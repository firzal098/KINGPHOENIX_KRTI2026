#!/usr/bin/env python3
"""
Pixel-Innovation Extended Kalman Filter (EKF) Core for Gate Tracking.

Maintains a 2D persistent state in the fixed local ENU (world) frame:
    x_g = [X_g, Y_g]^T
Predicts gate corner image projections h(x_g) given known 3D gate geometry,
known nominal altitude Z_g, known gate orientation yaw_g, and current drone pose.
Applies image-space measurement updates using YOLO corner pixel keypoints.
Derives body FLU relative [forward, lateral] on-demand for the RL policy.
"""

from enum import Enum
import math
import numpy as np
import cv2


class TrackingState(Enum):
    BOOTSTRAP = "BOOTSTRAP"
    TRACKING = "TRACKING"
    REJECTED = "REJECTED"
    RECOVERY = "RECOVERY"


def quat_to_rot_matrix(q):
    """
    Converts quaternion [w, x, y, z] to 3x3 rotation matrix R_WB (Body FLU -> World ENU).
    Maps body vector v_B to world vector v_W: v_W = R_WB @ v_B.
    """
    w, x, y, z = float(q[0]), float(q[1]), float(q[2]), float(q[3])
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm < 1e-8:
        return np.eye(3, dtype=np.float64)
    w, x, y, z = w / norm, x / norm, y / norm, z / norm

    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z),       2.0 * (x * z + w * y)],
        [2.0 * (x * y + w * z),       1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)],
        [2.0 * (x * z - w * y),       2.0 * (y * z + w * x),       1.0 - 2.0 * (x * x + y * y)]
    ], dtype=np.float64)


def get_camera_to_body_rotation(camera_pitch_deg=0.0):
    """
    Constructs R_BC: rotation matrix mapping Camera RDF vectors to Body FLU vectors.
    RDF: +X Right, +Y Down, +Z Forward.
    FLU: +X Forward, +Y Left, +Z Up.
    camera_pitch_deg: camera tilt in degrees (e.g. +15 deg looking upward, 0 deg horizontal).
    v_B = R_BC @ v_C.
    """
    pitch_rad = math.radians(float(camera_pitch_deg))
    sin_p = math.sin(pitch_rad)
    cos_p = math.cos(pitch_rad)

    return np.array([
        [ 0.0,   sin_p,  cos_p],
        [-1.0,     0.0,    0.0],
        [ 0.0,  -cos_p,  sin_p]
    ], dtype=np.float64)


def world_to_camera_rdf(p_w, drone_pos_w, R_wb, R_bc, p_cb=None):
    """
    Transforms world 3D point p_w (in ENU) into camera optical frame (in RDF).
    Formula:
        p_c = R_bc.T @ (R_wb.T @ (p_w - drone_pos_w) - p_cb)
    """
    p_w = np.asarray(p_w, dtype=np.float64).reshape(3)
    drone_pos_w = np.asarray(drone_pos_w, dtype=np.float64).reshape(3)
    if p_cb is None:
        p_cb = np.zeros(3, dtype=np.float64)
    else:
        p_cb = np.asarray(p_cb, dtype=np.float64).reshape(3)

    # 1. World ENU -> Body FLU
    p_b = R_wb.T @ (p_w - drone_pos_w) - p_cb
    # 2. Body FLU -> Camera RDF
    p_c = R_bc.T @ p_b
    return p_c


def camera_rdf_to_world(p_c, drone_pos_w, R_wb, R_bc, p_cb=None):
    """
    Transforms camera optical 3D point p_c (in RDF) into world frame (in ENU).
    Inverse of world_to_camera_rdf:
        p_b = R_bc @ p_c + p_cb
        p_w = drone_pos_w + R_wb @ p_b
    """
    p_c = np.asarray(p_c, dtype=np.float64).reshape(3)
    drone_pos_w = np.asarray(drone_pos_w, dtype=np.float64).reshape(3)
    if p_cb is None:
        p_cb = np.zeros(3, dtype=np.float64)
    else:
        p_cb = np.asarray(p_cb, dtype=np.float64).reshape(3)

    p_b = R_bc @ p_c + p_cb
    p_w = drone_pos_w + R_wb @ p_b
    return p_w


class PixelInnovationEKF:
    """
    Pixel-Innovation Extended Kalman Filter for continuous gate tracking.
    """

    def __init__(self,
                 gate_width=1.9,
                 gate_height=2.0,
                 sigma_pixel=3.0,
                 process_noise_q=1e-4,
                 p0_sigma=2.0,
                 camera_pitch_deg=0.0,
                 p_cam_in_body=None,
                 max_consecutive_rejections=5,
                 cond_h_max=1000.0,
                 max_innovation_px_sanity=60.0):
        self.gate_w = float(gate_width)
        self.gate_h = float(gate_height)
        self.sigma_pixel = float(sigma_pixel)
        self.process_noise_q = float(process_noise_q)
        self.p0_sigma = float(p0_sigma)
        self.camera_pitch_deg = float(camera_pitch_deg)
        self.p_cb = np.zeros(3, dtype=np.float64) if p_cam_in_body is None else np.asarray(p_cam_in_body, dtype=np.float64).reshape(3)
        self.max_consecutive_rejections = int(max_consecutive_rejections)
        self.cond_h_max = float(cond_h_max)
        self.max_innovation_px_sanity = float(max_innovation_px_sanity)

        self.R_bc = get_camera_to_body_rotation(self.camera_pitch_deg)

        # 3D object model points in gate-centric frame for IPPE solvePnP:
        # [X_right, Y_down, Z_fwd] in OpenCV object frame
        hw = self.gate_w / 2.0
        hh = self.gate_h / 2.0
        self.object_points_4p = np.array([
            [-hw, -hh, 0.0],  # Corner 0: Top-Left (YOLO index 6)
            [ hw, -hh, 0.0],  # Corner 1: Top-Right (YOLO index 7)
            [ hw,  hh, 0.0],  # Corner 2: Bottom-Right (YOLO index 8)
            [-hw,  hh, 0.0]   # Corner 3: Bottom-Left (YOLO index 9)
        ], dtype=np.float32)

        # EKF State: [X_g, Y_g]^T in local ENU
        self.x = np.zeros((2, 1), dtype=np.float64)
        self.P = np.eye(2, dtype=np.float64) * (self.p0_sigma ** 2)

        # Active Track Context
        self.gate_z = 0.75
        self.gate_yaw = 0.0
        self.is_initialized = False
        self.state_fsm = TrackingState.BOOTSTRAP
        self.consecutive_rejections = 0

        # Telemetry / Diagnostic cache
        self.last_predicted_pixels = None
        self.last_measured_pixels = None
        self.last_innovation = None
        self.last_H = None
        self.last_K = None
        self.last_nis = 0.0
        self.last_cond_H = 0.0
        self.last_delta_x = np.zeros((2, 1), dtype=np.float64)

    def set_camera_pitch(self, camera_pitch_deg):
        self.camera_pitch_deg = float(camera_pitch_deg)
        self.R_bc = get_camera_to_body_rotation(self.camera_pitch_deg)

    def initialize_state(self, x_init, y_init, p0_sigma=None):
        self.x = np.array([[float(x_init)], [float(y_init)]], dtype=np.float64)
        sig = self.p0_sigma if p0_sigma is None else float(p0_sigma)
        self.P = np.eye(2, dtype=np.float64) * (sig ** 2)
        self.is_initialized = True
        self.state_fsm = TrackingState.TRACKING
        self.consecutive_rejections = 0

    def predict(self, dt):
        """
        EKF Prediction step for static world gate:
            x_{k|k-1} = x_{k-1|k-1}
            P_{k|k-1} = P_{k-1|k-1} + Q * dt
        """
        if not self.is_initialized:
            return
        dt = max(0.0, float(dt))
        # Q represents static gate prior / map uncertainty accumulation
        Q = np.eye(2, dtype=np.float64) * (self.process_noise_q * dt)
        self.P = self.P + Q

    def get_gate_corners_world(self, state_xy, gate_z=None, gate_yaw=None):
        """
        Constructs the 4 outer 3D corner coordinates in World ENU frame.
        Corners: Top-Left, Top-Right, Bottom-Right, Bottom-Left.
        """
        x_g = float(state_xy[0, 0] if isinstance(state_xy, np.ndarray) and state_xy.ndim == 2 else state_xy[0])
        y_g = float(state_xy[1, 0] if isinstance(state_xy, np.ndarray) and state_xy.ndim == 2 else state_xy[1])
        z_g = self.gate_z if gate_z is None else float(gate_z)
        yaw_g = self.gate_yaw if gate_yaw is None else float(gate_yaw)

        hw = self.gate_w / 2.0
        hh = self.gate_h / 2.0

        p_center = np.array([x_g, y_g, z_g], dtype=np.float64)

        # In horizontal plane:
        # Gate normal n_g = [cos(yaw), sin(yaw), 0]
        # Gate lateral u_lat = [-sin(yaw), cos(yaw), 0] (perpendicular to normal, points LEFT in ENU)
        u_lat = np.array([-math.sin(yaw_g), math.cos(yaw_g), 0.0], dtype=np.float64)
        u_vert = np.array([0.0, 0.0, 1.0], dtype=np.float64)

        # 4 Outer corners in ENU:
        # Looking along gate normal (+u_lat is LEFT, +u_vert is UP):
        # Top-Left: +u_lat, +u_vert (image u < cx, v < cy)
        # Top-Right: -u_lat, +u_vert (image u > cx, v < cy)
        # Bottom-Right: -u_lat, -u_vert (image u > cx, v > cy)
        # Bottom-Left: +u_lat, -u_vert (image u < cx, v > cy)
        c_tl = p_center + hw * u_lat + hh * u_vert
        c_tr = p_center - hw * u_lat + hh * u_vert
        c_br = p_center - hw * u_lat - hh * u_vert
        c_bl = p_center + hw * u_lat - hh * u_vert

        return [c_tl, c_tr, c_br, c_bl]

    def predict_gate_pixels(self, state_xy, drone_pos_w, R_wb, K_matrix,
                            gate_z=None, gate_yaw=None, visible_corner_indices=None):
        """
        Projects gate corners into camera pixel space h(x_g).
        Returns:
            predicted_pixels: np.ndarray of shape (2M, 1) where M is len(visible_corner_indices)
            valid: bool (False if any corner is behind the camera Z_c < 0.3m)
        """
        if visible_corner_indices is None:
            visible_corner_indices = [0, 1, 2, 3]

        corners_w = self.get_gate_corners_world(state_xy, gate_z, gate_yaw)
        fx = K_matrix[0, 0]
        fy = K_matrix[1, 1]
        cx = K_matrix[0, 2]
        cy = K_matrix[1, 2]

        pix_list = []
        for idx in visible_corner_indices:
            c_w = corners_w[idx]
            c_cam = world_to_camera_rdf(c_w, drone_pos_w, R_wb, self.R_bc, self.p_cb)

            # Optical depth validation (RDF Z is forward)
            if c_cam[2] < 0.3:
                return None, False

            u = fx * (c_cam[0] / c_cam[2]) + cx
            v = fy * (c_cam[1] / c_cam[2]) + cy
            pix_list.extend([u, v])

        return np.array(pix_list, dtype=np.float64).reshape(-1, 1), True

    def compute_jacobian_finite_difference(self, state_xy, drone_pos_w, R_wb, K_matrix,
                                           visible_corner_indices=None, epsilon=0.05):
        """
        Computes measurement Jacobian H = dh/dx in R^{2M x 2} via finite differences.
        Also evaluates SVD of H to check observability / conditioning.
        """
        if visible_corner_indices is None:
            visible_corner_indices = [0, 1, 2, 3]

        h0, ok0 = self.predict_gate_pixels(state_xy, drone_pos_w, R_wb, K_matrix,
                                           visible_corner_indices=visible_corner_indices)
        if not ok0 or h0 is None:
            return None, float('inf')

        # Perturb X_g
        x_plus_dx = state_xy.copy()
        x_plus_dx[0, 0] += epsilon
        h_x, ok_x = self.predict_gate_pixels(x_plus_dx, drone_pos_w, R_wb, K_matrix,
                                             visible_corner_indices=visible_corner_indices)
        if not ok_x or h_x is None:
            return None, float('inf')

        # Perturb Y_g
        x_plus_dy = state_xy.copy()
        x_plus_dy[1, 0] += epsilon
        h_y, ok_y = self.predict_gate_pixels(x_plus_dy, drone_pos_w, R_wb, K_matrix,
                                             visible_corner_indices=visible_corner_indices)
        if not ok_y or h_y is None:
            return None, float('inf')

        col_x = (h_x - h0) / epsilon
        col_y = (h_y - h0) / epsilon
        H = np.column_stack([col_x, col_y])  # Shape (2M, 2)

        # SVD conditioning check
        u, s, vh = np.linalg.svd(H)
        sigma_max = s[0]
        sigma_min = s[-1] if len(s) > 1 else 0.0
        cond = (sigma_max / sigma_min) if sigma_min > 1e-12 else float('inf')

        return H, cond

    def bootstrap_pnp(self, corners_2d, K_matrix, dist_coeffs, drone_pos_w, R_wb, prior_norm_w=None):
        """
        Initializes the gate state in World ENU from YOLO corner keypoints using IPPE solvePnP.
        Resolves planar ambiguity and projects camera-relative pose into world coordinates.
        corners_2d: np.ndarray shape (4, 2) containing [u, v] for TL, TR, BR, BL.
        Returns:
            success: bool
            p_gate_w: np.ndarray (3,) [X_g, Y_g, Z_g] in world ENU
        """
        corners_2d = np.ascontiguousarray(corners_2d, dtype=np.float32).reshape(4, 2)
        dist = np.zeros((5, 1), dtype=np.float64) if dist_coeffs is None else np.asarray(dist_coeffs, dtype=np.float64)

        success, rvecs, tvecs, reproj_errs = cv2.solvePnPGeneric(
            self.object_points_4p,
            corners_2d,
            np.asarray(K_matrix, dtype=np.float64),
            dist,
            flags=cv2.SOLVEPNP_IPPE
        )

        if not success or len(tvecs) == 0:
            return False, None

        # Select solution:
        # If two solutions exist, resolve planar ambiguity by checking reprojection error and alignment with prior normal
        best_idx = 0
        if len(tvecs) > 1:
            if reproj_errs is not None and len(reproj_errs) >= 2:
                err0 = float(reproj_errs[0][0])
                err1 = float(reproj_errs[1][0])
                if abs(err0 - err1) > 0.5:
                    best_idx = 0 if err0 < err1 else 1
                else:
                    # Compare normals
                    if prior_norm_w is not None:
                        R0, _ = cv2.Rodrigues(rvecs[0])
                        R1, _ = cv2.Rodrigues(rvecs[1])
                        # Object Z normal [0, 0, 1] in camera frame:
                        norm0_c = R0 @ np.array([0, 0, 1], dtype=np.float64)
                        norm1_c = R1 @ np.array([0, 0, 1], dtype=np.float64)
                        # Transform to world:
                        norm0_w = R_wb @ (self.R_bc @ norm0_c)
                        norm1_w = R_wb @ (self.R_bc @ norm1_c)
                        dot0 = np.dot(norm0_w, prior_norm_w)
                        dot1 = np.dot(norm1_w, prior_norm_w)
                        best_idx = 0 if dot0 > dot1 else 1

        t_cam = tvecs[best_idx].flatten()  # Center translation in camera optical RDF frame
        if t_cam[2] < 0.3:
            return False, None

        # Transform from camera RDF to World ENU
        p_gate_w = camera_rdf_to_world(t_cam, drone_pos_w, R_wb, self.R_bc, self.p_cb)

        # Initialize EKF
        self.initialize_state(p_gate_w[0], p_gate_w[1])
        return True, p_gate_w

    def update(self, measured_corners_dict, drone_pos_w, R_wb, K_matrix,
               gate_z=None, gate_yaw=None, P_drone=None):
        """
        Executes measurement update using pixel innovation.
        measured_corners_dict: dict mapping corner index (0..3) to (u, v, confidence)
        P_drone: optional 6x6 drone pose covariance for R_eff expansion (Stage 2)
        Returns:
            accepted: bool
            diagnostics: dict
        """
        if not self.is_initialized:
            return False, {"status": "NOT_INITIALIZED"}

        # Extract visible corners with conf >= thresh
        visible_indices = [i for i in [0, 1, 2, 3] if i in measured_corners_dict and measured_corners_dict[i] is not None]
        if len(visible_indices) < 3:
            # Observability requires at least 3 valid corners
            self.consecutive_rejections += 1
            if self.consecutive_rejections > self.max_consecutive_rejections:
                self.state_fsm = TrackingState.RECOVERY
            else:
                self.state_fsm = TrackingState.REJECTED
            return False, {"status": "INSUFFICIENT_CORNERS", "num_corners": len(visible_indices)}

        # Construct measurement vector z
        z_list = []
        for idx in visible_indices:
            u, v, _ = measured_corners_dict[idx]
            z_list.extend([float(u), float(v)])
        z = np.array(z_list, dtype=np.float64).reshape(-1, 1)

        # 1. Predicted pixels h(x_g)
        h0, ok_h = self.predict_gate_pixels(self.x, drone_pos_w, R_wb, K_matrix,
                                            gate_z=gate_z, gate_yaw=gate_yaw,
                                            visible_corner_indices=visible_indices)
        if not ok_h or h0 is None:
            self.consecutive_rejections += 1
            self.state_fsm = TrackingState.RECOVERY if self.consecutive_rejections > self.max_consecutive_rejections else TrackingState.REJECTED
            return False, {"status": "PROJECTION_FAILED"}

        # 2. Innovation
        y = z - h0
        max_abs_innov = np.max(np.abs(y))

        # Secondary sanity ceiling
        if max_abs_innov > self.max_innovation_px_sanity:
            self.consecutive_rejections += 1
            self.state_fsm = TrackingState.RECOVERY if self.consecutive_rejections > self.max_consecutive_rejections else TrackingState.REJECTED
            return False, {"status": "EXCEEDED_SANITY_CEILING", "max_abs_innov": max_abs_innov}

        # 3. Measurement Jacobian H
        H, cond_H = self.compute_jacobian_finite_difference(self.x, drone_pos_w, R_wb, K_matrix,
                                                            visible_corner_indices=visible_indices)
        if H is None or cond_H > self.cond_h_max:
            self.consecutive_rejections += 1
            self.state_fsm = TrackingState.RECOVERY if self.consecutive_rejections > self.max_consecutive_rejections else TrackingState.REJECTED
            return False, {"status": "POOR_CONDITIONING", "cond_H": cond_H}

        # 4. Measurement Covariance R_eff
        num_meas = len(z_list)
        R_yolo = np.eye(num_meas, dtype=np.float64) * (self.sigma_pixel ** 2)

        # Stage 2: add drone uncertainty H_d P_d H_d^T if provided
        R_eff = R_yolo
        if P_drone is not None:
            # Numerical H_d perturbation w.r.t drone position [x, y, z]
            eps_d = 0.05
            H_dp_list = []
            for d_idx in range(3):
                dp_plus = drone_pos_w.copy()
                dp_plus[d_idx] += eps_d
                h_dp, ok_dp = self.predict_gate_pixels(self.x, dp_plus, R_wb, K_matrix,
                                                       gate_z=gate_z, gate_yaw=gate_yaw,
                                                       visible_corner_indices=visible_indices)
                if ok_dp and h_dp is not None:
                    H_dp_list.append((h_dp - h0) / eps_d)
                else:
                    H_dp_list.append(np.zeros_like(h0))
            H_dp = np.column_stack(H_dp_list)  # (2M, 3)
            # Add positional variance component
            P_pos = P_drone[:3, :3] if P_drone.shape[0] >= 3 else np.eye(3) * 0.04
            R_eff = R_eff + H_dp @ P_pos @ H_dp.T

        # 5. Innovation Covariance S
        S = H @ self.P @ H.T + R_eff

        # 6. Normalized Innovation Squared (NIS) Mahalanobis check
        try:
            S_inv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            self.consecutive_rejections += 1
            return False, {"status": "SINGULAR_S"}

        nis = float((y.T @ S_inv @ y)[0, 0])

        # Chi-square 99% threshold for 2M degrees of freedom
        # 4 corners (8 DoF): 20.09. 3 corners (6 DoF): 16.81.
        dof = num_meas
        chi2_thresh = 20.09 if dof >= 8 else 16.81

        if nis > chi2_thresh:
            self.consecutive_rejections += 1
            if self.consecutive_rejections > self.max_consecutive_rejections:
                self.state_fsm = TrackingState.RECOVERY
            else:
                self.state_fsm = TrackingState.REJECTED
            return False, {"status": "NIS_GATED", "nis": nis, "thresh": chi2_thresh}

        # 7. Kalman Gain
        K = self.P @ H.T @ S_inv  # Shape (2, 2M)

        # 8. State Update
        delta_x = K @ y  # Shape (2, 1)
        self.x = self.x + delta_x

        # 9. Joseph-form Stabilized Covariance Update:
        # P = (I - K H) P (I - K H)^T + K R_eff K^T
        I_KH = np.eye(2, dtype=np.float64) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R_eff @ K.T

        # Success: reset rejection count and set tracking state
        self.consecutive_rejections = 0
        self.state_fsm = TrackingState.TRACKING

        # Cache telemetry
        self.last_predicted_pixels = h0
        self.last_measured_pixels = z
        self.last_innovation = y
        self.last_H = H
        self.last_K = K
        self.last_nis = nis
        self.last_cond_H = cond_H
        self.last_delta_x = delta_x

        return True, {
            "status": "ACCEPTED",
            "nis": nis,
            "cond_H": cond_H,
            "delta_x": delta_x.flatten().tolist(),
            "state": self.x.flatten().tolist()
        }

    def get_relative_body_flu(self, drone_pos_w, R_wb):
        """
        Derives the gate center in Body FLU coordinates [forward, lateral, up].
        f = p_g_b[0], l = p_g_b[1].
        """
        p_g_w = np.array([self.x[0, 0], self.x[1, 0], self.gate_z], dtype=np.float64)
        p_g_b = R_wb.T @ (p_g_w - np.asarray(drone_pos_w, dtype=np.float64).reshape(3))
        forward = float(p_g_b[0])
        lateral = float(p_g_b[1])
        up = float(p_g_b[2])
        return forward, lateral, up
