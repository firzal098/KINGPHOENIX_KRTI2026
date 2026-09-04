#!/usr/bin/env python3
"""
ROS 2 Jazzy Gate Estimator Node.
Supports two selectable estimation methods via parameter 'estimation_method':
  1. 'pixel_innovation' (Default): Continuous direct image-space measurement update EKF
     with persistent state [X_g, Y_g] in fixed local ENU, initialized directly from
     surveyed track prior constants without PnP bootstrapping, using SVD-conditioned Jacobian.
  2. 'pnp': Legacy 3D pose EKF solving IPPE on every frame.
Publishes /estimator/refined_gate_poses (PoseArray in 'map' ENU frame), preserving
100% downstream compatibility with Policy.hpp, GCS, and RViz.
"""

import math
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PolygonStamped, PoseStamped, PoseArray, Pose, Point, Quaternion
from std_msgs.msg import Int32, Float64MultiArray, Header
from visualization_msgs.msg import MarkerArray, Marker
from std_srvs.srv import Trigger

from cuda_gate_inference.pixel_innovation_ekf import (
    PixelInnovationEKF,
    TrackingState,
    quat_to_rot_matrix,
    get_camera_to_body_rotation
)


class GateTrackState:
    def __init__(self, gate_id, pos_enu, norm_enu, yaw_rad):
        self.id = gate_id
        self.position_enu = np.array(pos_enu, dtype=np.float64)
        self.prior_pos_enu = np.array(pos_enu, dtype=np.float64)
        self.normal_enu = np.array(norm_enu, dtype=np.float64) / max(1e-6, np.linalg.norm(norm_enu))
        self.yaw_rad = yaw_rad
        self.covariance = np.eye(3, dtype=np.float64) * 4.0

    def get_orientation_quaternion(self):
        """Constructs quaternion from gate normal vector in ENU."""
        yaw = self.yaw_rad
        qw = math.cos(yaw / 2.0)
        qz = math.sin(yaw / 2.0)
        return 0.0, 0.0, qz, qw


class GateEstimatorNode(Node):
    """
    ROS 2 Gate Estimator Node managing multi-gate track geometry and Pixel Innovation EKF.
    """

    def __init__(self, **kwargs):
        super().__init__('gate_estimator_node', **kwargs)

        # Parameters
        self.declare_parameter('estimation_method', 'pixel_innovation')  # 'pixel_innovation' or 'pnp'
        self.declare_parameter('gate_width_m', 1.9)
        self.declare_parameter('gate_height_m', 2.0)
        self.declare_parameter('sigma_pixel', 3.0)
        self.declare_parameter('process_noise_q', 1e-4)
        self.declare_parameter('p0_sigma', 2.0)
        self.declare_parameter('camera_pitch_deg', 15.0)
        self.declare_parameter('anchor_priors_at_takeoff', True)
        self.declare_parameter('cond_h_max', 1000.0)
        self.declare_parameter('max_innovation_px_sanity', 120.0)
        self.declare_parameter('max_consecutive_rejections', 5)
        self.declare_parameter('max_refine_distance_m', 36.0)
        self.declare_parameter('v7', True)

        self.estimation_method = str(self.get_parameter('estimation_method').value).lower()
        self.gate_w = float(self.get_parameter('gate_width_m').value)
        self.gate_h = float(self.get_parameter('gate_height_m').value)
        self.sigma_pixel = float(self.get_parameter('sigma_pixel').value)
        self.process_noise_q = float(self.get_parameter('process_noise_q').value)
        self.p0_sigma = float(self.get_parameter('p0_sigma').value)
        self.camera_pitch_deg = float(self.get_parameter('camera_pitch_deg').value)
        self.anchor_at_takeoff = bool(self.get_parameter('anchor_priors_at_takeoff').value)
        self.cond_h_max = float(self.get_parameter('cond_h_max').value)
        self.max_innov_sanity = float(self.get_parameter('max_innovation_px_sanity').value)
        self.max_rejections = int(self.get_parameter('max_consecutive_rejections').value)
        self.max_refine_dist = float(self.get_parameter('max_refine_distance_m').value)
        self.v7 = bool(self.get_parameter('v7').value)

        # Dynamic parameter reconfiguration
        self.add_on_set_parameters_callback(self.parameters_callback)

        # Core EKF Instance
        self.ekf = PixelInnovationEKF(
            gate_width=self.gate_w,
            gate_height=self.gate_h,
            sigma_pixel=self.sigma_pixel,
            process_noise_q=self.process_noise_q,
            p0_sigma=self.p0_sigma,
            camera_pitch_deg=self.camera_pitch_deg,
            max_consecutive_rejections=self.max_rejections,
            cond_h_max=self.cond_h_max,
            max_innovation_px_sanity=self.max_innov_sanity
        )

        # State Variables
        self.camera_matrix = None
        self.dist_coeffs = None
        self.drone_pos_w = np.zeros(3, dtype=np.float64)
        self.drone_quat_w = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)  # [w, x, y, z]
        self.R_wb = np.eye(3, dtype=np.float64)
        self.drone_pose_received = False

        self.initial_drone_pos = np.zeros(3, dtype=np.float64)
        self.initial_drone_rot = np.eye(3, dtype=np.float64)
        self.initial_pose_captured = False

        self.active_target_gate_idx = 0
        self.gates = []
        self.last_predict_time = self.get_clock().now()

        # Initialize Default Gates
        self.initialize_gate_priors()

        # QoS Profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Subscriptions
        self.sub_corners = self.create_subscription(
            PolygonStamped, '/perception/gate_corners_2d', self.corners_callback, 10
        )
        self.sub_cam_info = self.create_subscription(
            CameraInfo, '/camera/camera_info', self.camera_info_callback, 10
        )
        self.sub_drone_pose = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self.drone_pose_callback, sensor_qos
        )
        self.sub_target_gate = self.create_subscription(
            Int32, '/controller/target_gate_index', self.target_gate_callback, 10
        )

        # Publishers
        self.pub_refined_poses = self.create_publisher(
            PoseArray, '/estimator/refined_gate_poses', 10
        )
        self.pub_markers = self.create_publisher(
            MarkerArray, '/estimator/gate_markers', 10
        )
        self.pub_diagnostics = self.create_publisher(
            Float64MultiArray, '/estimator/pixel_ekf_diagnostics', 10
        )
        self.pub_projected_corners = self.create_publisher(
            Float64MultiArray, '/estimator/projected_gate_corners', 10
        )

        # Reset Service
        self.srv_reset = self.create_service(
            Trigger, '/estimator/reset_gates', self.handle_reset_service
        )

        # 30 Hz Timer for Publishing Estimates
        self.pub_timer = self.create_timer(0.03333, self.timer_callback)

        self.get_logger().info(
            f"Gate Estimator Node Initialized (Method: {self.estimation_method.upper()}, "
            f"Pitch: {self.camera_pitch_deg:.1f}°, Gate: {self.gate_w}x{self.gate_h}m)"
        )

    def initialize_gate_priors(self):
        """Initializes the 5 surveyed gate track priors anchored at initial drone pose."""
        self.gates = []
        # Priors in RDF frame relative to drone: [x_right, y_down, z_fwd], normal: [nx, ny, nz]
        priors_rdf = [
            (1,  0.41, -0.75, 29.33,  0.0, 0.0, 1.0),
            (2,  5.46, -0.75, 19.31,  0.0, 0.0, 1.0),
            (3,  9.49, -0.75, 10.51,  1.0, 0.0, 0.0),
            (4, 12.41, -0.75, 12.43,  0.0, 0.0, 1.0),
            (5, 17.47, -0.75, 29.38,  0.0, 0.0, 1.0)
        ]

        for gid, x_rdf, y_rdf, z_rdf, nx_rdf, ny_rdf, nz_rdf in priors_rdf:
            # Convert RDF -> ENU: x_enu = z_rdf, y_enu = -x_rdf, z_enu = -y_rdf (+0.75m)
            rel_pos_enu = np.array([z_rdf, -x_rdf, -y_rdf], dtype=np.float64)
            rel_norm_enu = np.array([nz_rdf, -nx_rdf, -ny_rdf], dtype=np.float64)

            # Anchor at initial drone pose
            pos_enu = self.initial_drone_pos + self.initial_drone_rot @ rel_pos_enu
            norm_enu = self.initial_drone_rot @ rel_norm_enu
            norm_enu /= max(1e-6, np.linalg.norm(norm_enu))

            yaw_rad = math.atan2(norm_enu[1], norm_enu[0])
            self.gates.append(GateTrackState(gid, pos_enu, norm_enu, yaw_rad))

        self.sync_ekf_with_active_gate()

    def sync_ekf_with_active_gate(self):
        """Syncs the PixelInnovationEKF context with the currently active target gate directly using defined prior constants."""
        if 0 <= self.active_target_gate_idx < len(self.gates):
            active_gate = self.gates[self.active_target_gate_idx]
            self.ekf.gate_z = float(active_gate.position_enu[2])
            self.ekf.gate_yaw = float(active_gate.yaw_rad)
            # Initialize directly from defined prior position constants without PnP bootstrap
            self.ekf.initialize_state(active_gate.position_enu[0], active_gate.position_enu[1], self.p0_sigma)
            self.ekf.state_fsm = TrackingState.TRACKING
            self.get_logger().info(
                f"Gate {active_gate.id} initialized from defined prior constants at: "
                f"[{active_gate.position_enu[0]:.2f}, {active_gate.position_enu[1]:.2f}, {active_gate.position_enu[2]:.2f}]"
            )

    @staticmethod
    def _extract_param_value(param):
        val = getattr(param, 'value', None)
        if hasattr(val, 'type') and hasattr(val, 'double_value'):
            if val.type == 3:
                return val.double_value
            elif val.type == 2:
                return val.integer_value
            elif val.type == 4:
                return val.string_value
            elif val.type == 1:
                return val.bool_value
        return val

    def parameters_callback(self, params):
        for param in params:
            val = self._extract_param_value(param)
            if val is None:
                continue
            if param.name == 'camera_pitch_deg':
                self.camera_pitch_deg = float(val)
                self.ekf.set_camera_pitch(self.camera_pitch_deg)
                self.get_logger().info(f"Dynamic param update: camera_pitch_deg = {self.camera_pitch_deg:.1f}°")
            elif param.name == 'sigma_pixel':
                self.sigma_pixel = float(val)
                self.ekf.sigma_pixel = self.sigma_pixel
                self.get_logger().info(f"Dynamic param update: sigma_pixel = {self.sigma_pixel:.2f}px")
            elif param.name == 'process_noise_q':
                self.process_noise_q = float(val)
                self.ekf.process_noise_q = self.process_noise_q
                self.get_logger().info(f"Dynamic param update: process_noise_q = {self.process_noise_q:.2e}")
        return SetParametersResult(successful=True)

    def camera_info_callback(self, msg: CameraInfo):
        first_time = (self.camera_matrix is None)
        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape((3, 3))
        self.dist_coeffs = np.array(msg.d, dtype=np.float64) if len(msg.d) > 0 else np.zeros((5, 1))
        if first_time:
            self.get_logger().info(
                f"Camera intrinsics registered: fx={self.camera_matrix[0,0]:.2f}, fy={self.camera_matrix[1,1]:.2f}, "
                f"cx={self.camera_matrix[0,2]:.1f}, cy={self.camera_matrix[1,2]:.1f}"
            )

    def drone_pose_callback(self, msg: PoseStamped):
        self.drone_pos_w = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ], dtype=np.float64)

        self.drone_quat_w = np.array([
            msg.pose.orientation.w,
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z
        ], dtype=np.float64)

        self.R_wb = quat_to_rot_matrix(self.drone_quat_w)
        self.drone_pose_received = True

        if not self.initial_pose_captured:
            if self.anchor_at_takeoff:
                self.initial_drone_pos = self.drone_pos_w.copy()
                self.initial_drone_rot = self.R_wb.copy()
                self.initialize_gate_priors()
                self.get_logger().info(
                    f"Initial drone pose captured: [{self.initial_drone_pos[0]:.2f}, "
                    f"{self.initial_drone_pos[1]:.2f}, {self.initial_drone_pos[2]:.2f}]. Anchored 5 gates."
                )
            else:
                self.get_logger().info(
                    "Using fixed surveyed prior constants in map frame (anchor_priors_at_takeoff=False)."
                )
            self.initial_pose_captured = True

    def target_gate_callback(self, msg: Int32):
        if msg.data >= 0 and msg.data != self.active_target_gate_idx:
            old_idx = self.active_target_gate_idx
            self.active_target_gate_idx = msg.data
            self.get_logger().info(f"Active target gate transitioned: Gate {old_idx + 1} -> Gate {self.active_target_gate_idx + 1}")
            self.sync_ekf_with_active_gate()

    def handle_reset_service(self, request, response):
        if self.anchor_at_takeoff and self.drone_pose_received:
            self.initial_drone_pos = self.drone_pos_w.copy()
            self.initial_drone_rot = self.R_wb.copy()
        self.active_target_gate_idx = 0
        self.initialize_gate_priors()
        response.success = True
        response.message = "Gate track priors successfully reset."
        self.get_logger().info("Reset gates service triggered.")
        return response

    def corners_callback(self, msg: PolygonStamped):
        """Processes 2D corner keypoints from YOLO and executes measurement update."""
        if not self.drone_pose_received or self.camera_matrix is None:
            return

        now = self.get_clock().now()
        dt = (now - self.last_predict_time).nanoseconds / 1e9
        self.last_predict_time = now

        # EKF static covariance prediction
        self.ekf.predict(dt)

        if len(msg.polygon.points) < 3:
            return

        target_idx = self.active_target_gate_idx
        if target_idx < 0 or target_idx >= len(self.gates):
            return

        active_gate = self.gates[target_idx]

        # Gate ID Verification: ensure incoming corners strictly belong to the active target gate
        if msg.header.frame_id and msg.header.frame_id.startswith("gate_"):
            try:
                msg_gate_id = int(msg.header.frame_id.split("_")[1])
                if msg_gate_id != active_gate.id:
                    # Ignore corner keypoints from other gates
                    return
            except ValueError:
                pass

        # Distance Gating: Skip updates if drone is further than max_refine_dist from gate prior
        dist_to_gate = float(np.linalg.norm(active_gate.position_enu - self.drone_pos_w))
        if dist_to_gate > self.max_refine_dist:
            return

        # Special Rule: Skip direct visual updates for Gate 4 (it inherits offset from Gate 3)
        if target_idx == 3:
            return

        # Parse corner dictionary: 0: TL, 1: TR, 2: BR, 3: BL
        meas_dict = {}
        for k in range(min(4, len(msg.polygon.points))):
            pt = msg.polygon.points[k]
            # pt.x = u, pt.y = v, pt.z = confidence
            if pt.z >= 0.15:
                meas_dict[k] = (float(pt.x), float(pt.y), float(pt.z))

        # Mode Selection: Pixel Innovation vs PnP
        if self.estimation_method == 'pixel_innovation':
            self.process_pixel_innovation(meas_dict, active_gate)
        else:
            self.process_legacy_pnp(meas_dict, active_gate)

    def process_pixel_innovation(self, meas_dict, active_gate):
        """Executes Pixel Innovation EKF directly from defined prior constants (no PnP bootstrap)."""
        # Ensure EKF state is initialized with defined prior constants
        if not self.ekf.is_initialized or self.ekf.state_fsm == TrackingState.BOOTSTRAP:
            self.ekf.initialize_state(
                active_gate.prior_pos_enu[0],
                active_gate.prior_pos_enu[1],
                self.p0_sigma
            )
            self.ekf.gate_z = float(active_gate.position_enu[2])
            self.ekf.gate_yaw = float(active_gate.yaw_rad)
            self.ekf.state_fsm = TrackingState.TRACKING

        # In recovery state, reset directly to defined prior constants without PnP bootstrap
        if self.ekf.state_fsm == TrackingState.RECOVERY:
            self.ekf.initialize_state(
                active_gate.prior_pos_enu[0],
                active_gate.prior_pos_enu[1],
                self.p0_sigma
            )
            self.ekf.gate_z = float(active_gate.position_enu[2])
            self.ekf.gate_yaw = float(active_gate.yaw_rad)
            self.ekf.state_fsm = TrackingState.TRACKING
            self.get_logger().warn(
                f"Gate {active_gate.id} tracking recovery: reset to defined prior constants at "
                f"[{active_gate.prior_pos_enu[0]:.2f}, {active_gate.prior_pos_enu[1]:.2f}]",
                throttle_duration_sec=2.0
            )

        # Continuous Tracking Update directly via pixel innovation
        accepted, diag = self.ekf.update(
            meas_dict,
            self.drone_pos_w,
            self.R_wb,
            self.camera_matrix,
            gate_z=float(active_gate.position_enu[2]),
            gate_yaw=float(active_gate.yaw_rad)
        )

        if accepted:
            # Update active gate state in world ENU
            active_gate.position_enu[0] = float(self.ekf.x[0, 0])
            active_gate.position_enu[1] = float(self.ekf.x[1, 0])

            # Anti-Drift Clamping: Max 1.5m deviation from surveyed prior
            dev = active_gate.position_enu[:2] - active_gate.prior_pos_enu[:2]
            horiz_dev = float(np.linalg.norm(dev))
            if horiz_dev > 1.5:
                active_gate.position_enu[:2] = active_gate.prior_pos_enu[:2] + dev * (1.5 / horiz_dev)
                self.ekf.x[0, 0] = active_gate.position_enu[0]
                self.ekf.x[1, 0] = active_gate.position_enu[1]

            # Track offset propagation to downstream gates (Gate 1/2 -> 3, 4, 5)
            self.propagate_track_offsets(self.active_target_gate_idx)
        else:
            status = diag.get("status", "UNKNOWN")
            if status == "EXCEEDED_SANITY_CEILING":
                self.get_logger().warn(
                    f"Gate {active_gate.id} measurement rejected: max innovation {diag.get('max_abs_innov', 0.0):.1f}px "
                    f"> sanity ceiling {self.max_innov_sanity:.1f}px",
                    throttle_duration_sec=2.0
                )
            elif status == "NIS_GATED":
                self.get_logger().warn(
                    f"Gate {active_gate.id} measurement rejected by NIS gate: NIS {diag.get('nis', 0.0):.1f} "
                    f"> chi2 thresh {diag.get('thresh', 0.0):.1f}",
                    throttle_duration_sec=2.0
                )
            elif status == "POOR_CONDITIONING":
                self.get_logger().warn(
                    f"Gate {active_gate.id} measurement rejected: condition number {diag.get('cond_H', 0.0):.1f} "
                    f"> {self.cond_h_max:.1f}",
                    throttle_duration_sec=2.0
                )

        # Publish Diagnostics
        self.publish_diagnostics(diag, active_gate)

    def process_legacy_pnp(self, meas_dict, active_gate):
        """Fallback legacy PnP estimator mode."""
        if len(meas_dict) < 4:
            return

        corners_2d = np.array([[meas_dict[k][0], meas_dict[k][1]] for k in range(4)], dtype=np.float32)
        success, rvec, tvec = cv2.solvePnP(
            self.ekf.object_points_4p,
            corners_2d,
            self.camera_matrix,
            self.dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE
        )
        if not success:
            return

        t_cam = tvec.flatten()
        if t_cam[2] < 0.3:
            return

        # Camera -> World
        R_bc = get_camera_to_body_rotation(self.camera_pitch_deg)
        p_meas_w = self.drone_pos_w + self.R_wb @ (R_bc @ t_cam)

        # 3D Kalman Update
        z_meas = np.array(p_meas_w, dtype=np.float64)
        y = z_meas - active_gate.position_enu
        R_meas = np.eye(3, dtype=np.float64) * 4.0
        S = active_gate.covariance + R_meas
        K = active_gate.covariance @ np.linalg.inv(S)

        active_gate.position_enu += K @ y
        active_gate.covariance = (np.eye(3, dtype=np.float64) - K) @ active_gate.covariance
        self.propagate_track_offsets(self.active_target_gate_idx)

    def propagate_track_offsets(self, target_idx):
        """Propagates refined horizontal offsets to downstream gates (matching original behavior)."""
        if target_idx in [0, 1] and len(self.gates) >= 5:
            offset_1 = self.gates[0].position_enu[:2] - self.gates[0].prior_pos_enu[:2]
            offset_2 = self.gates[1].position_enu[:2] - self.gates[1].prior_pos_enu[:2]
            blended = 0.5 * (offset_1 + offset_2) if target_idx == 1 else offset_1

            self.gates[2].position_enu[:2] = self.gates[2].prior_pos_enu[:2] + blended
            self.gates[3].position_enu[:2] = self.gates[3].prior_pos_enu[:2] + blended
            self.gates[4].position_enu[:2] = self.gates[4].prior_pos_enu[:2] + 0.40 * blended
        elif target_idx == 2 and len(self.gates) >= 4:
            offset_3 = self.gates[2].position_enu[:2] - self.gates[2].prior_pos_enu[:2]
            self.gates[3].position_enu[:2] = self.gates[3].prior_pos_enu[:2] + offset_3

    def timer_callback(self):
        """Publishes refined gate poses and RViz markers at 30 Hz."""
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'map'

        # 1. Refined Gate Poses (PoseArray in 'map' frame)
        pose_array = PoseArray()
        pose_array.header = header

        for gate in self.gates:
            p = Pose()
            p.position.x = float(gate.position_enu[0])
            p.position.y = float(gate.position_enu[1])
            p.position.z = float(gate.position_enu[2])
            qx, qy, qz, qw = gate.get_orientation_quaternion()
            p.orientation.x = qx
            p.orientation.y = qy
            p.orientation.z = qz
            p.orientation.w = qw
            pose_array.poses.append(p)

        self.pub_refined_poses.publish(pose_array)

        # 2. RViz Markers
        self.publish_rviz_markers(header)

        # 3. Projected Gate Corners on Camera Image (Prior vs EKF)
        self.publish_projected_gate_corners()

    def publish_rviz_markers(self, header):
        marker_array = MarkerArray()
        for gate in self.gates:
            box = Marker()
            box.header = header
            box.ns = "gate_boxes"
            box.id = gate.id
            box.type = Marker.CUBE
            box.action = Marker.ADD
            box.pose.position.x = float(gate.position_enu[0])
            box.pose.position.y = float(gate.position_enu[1])
            box.pose.position.z = float(gate.position_enu[2])
            qx, qy, qz, qw = gate.get_orientation_quaternion()
            box.pose.orientation.x = qx
            box.pose.orientation.y = qy
            box.pose.orientation.z = qz
            box.pose.orientation.w = qw
            box.scale.x = 0.1
            box.scale.y = self.gate_w
            box.scale.z = self.gate_h
            box.color.r = 0.0
            box.color.g = 0.8
            box.color.b = 1.0
            box.color.a = 0.65
            marker_array.markers.append(box)

            # Sub-gates for Gate 3 and 4
            sub_offsets = []
            if self.v7:
                if gate.id == 3:
                    sub_offsets = [1.0, 2.0]
                elif gate.id == 4:
                    sub_offsets = [1.0, 2.0, 3.0]

            for s_idx, s_off in enumerate(sub_offsets):
                sub_box = Marker()
                sub_box.header = header
                sub_box.ns = "sub_gate_boxes"
                sub_box.id = gate.id * 10 + (s_idx + 1)
                sub_box.type = Marker.CUBE
                sub_box.action = Marker.ADD
                sub_pos = gate.position_enu + s_off * gate.normal_enu
                sub_box.pose.position.x = float(sub_pos[0])
                sub_box.pose.position.y = float(sub_pos[1])
                sub_box.pose.position.z = float(sub_pos[2])
                sub_box.pose.orientation = box.pose.orientation
                sub_box.scale = box.scale
                sub_box.color.r = 0.2
                sub_box.color.g = 0.9
                sub_box.color.b = 0.8
                sub_box.color.a = 0.45
                marker_array.markers.append(sub_box)

        self.pub_markers.publish(marker_array)

    def publish_diagnostics(self, diag, active_gate):
        """Publishes real-time telemetry diagnostics for sim-to-real analysis."""
        f_rel, l_rel, _ = self.ekf.get_relative_body_flu(self.drone_pos_w, self.R_wb)
        fsm_code = {
            TrackingState.BOOTSTRAP: 0.0,
            TrackingState.TRACKING: 1.0,
            TrackingState.REJECTED: 2.0,
            TrackingState.RECOVERY: 3.0
        }.get(self.ekf.state_fsm, -1.0)

        diag_data = [
            float(self.drone_pos_w[0]),
            float(self.drone_pos_w[1]),
            float(self.drone_pos_w[2]),
            float(active_gate.position_enu[0]),
            float(active_gate.position_enu[1]),
            float(active_gate.position_enu[2]),
            float(f_rel),
            float(l_rel),
            float(self.ekf.last_nis),
            float(self.ekf.last_cond_H),
            float(fsm_code),
            float(self.ekf.P[0, 0]),
            float(self.ekf.P[1, 1])
        ]

        msg = Float64MultiArray()
        msg.data = diag_data
        self.pub_diagnostics.publish(msg)

    def publish_projected_gate_corners(self):
        """Projects active gate prior and EKF estimated corners into camera pixels for visual debugging."""
        if (self.camera_matrix is None or not self.drone_pose_received
                or not (0 <= self.active_target_gate_idx < len(self.gates))):
            return

        active_gate = self.gates[self.active_target_gate_idx]

        # 1. Project EKF estimated corners
        ekf_xy = self.ekf.x if self.ekf.is_initialized else active_gate.prior_pos_enu[:2]
        pix_ekf, ok_ekf = self.ekf.predict_gate_pixels(
            ekf_xy, self.drone_pos_w, self.R_wb, self.camera_matrix,
            gate_z=float(active_gate.position_enu[2]),
            gate_yaw=float(active_gate.yaw_rad)
        )

        # 2. Project Prior surveyed corners
        pix_prior, ok_prior = self.ekf.predict_gate_pixels(
            active_gate.prior_pos_enu[:2], self.drone_pos_w, self.R_wb, self.camera_matrix,
            gate_z=float(active_gate.prior_pos_enu[2]),
            gate_yaw=float(active_gate.yaw_rad)
        )

        if ok_ekf and ok_prior and pix_ekf is not None and pix_prior is not None:
            fsm_code = {
                TrackingState.BOOTSTRAP: 0.0,
                TrackingState.TRACKING: 1.0,
                TrackingState.REJECTED: 2.0,
                TrackingState.RECOVERY: 3.0
            }.get(self.ekf.state_fsm, -1.0)

            max_innov = float(np.max(np.abs(self.ekf.last_innovation))) if self.ekf.last_innovation is not None else 0.0

            # Message payload: [gate_id, fsm_code, nis, max_innov, 8 ekf px, 8 prior px]
            data = [
                float(active_gate.id),
                float(fsm_code),
                float(self.ekf.last_nis),
                float(max_innov)
            ]
            data.extend(pix_ekf.flatten().tolist())
            data.extend(pix_prior.flatten().tolist())

            msg = Float64MultiArray()
            msg.data = data
            self.pub_projected_corners.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GateEstimatorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
