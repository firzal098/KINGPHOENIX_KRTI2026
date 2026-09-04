#!/usr/bin/env python3
"""
Standalone Gate Pixel Projection Visualizer Node.

Tests 3D-to-2D gate corner projection onto live camera feed WITHOUT running
the Gate Estimator EKF or YOLO inference.

Subscribes:
  - /camera/image_raw (sensor_msgs/Image - BEST_EFFORT)
  - /camera/camera_info (sensor_msgs/CameraInfo)
  - /mavros/local_position/pose (geometry_msgs/PoseStamped - BEST_EFFORT)
  - /controller/target_gate_index (std_msgs/Int32, optional)

Publishes:
  - /perception/debug_image (sensor_msgs/Image) - Always at 20 Hz
  - /perception/debug_image/compressed (sensor_msgs/CompressedImage) - Always at 20 Hz
"""

import math
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header, Int32

from cuda_gate_inference.pixel_innovation_ekf import (
    quat_to_rot_matrix,
    get_camera_to_body_rotation,
    world_to_camera_rdf
)


# Standard gate sets
DEFINED_PRIORS = [
    # (id, x_enu, y_enu, z_enu, yaw_rad)
    (1, 29.33,  -0.41, 0.75, 0.0),
    (2, 19.31,  -5.46, 0.75, 0.0),
    (3, 10.51,  -9.49, 0.75, math.pi / 2.0),
    (4, 12.43, -12.41, 0.75, 0.0),
    (5, 29.38, -17.47, 0.75, 0.0),
]

WEBOTS_GATES = [
    (1, 13.14,   0.12, 0.99, 0.0),
    (2,  8.09,  10.14, 0.99, 0.0),
    (3,  5.01,  17.99, 0.99, math.pi / 2.0),
    (4,  1.14,  15.02, 0.99, 0.0),
    (5, -3.92,   0.07, 0.99, 0.0),
]


class TestGateProjectionNode(Node):
    def __init__(self, **kwargs):
        super().__init__('test_gate_projection_node', **kwargs)

        # Parameters
        self.declare_parameter('camera_pitch_deg', 15.0)
        self.declare_parameter('gate_width_m', 1.9)
        self.declare_parameter('gate_height_m', 2.0)
        self.declare_parameter('prior_source', 'defined_constants')  # 'defined_constants', 'webots_world', 'custom'
        self.declare_parameter('show_all_gates', True)
        self.declare_parameter('target_gate_idx', 0)
        self.declare_parameter('custom_gate_x', 29.33)
        self.declare_parameter('custom_gate_y', -0.41)
        self.declare_parameter('custom_gate_z', 0.75)
        self.declare_parameter('custom_gate_yaw_deg', 0.0)

        self.camera_pitch_deg = float(self.get_parameter('camera_pitch_deg').value)
        self.gate_w = float(self.get_parameter('gate_width_m').value)
        self.gate_h = float(self.get_parameter('gate_height_m').value)
        self.prior_source = str(self.get_parameter('prior_source').value).lower()
        self.show_all_gates = bool(self.get_parameter('show_all_gates').value)
        self.target_gate_idx = int(self.get_parameter('target_gate_idx').value)

        # Dynamic parameter reconfiguration callback
        self.add_on_set_parameters_callback(self.parameters_callback)

        # State
        self.camera_matrix = None
        self.intrinsics_from_topic = False
        self.drone_pos_w = np.zeros(3, dtype=np.float64)
        self.drone_quat_w = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        self.R_wb = np.eye(3, dtype=np.float64)
        self.drone_pose_received = False
        self.last_log_time = self.get_clock().now()

        # Frame buffers for continuous rendering
        self.latest_raw_image = None
        self.latest_raw_header = None
        self.last_image_time = None

        # QoS for sensor data (BEST_EFFORT is mandatory for camera streams & telemetry)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Subscriptions
        self.sub_image = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, sensor_qos
        )
        self.sub_cam_info = self.create_subscription(
            CameraInfo, '/camera/camera_info', self.cam_info_callback, 10
        )
        self.sub_drone_pose = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self.drone_pose_callback, sensor_qos
        )
        self.sub_target_idx = self.create_subscription(
            Int32, '/controller/target_gate_index', self.target_idx_callback, 10
        )

        # Publishers (Always active)
        self.pub_debug_img = self.create_publisher(Image, '/perception/debug_image', 10)
        self.pub_debug_comp = self.create_publisher(CompressedImage, '/perception/debug_image/compressed', 10)

        # 20 Hz Continuous Render Timer (ensures topics are always streaming)
        self.render_timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info(
            f"=== Test Gate Projection Node Started ===\n"
            f"  Camera Pitch: {self.camera_pitch_deg:.1f} deg\n"
            f"  Gate Geometry: {self.gate_w:.2f}m W x {self.gate_h:.2f}m H\n"
            f"  Prior Source: {self.prior_source}\n"
            f"  Show All Gates: {self.show_all_gates}\n"
            f"Continuous 20 Hz stream active on /perception/debug_image & /perception/debug_image/compressed"
        )

    @staticmethod
    def _extract_param_value(param):
        val = getattr(param, 'value', None)
        if hasattr(val, 'type') and hasattr(val, 'double_value'):
            # rcl_interfaces.msg.ParameterValue
            if val.type == 3:  # PARAMETER_DOUBLE
                return val.double_value
            elif val.type == 2:  # PARAMETER_INTEGER
                return val.integer_value
            elif val.type == 4:  # PARAMETER_STRING
                return val.string_value
            elif val.type == 1:  # PARAMETER_BOOL
                return val.bool_value
        return val

    def parameters_callback(self, params):
        for param in params:
            val = self._extract_param_value(param)
            if val is None:
                continue
            if param.name == 'camera_pitch_deg':
                self.camera_pitch_deg = float(val)
                self.get_logger().info(f"Dynamic param update: camera_pitch_deg = {self.camera_pitch_deg:.1f} deg")
            elif param.name == 'gate_width_m':
                self.gate_w = float(val)
                self.get_logger().info(f"Dynamic param update: gate_width_m = {self.gate_w:.2f}m")
            elif param.name == 'gate_height_m':
                self.gate_h = float(val)
                self.get_logger().info(f"Dynamic param update: gate_height_m = {self.gate_h:.2f}m")
            elif param.name == 'prior_source':
                self.prior_source = str(val).lower()
                self.get_logger().info(f"Dynamic param update: prior_source = {self.prior_source}")
            elif param.name == 'show_all_gates':
                self.show_all_gates = bool(val)
                self.get_logger().info(f"Dynamic param update: show_all_gates = {self.show_all_gates}")
            elif param.name == 'target_gate_idx':
                self.target_gate_idx = int(val)
                self.get_logger().info(f"Dynamic param update: target_gate_idx = {self.target_gate_idx}")
        return SetParametersResult(successful=True)

    def cam_info_callback(self, msg: CameraInfo):
        first_time = not self.intrinsics_from_topic
        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape((3, 3))
        self.intrinsics_from_topic = True
        if first_time:
            self.get_logger().info(
                f"Camera intrinsics received from topic: fx={self.camera_matrix[0,0]:.1f}, "
                f"fy={self.camera_matrix[1,1]:.1f}, cx={self.camera_matrix[0,2]:.1f}, cy={self.camera_matrix[1,2]:.1f}"
            )

    def drone_pose_callback(self, msg: PoseStamped):
        p = msg.pose.position
        o = msg.pose.orientation
        self.drone_pos_w = np.array([p.x, p.y, p.z], dtype=np.float64)
        self.drone_quat_w = np.array([o.w, o.x, o.y, o.z], dtype=np.float64)
        self.R_wb = quat_to_rot_matrix(self.drone_quat_w)
        self.drone_pose_received = True

    def target_idx_callback(self, msg: Int32):
        if msg.data >= 0:
            self.target_gate_idx = msg.data

    def get_gate_list(self):
        """Return list of (gate_id, pos_enu, yaw_rad) to project."""
        if self.prior_source == 'custom':
            gx = float(self.get_parameter('custom_gate_x').value)
            gy = float(self.get_parameter('custom_gate_y').value)
            gz = float(self.get_parameter('custom_gate_z').value)
            gyaw = math.radians(float(self.get_parameter('custom_gate_yaw_deg').value))
            return [(1, np.array([gx, gy, gz]), gyaw)]
        elif self.prior_source == 'webots_world':
            return [(gid, np.array([x, y, z]), yaw) for gid, x, y, z, yaw in WEBOTS_GATES]
        else:
            return [(gid, np.array([x, y, z]), yaw) for gid, x, y, z, yaw in DEFINED_PRIORS]

    def compute_gate_corners_world(self, center_enu, yaw_rad):
        """Compute 4 corners in ENU: [TL, TR, BR, BL]."""
        hw = self.gate_w / 2.0
        hh = self.gate_h / 2.0
        u_lat = np.array([-math.sin(yaw_rad), math.cos(yaw_rad), 0.0], dtype=np.float64)
        u_vert = np.array([0.0, 0.0, 1.0], dtype=np.float64)

        c_tl = center_enu + hw * u_lat + hh * u_vert
        c_tr = center_enu - hw * u_lat + hh * u_vert
        c_br = center_enu - hw * u_lat - hh * u_vert
        c_bl = center_enu + hw * u_lat - hh * u_vert
        return [c_tl, c_tr, c_br, c_bl]

    def project_corner(self, corner_enu, R_bc, K):
        """Transform corner ENU -> camera RDF -> 2D pixel (u, v) using camera matrix K."""
        p_c = world_to_camera_rdf(corner_enu, self.drone_pos_w, self.R_wb, R_bc)
        if p_c[2] < 0.2:
            return None, p_c[2]

        fx = K[0, 0]
        fy = K[1, 1]
        cx = K[0, 2]
        cy = K[1, 2]

        u = fx * (p_c[0] / p_c[2]) + cx
        v = fy * (p_c[1] / p_c[2]) + cy
        return (float(u), float(v)), p_c[2]

    def image_callback(self, msg: Image):
        # Cache incoming frame and process
        img_arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, -1))
        if msg.encoding == 'rgb8':
            self.latest_raw_image = cv2.cvtColor(img_arr, cv2.COLOR_RGB2BGR)
        else:
            self.latest_raw_image = img_arr.copy()
        self.latest_raw_header = msg.header
        self.last_image_time = self.get_clock().now()

        # Render immediately on new camera frame
        self.render_and_publish()

    def timer_callback(self):
        # If no image received in last 0.2s, render synthetic frame to keep stream alive
        now = self.get_clock().now()
        is_live = (self.last_image_time is not None and (now - self.last_image_time).nanoseconds * 1e-9 < 0.2)
        if not is_live:
            self.render_and_publish()

    def generate_synthetic_canvas(self):
        """Generates a clean synthetic camera frame if live video stream is not arriving."""
        w, h = 640, 640
        canvas = np.zeros((h, w, 3), dtype=np.uint8)
        # Sky (dark blue gradient)
        canvas[:int(h * 0.6), :] = [50, 30, 20]
        # Ground (dark grey-green)
        canvas[int(h * 0.6):, :] = [30, 45, 30]
        # Horizon line
        cv2.line(canvas, (0, int(h * 0.6)), (w, int(h * 0.6)), (80, 80, 80), 1)

        cv2.putText(canvas, "[SYNTHETIC CANVAS - Awaiting /camera/image_raw]", (w // 2 - 210, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 180, 255), 1, cv2.LINE_AA)
        return canvas

    def render_and_publish(self):
        """Projects gates and publishes to /perception/debug_image & compressed."""
        is_synthetic = (self.latest_raw_image is None)
        if is_synthetic:
            vis_img = self.generate_synthetic_canvas()
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = 'camera_front_optical_frame'
        else:
            vis_img = self.latest_raw_image.copy()
            header = self.latest_raw_header

        h_img, w_img = vis_img.shape[:2]

        # Determine active camera matrix K (from topic or dynamically computed for this frame)
        if self.camera_matrix is not None:
            active_K = self.camera_matrix
            intrinsics_src = "TOPIC" if self.intrinsics_from_topic else "MANUAL"
        else:
            fov = 1.03
            cx = w_img / 2.0
            cy = h_img / 2.0
            fx = cx / math.tan(fov / 2.0)
            fy = fx
            active_K = np.array([
                [fx, 0.0, cx],
                [0.0, fy, cy],
                [0.0, 0.0, 1.0]
            ], dtype=np.float64)
            intrinsics_src = f"FALLBACK ({w_img}x{h_img})"

        R_bc = get_camera_to_body_rotation(self.camera_pitch_deg)
        gates_to_test = self.get_gate_list()

        corner_colors = [
            (255, 100, 0),   # TL - Cyan/Sky Blue
            (0, 255, 0),     # TR - Bright Green
            (0, 0, 255),     # BR - Bright Red
            (0, 255, 255)    # BL - Bright Yellow
        ]
        corner_names = ["TL", "TR", "BR", "BL"]

        gate_palette = [
            (255, 255, 0),   # G1: Cyan
            (0, 200, 255),   # G2: Orange
            (255, 0, 255),   # G3: Magenta
            (0, 255, 128),   # G4: Spring Green
            (180, 105, 255)  # G5: Hot Pink
        ]

        active_target_hud_info = None

        for idx, (gid, pos_enu, yaw_rad) in enumerate(gates_to_test):
            is_target = (idx == self.target_gate_idx)
            if not self.show_all_gates and not is_target:
                continue

            dist = float(np.linalg.norm(pos_enu - self.drone_pos_w))
            corners_w = self.compute_gate_corners_world(pos_enu, yaw_rad)

            proj_corners = []
            all_valid = True
            depths = []

            for c_w in corners_w:
                px, depth = self.project_corner(c_w, R_bc, active_K)
                depths.append(depth)
                if px is None:
                    all_valid = False
                    break
                proj_corners.append(px)

            gate_col = gate_palette[(gid - 1) % len(gate_palette)]
            line_thickness = 3 if is_target else 2

            if all_valid:
                pts = np.array(proj_corners, dtype=np.int32)

                # Draw Gate Polygon
                cv2.polylines(vis_img, [pts], isClosed=True, color=gate_col, thickness=line_thickness)

                # Draw Gate Corners
                for c_idx, pt in enumerate(pts):
                    c_col = corner_colors[c_idx]
                    cv2.circle(vis_img, (int(pt[0]), int(pt[1])), 5, c_col, -1)
                    cv2.circle(vis_img, (int(pt[0]), int(pt[1])), 7, (0, 0, 0), 1)
                    offset_x = -18 if c_idx in [0, 3] else 6
                    offset_y = -8 if c_idx in [0, 1] else 16
                    cv2.putText(vis_img, corner_names[c_idx],
                                (int(pt[0]) + offset_x, int(pt[1]) + offset_y),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, c_col, 1, cv2.LINE_AA)

                # Draw Gate Center & Label
                center_px, _ = self.project_corner(pos_enu, R_bc, active_K)
                if center_px is not None:
                    cx_i, cy_i = int(center_px[0]), int(center_px[1])
                    cv2.drawMarker(vis_img, (cx_i, cy_i), gate_col, cv2.MARKER_CROSS, 14, 2)
                    tag = f"G{gid}: {dist:.1f}m"
                    if is_target:
                        tag = f"[TARGET G{gid}] {dist:.1f}m"
                    cv2.putText(vis_img, tag, (pts[0, 0], max(20, pts[0, 1] - 12)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55, gate_col, 2, cv2.LINE_AA)

                if is_target:
                    active_target_hud_info = {
                        'id': gid,
                        'dist': dist,
                        'in_view': True,
                        'corners': proj_corners,
                        'center_px': center_px,
                        'depth': np.mean(depths)
                    }
            else:
                if is_target:
                    active_target_hud_info = {
                        'id': gid,
                        'dist': dist,
                        'in_view': False,
                        'corners': [],
                        'center_px': None,
                        'depth': depths[0] if depths else 0.0
                    }

        # ----------------- HUD Telemetry Box (Top-Left) -----------------
        hud_w, hud_h = 350, 136
        overlay = vis_img.copy()
        cv2.rectangle(overlay, (10, 10), (10 + hud_w, 10 + hud_h), (20, 20, 20), -1)
        cv2.addWeighted(overlay, 0.70, vis_img, 0.30, 0, vis_img)
        cv2.rectangle(vis_img, (10, 10), (10 + hud_w, 10 + hud_h), (100, 100, 100), 1)

        yaw_drone_deg = math.degrees(math.atan2(self.R_wb[1, 0], self.R_wb[0, 0]))

        cv2.putText(vis_img, "DIRECT PRIOR PROJECTION TEST", (16, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(vis_img, f"Drone: [{self.drone_pos_w[0]:.2f}, {self.drone_pos_w[1]:.2f}, {self.drone_pos_w[2]:.2f}] Yaw:{yaw_drone_deg:.1f}*", (16, 46),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (220, 220, 220), 1, cv2.LINE_AA)
        cv2.putText(vis_img, f"Source: {self.prior_source.upper()}  Pitch: {self.camera_pitch_deg:.1f}*", (16, 64),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (180, 220, 180), 1, cv2.LINE_AA)
        cv2.putText(vis_img, f"K: fx={active_K[0,0]:.1f} cx={active_K[0,2]:.0f} cy={active_K[1,2]:.0f} [{intrinsics_src}]", (16, 80),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (160, 200, 255), 1, cv2.LINE_AA)

        if active_target_hud_info is not None:
            gid = active_target_hud_info['id']
            dist = active_target_hud_info['dist']
            if active_target_hud_info['in_view']:
                c = active_target_hud_info['corners']
                cv2.putText(vis_img, f"Target G{gid}: Dist={dist:.1f}m (IN VIEW)", (16, 98),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.40, (0, 255, 0), 1, cv2.LINE_AA)
                cv2.putText(vis_img, f"TL:({c[0][0]:.0f},{c[0][1]:.0f}) TR:({c[1][0]:.0f},{c[1][1]:.0f})", (16, 114),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.36, (200, 200, 200), 1, cv2.LINE_AA)
                cv2.putText(vis_img, f"BR:({c[2][0]:.0f},{c[2][1]:.0f}) BL:({c[3][0]:.0f},{c[3][1]:.0f})", (16, 130),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.36, (200, 200, 200), 1, cv2.LINE_AA)
            else:
                cv2.putText(vis_img, f"Target G{gid}: Dist={dist:.1f}m (OUT OF VIEW / BEHIND)", (16, 98),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.40, (0, 0, 255), 1, cv2.LINE_AA)
                cv2.putText(vis_img, f"Z_cam={active_target_hud_info['depth']:.2f}m (< 0.2m)", (16, 116),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.36, (180, 180, 180), 1, cv2.LINE_AA)

        # Publish Debug Image (raw)
        debug_msg = Image()
        debug_msg.header = header
        debug_msg.height = vis_img.shape[0]
        debug_msg.width = vis_img.shape[1]
        debug_msg.encoding = "bgr8"
        debug_msg.is_bigendian = 0
        debug_msg.step = vis_img.shape[1] * 3
        debug_msg.data = vis_img.tobytes()
        self.pub_debug_img.publish(debug_msg)

        # Publish Debug Image (compressed)
        comp_msg = CompressedImage()
        comp_msg.header = header
        comp_msg.format = "jpeg"
        ret, jpeg_buf = cv2.imencode('.jpg', vis_img, [int(cv2.IMWRITE_JPEG_QUALITY), 75])
        if ret:
            comp_msg.data = jpeg_buf.tobytes()
            self.pub_debug_comp.publish(comp_msg)

        # Throttled console log every 2 seconds
        now = self.get_clock().now()
        if (now - self.last_log_time).nanoseconds * 1e-9 >= 2.0:
            self.last_log_time = now
            status_str = "LIVE" if not is_synthetic else "SYNTHETIC"
            k_str = f"K(fx={active_K[0,0]:.1f}, cy={active_K[1,2]:.1f}) [{intrinsics_src}]"
            if active_target_hud_info and active_target_hud_info['in_view']:
                c = active_target_hud_info['corners']
                self.get_logger().info(
                    f"[{status_str} | Pitch:{self.camera_pitch_deg:.1f}° | {k_str}] "
                    f"Target Gate {active_target_hud_info['id']} ({active_target_hud_info['dist']:.1f}m): "
                    f"TL=({c[0][0]:.0f},{c[0][1]:.0f}), TR=({c[1][0]:.0f},{c[1][1]:.0f}), "
                    f"BR=({c[2][0]:.0f},{c[2][1]:.0f}), BL=({c[3][0]:.0f},{c[3][1]:.0f})"
                )
            elif active_target_hud_info:
                self.get_logger().info(
                    f"[{status_str} | Pitch:{self.camera_pitch_deg:.1f}° | {k_str}] "
                    f"Target Gate {active_target_hud_info['id']} at {active_target_hud_info['dist']:.1f}m "
                    f"is behind camera or outside frustum (Z_cam={active_target_hud_info['depth']:.2f}m)"
                )


def main(args=None):
    rclpy.init(args=args)
    node = TestGateProjectionNode()
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
