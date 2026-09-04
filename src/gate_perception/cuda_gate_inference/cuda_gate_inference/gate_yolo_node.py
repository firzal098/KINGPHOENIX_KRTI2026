#!/usr/bin/env python3
"""
ROS 2 Jazzy Gate YOLO Keypoint Detection Node.
Subscribes to camera image stream, performs ONNX keypoint inference (YOLOv8 pose schema),
and publishes 2D corner pixel coordinates on /perception/gate_corners_2d.
Also publishes annotated debug images for monitoring.
"""

import os
import math
import numpy as np
import cv2
import onnxruntime as ort

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from ament_index_python.packages import get_package_share_directory

from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from geometry_msgs.msg import PolygonStamped, Point32, PoseArray, PoseStamped, Pose
from std_msgs.msg import Float64MultiArray, Int32

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


def nms_boxes(boxes, scores, iou_threshold=0.45):
    """
    Standard Non-Maximum Suppression algorithm for 2D bounding boxes.
    boxes format: [x1, y1, x2, y2]
    """
    if len(boxes) == 0:
        return []

    x1 = boxes[:, 0]
    y1 = boxes[:, 1]
    x2 = boxes[:, 2]
    y2 = boxes[:, 3]

    areas = (x2 - x1) * (y2 - y1)
    order = scores.argsort()[::-1]

    keep = []
    while order.size > 0:
        i = order[0]
        keep.append(i)
        if order.size == 1:
            break

        xx1 = np.maximum(x1[i], x1[order[1:]])
        yy1 = np.maximum(y1[i], y1[order[1:]])
        xx2 = np.minimum(x2[i], x2[order[1:]])
        yy2 = np.minimum(y2[i], y2[order[1:]])

        w = np.maximum(0.0, xx2 - xx1)
        h = np.maximum(0.0, yy2 - yy1)
        inter = w * h

        ovr = inter / (areas[i] + areas[order[1:]] - inter + 1e-6)
        inds = np.where(ovr <= iou_threshold)[0]
        order = order[inds + 1]

    return keep


def rvec_to_quaternion(rvec):
    """
    Converts OpenCV Rodrigues rotation vector to ROS quaternion [x, y, z, w].
    """
    R, _ = cv2.Rodrigues(rvec)
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        S = np.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * S
        qx = (R[2, 1] - R[1, 2]) / S
        qy = (R[0, 2] - R[2, 0]) / S
        qz = (R[1, 0] - R[0, 1]) / S
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        qw = (R[2, 1] - R[1, 2]) / S
        qx = 0.25 * S
        qy = (R[0, 1] + R[1, 0]) / S
        qz = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        qw = (R[0, 2] - R[2, 0]) / S
        qx = (R[0, 1] + R[1, 0]) / S
        qy = 0.25 * S
        qz = (R[1, 2] + R[2, 1]) / S
    else:
        S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        qw = (R[1, 0] - R[0, 1]) / S
        qx = (R[0, 2] + R[2, 0]) / S
        qy = (R[1, 2] + R[2, 1]) / S
        qz = 0.25 * S
    return qx, qy, qz, qw


class GateYoloNode(Node):
    """
    Pure 2D YOLO Inference Node for Gate Corner Keypoints.
    """

    def __init__(self):
        super().__init__('gate_yolo_node')

        _default_model = os.path.join(
            get_package_share_directory('cuda_gate_inference'), 'resource', 'gate_keypoints.onnx'
        )
        self.declare_parameter('model_path', _default_model)
        self.declare_parameter('conf_threshold', 0.50)
        self.declare_parameter('corner_conf_threshold', 0.15)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('gate_width_m', 1.9)
        self.declare_parameter('gate_height_m', 2.0)
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('publish_pnp_fallback', True)

        self.model_path = str(self.get_parameter('model_path').value)
        self.conf_threshold = float(self.get_parameter('conf_threshold').value)
        self.corner_conf_thresh = float(self.get_parameter('corner_conf_threshold').value)
        self.iou_threshold = float(self.get_parameter('iou_threshold').value)
        self.gate_w = float(self.get_parameter('gate_width_m').value)
        self.gate_h = float(self.get_parameter('gate_height_m').value)
        self.publish_debug = bool(self.get_parameter('publish_debug_image').value)
        self.publish_pnp_fallback = bool(self.get_parameter('publish_pnp_fallback').value)

        hw = self.gate_w / 2.0
        hh = self.gate_h / 2.0
        self.object_points_4p = np.array([
            [-hw, -hh, 0.0],  # Corner 6 (Top-Left)
            [ hw, -hh, 0.0],  # Corner 7 (Top-Right)
            [ hw,  hh, 0.0],  # Corner 8 (Bottom-Right)
            [-hw,  hh, 0.0]   # Corner 9 (Bottom-Left)
        ], dtype=np.float32)

        self.camera_matrix = None
        self.dist_coeffs = None

        self.init_onnx_session()

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.sub_image = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, qos_profile
        )
        self.sub_info = self.create_subscription(
            CameraInfo, '/camera/camera_info', self.camera_info_callback, 10
        )

        # Main 2D Corner Keypoint Publisher
        self.pub_corners_2d = self.create_publisher(
            PolygonStamped, '/perception/gate_corners_2d', 10
        )

        # Fallback 3D PnP Publishers (for smooth legacy compatibility)
        if self.publish_pnp_fallback:
            self.pub_poses_3d = self.create_publisher(
                PoseArray, '/perception/gate_poses_3d', 10
            )
            self.pub_covs_3d = self.create_publisher(
                Float64MultiArray, '/perception/gate_covariances_3d', 10
            )
            self.pub_primary_pose = self.create_publisher(
                PoseStamped, '/perception/gate_pose', 10
            )

        # Debug Visualizations
        if self.publish_debug:
            self.pub_debug_img = self.create_publisher(
                Image, '/perception/debug_image', 10
            )
            self.pub_debug_compressed = self.create_publisher(
                CompressedImage, '/perception/debug_image/compressed', 10
            )

        # Subscription to projected gate corners from gate_estimator_node
        self.sub_projected_corners = self.create_subscription(
            Float64MultiArray, '/estimator/projected_gate_corners', self.projected_corners_callback, 10
        )
        self.latest_projected_corners = None
        self.last_projected_corners_time = self.get_clock().now()
        self.last_target_det_center = None

        # Direct prior projection parameters & subscriptions
        self.declare_parameter('camera_pitch_deg', 15.0)
        self.declare_parameter('prior_source', 'defined_constants')
        self.declare_parameter('show_all_projected_gates', True)
        self.camera_pitch_deg = float(self.get_parameter('camera_pitch_deg').value)
        self.prior_source = str(self.get_parameter('prior_source').value).lower()
        self.show_all_projected_gates = bool(self.get_parameter('show_all_projected_gates').value)

        # Dynamic parameter reconfiguration callback
        self.add_on_set_parameters_callback(self.parameters_callback)

        self.drone_pos_w = np.zeros(3, dtype=np.float64)
        self.drone_quat_w = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        self.R_wb = np.eye(3, dtype=np.float64)
        self.initial_drone_pos = np.zeros(3, dtype=np.float64)
        self.initial_drone_rot = np.eye(3, dtype=np.float64)
        self.initial_pose_captured = False
        self.active_target_gate_idx = 0

        self.sub_drone_pose = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self.drone_pose_callback, qos_profile
        )
        self.sub_target_gate = self.create_subscription(
            Int32, '/controller/target_gate_index', self.target_gate_callback, 10
        )

        self.get_logger().info(f"Gate YOLO Node initialized using ONNX model: {self.model_path}")

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
                self.get_logger().info(f"Dynamic param update: camera_pitch_deg = {self.camera_pitch_deg:.1f}°")
            elif param.name == 'conf_threshold':
                self.conf_threshold = float(val)
                self.get_logger().info(f"Dynamic param update: conf_threshold = {self.conf_threshold:.2f}")
            elif param.name == 'corner_conf_threshold':
                self.corner_conf_thresh = float(val)
                self.get_logger().info(f"Dynamic param update: corner_conf_threshold = {self.corner_conf_thresh:.2f}")
            elif param.name == 'prior_source':
                self.prior_source = str(val).lower()
                self.get_logger().info(f"Dynamic param update: prior_source = {self.prior_source}")
            elif param.name == 'show_all_projected_gates':
                self.show_all_projected_gates = bool(val)
                self.get_logger().info(f"Dynamic param update: show_all_projected_gates = {self.show_all_projected_gates}")
        return SetParametersResult(successful=True)

    def get_gate_list(self):
        """Returns list of (gate_id, pos_enu, yaw_rad) for all surveyed gates anchored to takeoff pose."""
        if self.prior_source == 'webots_world':
            return [(gid, np.array([x, y, z], dtype=np.float64), yaw) for gid, x, y, z, yaw in WEBOTS_GATES]
        else:
            # Priors in RDF relative to drone takeoff pose: [x_right, y_down, z_fwd], normal: [nx, ny, nz]
            priors_rdf = [
                (1,  0.41, -0.75, 29.33,  0.0, 0.0, 1.0),
                (2,  5.46, -0.75, 19.31,  0.0, 0.0, 1.0),
                (3,  9.49, -0.75, 10.51,  1.0, 0.0, 0.0),
                (4, 12.41, -0.75, 12.43,  0.0, 0.0, 1.0),
                (5, 17.47, -0.75, 29.38,  0.0, 0.0, 1.0)
            ]
            gates = []
            for gid, x_rdf, y_rdf, z_rdf, nx_rdf, ny_rdf, nz_rdf in priors_rdf:
                rel_pos_enu = np.array([z_rdf, -x_rdf, -y_rdf], dtype=np.float64)
                rel_norm_enu = np.array([nz_rdf, -nx_rdf, -ny_rdf], dtype=np.float64)

                pos_enu = self.initial_drone_pos + self.initial_drone_rot @ rel_pos_enu
                norm_enu = self.initial_drone_rot @ rel_norm_enu
                norm_enu /= max(1e-6, np.linalg.norm(norm_enu))

                yaw_rad = math.atan2(norm_enu[1], norm_enu[0])
                gates.append((gid, pos_enu, yaw_rad))
            return gates

    def compute_gate_corners_world(self, center_enu, yaw_rad):
        """Constructs 4 outer corners in World ENU frame: [TL, TR, BR, BL]."""
        hw = self.gate_w / 2.0
        hh = self.gate_h / 2.0
        u_lat = np.array([-math.sin(yaw_rad), math.cos(yaw_rad), 0.0], dtype=np.float64)
        u_vert = np.array([0.0, 0.0, 1.0], dtype=np.float64)
        c_tl = center_enu + hw * u_lat + hh * u_vert
        c_tr = center_enu - hw * u_lat + hh * u_vert
        c_br = center_enu - hw * u_lat - hh * u_vert
        c_bl = center_enu + hw * u_lat - hh * u_vert
        return [c_tl, c_tr, c_br, c_bl]

    def project_corner(self, corner_enu, R_bc, K, drone_pos, R_wb):
        """Transforms corner ENU -> camera RDF -> 2D pixel (u, v)."""
        p_c = world_to_camera_rdf(corner_enu, drone_pos, R_wb, R_bc)
        if p_c[2] < 0.2:
            return None, float(p_c[2])
        fx = K[0, 0]
        fy = K[1, 1]
        cx = K[0, 2]
        cy = K[1, 2]
        u = fx * (p_c[0] / p_c[2]) + cx
        v = fy * (p_c[1] / p_c[2]) + cy
        return (float(u), float(v)), float(p_c[2])

    def get_projected_gates(self, img_w=640, img_h=640):
        """
        Projects all known gates into camera pixel space using current drone pose and intrinsics.
        Returns dict: {gate_id: {'corners': np.ndarray (4, 2), 'center': (cx, cy), 'bbox': (cx, cy, w, h), 'dist': float, 'depth': float, 'yaw': float}}
        """
        if self.camera_matrix is not None:
            K = self.camera_matrix
        else:
            fov = 1.03
            cx = img_w / 2.0
            cy = img_h / 2.0
            fx = cx / math.tan(fov / 2.0)
            fy = fx
            K = np.array([
                [fx, 0.0, cx],
                [0.0, fy, cy],
                [0.0, 0.0, 1.0]
            ], dtype=np.float64)

        drone_pos = self.drone_pos_w if self.drone_pos_w is not None else np.zeros(3, dtype=np.float64)
        R_wb = self.R_wb if self.R_wb is not None else np.eye(3, dtype=np.float64)
        R_bc = get_camera_to_body_rotation(self.camera_pitch_deg)
        gates = self.get_gate_list()

        projected = {}
        for gid, pos_enu, yaw_rad in gates:
            dist = float(np.linalg.norm(pos_enu - drone_pos))
            corners_w = self.compute_gate_corners_world(pos_enu, yaw_rad)

            corners_2d = []
            depths = []
            all_valid = True
            for c_w in corners_w:
                px, depth = self.project_corner(c_w, R_bc, K, drone_pos, R_wb)
                depths.append(depth)
                if px is None:
                    all_valid = False
                    break
                corners_2d.append(px)

            if not all_valid:
                continue

            corners_arr = np.array(corners_2d, dtype=np.float32)  # (4, 2) [TL, TR, BR, BL]
            min_u, max_u = np.min(corners_arr[:, 0]), np.max(corners_arr[:, 0])
            min_v, max_v = np.min(corners_arr[:, 1]), np.max(corners_arr[:, 1])
            cx = float(np.mean(corners_arr[:, 0]))
            cy = float(np.mean(corners_arr[:, 1]))
            w = float(max_u - min_u)
            h = float(max_v - min_v)

            # Keep if inside or near viewport (-400 to +1040)
            if -400 <= cx <= 1040 and -400 <= cy <= 1040:
                projected[gid] = {
                    'corners': corners_arr,
                    'center': (cx, cy),
                    'bbox': (cx, cy, w, h),
                    'dist': dist,
                    'depth': float(np.mean(depths)),
                    'yaw': yaw_rad
                }

        return projected

    def drone_pose_callback(self, msg: PoseStamped):
        p = msg.pose.position
        o = msg.pose.orientation
        self.drone_pos_w = np.array([p.x, p.y, p.z], dtype=np.float64)
        self.drone_quat_w = np.array([o.w, o.x, o.y, o.z], dtype=np.float64)
        self.R_wb = quat_to_rot_matrix(self.drone_quat_w)

        if not self.initial_pose_captured:
            self.initial_drone_pos = self.drone_pos_w.copy()
            self.initial_drone_rot = self.R_wb.copy()
            self.initial_pose_captured = True
            self.get_logger().info(
                f"Captured initial takeoff pose offset: [{self.initial_drone_pos[0]:.2f}, "
                f"{self.initial_drone_pos[1]:.2f}, {self.initial_drone_pos[2]:.2f}]. Anchoring gate priors."
            )

    def target_gate_callback(self, msg: Int32):
        if msg.data >= 0:
            self.active_target_gate_idx = msg.data

    def projected_corners_callback(self, msg: Float64MultiArray):
        if len(msg.data) >= 20:
            self.latest_projected_corners = {
                'gate_id': int(msg.data[0]),
                'fsm_code': int(msg.data[1]),
                'nis': float(msg.data[2]),
                'max_innov': float(msg.data[3]),
                'ekf_pixels': np.array(msg.data[4:12], dtype=np.float32).reshape((4, 2)),
                'prior_pixels': np.array(msg.data[12:20], dtype=np.float32).reshape((4, 2))
            }
            self.last_projected_corners_time = self.get_clock().now()

    def init_onnx_session(self):
        if not os.path.exists(self.model_path):
            self.get_logger().error(f"ONNX model file not found at: {self.model_path}")
            alt_path = os.path.join(os.getcwd(), self.model_path)
            if os.path.exists(alt_path):
                self.model_path = alt_path

        opts = ort.SessionOptions()
        opts.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        providers = ["CUDAExecutionProvider", "CPUExecutionProvider"]

        try:
            self.ort_session = ort.InferenceSession(self.model_path, sess_options=opts, providers=providers)
            self.input_name = self.ort_session.get_inputs()[0].name
            self.get_logger().info(f"Loaded ONNX model successfully with active provider: {self.ort_session.get_providers()[0]}")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize ONNX session: {str(e)}")
            self.ort_session = None

    def camera_info_callback(self, msg: CameraInfo):
        first_time = (self.camera_matrix is None)
        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape((3, 3))
        self.dist_coeffs = np.array(msg.d, dtype=np.float64) if len(msg.d) > 0 else np.zeros((5, 1))
        if first_time:
            self.get_logger().info(
                f"Camera intrinsics registered: fx={self.camera_matrix[0,0]:.2f}, fy={self.camera_matrix[1,1]:.2f}, "
                f"cx={self.camera_matrix[0,2]:.1f}, cy={self.camera_matrix[1,2]:.1f}"
            )

    def image_callback(self, msg: Image):
        if self.ort_session is None:
            return

        try:
            if msg.encoding in ['bgr8', 'rgb8']:
                img_np = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
                frame = cv2.cvtColor(img_np, cv2.COLOR_RGB2BGR) if msg.encoding == 'rgb8' else img_np
            else:
                self.get_logger().warn(f"Unsupported image encoding: {msg.encoding}", throttle_duration_sec=2.0)
                return
        except Exception as e:
            self.get_logger().error(f"Failed to decode ROS Image message: {str(e)}")
            return

        orig_h, orig_w = frame.shape[:2]
        x_scale = orig_w / 640.0
        y_scale = orig_h / 640.0

        # Fast C++ preprocessing via cv2.dnn.blobFromImage
        input_tensor = cv2.dnn.blobFromImage(
            frame,
            scalefactor=1.0 / 255.0,
            size=(640, 640),
            mean=(0, 0, 0),
            swapRB=True,
            crop=False
        )

        # Inference
        outputs = self.ort_session.run(None, {self.input_name: input_tensor})[0]
        if outputs.ndim == 3 and outputs.shape[1] == 38:
            outputs = outputs[0].transpose(1, 0)
        elif outputs.ndim == 3:
            outputs = outputs[0]

        scores = outputs[:, 4]
        mask = scores >= self.conf_threshold
        filtered_outputs = outputs[mask]

        detections = []
        if len(filtered_outputs) > 0:
            boxes_640 = filtered_outputs[:, :4]
            scores_filt = filtered_outputs[:, 4]
            keypoints_filt = filtered_outputs[:, 5:]

            x1 = boxes_640[:, 0] - boxes_640[:, 2] / 2.0
            y1 = boxes_640[:, 1] - boxes_640[:, 3] / 2.0
            x2 = boxes_640[:, 0] + boxes_640[:, 2] / 2.0
            y2 = boxes_640[:, 1] + boxes_640[:, 3] / 2.0
            boxes_xyxy = np.column_stack([x1, y1, x2, y2])

            keep_indices = nms_boxes(boxes_xyxy, scores_filt, self.iou_threshold)

            for idx in keep_indices:
                kp_data = keypoints_filt[idx].reshape(11, 3)
                corner_confs = kp_data[6:10, 2]
                valid_corners = np.sum(corner_confs >= self.corner_conf_thresh)

                # At least 3 corners required
                if valid_corners < 3:
                    continue

                cx, cy, w, h = boxes_640[idx]
                detections.append({
                    'bbox': (cx * x_scale, cy * y_scale, w * x_scale, h * y_scale),
                    'score': float(scores_filt[idx]),
                    'kp_x': kp_data[:, 0] * x_scale,
                    'kp_y': kp_data[:, 1] * y_scale,
                    'kp_vis': kp_data[:, 2],
                    'pose_3d': None
                })

        # 1. Project all visible gate priors into current camera pixel frame
        projected_priors = self.get_projected_gates(orig_w, orig_h)
        target_gate_id = self.active_target_gate_idx + 1

        self.get_logger().info(
            f"PERCEPTION DIAG: projected_priors={list(projected_priors.keys())}, "
            f"target_gate_id={target_gate_id}, "
            f"drone_pos={self.drone_pos_w.tolist() if self.drone_pos_w is not None else None}, "
            f"camera_pitch={self.camera_pitch_deg:.1f}°",
            throttle_duration_sec=2.0
        )

        # 2. Prior-guided Association: match each YOLO detection to closest projected gate prior

        for det in detections:
            det['assigned_gate_id'] = None
            det['assoc_cost'] = float('inf')

            cx_det, cy_det, w_det, h_det = det['bbox']

            best_gid = None
            best_cost = float('inf')

            for gid, p_info in projected_priors.items():
                cx_p, cy_p = p_info['center']
                w_p, h_p = p_info['bbox'][2], p_info['bbox'][3]

                u_diff = abs(cx_det - cx_p)
                v_diff = abs(cy_det - cy_p)
                dist_2d = math.hypot(u_diff, v_diff)

                # Scale consistency: ratio between detected width and projected prior width
                scale_ratio = min(w_det, max(1.0, w_p)) / max(w_det, max(1.0, w_p))

                # Weight 2D center distance + scale penalty
                cost = dist_2d + 50.0 * (1.0 - scale_ratio)

                # Association gate: allow up to 250 pixels deviation (absorbs ground pitch differences)
                if dist_2d < 250.0 and cost < best_cost:
                    best_cost = cost
                    best_gid = gid

            if best_gid is not None:
                det['assigned_gate_id'] = best_gid
                det['assoc_cost'] = best_cost

        # 3. Find detection explicitly matched to the active target gate
        target_det = None
        for det in detections:
            if det.get('assigned_gate_id') == target_gate_id:
                if target_det is None or det['score'] > target_det['score']:
                    target_det = det

        # Fallback association: if active target gate not assigned but only 1 detection exists and target prior is in view
        if target_det is None and len(detections) == 1 and target_gate_id in projected_priors:
            p_target = projected_priors[target_gate_id]
            cx_det, cy_det = detections[0]['bbox'][0], detections[0]['bbox'][1]
            dist_2d = math.hypot(cx_det - p_target['center'][0], cy_det - p_target['center'][1])
            if dist_2d < 300.0:
                target_det = detections[0]
                target_det['assigned_gate_id'] = target_gate_id

        # 4. Publish 2D Corner Keypoints ONLY if target gate is matched
        if target_det is not None:
            poly_msg = PolygonStamped()
            poly_msg.header = msg.header
            poly_msg.header.frame_id = f"gate_{target_gate_id}"
            # 4 Points in canonical order: TL, TR, BR, BL
            # YOLO model keypoint mapping:
            # Index 7: Top-Left (TL)
            # Index 8: Top-Right (TR)
            # Index 9: Bottom-Right (BR)
            # Index 6: Bottom-Left (BL)
            corner_indices = [7, 8, 9, 6]
            for k_idx in corner_indices:
                pt = Point32()
                pt.x = float(target_det['kp_x'][k_idx])
                pt.y = float(target_det['kp_y'][k_idx])
                pt.z = float(target_det['kp_vis'][k_idx])
                poly_msg.polygon.points.append(pt)
            self.pub_corners_2d.publish(poly_msg)
            self.last_target_det_center = (target_det['bbox'][0], target_det['bbox'][1])

        # Fallback PnP solving (if enabled and requested by legacy nodes)
        if self.publish_pnp_fallback and self.camera_matrix is not None:
            pose_3d_msg = PoseArray()
            pose_3d_msg.header = msg.header
            cov_3d_msg = Float64MultiArray()
            best_pose = None
            best_pnp_score = -1.0

            corner_indices = [7, 8, 9, 6]
            for det in detections:
                corners_2d = np.ascontiguousarray(
                    np.column_stack([det['kp_x'][corner_indices], det['kp_y'][corner_indices]]), dtype=np.float32
                )
                corner_vis = det['kp_vis'][corner_indices]

                if np.sum(corner_vis >= self.corner_conf_thresh) >= 4:
                    success, rvec, tvec = cv2.solvePnP(
                        self.object_points_4p,
                        corners_2d,
                        self.camera_matrix,
                        self.dist_coeffs,
                        flags=cv2.SOLVEPNP_IPPE
                    )
                    if success:
                        qx, qy, qz, qw = rvec_to_quaternion(rvec)
                        tx, ty, tz = float(tvec[0][0]), float(tvec[1][0]), float(tvec[2][0])
                        gate_pose = Pose()
                        gate_pose.position.x = tx
                        gate_pose.position.y = ty
                        gate_pose.position.z = tz
                        gate_pose.orientation.x = qx
                        gate_pose.orientation.y = qy
                        gate_pose.orientation.z = qz
                        gate_pose.orientation.w = qw
                        pose_3d_msg.poses.append(gate_pose)

                        d = math.sqrt(tx*tx + ty*ty + tz*tz)
                        sig = 8.0 if d > 20.0 else (1.5 + (d / 20.0) * 6.5)
                        R_cam = np.eye(3, dtype=np.float64) * (sig ** 2)
                        cov_3d_msg.data.extend(R_cam.flatten().tolist())

                        det['pose_3d'] = (tx, ty, tz)
                        if det['score'] > best_pnp_score:
                            best_pnp_score = det['score']
                            best_pose = gate_pose

            self.pub_poses_3d.publish(pose_3d_msg)
            self.pub_covs_3d.publish(cov_3d_msg)
            if best_pose is not None:
                p_msg = PoseStamped()
                p_msg.header = msg.header
                p_msg.pose = best_pose
                self.pub_primary_pose.publish(p_msg)

        # 5. Rich Debug Overlays (Always streaming)
        if self.publish_debug:
            self.publish_debug_overlay(frame, detections, target_det, projected_priors, target_gate_id, msg.header, publish_raw=True, publish_compressed=True)

    def publish_debug_overlay(self, frame, detections, target_det, projected_priors, target_gate_id, header, publish_raw=True, publish_compressed=True):
        vis_img = frame.copy()

        gate_palette = [
            (255, 255, 0),   # G1: Cyan (B=255, G=255, R=0)
            (0, 200, 255),   # G2: Orange
            (255, 0, 255),   # G3: Magenta
            (0, 255, 128),   # G4: Spring Green
            (180, 105, 255)  # G5: Hot Pink
        ]
        corner_colors = [
            (255, 100, 0),   # TL - Sky Blue
            (0, 255, 0),     # TR - Bright Green
            (0, 0, 255),     # BR - Bright Red
            (0, 255, 255)    # BL - Bright Yellow
        ]
        corner_names = ["TL", "TR", "BR", "BL"]

        # 1. Render all visible projected gate priors (from test_gate_projection)
        for gid, p_info in projected_priors.items():
            is_target = (gid == target_gate_id)
            if not self.show_all_projected_gates and not is_target:
                continue

            pts = np.clip(p_info['corners'], -2000, 4000).astype(np.int32)
            gate_col = gate_palette[(gid - 1) % len(gate_palette)]
            line_thickness = 3 if is_target else 2

            # Draw polygon
            cv2.polylines(vis_img, [pts], isClosed=True, color=gate_col, thickness=line_thickness)

            # Draw corners
            for c_idx, pt in enumerate(pts):
                c_col = corner_colors[c_idx]
                cv2.circle(vis_img, (int(pt[0]), int(pt[1])), 5, c_col, -1)
                cv2.circle(vis_img, (int(pt[0]), int(pt[1])), 7, (0, 0, 0), 1)
                offset_x = -18 if c_idx in [0, 3] else 6
                offset_y = -8 if c_idx in [0, 1] else 16
                cv2.putText(vis_img, corner_names[c_idx],
                            (int(pt[0]) + offset_x, int(pt[1]) + offset_y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, c_col, 1, cv2.LINE_AA)

            # Center crosshair & label
            cx_i, cy_i = int(p_info['center'][0]), int(p_info['center'][1])
            cv2.drawMarker(vis_img, (cx_i, cy_i), gate_col, cv2.MARKER_CROSS, 12, 2)
            dist_val = p_info['dist']
            tag = f"[TARGET G{gid}] {dist_val:.1f}m" if is_target else f"Prior G{gid}: {dist_val:.1f}m"
            cv2.putText(vis_img, tag, (int(pts[0, 0]), int(max(20, pts[0, 1] - 10))),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.50, gate_col, 2, cv2.LINE_AA)

        # 2. Draw YOLO Bounding Boxes & Assigned Labels
        for det in detections:
            cx, cy, w, h = det['bbox']
            x1, y1 = int(cx - w / 2), int(cy - h / 2)
            x2, y2 = int(cx + w / 2), int(cy + h / 2)

            assigned_gid = det.get('assigned_gate_id', None)
            is_target = (det is target_det)

            if is_target:
                box_col = (0, 255, 0)  # Bright Green for target detection
                label = f"Target G{assigned_gid}: {det['score']:.2f}"
            elif assigned_gid is not None:
                box_col = gate_palette[(assigned_gid - 1) % len(gate_palette)]
                label = f"YOLO G{assigned_gid}: {det['score']:.2f}"
            else:
                box_col = (140, 140, 140)  # Gray for unassigned
                label = f"Gate?: {det['score']:.2f}"

            cv2.rectangle(vis_img, (x1, y1), (x2, y2), box_col, 2)
            cv2.putText(vis_img, label, (x1, max(15, y1 - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.48, box_col, 2, cv2.LINE_AA)

            # Keypoints
            for i in range(len(det['kp_x'])):
                kx, ky, vis = int(det['kp_x'][i]), int(det['kp_y'][i]), det['kp_vis'][i]
                if vis >= self.corner_conf_thresh:
                    kp_col = (0, 0, 255) if 6 <= i <= 9 else (255, 250, 0)
                    cv2.circle(vis_img, (kx, ky), 4, kp_col, -1)

        # 3. Draw EKF Refined Polygon & Innovation Vectors for active target gate
        fsm_str = "SEARCHING"
        fsm_col = (0, 165, 255)
        nis_val = 0.0
        max_innov_val = 0.0

        if self.latest_projected_corners is not None:
            age_sec = (self.get_clock().now() - self.last_projected_corners_time).nanoseconds * 1e-9
            if age_sec < 1.5:
                proj = self.latest_projected_corners
                fsm_code = proj['fsm_code']
                nis_val = proj['nis']
                max_innov_val = proj['max_innov']
                ekf_pts = np.clip(proj['ekf_pixels'], -2000, 4000).astype(np.int32)

                # Draw EKF refined polygon in Magenta
                cv2.polylines(vis_img, [ekf_pts], isClosed=True, color=(255, 0, 255), thickness=2)
                for pt in ekf_pts:
                    cv2.circle(vis_img, (int(pt[0]), int(pt[1])), 4, (255, 0, 255), -1)
                cv2.putText(vis_img, f"EKF G{target_gate_id}",
                            (ekf_pts[1, 0] - 10, max(15, ekf_pts[1, 1] - 8)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 0, 255), 1, cv2.LINE_AA)

                fsm_str = {0: "BOOTSTRAP", 1: "TRACKING", 2: "REJECTED", 3: "RECOVERY"}.get(int(fsm_code), "UNKNOWN")
                fsm_col = {
                    0: (0, 255, 255),
                    1: (0, 255, 0),
                    2: (0, 165, 255),
                    3: (0, 0, 255)
                }.get(int(fsm_code), (200, 200, 200))

                # Draw innovation arrows from EKF corners to YOLO detected corners
                if target_det is not None:
                    k_to_yolo = [7, 8, 9, 6]
                    for k in range(4):
                        yolo_idx = k_to_yolo[k]
                        if target_det['kp_vis'][yolo_idx] >= self.corner_conf_thresh:
                            p_yolo = (int(target_det['kp_x'][yolo_idx]), int(target_det['kp_y'][yolo_idx]))
                            p_ekf = (int(ekf_pts[k, 0]), int(ekf_pts[k, 1]))
                            cv2.arrowedLine(vis_img, p_ekf, p_yolo, (0, 255, 255), 1, tipLength=0.25)

        # If no EKF overlay but target_det and prior exist, draw arrows from Prior to YOLO
        elif target_det is not None and target_gate_id in projected_priors:
            prior_corners = np.clip(projected_priors[target_gate_id]['corners'], -2000, 4000).astype(np.int32)
            k_to_yolo = [7, 8, 9, 6]
            for k in range(4):
                yolo_idx = k_to_yolo[k]
                if target_det['kp_vis'][yolo_idx] >= self.corner_conf_thresh:
                    p_yolo = (int(target_det['kp_x'][yolo_idx]), int(target_det['kp_y'][yolo_idx]))
                    p_prior = (int(prior_corners[k, 0]), int(prior_corners[k, 1]))
                    cv2.arrowedLine(vis_img, p_prior, p_yolo, (0, 255, 255), 1, tipLength=0.25)

        # 4. Telemetry HUD Box (Top-Left)
        box_w, box_h = 285, 82
        overlay = vis_img.copy()
        cv2.rectangle(overlay, (10, 10), (10 + box_w, 10 + box_h), (20, 20, 20), -1)
        cv2.addWeighted(overlay, 0.70, vis_img, 0.30, 0, vis_img)
        cv2.rectangle(vis_img, (10, 10), (10 + box_w, 10 + box_h), (80, 80, 80), 1)

        target_dist = projected_priors[target_gate_id]['dist'] if target_gate_id in projected_priors else 0.0

        cv2.putText(vis_img, f"TARGET: GATE {target_gate_id} ({fsm_str})", (18, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, fsm_col, 1, cv2.LINE_AA)
        cv2.putText(vis_img, f"Dist: {target_dist:.1f}m  CamPitch: {self.camera_pitch_deg:.1f}*", (18, 46),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (220, 220, 220), 1, cv2.LINE_AA)
        cv2.putText(vis_img, f"NIS: {nis_val:.2f}  MaxInn: {max_innov_val:.1f}px", (18, 64),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (220, 220, 220), 1, cv2.LINE_AA)
        cv2.putText(vis_img, "Colors: Cyan:Priors Mag:EKF Grn:Target Yel:Inn", (18, 80),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.33, (180, 180, 180), 1, cv2.LINE_AA)

        if publish_raw:
            debug_msg = Image()
            debug_msg.header = header
            debug_msg.height = vis_img.shape[0]
            debug_msg.width = vis_img.shape[1]
            debug_msg.encoding = "bgr8"
            debug_msg.is_bigendian = 0
            debug_msg.step = vis_img.shape[1] * 3
            debug_msg.data = vis_img.tobytes()
            self.pub_debug_img.publish(debug_msg)

        if publish_compressed:
            comp_msg = CompressedImage()
            comp_msg.header = header
            comp_msg.format = "jpeg"
            ret, jpeg_buf = cv2.imencode('.jpg', vis_img, [int(cv2.IMWRITE_JPEG_QUALITY), 75])
            if ret:
                comp_msg.data = jpeg_buf.tobytes()
                self.pub_debug_compressed.publish(comp_msg)


def main(args=None):
    rclpy.init(args=args)
    node = GateYoloNode()
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
