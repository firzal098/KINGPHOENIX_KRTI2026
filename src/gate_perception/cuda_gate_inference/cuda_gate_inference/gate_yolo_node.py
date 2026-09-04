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

        # Direct prior projection fallback parameters & subscriptions (works without gate_estimator_node)
        self.declare_parameter('camera_pitch_deg', 15.0)
        self.declare_parameter('prior_source', 'defined_constants')
        self.camera_pitch_deg = float(self.get_parameter('camera_pitch_deg').value)
        self.prior_source = str(self.get_parameter('prior_source').value).lower()

        self.drone_pos_w = None
        self.drone_quat_w = None
        self.R_wb = None
        self.active_target_gate_idx = 0

        self.sub_drone_pose = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self.drone_pose_callback, qos_profile
        )
        self.sub_target_gate = self.create_subscription(
            Int32, '/controller/target_gate_index', self.target_gate_callback, 10
        )

        self.get_logger().info(f"Gate YOLO Node initialized using ONNX model: {self.model_path}")

    def drone_pose_callback(self, msg: PoseStamped):
        p = msg.pose.position
        o = msg.pose.orientation
        self.drone_pos_w = np.array([p.x, p.y, p.z], dtype=np.float64)
        self.drone_quat_w = np.array([o.w, o.x, o.y, o.z], dtype=np.float64)
        self.R_wb = quat_to_rot_matrix(self.drone_quat_w)

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
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape((3, 3))
            self.dist_coeffs = np.array(msg.d, dtype=np.float64) if len(msg.d) > 0 else np.zeros((5, 1))
            self.get_logger().info(f"Camera intrinsics registered: fx={self.camera_matrix[0,0]:.2f}, fy={self.camera_matrix[1,1]:.2f}")

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

        # 1. Associate and publish 2D Corner Keypoints (PolygonStamped)
        best_det = None
        if len(detections) > 0:
            if self.latest_projected_corners is not None:
                ekf_px = self.latest_projected_corners['ekf_pixels']
                target_u = float(np.mean(ekf_px[:, 0]))
                target_v = float(np.mean(ekf_px[:, 1]))
                proj_w = float(np.max(ekf_px[:, 0]) - np.min(ekf_px[:, 0]))

                def assoc_cost(d):
                    cx, cy, w, h = d['bbox']
                    # Horizontal error is primary (yaw/lateral track alignment)
                    u_err = abs(cx - target_u)
                    # Vertical error (altitude/pitch alignment)
                    v_err = abs(cy - target_v)
                    # Gate width/scale error (distance to gate cue)
                    w_err = abs(w - proj_w)

                    cost = u_err + 0.4 * v_err + 1.2 * w_err

                    # Temporal tracking hysteresis: strongly prefer continuing to track same gate
                    if self.last_target_det_center is not None:
                        prev_dist = math.hypot(cx - self.last_target_det_center[0],
                                               cy - self.last_target_det_center[1])
                        if prev_dist < 80.0:
                            cost -= 60.0

                    return cost

                best_det = min(detections, key=assoc_cost)
                self.last_target_det_center = (best_det['bbox'][0], best_det['bbox'][1])
            else:
                best_det = max(detections, key=lambda d: d['score'])
                self.last_target_det_center = (best_det['bbox'][0], best_det['bbox'][1])

            poly_msg = PolygonStamped()
            poly_msg.header = msg.header
            # 4 Points in canonical order: TL, TR, BR, BL
            # YOLO model keypoint mapping:
            # Index 7: Top-Left (TL)
            # Index 8: Top-Right (TR)
            # Index 9: Bottom-Right (BR)
            # Index 6: Bottom-Left (BL)
            corner_indices = [7, 8, 9, 6]
            for k_idx in corner_indices:
                pt = Point32()
                pt.x = float(best_det['kp_x'][k_idx])
                pt.y = float(best_det['kp_y'][k_idx])
                pt.z = float(best_det['kp_vis'][k_idx])
                poly_msg.polygon.points.append(pt)
            self.pub_corners_2d.publish(poly_msg)

        # 2. Fallback PnP solving (if enabled and requested by legacy nodes)
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

        # 3. Debug Overlays (Publish continuously for GCS and ROS tools)
        if self.publish_debug:
            self.publish_debug_overlay(frame, detections, best_det, msg.header, publish_raw=True, publish_compressed=True)

    def publish_debug_overlay(self, frame, detections, best_det, header, publish_raw=True, publish_compressed=True):
        vis_img = frame.copy()

        # 1. Draw YOLO Bounding Boxes and Detected Keypoints
        for det in detections:
            cx, cy, w, h = det['bbox']
            x1, y1 = int(cx - w / 2), int(cy - h / 2)
            x2, y2 = int(cx + w / 2), int(cy + h / 2)

            is_target = (det is best_det)
            box_col = (0, 255, 0) if is_target else (160, 160, 160)
            tag = "Target Gate" if is_target else "Gate"
            cv2.rectangle(vis_img, (x1, y1), (x2, y2), box_col, 2)
            cv2.putText(vis_img, f"{tag}: {det['score']:.2f}", (x1, max(15, y1 - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, box_col, 2)

            for i in range(len(det['kp_x'])):
                kx, ky, vis = int(det['kp_x'][i]), int(det['kp_y'][i]), det['kp_vis'][i]
                if vis >= self.corner_conf_thresh:
                    # Red for main corners 6..9, orange/cyan for inner keypoints
                    color = (0, 0, 255) if 6 <= i <= 9 else (255, 250, 0)
                    cv2.circle(vis_img, (kx, ky), 4, color, -1)

        # 2. Draw Projected Gate Pixels (Prior Surveyed vs EKF Refined)
        drawn_ekf_overlay = False
        if self.latest_projected_corners is not None:
            age_sec = (self.get_clock().now() - self.last_projected_corners_time).nanoseconds * 1e-9
            if age_sec < 1.0:
                drawn_ekf_overlay = True
                proj = self.latest_projected_corners
                gate_id = proj['gate_id']
                fsm_code = proj['fsm_code']
                nis = proj['nis']
                max_innov = proj['max_innov']

                prior_pts = np.clip(proj['prior_pixels'], -2000, 4000).astype(np.int32)
                ekf_pts = np.clip(proj['ekf_pixels'], -2000, 4000).astype(np.int32)

                # A. Prior Gate (Cyan polygon: B=255, G=255, R=0)
                cv2.polylines(vis_img, [prior_pts], isClosed=True, color=(255, 255, 0), thickness=2)
                for pt in prior_pts:
                    cv2.circle(vis_img, (int(pt[0]), int(pt[1])), 4, (255, 255, 0), -1)
                cv2.putText(vis_img, f"Prior G{gate_id}",
                            (prior_pts[0, 0] - 10, max(15, prior_pts[0, 1] - 8)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 1, cv2.LINE_AA)

                # B. EKF Refined Gate (Magenta polygon: B=255, G=0, R=255)
                cv2.polylines(vis_img, [ekf_pts], isClosed=True, color=(255, 0, 255), thickness=2)
                for pt in ekf_pts:
                    cv2.circle(vis_img, (int(pt[0]), int(pt[1])), 4, (255, 0, 255), -1)
                cv2.putText(vis_img, f"EKF G{gate_id}",
                            (ekf_pts[1, 0] - 10, max(15, ekf_pts[1, 1] - 8)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 0, 255), 1, cv2.LINE_AA)

                # C. Correction Vectors: Prior -> EKF (Light Gray dashed/thin line)
                for k in range(4):
                    p_prior = (int(prior_pts[k, 0]), int(prior_pts[k, 1]))
                    p_ekf = (int(ekf_pts[k, 0]), int(ekf_pts[k, 1]))
                    if math.hypot(p_ekf[0] - p_prior[0], p_ekf[1] - p_prior[1]) >= 2:
                        cv2.line(vis_img, p_prior, p_ekf, (200, 200, 200), 1, cv2.LINE_AA)

                # D. Innovation Vectors: EKF -> YOLO detection (Yellow arrow)
                # Canonical corner mapping:
                # EKF corners: 0: TL, 1: TR, 2: BR, 3: BL
                # YOLO keypoints: 7: TL, 8: TR, 9: BR, 6: BL
                if best_det is not None:
                    k_to_yolo = [7, 8, 9, 6]
                    for k in range(4):
                        yolo_idx = k_to_yolo[k]
                        if best_det['kp_vis'][yolo_idx] >= self.corner_conf_thresh:
                            p_yolo = (int(best_det['kp_x'][yolo_idx]), int(best_det['kp_y'][yolo_idx]))
                            p_ekf = (int(ekf_pts[k, 0]), int(ekf_pts[k, 1]))
                            cv2.arrowedLine(vis_img, p_ekf, p_yolo, (0, 255, 255), 1, tipLength=0.25)

                # E. Telemetry HUD Box (Top-Left)
                box_w, box_h = 245, 76
                overlay = vis_img.copy()
                cv2.rectangle(overlay, (10, 10), (10 + box_w, 10 + box_h), (20, 20, 20), -1)
                cv2.addWeighted(overlay, 0.65, vis_img, 0.35, 0, vis_img)
                cv2.rectangle(vis_img, (10, 10), (10 + box_w, 10 + box_h), (80, 80, 80), 1)

                fsm_str = {0: "BOOTSTRAP", 1: "TRACKING", 2: "REJECTED", 3: "RECOVERY"}.get(int(fsm_code), "UNKNOWN")
                fsm_col = {
                    0: (0, 255, 255),   # Yellow
                    1: (0, 255, 0),     # Green
                    2: (0, 165, 255),   # Orange
                    3: (0, 0, 255)      # Red
                }.get(int(fsm_code), (200, 200, 200))

                cv2.putText(vis_img, f"GATE {gate_id}: {fsm_str}", (18, 28),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, fsm_col, 1, cv2.LINE_AA)
                cv2.putText(vis_img, f"NIS: {nis:.2f}  MaxInn: {max_innov:.1f}px", (18, 46),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.40, (220, 220, 220), 1, cv2.LINE_AA)
                cv2.putText(vis_img, "Cyan:Prior  Mag:EKF  Yel:Inn", (18, 64),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.38, (180, 180, 180), 1, cv2.LINE_AA)

        # Fallback: Direct Prior Projection when gate_estimator_node is not running
        if not drawn_ekf_overlay and self.drone_pos_w is not None and self.camera_matrix is not None:
            self.draw_direct_prior_projection(vis_img, best_det)

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

    def draw_direct_prior_projection(self, vis_img, best_det):
        """Directly projects target gate prior into camera image without requiring gate_estimator_node."""
        gate_list = WEBOTS_GATES if self.prior_source == 'webots_world' else DEFINED_PRIORS
        if not (0 <= self.active_target_gate_idx < len(gate_list)):
            return

        gid, gx, gy, gz, gyaw = gate_list[self.active_target_gate_idx]
        pos_enu = np.array([gx, gy, gz], dtype=np.float64)
        dist = float(np.linalg.norm(pos_enu - self.drone_pos_w))

        hw = self.gate_w / 2.0
        hh = self.gate_h / 2.0
        u_lat = np.array([-math.sin(gyaw), math.cos(gyaw), 0.0], dtype=np.float64)
        u_vert = np.array([0.0, 0.0, 1.0], dtype=np.float64)
        corners_w = [
            pos_enu + hw * u_lat + hh * u_vert,  # TL
            pos_enu - hw * u_lat + hh * u_vert,  # TR
            pos_enu - hw * u_lat - hh * u_vert,  # BR
            pos_enu + hw * u_lat - hh * u_vert,  # BL
        ]

        R_bc = get_camera_to_body_rotation(self.camera_pitch_deg)
        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]

        proj_pts = []
        all_ok = True
        depths = []
        for c_w in corners_w:
            p_c = world_to_camera_rdf(c_w, self.drone_pos_w, self.R_wb, R_bc)
            depths.append(p_c[2])
            if p_c[2] < 0.2:
                all_ok = False
                break
            u = fx * (p_c[0] / p_c[2]) + cx
            v = fy * (p_c[1] / p_c[2]) + cy
            proj_pts.append([u, v])

        # Telemetry HUD Box (Top-Left)
        box_w, box_h = 280, 80
        overlay = vis_img.copy()
        cv2.rectangle(overlay, (10, 10), (10 + box_w, 10 + box_h), (20, 20, 20), -1)
        cv2.addWeighted(overlay, 0.70, vis_img, 0.30, 0, vis_img)
        cv2.rectangle(vis_img, (10, 10), (10 + box_w, 10 + box_h), (80, 80, 80), 1)

        cv2.putText(vis_img, f"DIRECT PRIOR: GATE {gid} ({self.prior_source})", (18, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(vis_img, f"Dist: {dist:.1f}m  CamPitch: {self.camera_pitch_deg:.1f}*", (18, 46),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (220, 220, 220), 1, cv2.LINE_AA)

        if all_ok:
            pts = np.clip(np.array(proj_pts, dtype=np.float32), -2000, 4000).astype(np.int32)
            cv2.polylines(vis_img, [pts], isClosed=True, color=(255, 255, 0), thickness=2)
            corner_names = ["TL", "TR", "BR", "BL"]
            for k in range(4):
                cv2.circle(vis_img, (int(pts[k, 0]), int(pts[k, 1])), 4, (255, 255, 0), -1)
                cv2.putText(vis_img, corner_names[k], (int(pts[k, 0]) - 10, int(pts[k, 1]) - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.38, (255, 255, 0), 1, cv2.LINE_AA)
            cv2.putText(vis_img, f"Prior G{gid}", (pts[0, 0] - 10, max(15, pts[0, 1] - 12)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 1, cv2.LINE_AA)

            # Draw vectors from Prior to YOLO detected corners (Yellow arrows)
            if best_det is not None:
                k_to_yolo = [7, 8, 9, 6]
                for k in range(4):
                    yolo_idx = k_to_yolo[k]
                    if best_det['kp_vis'][yolo_idx] >= self.corner_conf_thresh:
                        p_yolo = (int(best_det['kp_x'][yolo_idx]), int(best_det['kp_y'][yolo_idx]))
                        p_prior = (int(pts[k, 0]), int(pts[k, 1]))
                        cv2.arrowedLine(vis_img, p_prior, p_yolo, (0, 255, 255), 1, tipLength=0.25)

            cv2.putText(vis_img, f"TL:({pts[0,0]},{pts[0,1]}) TR:({pts[1,0]},{pts[1,1]})", (18, 64),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.36, (200, 200, 200), 1, cv2.LINE_AA)
        else:
            depth_val = depths[0] if depths else 0.0
            cv2.putText(vis_img, f"Gate {gid} behind/outside view (Z_cam={depth_val:.2f}m)", (18, 64),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.36, (0, 100, 255), 1, cv2.LINE_AA)


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
