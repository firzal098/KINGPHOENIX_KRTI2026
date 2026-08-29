#!/usr/bin/env python3
"""
ROS 2 Jazzy Gate Perception Node
Subscribes to Webots/Camera UDP bridge images and camera info, performs ONNX keypoint 
detection (YOLOv8 pose schema), decodes gate corners, estimates 3D pose using IPPE/PnP, 
and publishes standard ROS 2 messages for downstream estimators and state machines.
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
from geometry_msgs.msg import PoseArray, PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Float64MultiArray

try:
    from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose, BoundingBox2D
    HAS_VISION_MSGS = True
except ImportError:
    HAS_VISION_MSGS = False
    Detection2DArray = None
    Detection2D = None
    ObjectHypothesisWithPose = None
    BoundingBox2D = None


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


class GatePerceptionNode(Node):
    def __init__(self):
        super().__init__('gate_perception_node')

        _default_model = os.path.join(
            get_package_share_directory('cuda_gate_inference'), 'resource', 'gate_keypoints.onnx'
        )
        self.declare_parameter('model_path', _default_model)
        self.declare_parameter('conf_threshold', 0.7)
        self.declare_parameter('corner_conf_threshold', 0.20)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('gate_width_m', 1.9)
        self.declare_parameter('gate_height_m', 2.0)
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('enable_monte_carlo_cov', True)
        self.declare_parameter('mc_samples', 20)
        self.declare_parameter('corner_noise_sigma', 2.0)

        self.model_path = str(self.get_parameter('model_path').value)
        self.conf_threshold = float(self.get_parameter('conf_threshold').value)
        self.corner_conf_thresh = float(self.get_parameter('corner_conf_threshold').value)
        self.iou_threshold = float(self.get_parameter('iou_threshold').value)
        self.gate_w = float(self.get_parameter('gate_width_m').value)
        self.gate_h = float(self.get_parameter('gate_height_m').value)
        self.publish_debug = bool(self.get_parameter('publish_debug_image').value)
        self.enable_mc_cov = bool(self.get_parameter('enable_monte_carlo_cov').value)
        self.mc_samples = int(self.get_parameter('mc_samples').value)
        self.corner_noise_sigma = float(self.get_parameter('corner_noise_sigma').value)

        # 3D object model points for the 4 physical gate corners (top-left, top-right, bottom-right, bottom-left)
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

        if HAS_VISION_MSGS:
            self.pub_detections_2d = self.create_publisher(
                Detection2DArray, '/perception/gate_detections_2d', 10
            )
        else:
            self.pub_detections_2d = None

        self.pub_poses_3d = self.create_publisher(
            PoseArray, '/perception/gate_poses_3d', 10
        )
        self.pub_covs_3d = self.create_publisher(
            Float64MultiArray, '/perception/gate_covariances_3d', 10
        )
        self.pub_primary_pose = self.create_publisher(
            PoseStamped, '/perception/gate_pose', 10
        )
        if self.publish_debug:
            self.pub_debug_img = self.create_publisher(
                Image, '/perception/debug_image', 10
            )
            self.pub_debug_compressed = self.create_publisher(
                CompressedImage, '/perception/debug_image/compressed', 10
            )

        self.get_logger().info(f"Gate Perception Node initialized using ONNX model: {self.model_path}")

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

        # Fast C++ preprocessing via cv2.dnn.blobFromImage (resizes, swaps RB to RGB, scales by 1/255, NCHW format)
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

                # Strict requirement: all 4 corners must be detected
                if valid_corners < 4:
                    continue

                cx, cy, w, h = boxes_640[idx]
                orig_cx = cx * x_scale
                orig_cy = cy * y_scale
                orig_w = w * x_scale
                orig_h = h * y_scale

                detections.append({
                    'bbox': (orig_cx, orig_cy, orig_w, orig_h),
                    'score': float(scores_filt[idx]),
                    'kp_x': kp_data[:, 0] * x_scale,
                    'kp_y': kp_data[:, 1] * y_scale,
                    'kp_vis': kp_data[:, 2],
                    'pose_3d': None
                })

        det_2d_msg = Detection2DArray() if HAS_VISION_MSGS else None
        if det_2d_msg is not None:
            det_2d_msg.header = msg.header

        pose_3d_msg = PoseArray()
        pose_3d_msg.header = msg.header
        cov_3d_msg = Float64MultiArray()

        best_detection_pose = None
        best_score = -1.0

        for det in detections:
            if det_2d_msg is not None:
                d2d = Detection2D()
                d2d.header = msg.header
                d2d.bbox.center.position.x = float(det['bbox'][0])
                d2d.bbox.center.position.y = float(det['bbox'][1])
                d2d.bbox.size_x = float(det['bbox'][2])
                d2d.bbox.size_y = float(det['bbox'][3])

                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = str("gate")
                hyp.hypothesis.score = float(det['score'])
                d2d.results.append(hyp)
                det_2d_msg.detections.append(d2d)

            # 3D Pose estimation via IPPE & Monte-Carlo Corner Perturbation
            if self.camera_matrix is not None:
                corners_2d = np.ascontiguousarray(
                    np.column_stack([det['kp_x'][6:10], det['kp_y'][6:10]]), dtype=np.float32
                )
                corner_vis = det['kp_vis'][6:10]

                if np.all(corner_vis >= self.corner_conf_thresh):
                    success, rvec, tvec = cv2.solvePnP(
                        self.object_points_4p,
                        corners_2d,
                        self.camera_matrix,
                        self.dist_coeffs,
                        flags=cv2.SOLVEPNP_IPPE
                    )
                else:
                    success = False

                if success:
                    qx, qy, qz, qw = rvec_to_quaternion(rvec)
                    tx, ty, tz = float(tvec[0][0]), float(tvec[1][0]), float(tvec[2][0])
                    t_nom = np.array([tx, ty, tz], dtype=np.float64)

                    # Swift Monte-Carlo Corner Perturbation for 3x3 Measurement Covariance (R_cam)
                    if self.enable_mc_cov:
                        sampled_translations = []
                        for _ in range(self.mc_samples):
                            # Add zero-mean Gaussian jitter to the 4 corner pixels
                            noise = np.random.normal(0.0, self.corner_noise_sigma, size=corners_2d.shape).astype(np.float32)
                            perturbed_corners = corners_2d + noise

                            ok_p, rvec_p, tvec_p = cv2.solvePnP(
                                self.object_points_4p,
                                perturbed_corners,
                                self.camera_matrix,
                                self.dist_coeffs,
                                flags=cv2.SOLVEPNP_IPPE
                            )
                            if ok_p and tvec_p[2][0] > 0.3:
                                sampled_translations.append(tvec_p.flatten())

                        if len(sampled_translations) >= max(5, self.mc_samples // 2):
                            samples = np.array(sampled_translations, dtype=np.float64)
                            diff = samples - t_nom
                            R_cam = (diff.T @ diff) / float(len(samples) - 1)
                            # Add minimal regularization floor (0.05m)^2
                            R_cam += np.eye(3, dtype=np.float64) * 0.0025
                        else:
                            d = np.linalg.norm(t_nom)
                            sig = 8.0 if d > 20.0 else (1.5 + (d / 20.0) * 6.5)
                            R_cam = np.eye(3, dtype=np.float64) * (sig ** 2)
                    else:
                        d = np.linalg.norm(t_nom)
                        sig = 8.0 if d > 20.0 else (1.5 + (d / 20.0) * 6.5)
                        R_cam = np.eye(3, dtype=np.float64) * (sig ** 2)

                    gate_pose = Pose()
                    gate_pose.position.x = tx
                    gate_pose.position.y = ty
                    gate_pose.position.z = tz
                    gate_pose.orientation.x = qx
                    gate_pose.orientation.y = qy
                    gate_pose.orientation.z = qz
                    gate_pose.orientation.w = qw

                    det['pose_3d'] = (tx, ty, tz)
                    pose_3d_msg.poses.append(gate_pose)
                    cov_3d_msg.data.extend(R_cam.flatten().tolist())

                    if det['score'] > best_score:
                        best_score = det['score']
                        best_detection_pose = gate_pose

        if self.pub_detections_2d is not None and det_2d_msg is not None:
            self.pub_detections_2d.publish(det_2d_msg)

        self.pub_poses_3d.publish(pose_3d_msg)
        self.pub_covs_3d.publish(cov_3d_msg)

        if best_detection_pose is not None:
            primary_pose_msg = PoseStamped()
            primary_pose_msg.header = msg.header
            primary_pose_msg.pose = best_detection_pose
            self.pub_primary_pose.publish(primary_pose_msg)

        # Gate debug drawing and publication: only execute if subscribers exist
        has_img_subs = self.pub_debug_img.get_subscription_count() > 0
        has_compressed_subs = self.pub_debug_compressed.get_subscription_count() > 0
        if self.publish_debug and (has_img_subs or has_compressed_subs):
            self.publish_debug_overlay(frame, detections, msg.header, has_img_subs, has_compressed_subs)

    def publish_empty_results(self, header):
        if self.pub_detections_2d is not None and HAS_VISION_MSGS:
            det_msg = Detection2DArray()
            det_msg.header = header
            self.pub_detections_2d.publish(det_msg)

        pose_msg = PoseArray()
        pose_msg.header = header
        self.pub_poses_3d.publish(pose_msg)

        cov_msg = Float64MultiArray()
        self.pub_covs_3d.publish(cov_msg)

    def publish_debug_overlay(self, frame, detections, header, publish_raw=True, publish_compressed=True):
        vis_img = frame.copy()
        for det in detections:
            cx, cy, w, h = det['bbox']
            x1, y1 = int(cx - w / 2), int(cy - h / 2)
            x2, y2 = int(cx + w / 2), int(cy + h / 2)

            cv2.rectangle(vis_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(vis_img, f"Gate: {det['score']:.2f}", (x1, max(15, y1 - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            if det.get('pose_3d') is not None:
                tx, ty, tz = det['pose_3d']
                pos_str = f"XYZ: [{tx:+.2f}, {ty:+.2f}, {tz:+.2f}]m"
                cv2.putText(vis_img, pos_str, (x1, min(vis_img.shape[0] - 6, y2 + 18)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 2)

            for i in range(len(det['kp_x'])):
                kx, ky, vis = int(det['kp_x'][i]), int(det['kp_y'][i]), det['kp_vis'][i]
                if vis >= self.corner_conf_thresh:
                    color = (0, 0, 255) if 6 <= i <= 9 else (255, 250, 0)
                    cv2.circle(vis_img, (kx, ky), 4, color, -1)

        # 1. Raw Image
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

        # 2. Compressed JPEG Image (fast web streaming for GCS)
        if publish_compressed:
            compressed_msg = CompressedImage()
            compressed_msg.header = header
            compressed_msg.format = "jpeg"
            ret, jpeg_buf = cv2.imencode('.jpg', vis_img, [int(cv2.IMWRITE_JPEG_QUALITY), 75])
            if ret:
                compressed_msg.data = jpeg_buf.tobytes()
                self.pub_debug_compressed.publish(compressed_msg)


def main(args=None):
    rclpy.init(args=args)
    node = GatePerceptionNode()
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