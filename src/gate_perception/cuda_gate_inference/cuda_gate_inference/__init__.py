#!/usr/bin/env python3
"""
ROS 2 Jazzy Gate Perception Node (Pure 2D YOLO Inference + Gate Memory Projection Overlay)
Subscribes to camera images, executes ONNX YOLOv8 keypoint detection on CUDA/CPU,
publishes 2D gate corner pixels for the C++ estimator, and renders live FPV debug images
with half-transparent YOLO detections and projected 3D gate memory positions.
"""

import os
import numpy as np
import cv2
import onnxruntime as ort

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from ament_index_python.packages import get_package_share_directory

from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Float64MultiArray

try:
    from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
    HAS_VISION_MSGS = True
except ImportError:
    HAS_VISION_MSGS = False
    Detection2DArray = None
    Detection2D = None
    ObjectHypothesisWithPose = None


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


class GatePerceptionNode(Node):
    def __init__(self):
        super().__init__('gate_perception_node')

        _default_model = os.path.join(
            get_package_share_directory('cuda_gate_inference'), 'resource', 'gate_keypoints.onnx'
        )
        self.declare_parameter('model_path', _default_model)
        self.declare_parameter('method', 'pnp') # 'pnp' or 'pixel_innovation'
        self.declare_parameter('conf_threshold', 0.50)
        self.declare_parameter('corner_conf_threshold', 0.15)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('publish_debug_image', True)

        self.model_path = str(self.get_parameter('model_path').value)
        self.method = str(self.get_parameter('method').value)
        self.conf_threshold = float(self.get_parameter('conf_threshold').value)
        self.corner_conf_thresh = float(self.get_parameter('corner_conf_threshold').value)
        self.iou_threshold = float(self.get_parameter('iou_threshold').value)
        self.publish_debug = bool(self.get_parameter('publish_debug_image').value)

        self.latest_projected_pixels = []

        self.init_onnx_session()

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.sub_image = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, qos_profile
        )

        # Subscribe to 3D-to-2D projected gate memory pixels from C++ Estimator
        self.sub_projected_pixels = self.create_subscription(
            Float64MultiArray, '/estimator/projected_gate_pixels', self.projected_pixels_callback, 10
        )

        # 2D Keypoint publisher for downstream C++ Estimator
        # Format per detected gate: [score, u0, v0, c0, u1, v1, c1, u2, v2, c2, u3, v3, c3]
        self.pub_corners_2d = self.create_publisher(
            Float64MultiArray, '/perception/gate_corners_2d', 10
        )

        if HAS_VISION_MSGS:
            self.pub_detections_2d = self.create_publisher(
                Detection2DArray, '/perception/gate_detections_2d', 10
            )
        else:
            self.pub_detections_2d = None

        if self.publish_debug:
            self.pub_debug_img = self.create_publisher(
                Image, '/perception/debug_image', 10
            )
            self.pub_debug_compressed = self.create_publisher(
                CompressedImage, '/perception/debug_image/compressed', 10
            )

        self.get_logger().info(
            f"Gate YOLO Perception Node initialized (Method: '{self.method}', Model: {self.model_path})"
        )

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

    def projected_pixels_callback(self, msg: Float64MultiArray):
        if msg and len(msg.data) > 0:
            self.latest_projected_pixels = list(msg.data)

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

        # Fast preprocessing via cv2.dnn.blobFromImage (resizes, swaps RB to RGB, scales by 1/255, NCHW format)
        input_tensor = cv2.dnn.blobFromImage(
            frame,
            scalefactor=1.0 / 255.0,
            size=(640, 640),
            mean=(0, 0, 0),
            swapRB=True,
            crop=False
        )

        # Execute ONNX TensorRT / CUDA inference
        outputs = self.ort_session.run(None, {self.input_name: input_tensor})[0]
        
        if outputs.ndim == 3 and outputs.shape[1] == 38:
            outputs = outputs[0].transpose(1, 0)
        elif outputs.ndim == 3:
            outputs = outputs[0]

        scores = outputs[:, 4]
        mask = scores >= self.conf_threshold
        filtered_outputs = outputs[mask]

        detections = []
        corners_data = []

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

                # Strict requirement: all 4 corners (TL, TR, BR, BL) must be detected
                if valid_corners < 4:
                    continue

                cx, cy, w, h = boxes_640[idx]
                orig_cx = cx * x_scale
                orig_cy = cy * y_scale
                orig_w = w * x_scale
                orig_h = h * y_scale

                score = float(scores_filt[idx])
                
                # 4 physical corners in image space: 6 (TL), 7 (TR), 8 (BR), 9 (BL)
                u0, v0, c0 = float(kp_data[6, 0] * x_scale), float(kp_data[6, 1] * y_scale), float(kp_data[6, 2])
                u1, v1, c1 = float(kp_data[7, 0] * x_scale), float(kp_data[7, 1] * y_scale), float(kp_data[7, 2])
                u2, v2, c2 = float(kp_data[8, 0] * x_scale), float(kp_data[8, 1] * y_scale), float(kp_data[8, 2])
                u3, v3, c3 = float(kp_data[9, 0] * x_scale), float(kp_data[9, 1] * y_scale), float(kp_data[9, 2])

                # Pack 13 floats per gate: [score, u0, v0, c0, u1, v1, c1, u2, v2, c2, u3, v3, c3]
                corners_data.extend([score, u0, v0, c0, u1, v1, c1, u2, v2, c2, u3, v3, c3])

                detections.append({
                    'bbox': (orig_cx, orig_cy, orig_w, orig_h),
                    'score': score,
                    'kp_x': kp_data[:, 0] * x_scale,
                    'kp_y': kp_data[:, 1] * y_scale,
                    'kp_vis': kp_data[:, 2],
                })

        # Publish 2D corner measurements for C++ Estimator
        corners_msg = Float64MultiArray()
        corners_msg.data = corners_data
        self.pub_corners_2d.publish(corners_msg)

        # Publish 2D detections if vision_msgs is available
        if self.pub_detections_2d is not None and HAS_VISION_MSGS:
            det_2d_msg = Detection2DArray()
            det_2d_msg.header = msg.header
            for det in detections:
                d2d = Detection2D()
                d2d.header = msg.header
                d2d.bbox.center.position.x = float(det['bbox'][0])
                d2d.bbox.center.position.y = float(det['bbox'][1])
                d2d.bbox.size_x = float(det['bbox'][2])
                d2d.bbox.size_y = float(det['bbox'][3])

                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = "gate"
                hyp.hypothesis.score = float(det['score'])
                d2d.results.append(hyp)
                det_2d_msg.detections.append(d2d)
            self.pub_detections_2d.publish(det_2d_msg)

        # Gate debug drawing and publication: only execute if subscribers exist
        has_img_subs = self.pub_debug_img.get_subscription_count() > 0 if self.publish_debug else False
        has_compressed_subs = self.pub_debug_compressed.get_subscription_count() > 0 if self.publish_debug else False
        if self.publish_debug and (has_img_subs or has_compressed_subs):
            self.publish_debug_overlay(frame, detections, msg.header, has_img_subs, has_compressed_subs)

    def publish_debug_overlay(self, frame, detections, header, publish_raw=True, publish_compressed=True):
        vis_img = frame.copy()

        # 1. Render YOLO Detections (Solid for PnP, Half-Transparent for Pixel Innovation)
        if len(detections) > 0:
            if self.method in ['pixel_innovation', 'pixel-innovation', 'pixel_innovation_advanced', 'pixel-innovation-advanced']:
                overlay = vis_img.copy()
                for det in detections:
                    cx, cy, w, h = det['bbox']
                    x1, y1 = int(cx - w / 2), int(cy - h / 2)
                    x2, y2 = int(cx + w / 2), int(cy + h / 2)

                    cv2.rectangle(overlay, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(overlay, f"Gate YOLO: {det['score']:.2f}", (x1, max(15, y1 - 6)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)

                    for i in range(len(det['kp_x'])):
                        kx, ky, vis = int(det['kp_x'][i]), int(det['kp_y'][i]), det['kp_vis'][i]
                        if vis >= self.corner_conf_thresh:
                            color = (0, 0, 255) if 6 <= i <= 9 else (255, 250, 0)
                            cv2.circle(overlay, (kx, ky), 4, color, -1)
                # Alpha blend overlay layer at 40% opacity
                cv2.addWeighted(overlay, 0.40, vis_img, 0.60, 0, vis_img)
            else:
                for det in detections:
                    cx, cy, w, h = det['bbox']
                    x1, y1 = int(cx - w / 2), int(cy - h / 2)
                    x2, y2 = int(cx + w / 2), int(cy + h / 2)

                    cv2.rectangle(vis_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(vis_img, f"Gate: {det['score']:.2f}", (x1, max(15, y1 - 6)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                    for i in range(len(det['kp_x'])):
                        kx, ky, vis = int(det['kp_x'][i]), int(det['kp_y'][i]), det['kp_vis'][i]
                        if vis >= self.corner_conf_thresh:
                            color = (0, 0, 255) if 6 <= i <= 9 else (255, 250, 0)
                            cv2.circle(vis_img, (kx, ky), 4, color, -1)

        # 2. Render 3D-to-2D Projected Gate Positions from Estimator Memory
        # Format per visible gate (12 floats): [id, is_active, u_c, v_c, u_tl, v_tl, u_tr, v_tr, u_br, v_br, u_bl, v_bl]
        if len(self.latest_projected_pixels) >= 12 and (len(self.latest_projected_pixels) % 12 == 0):
            num_mem_gates = len(self.latest_projected_pixels) // 12
            for g in range(num_mem_gates):
                off = g * 12
                gate_id = int(self.latest_projected_pixels[off + 0])
                is_active = bool(self.latest_projected_pixels[off + 1] > 0.5)
                u_c, v_c = self.latest_projected_pixels[off + 2], self.latest_projected_pixels[off + 3]
                u_tl, v_tl = self.latest_projected_pixels[off + 4], self.latest_projected_pixels[off + 5]
                u_tr, v_tr = self.latest_projected_pixels[off + 6], self.latest_projected_pixels[off + 7]
                u_br, v_br = self.latest_projected_pixels[off + 8], self.latest_projected_pixels[off + 9]
                u_bl, v_bl = self.latest_projected_pixels[off + 10], self.latest_projected_pixels[off + 11]

                # Active target gate is bright Cyan (255, 255, 0), other gates are Orange (0, 165, 255)
                color_box = (255, 255, 0) if is_active else (0, 165, 255)
                thickness = 2 if is_active else 1

                corners = [(u_tl, v_tl), (u_tr, v_tr), (u_br, v_br), (u_bl, v_bl)]
                valid_pts = []
                for pt in corners:
                    if pt[0] > -500 and pt[1] > -500:
                        valid_pts.append((int(pt[0]), int(pt[1])))

                # Draw projected 3D wireframe box
                if len(valid_pts) == 4:
                    pts_arr = np.array(valid_pts, np.int32).reshape((-1, 1, 2))
                    cv2.polylines(vis_img, [pts_arr], isClosed=True, color=color_box, thickness=thickness)

                # Draw projected corner dots
                for pt in valid_pts:
                    cv2.circle(vis_img, pt, 4, color_box, -1)
                    cv2.circle(vis_img, pt, 5, (0, 0, 0), 1)

                # Draw projected center crosshair and label
                if u_c > -500 and v_c > -500:
                    cx_i, cy_i = int(u_c), int(v_c)
                    cv2.circle(vis_img, (cx_i, cy_i), 4, (0, 255, 255) if is_active else color_box, -1)
                    cv2.drawMarker(vis_img, (cx_i, cy_i), color_box, markerType=cv2.MARKER_CROSS, markerSize=10, thickness=2)

                    tag = f"Gate {gate_id} [Target Mem]" if is_active else f"Gate {gate_id} [Mem]"
                    label_y = max(18, (valid_pts[0][1] - 6) if len(valid_pts) > 0 else (cy_i - 8))
                    label_x = max(10, (valid_pts[0][0]) if len(valid_pts) > 0 else (cx_i - 20))
                    cv2.putText(vis_img, tag, (label_x, label_y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color_box, 1)

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