#!/usr/bin/env python3
import unittest
import numpy as np
import cv2
import math

class TestDebugOverlay(unittest.TestCase):
    def test_overlay_rendering(self):
        vis_img = np.zeros((480, 640, 3), dtype=np.uint8)
        detections = [{
            'bbox': (320, 240, 100, 100),
            'score': 0.95,
            'kp_x': np.array([0,0,0,0,0,0, 270, 370, 370, 270], dtype=np.float32),
            'kp_y': np.array([0,0,0,0,0,0, 190, 190, 290, 290], dtype=np.float32),
            'kp_vis': np.array([0,0,0,0,0,0, 0.9, 0.9, 0.9, 0.9], dtype=np.float32)
        }]

        latest_projected_corners = {
            'gate_id': 1,
            'fsm_code': 1,
            'nis': 4.25,
            'max_innov': 8.5,
            'ekf_pixels': np.array([[265, 185], [375, 185], [375, 295], [265, 295]], dtype=np.float32),
            'prior_pixels': np.array([[260, 180], [380, 180], [380, 300], [260, 300]], dtype=np.float32)
        }

        # 1. Bboxes
        for det in detections:
            cx, cy, w, h = det['bbox']
            x1, y1 = int(cx - w / 2), int(cy - h / 2)
            x2, y2 = int(cx + w / 2), int(cy + h / 2)
            cv2.rectangle(vis_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            for i in range(len(det['kp_x'])):
                kx, ky, vis = int(det['kp_x'][i]), int(det['kp_y'][i]), det['kp_vis'][i]
                if vis >= 0.15:
                    color = (0, 0, 255) if 6 <= i <= 9 else (255, 250, 0)
                    cv2.circle(vis_img, (kx, ky), 4, color, -1)

        # 2. Projected corners
        proj = latest_projected_corners
        gate_id = proj['gate_id']
        fsm_code = proj['fsm_code']
        nis = proj['nis']
        max_innov = proj['max_innov']

        prior_pts = np.clip(proj['prior_pixels'], -2000, 4000).astype(np.int32)
        ekf_pts = np.clip(proj['ekf_pixels'], -2000, 4000).astype(np.int32)

        # A. Prior Gate (Cyan)
        cv2.polylines(vis_img, [prior_pts], isClosed=True, color=(255, 255, 0), thickness=2)
        for pt in prior_pts:
            cv2.circle(vis_img, (int(pt[0]), int(pt[1])), 4, (255, 255, 0), -1)

        # B. EKF Refined Gate (Magenta)
        cv2.polylines(vis_img, [ekf_pts], isClosed=True, color=(255, 0, 255), thickness=2)
        for pt in ekf_pts:
            cv2.circle(vis_img, (int(pt[0]), int(pt[1])), 4, (255, 0, 255), -1)

        # C. Prior -> EKF shift
        for k in range(4):
            p_prior = (int(prior_pts[k, 0]), int(prior_pts[k, 1]))
            p_ekf = (int(ekf_pts[k, 0]), int(ekf_pts[k, 1]))
            if math.hypot(p_ekf[0] - p_prior[0], p_ekf[1] - p_prior[1]) >= 2:
                cv2.line(vis_img, p_prior, p_ekf, (200, 200, 200), 1, cv2.LINE_AA)

        # D. Innovation EKF -> YOLO
        best_det = detections[0]
        for k in range(4):
            kp_idx = k + 6
            if best_det['kp_vis'][kp_idx] >= 0.15:
                p_yolo = (int(best_det['kp_x'][kp_idx]), int(best_det['kp_y'][kp_idx]))
                p_ekf = (int(ekf_pts[k, 0]), int(ekf_pts[k, 1]))
                cv2.arrowedLine(vis_img, p_ekf, p_yolo, (0, 255, 255), 1, tipLength=0.25)

        # E. HUD Box
        box_w, box_h = 245, 76
        overlay = vis_img.copy()
        cv2.rectangle(overlay, (10, 10), (10 + box_w, 10 + box_h), (20, 20, 20), -1)
        cv2.addWeighted(overlay, 0.65, vis_img, 0.35, 0, vis_img)
        cv2.rectangle(vis_img, (10, 10), (10 + box_w, 10 + box_h), (80, 80, 80), 1)

        cv2.putText(vis_img, f"GATE {gate_id}: TRACKING", (18, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1, cv2.LINE_AA)
        cv2.putText(vis_img, f"NIS: {nis:.2f}  MaxInn: {max_innov:.1f}px", (18, 46),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.40, (220, 220, 220), 1, cv2.LINE_AA)
        cv2.putText(vis_img, "Cyan:Prior  Mag:EKF  Yel:Inn", (18, 64),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (180, 180, 180), 1, cv2.LINE_AA)

        ret, buf = cv2.imencode('.jpg', vis_img)
        self.assertTrue(ret)
        self.assertGreater(len(buf), 1000)

if __name__ == '__main__':
    unittest.main()
