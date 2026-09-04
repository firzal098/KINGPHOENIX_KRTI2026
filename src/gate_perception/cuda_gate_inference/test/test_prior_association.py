#!/usr/bin/env python3
import math
import unittest
import numpy as np

import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../cuda_gate_inference')))

from pixel_innovation_ekf import (
    quat_to_rot_matrix,
    get_camera_to_body_rotation,
    world_to_camera_rdf,
)


class TestPriorAssociation(unittest.TestCase):
    """
    Tests 3D-to-2D multi-gate prior projection and 2D bounding-box / corner association.
    """

    def setUp(self):
        self.fx = 547.44
        self.fy = 547.44
        self.cx = 320.0
        self.cy = 320.0
        self.K = np.array([
            [self.fx, 0.0, self.cx],
            [0.0, self.fy, self.cy],
            [0.0, 0.0, 1.0]
        ], dtype=np.float64)

        # Gate 1 and Gate 2 defined priors in ENU
        self.priors = [
            (1, np.array([29.33, -0.41, 0.75]), 0.0),
            (2, np.array([19.31, -5.46, 0.75]), 0.0),
        ]

        self.drone_pos = np.array([0.0, 0.0, 0.0])
        self.drone_quat = np.array([1.0, 0.0, 0.0, 0.0])
        self.R_wb = quat_to_rot_matrix(self.drone_quat)
        self.camera_pitch_deg = 23.0
        self.R_bc = get_camera_to_body_rotation(self.camera_pitch_deg)

        self.gate_w = 1.9
        self.gate_h = 2.0

    def compute_corners(self, center_enu, yaw_rad):
        hw = self.gate_w / 2.0
        hh = self.gate_h / 2.0
        u_lat = np.array([-math.sin(yaw_rad), math.cos(yaw_rad), 0.0], dtype=np.float64)
        u_vert = np.array([0.0, 0.0, 1.0], dtype=np.float64)
        return [
            center_enu + hw * u_lat + hh * u_vert,  # TL
            center_enu - hw * u_lat + hh * u_vert,  # TR
            center_enu - hw * u_lat - hh * u_vert,  # BR
            center_enu + hw * u_lat - hh * u_vert   # BL
        ]

    def project_gate(self, center_enu, yaw_rad):
        corners_w = self.compute_corners(center_enu, yaw_rad)
        pix = []
        for c in corners_w:
            p_c = world_to_camera_rdf(c, self.drone_pos, self.R_wb, self.R_bc)
            if p_c[2] < 0.2:
                return None
            u = self.fx * (p_c[0] / p_c[2]) + self.cx
            v = self.fy * (p_c[1] / p_c[2]) + self.cy
            pix.append((float(u), float(v)))
        return np.array(pix, dtype=np.float32)

    def test_multi_gate_projections(self):
        """Verify that Gate 1 and Gate 2 project to distinct, valid pixel regions."""
        g1_corners = self.project_gate(self.priors[0][1], self.priors[0][2])
        g2_corners = self.project_gate(self.priors[1][1], self.priors[1][2])

        self.assertIsNotNone(g1_corners)
        self.assertIsNotNone(g2_corners)

        # Gate 1 center is roughly near image horizontal center (y=-0.41m)
        g1_cx = np.mean(g1_corners[:, 0])
        self.assertAlmostEqual(g1_cx, 320.0, delta=20.0)

        # Gate 2 is at y=-5.46m (right side of drone body in ENU), so cx should be to the right (cx > 400)
        g2_cx = np.mean(g2_corners[:, 0])
        self.assertGreater(g2_cx, 400.0)

    def test_prior_guided_association(self):
        """Simulate two YOLO detections and ensure they are assigned to the correct gates."""
        g1_corners = self.project_gate(self.priors[0][1], self.priors[0][2])
        g2_corners = self.project_gate(self.priors[1][1], self.priors[1][2])

        g1_cx, g1_cy = np.mean(g1_corners[:, 0]), np.mean(g1_corners[:, 1])
        g1_w = np.max(g1_corners[:, 0]) - np.min(g1_corners[:, 0])

        g2_cx, g2_cy = np.mean(g2_corners[:, 0]), np.mean(g2_corners[:, 1])
        g2_w = np.max(g2_corners[:, 0]) - np.min(g2_corners[:, 0])

        # Create simulated detections with minor noise (e.g. 5 px jitter)
        detections = [
            {'bbox': (g2_cx + 4.0, g2_cy - 3.0, g2_w * 1.05, g2_w * 1.05), 'score': 0.92, 'id': 'det_near_g2'},
            {'bbox': (g1_cx - 2.0, g1_cy + 3.0, g1_w * 0.98, g1_w * 0.98), 'score': 0.81, 'id': 'det_near_g1'},
        ]

        projected = {
            1: {'center': (g1_cx, g1_cy), 'bbox': (g1_cx, g1_cy, g1_w, g1_w)},
            2: {'center': (g2_cx, g2_cy), 'bbox': (g2_cx, g2_cy, g2_w, g2_w)},
        }

        # Association logic
        for det in detections:
            bbox = det['bbox']
            cx_d, cy_d, w_d = bbox[0], bbox[1], bbox[2]
            best_gid = None
            best_cost = float('inf')
            for gid, p_info in projected.items():
                cx_p, cy_p = p_info['center']
                w_p = p_info['bbox'][2]
                dist_2d = math.hypot(cx_d - cx_p, cy_d - cy_p)
                scale_ratio = min(w_d, max(1.0, w_p)) / max(w_d, max(1.0, w_p))
                cost = dist_2d + 50.0 * (1.0 - scale_ratio)
                if dist_2d < 250.0 and cost < best_cost:
                    best_cost = cost
                    best_gid = gid
            det['assigned_gate_id'] = best_gid

        # Verify correct assignment regardless of score (det_near_g2 has higher score 0.92, but must map to G2!)
        self.assertEqual(detections[0]['assigned_gate_id'], 2)
        self.assertEqual(detections[1]['assigned_gate_id'], 1)

    def test_target_gate_filtering(self):
        """Ensure that when tracking Gate 1, only Gate 1's detection is extracted."""
        active_target_gate_id = 1
        detections = [
            {'assigned_gate_id': 2, 'score': 0.95, 'name': 'Gate 2'},
            {'assigned_gate_id': 1, 'score': 0.78, 'name': 'Gate 1'},
        ]

        # Target selection
        target_det = None
        for det in detections:
            if det.get('assigned_gate_id') == active_target_gate_id:
                target_det = det
                break

        self.assertIsNotNone(target_det)
        self.assertEqual(target_det['name'], 'Gate 1')


if __name__ == '__main__':
    unittest.main()
