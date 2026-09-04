#!/usr/bin/env python3
"""
Comprehensive Synthetic Test Suite for Pixel-Innovation EKF.
Validates:
1. Round-trip coordinate frame transformations (World ENU <-> Body FLU <-> Camera RDF).
2. Explicit axis and sign conventions (forward, lateral left, yaw rotation, pitch tilt).
3. Innovation sign test (confirming positive perturbations push delta_x back toward ground truth).
4. Bootstrap IPPE solvePnP in World ENU coordinates.
5. SVD conditioning and observability check on H.
6. 100-trial Monte Carlo convergence and NIS distribution consistency.
7. Missing corner reduction (4 corners vs 3 corners).
8. FSM tracking state machine transitions (BOOTSTRAP -> TRACKING -> REJECTED -> RECOVERY).
"""

import math
import sys
import unittest
import numpy as np

# Allow importing pixel_innovation_ekf from sibling package directory
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from cuda_gate_inference.pixel_innovation_ekf import (
    PixelInnovationEKF,
    TrackingState,
    quat_to_rot_matrix,
    get_camera_to_body_rotation,
    world_to_camera_rdf,
    camera_rdf_to_world
)


class TestPixelInnovationEKF(unittest.TestCase):

    def setUp(self):
        self.camera_matrix = np.array([
            [500.0,   0.0, 320.0],
            [  0.0, 500.0, 240.0],
            [  0.0,   0.0,   1.0]
        ], dtype=np.float64)
        self.dist_coeffs = np.zeros(5, dtype=np.float64)
        self.gate_w = 1.9
        self.gate_h = 2.0
        self.ekf = PixelInnovationEKF(
            gate_width=self.gate_w,
            gate_height=self.gate_h,
            sigma_pixel=2.0,
            process_noise_q=1e-4,
            p0_sigma=2.0,
            camera_pitch_deg=0.0
        )

    def test_1_round_trip_transforms(self):
        """1. Verify World -> Camera -> World round-trip invertibility to machine precision."""
        drone_pos = np.array([5.2, -3.1, 1.8], dtype=np.float64)
        # 30 deg yaw, 5 deg pitch
        yaw = math.radians(30.0)
        pitch = math.radians(5.0)
        R_wb = np.array([
            [math.cos(yaw) * math.cos(pitch), -math.sin(yaw), math.cos(yaw) * math.sin(pitch)],
            [math.sin(yaw) * math.cos(pitch),  math.cos(yaw), math.sin(yaw) * math.sin(pitch)],
            [-math.sin(pitch),                             0.0,                  math.cos(pitch)]
        ], dtype=np.float64)
        R_bc = get_camera_to_body_rotation(camera_pitch_deg=12.0)
        p_cb = np.array([0.15, 0.0, -0.05], dtype=np.float64)

        test_points = [
            np.array([15.0, 2.0, 1.5]),
            np.array([25.0, -10.0, 0.5]),
            np.array([8.0, 0.0, 2.2])
        ]

        for p_w in test_points:
            p_c = world_to_camera_rdf(p_w, drone_pos, R_wb, R_bc, p_cb)
            p_w_recovered = camera_rdf_to_world(p_c, drone_pos, R_wb, R_bc, p_cb)
            err = np.linalg.norm(p_w - p_w_recovered)
            self.assertLess(err, 1e-12, f"Round-trip transform failed with error {err}")

    def test_2_axis_and_sign_conventions(self):
        """2. Explicitly test that sign conventions match physics and frame definitions."""
        drone_pos = np.array([0.0, 0.0, 0.0], dtype=np.float64)
        R_wb = np.eye(3, dtype=np.float64)  # Drone at origin, yaw=0
        R_bc = get_camera_to_body_rotation(camera_pitch_deg=0.0)

        # Gate 10m directly forward in World ENU (+X)
        gate_ahead = np.array([10.0, 0.0, 0.0])
        p_c_ahead = world_to_camera_rdf(gate_ahead, drone_pos, R_wb, R_bc)
        # RDF: +X Right, +Y Down, +Z Forward.
        self.assertAlmostEqual(p_c_ahead[0], 0.0, places=6)
        self.assertAlmostEqual(p_c_ahead[1], 0.0, places=6)
        self.assertAlmostEqual(p_c_ahead[2], 10.0, places=6)

        # Project gate center: should be exactly at optical center cx, cy
        u = 500.0 * (p_c_ahead[0] / p_c_ahead[2]) + 320.0
        v = 500.0 * (p_c_ahead[1] / p_c_ahead[2]) + 240.0
        self.assertAlmostEqual(u, 320.0, places=4)
        self.assertAlmostEqual(v, 240.0, places=4)

        # Gate shifted +1m to the left in World ENU (Body +Y)
        gate_left = np.array([10.0, 1.0, 0.0])
        p_c_left = world_to_camera_rdf(gate_left, drone_pos, R_wb, R_bc)
        # In RDF (+X Right), a point to the left must have negative X_c!
        self.assertAlmostEqual(p_c_left[0], -1.0, places=6)
        u_left = 500.0 * (p_c_left[0] / p_c_left[2]) + 320.0
        self.assertLess(u_left, 320.0, "Gate shifted left must have u < cx (shifted left on screen)")

        # Drone yaws +90 deg (facing +Y / North)
        # World gate at [10, 0, 0] is now to the drone's RIGHT (-Y body)
        yaw_90 = math.radians(90.0)
        R_wb_90 = np.array([
            [math.cos(yaw_90), -math.sin(yaw_90), 0.0],
            [math.sin(yaw_90),  math.cos(yaw_90), 0.0],
            [0.0,                           0.0,  1.0]
        ], dtype=np.float64)
        p_c_yaw90 = world_to_camera_rdf(gate_ahead, drone_pos, R_wb_90, R_bc)
        # Drone facing North, gate is East (to its right). In RDF, right is +X_c!
        self.assertAlmostEqual(p_c_yaw90[0], 10.0, places=6)
        self.assertAlmostEqual(p_c_yaw90[2], 0.0, places=6)

        # Camera pitch +10 deg (tilted up).
        # A gate at the same height directly ahead should appear LOWER in the camera frame (v > cy).
        R_bc_pitch10 = get_camera_to_body_rotation(camera_pitch_deg=10.0)
        p_c_pitch = world_to_camera_rdf(gate_ahead, drone_pos, R_wb, R_bc_pitch10)
        v_pitch = 500.0 * (p_c_pitch[1] / p_c_pitch[2]) + 240.0
        self.assertGreater(v_pitch, 240.0, "Camera pitched up means horizon and gate shift downward (higher v)")

    def test_3_innovation_sign_correction(self):
        """3. Innovation Sign Test: Perturbing prior gate forward pushes delta_x back toward ground truth."""
        drone_pos = np.array([0.0, 0.0, 0.0], dtype=np.float64)
        R_wb = np.eye(3, dtype=np.float64)

        # True gate at X=10m, Y=0m
        true_state = np.array([[10.0], [0.0]])
        z_true, ok = self.ekf.predict_gate_pixels(true_state, drone_pos, R_wb, self.camera_matrix)
        self.assertTrue(ok)

        # Perturbed filter state: +1.0m forward (X=11.0m)
        self.ekf.initialize_state(11.0, 0.0, p0_sigma=1.0)
        meas_dict = {i: (float(z_true[2*i, 0]), float(z_true[2*i+1, 0]), 1.0) for i in range(4)}

        accepted, diag = self.ekf.update(meas_dict, drone_pos, R_wb, self.camera_matrix)
        self.assertTrue(accepted)
        delta_x = diag["delta_x"][0]  # Correction along X

        # Since prior was too far (+1.0m), correction delta_x MUST BE NEGATIVE to pull it back toward 10.0m!
        self.assertLess(delta_x, 0.0, f"Innovation sign wrong! Prior was +11m (truth 10m), but delta_x={delta_x} was not negative.")
        self.assertLess(self.ekf.x[0, 0], 11.0, "Updated state must be closer to ground truth 10.0m than 11.0m")

    def test_4_bootstrap_ippe_world_conversion(self):
        """4. Verify solvePnP IPPE outputs correctly convert from camera RDF to World ENU."""
        drone_pos = np.array([2.0, 3.0, 1.0], dtype=np.float64)
        yaw = math.radians(20.0)
        R_wb = np.array([
            [math.cos(yaw), -math.sin(yaw), 0.0],
            [math.sin(yaw),  math.cos(yaw), 0.0],
            [0.0,                     0.0,  1.0]
        ], dtype=np.float64)

        # True gate center in World ENU
        true_x, true_y, true_z = 12.0, 6.0, 0.75
        self.ekf.gate_z = true_z
        self.ekf.gate_yaw = yaw

        # Project true corners to image plane
        true_state = np.array([[true_x], [true_y]])
        z_pix, ok = self.ekf.predict_gate_pixels(true_state, drone_pos, R_wb, self.camera_matrix)
        self.assertTrue(ok)

        corners_2d = z_pix.reshape(4, 2)
        success, p_gate_w = self.ekf.bootstrap_pnp(corners_2d, self.camera_matrix, self.dist_coeffs, drone_pos, R_wb)

        self.assertTrue(success)
        self.assertIsNotNone(p_gate_w)
        err = np.linalg.norm(p_gate_w[:2] - np.array([true_x, true_y]))
        self.assertLess(err, 0.05, f"Bootstrap PnP world error {err}m exceeds tolerance 0.05m")
        self.assertEqual(self.ekf.state_fsm, TrackingState.TRACKING)

    def test_5_svd_conditioning_and_observability(self):
        """5. Verify Jacobian SVD condition number and rejection of degenerate geometry."""
        drone_pos = np.array([0.0, 0.0, 0.0], dtype=np.float64)
        R_wb = np.eye(3, dtype=np.float64)
        state_xy = np.array([[10.0], [0.0]])

        # Nominal front-facing gate: H should have excellent condition number (cond < 50)
        H_nom, cond_nom = self.ekf.compute_jacobian_finite_difference(state_xy, drone_pos, R_wb, self.camera_matrix)
        self.assertIsNotNone(H_nom)
        self.assertLess(cond_nom, 50.0, f"Nominal condition number {cond_nom} should be well under 50")

        # Verify SVD singular values are positive and non-zero
        u, s, vh = np.linalg.svd(H_nom)
        self.assertGreater(s[0], 0.0)
        self.assertGreater(s[1], 0.0)
        self.assertAlmostEqual(cond_nom, s[0] / s[-1], places=5)

        # Test that setting a strict cond_h_max threshold triggers POOR_CONDITIONING rejection
        self.ekf.cond_h_max = 1.0  # Impossible threshold (condition number is always >= 1.0)
        z_nom, _ = self.ekf.predict_gate_pixels(state_xy, drone_pos, R_wb, self.camera_matrix)
        meas_dict = {i: (float(z_nom[2*i, 0]), float(z_nom[2*i+1, 0]), 1.0) for i in range(4)}
        self.ekf.initialize_state(10.0, 0.0)
        accepted, diag = self.ekf.update(meas_dict, drone_pos, R_wb, self.camera_matrix)
        self.assertFalse(accepted)
        self.assertEqual(diag["status"], "POOR_CONDITIONING")
        self.ekf.cond_h_max = 1000.0  # Reset

    def test_6_monte_carlo_convergence_and_nis(self):
        """6. 100-trial Monte Carlo test: Verify convergence to <0.1m and statistical consistency of NIS."""
        np.random.seed(42)
        true_x, true_y = 15.0, 2.0
        true_state = np.array([[true_x], [true_y]])
        drone_pos = np.array([0.0, 0.0, 0.0], dtype=np.float64)
        R_wb = np.eye(3, dtype=np.float64)

        z_clean, ok = self.ekf.predict_gate_pixels(true_state, drone_pos, R_wb, self.camera_matrix)
        self.assertTrue(ok)

        trials = 100
        steps = 15
        final_errors = []
        all_accepted_nis = []

        for trial in range(trials):
            # Prior with +1.5m X error, -1.0m Y error
            init_x = true_x + np.random.uniform(0.8, 1.5)
            init_y = true_y + np.random.uniform(-1.0, -0.5)
            self.ekf.initialize_state(init_x, init_y, p0_sigma=2.0)

            for step in range(steps):
                self.ekf.predict(dt=0.033)
                # Add Gaussian pixel noise sigma = 1.5 px
                noise = np.random.normal(0.0, 1.5, size=z_clean.shape)
                z_noisy = z_clean + noise
                meas_dict = {i: (float(z_noisy[2*i, 0]), float(z_noisy[2*i+1, 0]), 1.0) for i in range(4)}

                accepted, diag = self.ekf.update(meas_dict, drone_pos, R_wb, self.camera_matrix)
                if accepted:
                    all_accepted_nis.append(diag["nis"])

            err = np.hypot(self.ekf.x[0, 0] - true_x, self.ekf.x[1, 0] - true_y)
            final_errors.append(err)

        mean_error = np.mean(final_errors)
        self.assertLess(mean_error, 0.08, f"Mean Monte Carlo error {mean_error:.3f}m exceeds 0.08m")
        # Empirical mean NIS should be broadly around degrees of freedom (8)
        mean_nis = np.mean(all_accepted_nis)
        self.assertGreater(mean_nis, 2.0)
        self.assertLess(mean_nis, 16.0, f"Empirical mean NIS {mean_nis:.2f} is outside reasonable bounds (2.0 - 16.0)")

    def test_7_missing_corners(self):
        """7. Verify matrix dimensions with missing corners (4 corners vs 3 corners)."""
        drone_pos = np.array([0.0, 0.0, 0.0], dtype=np.float64)
        R_wb = np.eye(3, dtype=np.float64)
        true_state = np.array([[12.0], [0.5]])
        self.ekf.initialize_state(12.0, 0.5)

        z_4, _ = self.ekf.predict_gate_pixels(true_state, drone_pos, R_wb, self.camera_matrix, visible_corner_indices=[0, 1, 2, 3])
        self.assertEqual(z_4.shape, (8, 1))

        # Only 3 corners visible (corner 3 missing)
        z_3, _ = self.ekf.predict_gate_pixels(true_state, drone_pos, R_wb, self.camera_matrix, visible_corner_indices=[0, 1, 2])
        self.assertEqual(z_3.shape, (6, 1))

        meas_3 = {i: (float(z_3[2*i, 0]), float(z_3[2*i+1, 0]), 1.0) for i in range(3)}
        accepted, diag = self.ekf.update(meas_3, drone_pos, R_wb, self.camera_matrix)
        self.assertTrue(accepted)
        self.assertEqual(len(diag["delta_x"]), 2)

    def test_8_fsm_transitions(self):
        """8. Test FSM: TRACKING -> REJECTED (state unchanged) -> RECOVERY."""
        drone_pos = np.array([0.0, 0.0, 0.0], dtype=np.float64)
        R_wb = np.eye(3, dtype=np.float64)
        self.ekf.initialize_state(10.0, 0.0)
        self.assertEqual(self.ekf.state_fsm, TrackingState.TRACKING)

        # Inject massive outlier (150px offset)
        bad_meas = {i: (320.0 + 150.0, 240.0 + 150.0, 1.0) for i in range(4)}
        x_before = self.ekf.x.copy()

        accepted, diag = self.ekf.update(bad_meas, drone_pos, R_wb, self.camera_matrix)
        self.assertFalse(accepted)
        self.assertEqual(self.ekf.state_fsm, TrackingState.REJECTED)
        # State must remain unchanged during REJECTED
        np.testing.assert_array_equal(self.ekf.x, x_before)

        # 5 consecutive rejections trigger RECOVERY
        for _ in range(5):
            self.ekf.update(bad_meas, drone_pos, R_wb, self.camera_matrix)
        self.assertEqual(self.ekf.state_fsm, TrackingState.RECOVERY)


if __name__ == '__main__':
    unittest.main()
