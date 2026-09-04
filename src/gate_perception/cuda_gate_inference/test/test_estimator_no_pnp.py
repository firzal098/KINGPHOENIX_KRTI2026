import unittest
import numpy as np
from geometry_msgs.msg import PolygonStamped, Point32
from cuda_gate_inference.gate_estimator_node import GateEstimatorNode
from cuda_gate_inference.pixel_innovation_ekf import TrackingState
import rclpy

class TestEstimatorNoPnP(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_no_pnp_bootstrap_called(self):
        node = GateEstimatorNode()
        # Active gate initialized to gate 0 (Gate 1)
        self.assertEqual(node.ekf.state_fsm, TrackingState.TRACKING)
        
        # Simulate drone pose at origin looking along +X
        node.drone_pos_w = np.array([0.0, 0.0, 0.0], dtype=np.float64)
        node.drone_pose_received = True
        node.camera_matrix = np.array([
            [500.0, 0.0, 320.0],
            [0.0, 500.0, 240.0],
            [0.0, 0.0, 1.0]
        ], dtype=np.float64)
        node.dist_coeffs = np.zeros((5, 1))

        # Send 4 corner measurements
        msg = PolygonStamped()
        for u, v in [(300.0, 220.0), (340.0, 220.0), (340.0, 260.0), (300.0, 260.0)]:
            p = Point32()
            p.x = float(u)
            p.y = float(v)
            p.z = 1.0  # conf
            msg.polygon.points.append(p)

        # Trigger corners callback
        node.corners_callback(msg)

        # Confirm EKF stays in TRACKING or REJECTED, and never invoked bootstrap
        self.assertIn(node.ekf.state_fsm, [TrackingState.TRACKING, TrackingState.REJECTED])

        node.destroy_node()

if __name__ == '__main__':
    unittest.main()
