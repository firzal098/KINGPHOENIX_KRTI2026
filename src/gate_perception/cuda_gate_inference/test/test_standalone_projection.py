import math
import numpy as np
import pytest
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from sensor_msgs.msg import CameraInfo
from cuda_gate_inference.pixel_innovation_ekf import (
    quat_to_rot_matrix,
    get_camera_to_body_rotation,
    world_to_camera_rdf
)
from cuda_gate_inference.test_gate_projection import (
    TestGateProjectionNode,
    DEFINED_PRIORS,
    WEBOTS_GATES
)
# Prevent pytest from treating the ROS 2 node class as a test suite
TestGateProjectionNode.__test__ = False


def test_gate_projection_math():
    """Verify that a gate directly in front of the camera projects to the image center."""
    # Drone at (0, 0, 1), facing East (yaw = 0, so +X is forward in ENU)
    drone_pos_w = np.array([0.0, 0.0, 1.0])
    drone_quat_w = np.array([1.0, 0.0, 0.0, 0.0])  # identity
    R_wb = quat_to_rot_matrix(drone_quat_w)

    # Level camera
    R_bc = get_camera_to_body_rotation(0.0)

    # Gate center at 10m forward, at altitude 1.0m
    gate_center = np.array([10.0, 0.0, 1.0])
    p_c = world_to_camera_rdf(gate_center, drone_pos_w, R_wb, R_bc)

    # In camera RDF:
    # X_c should be 0 (center laterally)
    # Y_c should be 0 (center vertically)
    # Z_c should be +10 (10m depth)
    assert math.isclose(p_c[0], 0.0, abs_tol=1e-5)
    assert math.isclose(p_c[1], 0.0, abs_tol=1e-5)
    assert math.isclose(p_c[2], 10.0, abs_tol=1e-5)

    # Projection with standard 640x480 K
    fx = 565.4
    fy = 565.4
    cx = 320.0
    cy = 240.0
    u = fx * (p_c[0] / p_c[2]) + cx
    v = fy * (p_c[1] / p_c[2]) + cy
    assert math.isclose(u, 320.0, abs_tol=1e-4)
    assert math.isclose(v, 240.0, abs_tol=1e-4)


def test_camera_pitch_tilt():
    """Verify 15 deg camera pitch up shifts image projection downward."""
    drone_pos_w = np.array([0.0, 0.0, 1.0])
    R_wb = np.eye(3)
    gate_center = np.array([10.0, 0.0, 1.0])

    # With 15 deg up-pitch, looking up means an object at horizontal horizon appears lower on the image (higher v)
    R_bc_15 = get_camera_to_body_rotation(15.0)
    p_c_15 = world_to_camera_rdf(gate_center, drone_pos_w, R_wb, R_bc_15)

    assert p_c_15[2] > 0.0  # Still in front
    # Y_c should be positive (downward in RDF) because camera is pointed up above the horizon
    assert p_c_15[1] > 0.0

    cy = 240.0
    fy = 565.4
    v_15 = fy * (p_c_15[1] / p_c_15[2]) + cy
    assert v_15 > 240.0  # Appears below center line on screen


def test_gate1_projection_640x640_pitch15():
    """Verify Gate 1 at ~29.3m projects to bottom of frame (v in 440..483) with 640x640 K and 15 deg pitch."""
    drone_pos_w = np.zeros(3, dtype=np.float64)
    R_wb = np.eye(3, dtype=np.float64)

    # Webots 640x640 camera matrix
    fx = 585.756
    fy = 585.756
    cx = 320.0
    cy = 320.0
    K = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float64)

    R_bc_15 = get_camera_to_body_rotation(15.0)
    gate1_center = np.array([29.33, -0.41, 0.75])
    hw, hh = 0.95, 1.0  # 1.9m W x 2.0m H

    c_tl = gate1_center + np.array([0.0, hw, hh])
    c_br = gate1_center - np.array([0.0, hw, hh])

    p_c_tl = world_to_camera_rdf(c_tl, drone_pos_w, R_wb, R_bc_15)
    p_c_br = world_to_camera_rdf(c_br, drone_pos_w, R_wb, R_bc_15)

    u_tl = fx * (p_c_tl[0] / p_c_tl[2]) + cx
    v_tl = fy * (p_c_tl[1] / p_c_tl[2]) + cy
    u_br = fx * (p_c_br[0] / p_c_br[2]) + cx
    v_br = fy * (p_c_br[1] / p_c_br[2]) + cy

    # Gate is directly ahead, slightly right in image
    assert 305.0 < u_tl < 315.0
    assert 343.0 < u_br < 353.0

    # With +15 deg pitch up, Gate 1 is rendered near the bottom of the 640-height frame
    assert 435.0 < v_tl < 445.0
    assert 477.0 < v_br < 487.0
    # Gate dimensions in pixel space
    assert 35.0 < (u_br - u_tl) < 45.0
    assert 38.0 < (v_br - v_tl) < 46.0


def test_cam_info_callback_updates_intrinsics():
    """Verify cam_info_callback properly overrides fallback values and sets topic flag."""
    import rclpy
    if not rclpy.ok():
        rclpy.init()

    node = TestGateProjectionNode()
    assert node.camera_matrix is None
    assert not node.intrinsics_from_topic

    msg = CameraInfo()
    msg.k = [585.76, 0.0, 320.0, 0.0, 585.76, 320.0, 0.0, 0.0, 1.0]
    node.cam_info_callback(msg)

    assert node.camera_matrix is not None
    assert node.intrinsics_from_topic
    assert math.isclose(node.camera_matrix[0, 0], 585.76)
    assert math.isclose(node.camera_matrix[1, 2], 320.0)

    # Test dynamic parameter update
    param = Parameter()
    param.name = "camera_pitch_deg"
    param.value.type = ParameterType.PARAMETER_DOUBLE
    param.value.double_value = -5.0

    res = node.parameters_callback([param])
    assert res.successful
    assert math.isclose(node.camera_pitch_deg, -5.0)

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
