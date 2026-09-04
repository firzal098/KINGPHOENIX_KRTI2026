import math
import numpy as np
import pytest
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
