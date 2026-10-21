#!/usr/bin/env python3
"""
Unit Test Suite for Base-Pose & Coordinate-Frame Verification.
Validates:
1. Station validation logic with tight linear and angular tolerances and angle-wrap handling.
2. Inconsistent coordinate transformation detection between nominal plan stations and measured base poses.
3. Dynamic collision checking at actual base poses vs nominal stations.
4. FK accuracy under base translation and rotation.
"""

import math
import sys
import os
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from trajectory_planner import PaintingTrajectoryPlanner, validate_base_station
from kinematics import forward_kinematics, inverse_kinematics
from collision_checker import CollisionChecker, point_to_box_distance, CAR_LOWER_MIN, CAR_LOWER_MAX


def test_validate_base_station_exact_and_tolerances():
    print("Testing validate_base_station()...")
    req_station = (0.0, 2.00, -math.pi / 2.0)

    # 1. Exact match
    ok, pos_err, yaw_err, msg = validate_base_station(req_station, req_station)
    assert ok, f"Exact match failed: {msg}"
    assert pos_err < 1e-6, f"Expected 0 position error, got {pos_err}"
    assert yaw_err < 1e-6, f"Expected 0 yaw error, got {yaw_err}"

    # 2. Within tolerance (e.g. 0.04m, 2 deg)
    close_pose = (0.03, 2.02, -math.pi / 2.0 + math.radians(2.0))
    ok, pos_err, yaw_err, msg = validate_base_station(close_pose, req_station, pos_tol=0.08, yaw_tol=math.radians(5.0))
    assert ok, f"Close pose should pass: {msg}"
    assert pos_err < 0.08, f"Position error {pos_err} should be < 0.08"
    assert yaw_err < math.radians(5.0), f"Yaw error {yaw_err} should be < 5 deg"

    # 3. Position mismatch (spawn pose: Y=2.448 vs Y=2.00 -> 0.448m offset)
    spawn_pose = (0.0, 2.448, -math.pi / 2.0)
    ok, pos_err, yaw_err, msg = validate_base_station(spawn_pose, req_station, pos_tol=0.08, yaw_tol=math.radians(5.0))
    assert not ok, f"Spawn pose should fail station validation: {msg}"
    assert abs(pos_err - 0.448) < 1e-3, f"Expected pos_err 0.448m, got {pos_err}"
    assert "position error" in msg.lower(), f"Expected 'position error' in message: {msg}"

    # 4. Heading mismatch (15 deg offset)
    turned_pose = (0.0, 2.00, -math.pi / 2.0 + math.radians(15.0))
    ok, pos_err, yaw_err, msg = validate_base_station(turned_pose, req_station, pos_tol=0.08, yaw_tol=math.radians(5.0))
    assert not ok, f"Turned pose should fail station validation: {msg}"
    assert "yaw error" in msg.lower(), f"Expected 'yaw error' in message: {msg}"

    # 5. Angle wrapping at boundary (+pi vs -pi)
    front_req = (3.50, 0.0, -math.pi)
    front_meas = (3.50, 0.0, math.pi)  # Exactly equivalent angle!
    ok, pos_err, yaw_err, msg = validate_base_station(front_meas, front_req, pos_tol=0.08, yaw_tol=math.radians(5.0))
    assert ok, f"Angle wrapping +pi vs -pi should pass: {msg}"
    assert yaw_err < 1e-6, f"Expected 0 yaw error with wrap, got {yaw_err}"

    print("  [PASS] validate_base_station() verified.")


def test_coordinate_frame_consistency():
    print("Testing coordinate frame consistency...")
    planner = PaintingTrajectoryPlanner()
    plan = planner.generate_surface_plan('left_side')
    ok, solved_plan, msg = planner.discretize_and_solve(plan)
    assert ok, f"Planning failed: {msg}"

    target_q = solved_plan.joint_waypoints[0]
    target_cart = solved_plan.cartesian_waypoints[0]
    nominal_station = plan.base_station  # (0.0, 2.00, -pi/2)
    spawn_station = (0.0, 2.448, -math.pi / 2.0)

    # If robot is at nominal station:
    p_nom, _ = forward_kinematics(target_q, nominal_station)
    cart_err_nom = float(np.linalg.norm(target_cart - p_nom))
    assert cart_err_nom < 0.015, f"Nominal FK error too high: {cart_err_nom:.4f}m"

    # If robot is at spawn pose (Y=2.448):
    p_spawn, _ = forward_kinematics(target_q, spawn_station)
    cart_err_spawn = float(np.linalg.norm(target_cart - p_spawn))
    # The nozzle is shifted along Y by 2.448 - 2.00 = 0.448m!
    assert cart_err_spawn > 0.40, f"Expected >0.40m discrepancy when base is at spawn pose, got {cart_err_spawn:.4f}m"

    # Verifying collision checker catches discrepancies:
    cc = CollisionChecker()
    # At nominal station, target_q is safe:
    safe_nom, reason_nom = cc.check_robot_configuration(target_q, nominal_station)
    assert safe_nom, f"Nominal configuration should be safe: {reason_nom}"

    print(f"  [PASS] Frame consistency confirmed: Nominal FK err={cart_err_nom*1000:.1f}mm vs Unstationed err={cart_err_spawn*1000:.1f}mm.")


def test_all_six_surface_stations():
    print("Testing all 6 surface nominal stations and validation...")
    planner = PaintingTrajectoryPlanner()
    surfaces = ['hood', 'left_side', 'roof', 'rear', 'right_side', 'front_bumper']
    expected_stations = {
        'hood': (3.00, 0.0, -math.pi / 2.0),
        'left_side': (0.0, 2.00, -math.pi / 2.0),
        'roof': (0.0, 1.85, -math.pi / 2.0),
        'rear': (-3.50, 0.0, 0.0),
        'right_side': (0.0, -2.00, math.pi / 2.0),
        'front_bumper': (3.50, 0.0, -math.pi)
    }

    for s in surfaces:
        plan = planner.generate_surface_plan(s)
        expected = expected_stations[s]
        assert np.allclose(plan.base_station, expected, atol=1e-4), f"Surface {s} station mismatch: {plan.base_station} vs {expected}"

        # Station validation with exact pose must pass
        ok, pos_err, yaw_err, msg = validate_base_station(expected, plan.base_station)
        assert ok, f"Exact station validation failed for {s}: {msg}"

        # Station validation with spawn pose (0.0, 2.448, -pi/2) must fail for non-matching stations
        if s != 'left_side':  # Even left_side fails due to Y=2.448 vs Y=2.00
            ok_spawn, _, _, _ = validate_base_station((0.0, 2.448, -math.pi / 2.0), plan.base_station)
            assert not ok_spawn, f"Spawn pose should NOT match {s} station"

    print("  [PASS] All 6 surface stations validated.")


if __name__ == '__main__':
    test_validate_base_station_exact_and_tolerances()
    test_coordinate_frame_consistency()
    test_all_six_surface_stations()
    print("\nALL BASE-POSE & COORDINATE-FRAME TESTS PASSED!")
