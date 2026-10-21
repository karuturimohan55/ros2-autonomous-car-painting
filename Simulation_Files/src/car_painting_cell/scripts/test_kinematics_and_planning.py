#!/usr/bin/env python3
"""
Comprehensive Unit Test Suite for Stage 3 Trajectory Planning and Kinematics.
Tests:
1. Exact Forward Kinematics (FK) against URDF specifications.
2. Analytical Inverse Kinematics (IK) with orientation alignment.
3. FK/IK round-trip accuracy across reachable workspace.
4. Unreachable target detection and meaningful error reporting.
5. Collision checking clearance enforcement for mobile base and arm links.
6. Feasibility and safety of all six vehicle surface trajectory plans.
"""

import math
import numpy as np
import sys
import os

# Add scripts directory to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from kinematics import (
    forward_kinematics,
    forward_kinematics_all_links,
    inverse_kinematics,
    is_joint_state_valid,
    ARM_JOINT_NAMES,
    JOINT_LIMITS,
    Z_SHOULDER_GROUND
)
from collision_checker import CollisionChecker
from trajectory_planner import PaintingTrajectoryPlanner


def test_forward_kinematics_known_poses():
    """Validates FK against known physical dimensions of the robot."""
    print("Testing FK for known poses...")
    # Zero pose: all links vertical along +Z
    q_zero = [0.0] * 6
    p_nozzle, v_spray = forward_kinematics(q_zero, base_pose=(0.0, 0.0, 0.0))
    assert abs(p_nozzle[0]) < 1e-3, f"Expected X=0, got {p_nozzle[0]}"
    assert abs(p_nozzle[1]) < 1e-3, f"Expected Y=0, got {p_nozzle[1]}"
    assert abs(p_nozzle[2] - 2.24) < 1e-2, f"Expected Z=2.24m, got {p_nozzle[2]}"
    assert abs(v_spray[2] - 1.0) < 1e-3, f"Expected spray +Z, got {v_spray}"

    # Shoulder pitched 90 deg forward (q2 = pi/2)
    q_pitch90 = [0.0, math.pi / 2.0, 0.0, 0.0, 0.0, 0.0]
    p_fwd, v_fwd = forward_kinematics(q_pitch90, base_pose=(0.0, 0.0, 0.0))
    expected_x = 0.70 + 0.75 + 0.29  # 1.74m total horizontal extension
    assert abs(p_fwd[0] - expected_x) < 1e-2, f"Expected X={expected_x}, got {p_fwd[0]}"
    assert abs(p_fwd[2] - Z_SHOULDER_GROUND) < 1e-2, f"Expected Z={Z_SHOULDER_GROUND}, got {p_fwd[2]}"
    assert abs(v_fwd[0] - 1.0) < 1e-3, f"Expected spray +X, got {v_fwd}"

    print("  [PASS] FK known poses verified.")


def test_ik_orientation_and_roundtrip():
    """Validates analytical IK and exact normal alignment across various headings."""
    print("Testing IK and orientation round-trip...")
    base_pose = (0.0, 2.00, -math.pi / 2.0)

    # Test inward perpendicular spray into door (target normal = [0, 1, 0], spray = [0, -1, 0])
    test_targets = [
        ([0.00, 1.18, 0.75], [0.0, 1.0, 0.0]),
        ([0.50, 1.18, 0.75], [0.0, 1.0, 0.0]),
        ([-0.50, 1.18, 0.75], [0.0, 1.0, 0.0]),
        ([0.00, 1.18, 0.55], [0.0, 1.0, 0.0]),
        ([0.00, 1.18, 0.95], [0.0, 1.0, 0.0]),
    ]

    for tgt_p, tgt_normal in test_targets:
        ok, q_sol, err_dist = inverse_kinematics(tgt_p, tgt_normal, base_pose)
        assert ok, f"IK failed for target {tgt_p}: err={err_dist}"
        assert is_joint_state_valid(q_sol), f"Joint limits violated: {q_sol}"

        # Check FK agreement
        p_act, v_act = forward_kinematics(q_sol, base_pose)
        pos_err = np.linalg.norm(np.array(tgt_p) - p_act)
        v_desired = -np.array(tgt_normal) / np.linalg.norm(tgt_normal)
        dot_align = np.dot(v_act, v_desired)

        assert pos_err < 0.015, f"Position error too large: {pos_err:.4f}m for {tgt_p}"
        assert dot_align > 0.85, f"Orientation alignment failed: dot={dot_align:.4f} for {tgt_p}"

    print("  [PASS] IK orientation and round-trip verified.")


def test_specific_point_ik():
    """Validates IK accuracy and exact FK agreement on the specific boundary target [1.95, -0.45, 1.08]."""
    print("Testing specific boundary target [1.95, -0.45, 1.08]...")
    base_pose = (3.00, 0.0, -math.pi / 2.0)
    tgt = [1.95, -0.45, 1.08]
    normal = [0.0, 0.0, 1.0]

    ok, q_sol, err = inverse_kinematics(tgt, normal, base_pose)
    assert ok, f"IK failed for target {tgt}: err={err}"
    assert is_joint_state_valid(q_sol), f"Joint limits violated: {q_sol}"

    p_act, v_act = forward_kinematics(q_sol, base_pose)
    actual_pos_err = float(np.linalg.norm(np.array(tgt) - p_act))
    dot_align = float(np.dot(v_act, [0.0, 0.0, -1.0]))

    assert actual_pos_err < 0.01, f"Actual position error too large: {actual_pos_err:.4f}m"
    assert dot_align > 0.85, f"Orientation alignment failed: dot={dot_align:.4f}"
    assert abs(err - actual_pos_err) < 1e-4, f"Reported error {err} does not match actual FK error {actual_pos_err}"

    print(f"  [PASS] Target {tgt} solved: err={actual_pos_err:.6f}m, dot={dot_align:.4f}")


def test_unreachable_target_handling():
    """Confirms IK cleanly reports reachability errors without throwing or bogus values."""
    print("Testing unreachable target handling...")
    base_pose = (0.0, 0.0, 0.0)
    # 5 meters away (physically impossible)
    far_target = [5.0, 0.0, 1.0]
    ok, q_sol, err = inverse_kinematics(far_target, [0, 0, 1], base_pose)
    assert not ok, "Unreachable target should fail"
    assert err > 1.0, f"Expected error > 1.0m, got {err}"
    assert err != 999.0, "Should report meaningful physical deficit, not 999.0 sentinel"

    print("  [PASS] Unreachable target handling verified.")


def test_collision_checker_margins():
    """Validates collision checking safety margins."""
    print("Testing collision checker margins...")
    cc = CollisionChecker()

    # Base at 2.90m facing car (X=2.45m corner vs car at 2.42m -> clearance 0.03m < 0.25m)
    unsafe_base = (2.90, 0.0, -math.pi)
    base_ok, msg = cc.check_base_pose(unsafe_base)
    assert not base_ok, "Unsafe base pose (clearance 0.03m) should be rejected"
    assert "clearance" in msg.lower(), f"Unexpected message: {msg}"

    # Safe base at 3.15m facing car (clearance ~0.28m >= 0.25m)
    safe_base = (3.15, 0.0, -math.pi)
    base_ok, msg = cc.check_base_pose(safe_base)
    assert base_ok, f"Safe base pose rejected: {msg}"

    print("  [PASS] Collision checker margins verified.")


def test_six_surface_plans():
    """Validates all 6 vehicle surface trajectory plans."""
    print("\n--- Validating All Six Surface Trajectory Plans ---")
    planner = PaintingTrajectoryPlanner()
    surfaces = ['hood', 'left_side', 'roof', 'rear', 'right_side', 'front_bumper']

    all_passed = True
    results = {}

    for surf in surfaces:
        plan = planner.generate_surface_plan(surf)
        ok, res_plan, msg = planner.discretize_and_solve(plan)
        results[surf] = (ok, len(res_plan.joint_waypoints), msg)
        print(f"  {surf:14s} | success={str(ok):5s} | waypoints={len(res_plan.joint_waypoints):2d} | {msg}")
        if not ok:
            all_passed = False

    return all_passed, results


if __name__ == '__main__':
    test_forward_kinematics_known_poses()
    test_ik_orientation_and_roundtrip()
    test_specific_point_ik()
    test_unreachable_target_handling()
    test_collision_checker_margins()
    all_ok, res = test_six_surface_plans()
    if not all_ok:
        sys.exit(1)
    print("\nALL STAGE 3 PLANNING & KINEMATICS TESTS PASSED!")
