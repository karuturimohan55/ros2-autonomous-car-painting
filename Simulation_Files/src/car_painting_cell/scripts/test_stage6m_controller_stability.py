#!/usr/bin/env python3
"""
Stage 6M - Comprehensive Controller Stability and Hardening Test Suite.
Validates:
1. Trim saturation and anti-windup clamping ([-0.045, +0.045] rad).
2. Stale feedback handling, feedback loss, trim decay, and emergency spray shutoff.
3. Target changes: immediate reset of trim on new target and decay during interpolation.
4. URDF joint limits enforcement on both target angles and published commanded positions.
5. Convergence verification and deadband stability (zero hunting when |err| < 0.003 rad).
"""

import sys
import time
import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import String, Bool
from unittest.mock import MagicMock

from arm_controller import (
    ArmTrajectoryController,
    ARM_JOINT_NAMES,
    ARM_JOINT_LIMITS,
    PRESET_POSES,
    DEFAULT_JOINT_TOLERANCES,
    DEFAULT_MAX_NORM_TOLERANCE,
    DEFAULT_JOINT_VEL_TOLERANCES,
    DEFAULT_MAX_VEL_NORM_TOLERANCE,
    DEFAULT_SUSTAINED_SETTLING_SEC,
    MAX_FEEDBACK_AGE_SEC
)


def create_mock_controller():
    """Creates an ArmTrajectoryController instance with mocked publishers."""
    controller = ArmTrajectoryController()
    controller.traj_pub = MagicMock()
    controller.published_trajs = []
    controller.traj_pub.publish = lambda msg: controller.published_trajs.append(msg)
    return controller


def test_trim_saturation_and_anti_windup():
    """Test 1: Verify integral trim strictly clamps at ±0.045 rad without windup."""
    print("--- Test 1: Trim Saturation and Anti-Windup Clamping ---")
    ctrl = create_mock_controller()
    now = time.time()
    ctrl.has_valid_joint_feedback = True
    ctrl.last_feedback_time = now
    ctrl.motion_state = 'HOLDING'
    ctrl.target_angles = [0.0, 0.30, -0.60, 0.0, 0.30, 0.0]
    
    # Simulate persistent massive downward sag (error = +0.50 rad)
    ctrl.measured_angles = [0.0, 0.30, -1.10, 0.0, 0.30, 0.0]

    for _ in range(150):
        ctrl.control_loop()

    elbow_trim = ctrl.gravity_trim[2]
    assert abs(elbow_trim - 0.045) < 1e-6, (
        f"Elbow trim must saturate exactly at +0.045 rad upper limit! Got: {elbow_trim}"
    )

    # Test reverse direction (error = -0.50 rad)
    ctrl.measured_angles = [0.0, 0.30, -0.10, 0.0, 0.30, 0.0]
    for _ in range(150):
        ctrl.control_loop()

    elbow_trim_neg = ctrl.gravity_trim[2]
    assert abs(elbow_trim_neg - (-0.045)) < 1e-6, (
        f"Elbow trim must saturate exactly at -0.045 rad lower limit! Got: {elbow_trim_neg}"
    )

    print("  [PASS] Upper and lower trim saturation enforced strictly at ±0.045 rad with zero windup.")
    ctrl.destroy_node()


def test_stale_feedback_and_loss_handling():
    """Test 2: Verify stale feedback detection, state transition to FAILED, spray shutoff, and trim decay."""
    print("--- Test 2: Stale Feedback and Feedback Loss Handling ---")
    ctrl = create_mock_controller()
    now = time.time()
    ctrl.has_valid_joint_feedback = True
    ctrl.last_feedback_time = now
    ctrl.motion_state = 'HOLDING'
    ctrl.is_spraying = True
    ctrl.target_angles = [0.0, 0.30, -0.60, 0.0, 0.30, 0.0]
    ctrl.measured_angles = [0.0, 0.30, -0.62, 0.0, 0.30, 0.0]
    ctrl.gravity_trim[2] = 0.030

    # Advance time past MAX_FEEDBACK_AGE_SEC (0.5s limit)
    ctrl.get_time_sec = lambda: now + 0.65

    ctrl.control_loop()

    assert ctrl.has_valid_joint_feedback is False, "has_valid_joint_feedback must be False on stale feedback!"
    assert ctrl.motion_state == 'FAILED', f"motion_state must be 'FAILED', got: {ctrl.motion_state}"
    assert ctrl.target_reached is False, "target_reached must be False"
    assert ctrl.is_spraying is False, "Emergency spray shutoff must trigger on feedback loss!"

    # Verify trim decays safely when feedback is lost
    initial_trim = ctrl.gravity_trim[2]
    for _ in range(20):
        ctrl.control_loop()
    decayed_trim = ctrl.gravity_trim[2]
    assert decayed_trim < initial_trim, (
        f"Trim must decay safely when feedback is lost! Initial: {initial_trim}, Decayed: {decayed_trim}"
    )

    print("  [PASS] Stale feedback safely transitions to FAILED, cuts spray, and decays trim.")
    ctrl.destroy_node()


def test_target_changes_and_dynamic_reset():
    """Test 3: Verify trim resets on new target and decays during interpolation."""
    print("--- Test 3: Target Changes and Dynamic Reset ---")
    ctrl = create_mock_controller()
    now = time.time()
    ctrl.has_valid_joint_feedback = True
    ctrl.last_feedback_time = now
    ctrl.motion_state = 'HOLDING'
    ctrl.current_angles = list(PRESET_POSES['transport'])
    ctrl.target_angles = list(PRESET_POSES['transport'])
    ctrl.measured_angles = list(PRESET_POSES['transport'])
    ctrl.gravity_trim = [0.01, 0.02, 0.03, 0.0, 0.01, 0.0]

    # Command a new target pose
    new_target = PRESET_POSES['side_paint']
    ctrl.set_target_angles(new_target, 'side_paint', duration=2.0)

    assert ctrl.motion_state == 'INTERPOLATING', f"State must be INTERPOLATING, got: {ctrl.motion_state}"
    assert all(t == 0.0 for t in ctrl.gravity_trim), (
        f"All gravity trim elements must be strictly reset to 0.0 on new target! Got: {ctrl.gravity_trim}"
    )

    # Run interpolation loop and check that trim stays decayed
    for _ in range(10):
        ctrl.control_loop()
    assert all(abs(t) < 1e-4 for t in ctrl.gravity_trim), (
        f"Trim must remain near zero during interpolation! Got: {ctrl.gravity_trim}"
    )

    print("  [PASS] Target changes reset trim immediately and interpolation maintains zero bias.")
    ctrl.destroy_node()


def test_urdf_joint_limits_enforcement():
    """Test 4: Verify target and published positions NEVER exceed URDF joint limits."""
    print("--- Test 4: URDF Joint Limits Enforcement ---")
    ctrl = create_mock_controller()
    now = time.time()
    ctrl.has_valid_joint_feedback = True
    ctrl.last_feedback_time = now

    # 1. Test commanded target beyond URDF upper limits
    excessive_target = [
        4.00,   # arm_base max is 3.14159
        2.50,   # shoulder max is 1.80
        3.00,   # elbow max is 2.50
        4.00,   # wrist_1 max is 3.14159
        2.80,   # wrist_2 max is 2.00
        4.00    # wrist_3 max is 3.14159
    ]
    ctrl.set_target_angles(excessive_target, 'test_excessive', duration=1.0)

    for i, name in enumerate(ARM_JOINT_NAMES):
        max_lim = ARM_JOINT_LIMITS[name][1]
        assert ctrl.target_angles[i] <= max_lim, (
            f"Target for {name} ({ctrl.target_angles[i]}) exceeds URDF upper limit {max_lim}!"
        )

    # 2. Test commanded target beyond URDF lower limits
    negative_target = [
        -4.00,  # arm_base min is -3.14159
        -2.50,  # shoulder min is -1.80
        -3.00,  # elbow min is -2.50
        -4.00,  # wrist_1 min is -3.14159
        -2.80,  # wrist_2 min is -2.00
        -4.00   # wrist_3 min is -3.14159
    ]
    ctrl.set_target_angles(negative_target, 'test_negative', duration=1.0)
    for i, name in enumerate(ARM_JOINT_NAMES):
        min_lim = ARM_JOINT_LIMITS[name][0]
        assert ctrl.target_angles[i] >= min_lim, (
            f"Target for {name} ({ctrl.target_angles[i]}) exceeds URDF lower limit {min_lim}!"
        )

    # 3. Test trim addition near boundary does not push command past limit
    ctrl.motion_state = 'HOLDING'
    ctrl.current_angles = [0.0, 1.79, 0.0, 0.0, 0.0, 0.0]  # Shoulder near upper limit 1.80
    ctrl.gravity_trim = [0.0, 0.045, 0.0, 0.0, 0.0, 0.0]   # 1.79 + 0.045 = 1.835 > 1.80
    ctrl.control_loop()

    published_positions = ctrl.published_trajs[-1].points[0].positions
    cmd_shoulder = published_positions[1]
    assert cmd_shoulder <= 1.80, (
        f"Published shoulder position ({cmd_shoulder}) must NOT exceed URDF limit 1.80 rad!"
    )

    print("  [PASS] Joint limits strictly enforced on both targets and published trajectory points.")
    ctrl.destroy_node()


def test_convergence_and_deadband_stability():
    """Test 5: Verify convergence detection and deadband stability (|err| < 0.003 rad)."""
    print("--- Test 5: Convergence and Deadband Stability ---")
    ctrl = create_mock_controller()
    now = time.time()
    ctrl.has_valid_joint_feedback = True
    ctrl.last_feedback_time = now
    ctrl.motion_state = 'HOLDING'
    ctrl.target_angles = [0.0, 0.30, -0.60, 0.0, 0.30, 0.0]
    
    # Case A: Converged within deadband (error = 0.001 rad < 0.003 rad)
    ctrl.measured_angles = [0.0, 0.30, -0.601, 0.0, 0.30, 0.0]
    ctrl.gravity_trim[2] = 0.0200
    initial_trim = ctrl.gravity_trim[2]

    for _ in range(30):
        ctrl.control_loop()

    final_trim = ctrl.gravity_trim[2]
    assert abs(final_trim - initial_trim) < 1e-6, (
        f"Trim must remain completely static inside deadband! Initial: {initial_trim}, Final: {final_trim}"
    )
    assert ctrl.target_reached is True, "target_reached must be True"

    # Case B: check_target_convergence evaluation
    converged, max_err, err_norm, per_joint = ctrl.check_target_convergence(ctrl.target_angles)
    assert converged is True, f"Must be converged: max_err={max_err}, norm={err_norm}"
    assert err_norm < DEFAULT_MAX_NORM_TOLERANCE, f"Norm {err_norm} must be within {DEFAULT_MAX_NORM_TOLERANCE}"

    print("  [PASS] Zero hunting inside deadband (|err| < 0.003 rad) and robust convergence verification.")
    ctrl.destroy_node()


def test_velocity_rejection_and_sustained_settling():
    """Test 6: Verify rejection of false HOLDING with non-zero dynamic velocity and genuine stable convergence."""
    print("--- Test 6: Velocity Tolerance Verification and Sustained Settling Period ---")
    ctrl = create_mock_controller()
    now = time.time()
    ctrl.has_valid_joint_feedback = True
    ctrl.last_feedback_time = now
    ctrl.target_angles = list(PRESET_POSES['transport'])
    ctrl.active_pose_name = 'transport'
    ctrl.motion_state = 'SETTLING'
    ctrl.settling_start_time = now
    ctrl.settling_converged_start_time = None

    # Case 6A: Position converged, but elbow velocity is high (-0.85 rad/s > 0.50 rad/s limit)
    ctrl.measured_angles = [0.0, 0.305, -0.609, 0.0, 0.303, 0.0]
    ctrl.measured_velocities = [0.0, 0.20, -0.85, 0.0, 0.05, 0.0]

    converged, _, _, per_joint = ctrl.check_target_convergence(ctrl.target_angles)
    assert converged is False, "check_target_convergence MUST return False when elbow velocity is -0.85 rad/s!"

    ctrl.control_loop()
    assert ctrl.motion_state == 'SETTLING', f"motion_state must remain SETTLING, got {ctrl.motion_state}"
    assert ctrl.target_reached is False, "target_reached MUST remain False when velocity exceeds tolerance!"
    print("  [PASS] Case 6A: False HOLDING strictly rejected when elbow velocity (-0.85 rad/s) exceeds tolerance.")

    # Case 6B: Position converged, but shoulder velocity is high (+0.55 rad/s > 0.30 rad/s limit)
    ctrl.measured_velocities = [0.0, 0.55, -0.40, 0.0, 0.05, 0.0]
    converged_shoulder, _, _, _ = ctrl.check_target_convergence(ctrl.target_angles)
    assert converged_shoulder is False, "check_target_convergence MUST return False when shoulder velocity is +0.55 rad/s!"

    ctrl.control_loop()
    assert ctrl.motion_state == 'SETTLING'
    assert ctrl.target_reached is False
    print("  [PASS] Case 6B: False HOLDING strictly rejected when shoulder velocity (+0.55 rad/s) exceeds tolerance.")

    # Case 6C: Premature HOLDING rejection (settling duration < sustained_settling_time)
    # Positions and velocities are within justified steady-state bounds
    ctrl.measured_angles = [0.0, 0.305, -0.609, 0.0, 0.303, 0.0]
    ctrl.measured_velocities = [0.0, 0.207, -0.404, 0.0, 0.01, 0.0]  # Exact Gazebo steady-state values

    # First cycle: convergence begins, but duration is 0.0 < 0.25s
    ctrl.control_loop()
    assert ctrl.settling_converged_start_time is not None, "settling_converged_start_time must be initialized"
    assert ctrl.motion_state == 'SETTLING', "Controller MUST NOT report HOLDING on first converged cycle!"
    assert ctrl.target_reached is False, "target_reached MUST remain False before sustained period elapses!"
    print("  [PASS] Case 6C: Premature HOLDING strictly rejected before sustained settling period elapses.")

    # Case 6D: Genuine stable convergence after sustained settling period (>= 0.25s)
    # Advance time by 0.30s (> 0.25s sustained settling requirement)
    now_settled = now + 0.30
    ctrl.get_time_sec = lambda: now_settled
    ctrl.last_feedback_time = now_settled

    ctrl.control_loop()
    assert ctrl.motion_state == 'HOLDING', f"Controller MUST transition to HOLDING after sustained settling! Got: {ctrl.motion_state}"
    assert ctrl.target_reached is True, "target_reached MUST be True after sustained settling!"
    print("  [PASS] Case 6D: Genuine stable convergence verified (HOLDING and target_reached=True after 0.30s).")

    # Case 6E: Dynamic disturbance during HOLDING drops target_reached and re-enters SETTLING
    # Simulate a sudden external force/velocity spike
    ctrl.measured_velocities = [0.0, 0.80, -1.20, 0.0, 0.50, 0.0]
    ctrl.control_loop()
    assert ctrl.target_reached is False, "target_reached MUST immediately drop to False on disturbance!"
    assert ctrl.motion_state == 'SETTLING', f"Controller MUST drop out of HOLDING to SETTLING on disturbance! Got: {ctrl.motion_state}"
    assert ctrl.settling_converged_start_time is None, "settling_converged_start_time must be reset on disturbance"
    print("  [PASS] Case 6E: Dynamic disturbance during HOLDING safely dropped target_reached and triggered re-settling.")

    # Case 6F: Live failure regression test with exact measured telemetry values
    # Simulates exact values observed in live failure:
    # shoulder pos: 0.304945 rad, vel: +0.211190 rad/s
    # elbow pos: -0.609862 rad, vel: -0.421429 rad/s
    # wrist_2 pos: 0.301394 rad, vel: -0.003434 rad/s
    live_angles = [0.0, 0.304945, -0.609862, 0.0, 0.301394, 0.0]
    live_vels = [0.0, 0.211190, -0.421429, 0.0, -0.003434, 0.0]
    ctrl.measured_angles = list(live_angles)
    ctrl.measured_velocities = list(live_vels)
    ctrl.motion_state = 'SETTLING'
    now_f = now + 1.0
    ctrl.get_time_sec = lambda: now_f
    ctrl.last_feedback_time = now_f
    ctrl.settling_start_time = now_f
    ctrl.settling_converged_start_time = None

    conv, max_e, norm_e, _ = ctrl.check_target_convergence(ctrl.target_angles)
    assert conv is True, (
        f"Live values MUST converge under justified tolerances! Max err: {max_e:.4f}, Norm: {norm_e:.4f}"
    )

    # First cycle of live values
    ctrl.control_loop()
    assert ctrl.motion_state == 'SETTLING', "Must not report HOLDING before sustained period"
    assert ctrl.settling_converged_start_time == now_f

    # After sustained settling period (now_f + 0.30s)
    ctrl.get_time_sec = lambda: now_f + 0.30
    ctrl.last_feedback_time = now_f + 0.30
    ctrl.control_loop()
    assert ctrl.motion_state == 'HOLDING', f"Controller must achieve HOLDING! Got: {ctrl.motion_state}"
    assert ctrl.target_reached is True, "Target reached MUST be True after sustained settling period!"
    print("  [PASS] Case 6F: Live simulation telemetry values verified: stable convergence achieved.")

    ctrl.destroy_node()


def main():
    print("=" * 70)
    print(" RUNNING STAGE 6M CONTROLLER STABILITY & HARDENING TEST SUITE")
    print("=" * 70)
    rclpy.init()
    try:
        test_trim_saturation_and_anti_windup()
        test_stale_feedback_and_loss_handling()
        test_target_changes_and_dynamic_reset()
        test_urdf_joint_limits_enforcement()
        test_convergence_and_deadband_stability()
        test_velocity_rejection_and_sustained_settling()
    finally:
        if rclpy.ok():
            rclpy.shutdown()

    print("=" * 70)
    print(" ALL STAGE 6M STABILITY & HARDENING TESTS PASSED SUCCESSFULLY!")
    print("=" * 70)
    return 0


if __name__ == '__main__':
    sys.exit(main())
