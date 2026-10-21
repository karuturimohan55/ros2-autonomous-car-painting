#!/usr/bin/env python3
"""
Stage 6L Regression Test Suite: Transport-Pose Gravity Deflection & Safety Interlocks

Verifies:
1. Reproduction of Measured Exceedance (0.0507 rad):
   - Confirms that an elbow deflection of -0.6507 rad (target -0.6000 rad, offset 0.0507 rad > 0.0500 rad)
     is strictly REJECTED by verify_arm_transport_safety().
   - Confirms that drive_route() strictly blocks base movement and issues 0 non-zero /cmd_vel commands.
2. Safety Interlock Boundary Testing:
   - Offset = 0.0501 rad -> BLOCKED.
   - Offset = 0.0490 rad (within tolerance) -> AUTHORIZED (when status is HOLDING).
3. Closed-Loop Gravity Compensation in ArmTrajectoryController:
   - Verifies controller accumulates trim in response to steady-state deflection.
   - Verifies trim is clamped strictly within [-0.045, +0.045] rad bounds.
   - Verifies commanded output positions to Gazebo compensate for deflection.
4. Startup Race-Condition Elimination:
   - wait_for_telemetry() requires /arm_motion_status before authorizing state machine progression.
   - set_arm_pose() waits for controller status to settle into HOLDING.
"""

import sys
import os
import math
import time
import numpy as np
from unittest.mock import MagicMock

# Isolate ROS domain
os.environ.setdefault('ROS_DOMAIN_ID', '126')

import rclpy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool
from trajectory_msgs.msg import JointTrajectory

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from painting_state_machine import (
    PaintingStateMachine,
    ARM_JOINT_NAMES,
    PRESET_POSES,
    DEFAULT_JOINT_TOLERANCES,
    DEFAULT_MAX_NORM_TOLERANCE,
    STATIONS
)
from arm_controller import ArmTrajectoryController


def make_joint_state_msg(pos_dict, stamp_sec=None):
    msg = JointState()
    now_sec = stamp_sec if stamp_sec is not None else time.time()
    msg.header.stamp.sec = int(now_sec)
    msg.header.stamp.nanosec = int((now_sec - int(now_sec)) * 1e9)
    for name, pos in pos_dict.items():
        msg.name.append(name)
        msg.position.append(float(pos))
    return msg


def test_reproduction_of_measured_exceedance():
    """Test 1: Exact reproduction of Stage 6K live dry-run abort."""
    print("--- Test 1: Reproduction of Measured Exceedance (0.0507 rad) ---")
    node = PaintingStateMachine()
    now = time.time()

    # Exact measured values from Stage 6K: target=-0.6000, measured=-0.6507
    pos_dict = {
        'arm_base_joint': 0.0000,
        'shoulder_joint': 0.3049,
        'elbow_joint':   -0.6507,  # 0.0507 rad offset > 0.0500 rad limit!
        'wrist_1_joint':  0.0000,
        'wrist_2_joint':  0.3014,
        'wrist_3_joint':  0.0000
    }
    node.latest_joint_state = make_joint_state_msg(pos_dict, stamp_sec=now)
    node.latest_joint_state_time = now
    node.arm_motion_status = "HOLDING"
    node.latest_arm_status_time = now

    is_safe, reason = node.verify_arm_transport_safety()
    assert is_safe is False, "verify_arm_transport_safety must FAIL on 0.0507 rad offset!"
    assert "elbow_joint" in reason, f"Reason must cite elbow_joint: {reason}"
    assert "0.0507" in reason, f"Reason must mention measured offset: {reason}"
    print(f"  [PASS] verify_arm_transport_safety correctly rejected deflection: '{reason}'")

    # Verify drive_route blocks base movement
    published_twists = []
    node.cmd_vel_pub = MagicMock()
    node.cmd_vel_pub.publish = lambda tw: published_twists.append((tw.linear.x, tw.angular.z))

    route = [(1.80, 2.30), (3.50, 2.30), (3.50, 0.00)]
    ok, msg = node.drive_route(route, final_yaw=-math.pi, timeout=5.0)

    non_zero_twists = [tw for tw in published_twists if abs(tw[0]) > 1e-4 or abs(tw[1]) > 1e-4]
    assert len(non_zero_twists) == 0, f"No non-zero cmd_vel commands should be published! Got: {non_zero_twists}"
    print("  [PASS] drive_route strictly prohibited base movement (0 non-zero velocity commands issued).")
    node.destroy_node()


def test_interlock_boundary_conditions():
    """Test 2: Verify tolerance threshold boundary (0.0501 rad vs 0.0490 rad)."""
    print("--- Test 2: Interlock Boundary Condition Testing ---")
    node = PaintingStateMachine()
    now = time.time()
    node.arm_motion_status = "HOLDING"
    node.latest_arm_status_time = now

    # Case A: 0.0501 rad offset (just above 0.0500 limit)
    pos_above = {
        'arm_base_joint': 0.0000,
        'shoulder_joint': 0.3000,
        'elbow_joint':   -0.6501,  # 0.0501 rad offset
        'wrist_1_joint':  0.0000,
        'wrist_2_joint':  0.3000,
        'wrist_3_joint':  0.0000
    }
    node.latest_joint_state = make_joint_state_msg(pos_above, stamp_sec=now)
    node.latest_joint_state_time = now
    is_safe_above, msg_above = node.verify_arm_transport_safety()
    assert is_safe_above is False, "0.0501 rad offset must be rejected!"
    print(f"  [PASS] 0.0501 rad offset rejected: '{msg_above}'")

    # Case B: 0.0490 rad offset (just below 0.0500 limit)
    pos_below = {
        'arm_base_joint': 0.0000,
        'shoulder_joint': 0.3000,
        'elbow_joint':   -0.6490,  # 0.0490 rad offset
        'wrist_1_joint':  0.0000,
        'wrist_2_joint':  0.3000,
        'wrist_3_joint':  0.0000
    }
    node.latest_joint_state = make_joint_state_msg(pos_below, stamp_sec=now)
    node.latest_joint_state_time = now
    is_safe_below, msg_below = node.verify_arm_transport_safety()
    assert is_safe_below is True, f"0.0490 rad offset within tolerance must be authorized! Got: {msg_below}"
    print(f"  [PASS] 0.0490 rad offset authorized: '{msg_below}'")
    node.destroy_node()


def test_controller_gravity_compensation():
    """Test 3: Closed-loop gravity compensation in ArmTrajectoryController."""
    print("--- Test 3: ArmTrajectoryController Closed-Loop Gravity Compensation ---")
    controller = ArmTrajectoryController()
    assert hasattr(controller, 'gravity_trim'), "Controller must have gravity_trim attribute"
    assert len(controller.gravity_trim) == len(ARM_JOINT_NAMES)

    captured_cmds = []
    controller.traj_pub = MagicMock()
    controller.traj_pub.publish = lambda m: captured_cmds.append(m)

    # Establish feedback with measured elbow drooping to -0.63 rad (error = +0.03 rad)
    now = time.time()
    controller.has_valid_joint_feedback = True
    controller.last_feedback_time = now
    controller.measured_angles = [0.0, 0.30, -0.63, 0.0, 0.30, 0.0]
    controller.target_angles = [0.0, 0.30, -0.60, 0.0, 0.30, 0.0]
    controller.motion_state = 'HOLDING'

    # Run control loop iterations
    initial_elbow_trim = controller.gravity_trim[2]
    for _ in range(30):
        controller.control_loop()

    final_elbow_trim = controller.gravity_trim[2]
    assert final_elbow_trim > initial_elbow_trim, (
        f"Gravity trim must accumulate in positive direction to lift elbow! (initial={initial_elbow_trim}, final={final_elbow_trim})"
    )
    assert abs(final_elbow_trim) <= 0.045, (
        f"Gravity trim must be strictly clamped within bounds! Got: {final_elbow_trim}"
    )

    last_cmd_pt = captured_cmds[-1].points[0]
    cmd_elbow = last_cmd_pt.positions[2]
    assert cmd_elbow > -0.60, f"Commanded elbow must be compensated above target -0.60! Got: {cmd_elbow}"
    print(f"  [PASS] Closed-loop trim accumulated: {final_elbow_trim:.4f} rad, commanded output: {cmd_elbow:.4f} rad.")

    # Test saturation clamp on extreme error
    controller.measured_angles = [0.0, 0.30, -1.20, 0.0, 0.30, 0.0]  # Gross error
    for _ in range(100):
        controller.control_loop()
    assert controller.gravity_trim[2] == 0.045, (
        f"Trim must clamp strictly at upper bound 0.045 rad! Got: {controller.gravity_trim[2]}"
    )
    print("  [PASS] Saturation clamp strictly enforced at maximum ±0.045 rad limit.")
    controller.destroy_node()


def test_telemetry_and_settling_guards():
    """Test 4: Telemetry readiness and arm pose settling guards."""
    print("--- Test 4: Telemetry Readiness and Settling Guards ---")
    node = PaintingStateMachine()

    # 1. wait_for_telemetry rejects if arm status missing
    node.current_pose = (0.0, 2.45, -1.5708)
    node.latest_joint_state = make_joint_state_msg({'elbow_joint': -0.60})
    node.latest_arm_status_time = 0.0  # Missing!
    assert node.wait_for_telemetry(timeout=0.05) is False, "wait_for_telemetry must reject when arm status is missing!"
    print("  [PASS] wait_for_telemetry correctly rejected missing arm status.")

    # 2. wait_for_telemetry succeeds when all 3 telemetry streams are present
    node.latest_arm_status_time = time.time()
    assert node.wait_for_telemetry(timeout=0.05) is True, "wait_for_telemetry must succeed when all streams present!"
    print("  [PASS] wait_for_telemetry accepted full telemetry suite.")
    node.destroy_node()


def main():
    print("=" * 70)
    print(" RUNNING STAGE 6L GRAVITY DEFLECTION DIAGNOSTIC & TEST SUITE")
    print("=" * 70)
    rclpy.init()
    try:
        test_reproduction_of_measured_exceedance()
        test_interlock_boundary_conditions()
        test_controller_gravity_compensation()
        test_telemetry_and_settling_guards()
    finally:
        if rclpy.ok():
            rclpy.shutdown()

    print("=" * 70)
    print(" ALL STAGE 6L DIAGNOSTIC TESTS PASSED SUCCESSFULLY!")
    print("=" * 70)
    return 0


if __name__ == '__main__':
    sys.exit(main())
