#!/usr/bin/env python3
"""
Stage 5 Unit Test Suite: Arm Target-Arrival Verification, Tolerances, Staleness, and Timeout.
Tests:
1. Target Reached: Confirms target arrival when measured joints settle within per-joint tolerances.
2. Target NOT Reached: Confirms non-arrival when measured joints are stuck outside tolerance.
3. Settling Timeout & Emergency Spray Cutoff: Confirms transition to FAILED and immediate spray shutoff on timeout.
4. Stale Feedback Handling: Confirms rejection of commands and spray disable when feedback stops.
5. Interpolation Preservation: Confirms incoming joint telemetry does not corrupt active S-curve interpolation.
6. Spray-Cone Anchor: Confirms cone_pos remains strictly 0.0 in both spray states.
7. State Machine Convergence Guard: Confirms wait_for_arm_convergence blocks until tolerance or safely aborts.
"""

import sys
import os
import math
import numpy as np

# Isolate ROS domain to prevent interference from live simulation if running
os.environ.setdefault('ROS_DOMAIN_ID', '125')

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Bool
from trajectory_msgs.msg import JointTrajectory

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from arm_controller import (
    ArmTrajectoryController,
    ARM_JOINT_NAMES,
    PRESET_POSES,
    DEFAULT_JOINT_TOLERANCES,
    DEFAULT_MAX_NORM_TOLERANCE,
    MAX_FEEDBACK_AGE_SEC
)
from painting_state_machine import PaintingStateMachine


def make_joint_state_msg(angles_dict):
    msg = JointState()
    for k, v in angles_dict.items():
        msg.name.append(k)
        msg.position.append(float(v))
    return msg


def test_target_reached():
    print("Testing Target Reached Verification...")
    node = ArmTrajectoryController()
    target = PRESET_POSES['ready']  # [0.0, 0.30, -0.60, 0.0, 0.30, 0.0]

    # Establish valid initial feedback at home
    init_js = {n: 0.0 for n in ARM_JOINT_NAMES}
    node.joint_state_callback(make_joint_state_msg(init_js))
    assert node.has_valid_joint_feedback, "Feedback not established"

    # Command ready pose
    node.set_target_angles(target, 'ready', duration=0.5)
    assert node.motion_state == 'INTERPOLATING'
    assert not node.target_reached

    # Simulate measured angles settling close to target (within tolerance)
    settled_js = {
        'arm_base_joint': 0.001,
        'shoulder_joint': 0.315,  # +0.015 rad (within 0.05 tol)
        'elbow_joint':   -0.625,  # -0.025 rad (within 0.05 tol)
        'wrist_1_joint':  0.001,
        'wrist_2_joint':  0.304,  # +0.004 rad (within 0.04 tol)
        'wrist_3_joint':  0.000
    }
    node.joint_state_callback(make_joint_state_msg(settled_js))

    converged, max_err, err_norm, _ = node.check_target_convergence(target)
    assert converged, f"Expected convergence, got max_err={max_err}, err_norm={err_norm}"

    # Force settling state with sustained settling period satisfied and run control loop
    node.motion_state = 'SETTLING'
    now_t = node.get_time_sec()
    node.settling_start_time = now_t - 0.5
    node.settling_converged_start_time = now_t - 0.3
    node.control_loop()

    assert node.motion_state == 'HOLDING', f"Expected HOLDING, got {node.motion_state}"
    assert node.target_reached == True, "Expected target_reached to be True"
    print("  [PASS] Target reached verified successfully.")
    node.destroy_node()


def test_target_not_reached():
    print("Testing Target NOT Reached Detection...")
    node = ArmTrajectoryController()
    target = PRESET_POSES['ready']

    # Establish initial feedback
    node.joint_state_callback(make_joint_state_msg({n: 0.0 for n in ARM_JOINT_NAMES}))

    # Set measured angles far from target
    stuck_js = {
        'arm_base_joint': 0.0,
        'shoulder_joint': 0.10,  # 0.20 rad away (> 0.05 tol)
        'elbow_joint':   -0.20,  # 0.40 rad away (> 0.05 tol)
        'wrist_1_joint':  0.0,
        'wrist_2_joint':  0.10,
        'wrist_3_joint':  0.0
    }
    node.joint_state_callback(make_joint_state_msg(stuck_js))

    converged, max_err, err_norm, per_joint = node.check_target_convergence(target)
    assert not converged, "Expected target NOT reached"
    assert per_joint['shoulder_joint'] > DEFAULT_JOINT_TOLERANCES['shoulder_joint']
    assert per_joint['elbow_joint'] > DEFAULT_JOINT_TOLERANCES['elbow_joint']

    # Put in SETTLING and run control loop before timeout
    node.motion_state = 'SETTLING'
    node.settling_start_time = node.get_time_sec()
    node.control_loop()

    assert node.motion_state == 'SETTLING', "Should remain in SETTLING until timeout"
    assert node.target_reached == False, "target_reached must remain False"
    print("  [PASS] Target not reached verified successfully.")
    node.destroy_node()


def test_settling_timeout_and_spray_shutoff():
    print("Testing Settling Timeout and Emergency Spray Shutoff...")
    node = ArmTrajectoryController()
    target = PRESET_POSES['side_paint']

    # Establish feedback
    node.joint_state_callback(make_joint_state_msg({n: 0.0 for n in ARM_JOINT_NAMES}))

    # Turn on spraying
    node.is_spraying = True

    # Put in SETTLING state with expired timeout
    node.target_angles = list(target)
    node.motion_state = 'SETTLING'
    node.settling_timeout = 1.0
    node.settling_start_time = node.get_time_sec() - 1.5  # Expired

    # Run control loop with stuck measured positions
    node.control_loop()

    assert node.motion_state == 'FAILED', f"Expected FAILED, got {node.motion_state}"
    assert node.target_reached == False
    assert node.is_spraying == False, "Emergency spray shutoff must disable spraying!"
    print("  [PASS] Settling timeout and spray shutoff verified successfully.")
    node.destroy_node()


def test_stale_feedback_guard():
    print("Testing Stale Feedback Guard...")
    node = ArmTrajectoryController()

    # Initial valid feedback
    node.joint_state_callback(make_joint_state_msg({n: 0.0 for n in ARM_JOINT_NAMES}))
    assert node.has_valid_joint_feedback

    # Enable spray
    node.is_spraying = True

    # Age the feedback past MAX_FEEDBACK_AGE_SEC
    node.last_feedback_time = node.get_time_sec() - (MAX_FEEDBACK_AGE_SEC + 0.2)

    # Run control loop
    node.control_loop()

    assert not node.has_valid_joint_feedback, "Feedback must be flagged invalid when stale"
    assert node.motion_state == 'FAILED'
    assert node.is_spraying == False, "Stale feedback must force spray OFF"

    # Attempt to command new pose
    node.set_target_angles(PRESET_POSES['ready'], 'ready')
    assert node.motion_state == 'FAILED', "New command must be rejected when feedback is stale"
    print("  [PASS] Stale feedback guard verified successfully.")
    node.destroy_node()


def test_active_interpolation_preservation():
    print("Testing Active Interpolation Preservation...")
    node = ArmTrajectoryController()
    init_q = [0.0] * 6
    target_q = [0.5, 0.5, -0.5, 0.5, -0.5, 0.5]

    node.joint_state_callback(make_joint_state_msg({n: 0.0 for n in ARM_JOINT_NAMES}))
    node.set_target_angles(target_q, 'custom', duration=2.0)

    # Manually step interpolation to 50%
    node.trajectory_start_time = node.get_time_sec() - 1.0  # 1s elapsed out of 2s
    node.control_loop()

    interpolated_at_half = list(node.current_angles)
    assert 0.20 < interpolated_at_half[0] < 0.30, f"Expected midpoint ~0.25, got {interpolated_at_half[0]}"

    # Send a new joint state telemetry packet (say, robot lagging at 0.10)
    lagging_js = {n: 0.10 for n in ARM_JOINT_NAMES}
    node.joint_state_callback(make_joint_state_msg(lagging_js))

    # Verify current_angles was NOT overwritten by measured angles
    assert node.measured_angles[0] == 0.10
    assert abs(node.current_angles[0] - interpolated_at_half[0]) < 1e-6, "current_angles corrupted by joint_state_callback!"

    print("  [PASS] Active interpolation preservation verified successfully.")
    node.destroy_node()


def test_spray_cone_anchor():
    print("Testing Spray Cone Joint Fixed at 0.0...")
    node = ArmTrajectoryController()
    node.joint_state_callback(make_joint_state_msg({n: 0.0 for n in ARM_JOINT_NAMES}))

    captured_trajs = []
    node.traj_pub.publish = lambda msg: captured_trajs.append(msg)

    # Test spray OFF
    node.is_spraying = False
    node.control_loop()
    assert captured_trajs[-1].points[0].positions[-1] == 0.0, "cone_pos must be 0.0 when spray OFF"

    # Test spray ON
    node.is_spraying = True
    node.control_loop()
    assert captured_trajs[-1].points[0].positions[-1] == 0.0, "cone_pos must be 0.0 when spray ON"

    print("  [PASS] Spray cone anchor verified successfully.")
    node.destroy_node()


def test_state_machine_convergence():
    print("Testing State Machine wait_for_arm_convergence...")
    sm = PaintingStateMachine()
    target = [0.0, 0.30, -0.60, 0.0, 0.30, 0.0]

    # 1. Test successful convergence
    converged_js = make_joint_state_msg({
        'arm_base_joint': 0.000,
        'shoulder_joint': 0.310,
        'elbow_joint':   -0.615,
        'wrist_1_joint':  0.000,
        'wrist_2_joint':  0.302,
        'wrist_3_joint':  0.000
    })
    sm.joint_cb(converged_js)
    ok, err_norm = sm.wait_for_arm_convergence(target, timeout=0.5)
    assert ok, f"Expected convergence, got {ok} with err {err_norm}"

    # 2. Test timeout when joints are stuck
    stuck_js = make_joint_state_msg({
        'arm_base_joint': 0.0,
        'shoulder_joint': 0.0,
        'elbow_joint':    0.0,
        'wrist_1_joint':  0.0,
        'wrist_2_joint':  0.0,
        'wrist_3_joint':  0.0
    })
    sm.joint_cb(stuck_js)
    sm.is_spraying = True
    ok, err_norm = sm.wait_for_arm_convergence(target, timeout=0.1)
    assert not ok, "Expected timeout failure"
    assert err_norm > 0.5, f"Expected large error norm, got {err_norm}"

    print("  [PASS] State machine convergence checks verified successfully.")
    sm.destroy_node()


def run_all_tests():
    print("=" * 70)
    print("RUNNING STAGE 5 UNIT TESTS (ARRIVAL VERIFICATION & SAFETY)")
    print("=" * 70)
    rclpy.init()
    try:
        test_target_reached()
        test_target_not_reached()
        test_settling_timeout_and_spray_shutoff()
        test_stale_feedback_guard()
        test_active_interpolation_preservation()
        test_spray_cone_anchor()
        test_state_machine_convergence()
        print("=" * 70)
        print("ALL 7 STAGE 5 UNIT TESTS PASSED SUCCESSFULLY!")
        print("=" * 70)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    run_all_tests()
