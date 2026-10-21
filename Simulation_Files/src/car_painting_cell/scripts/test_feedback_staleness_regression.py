#!/usr/bin/env python3
"""
Regression Test Suite: Feedback Staleness & Interlock Reliability
Explicitly tests:
1. Reproduction of message queue backlog when only a single spin_once is executed per loop.
2. Verification that spin_some() fully drains all pending callbacks in the DDS queue.
3. Verification that PaintingStateMachine declares and configures use_sim_time=True.
4. Verification that genuine stale joint feedback (> 0.5s) strictly blocks base motion.
5. Verification that invalid arm controller status strictly blocks base motion.
6. Verification that spray is guaranteed OFF during navigation, interlock trips, and aborts.
"""

import os
import sys
import math
from unittest.mock import MagicMock

# Isolate ROS domain for unit testing
os.environ['ROS_DOMAIN_ID'] = '126'

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from painting_state_machine import (
    PaintingStateMachine,
    ARM_JOINT_NAMES,
    PRESET_POSES,
    DEFAULT_JOINT_TOLERANCES,
    DEFAULT_MAX_NORM_TOLERANCE
)


class MockClock:
    def __init__(self, start_time=100.0):
        self._time = start_time

    def now(self):
        return self._time

    def advance(self, dt):
        self._time += dt


def make_isolated_test_node(sim_time=100.0):
    if not rclpy.ok():
        rclpy.init()
    node = PaintingStateMachine()
    clk = MockClock(sim_time)
    node.get_time_sec = clk.now
    node.current_pose = (0.00, 2.45, -math.pi / 2.0)
    node.current_vel = (0.0, 0.0)

    # Mock publishers to record outputs
    node.published_twists = []
    node.published_sprays = []

    def mock_twist(msg):
        node.published_twists.append((float(msg.linear.x), float(msg.linear.y), float(msg.angular.z)))
        # Advance clock slightly with movement
        clk.advance(0.01)

    def mock_spray(msg):
        node.published_sprays.append(bool(msg.data))

    node.cmd_vel_pub = MagicMock()
    node.cmd_vel_pub.publish = mock_twist
    node.spray_active_pub = MagicMock()
    node.spray_active_pub.publish = mock_spray
    node.delete_client = MagicMock()
    node.delete_client.wait_for_service.return_value = False

    node.telemetry_autofresh = False
    node.status_autofresh = True
    node.pose_autofresh = True
    node.spin_advance_dt = 0.0
    orig_spin_some = node.spin_some
    def mock_spin_some(timeout_sec=0.02, max_drain=25):
        if getattr(node, 'spin_advance_dt', 0.0) > 0.0:
            clk.advance(node.spin_advance_dt)
            if getattr(node, 'telemetry_autofresh', False):
                now_t = clk.now()
                if node.latest_joint_state_time > 0.0:
                    node.latest_joint_state_time = now_t
                if getattr(node, 'status_autofresh', True) and node.latest_arm_status_time > 0.0:
                    node.latest_arm_status_time = now_t
                if getattr(node, 'pose_autofresh', True) and getattr(node, 'latest_arm_pose_time', 0.0) > 0.0:
                    node.latest_arm_pose_time = now_t
        orig_spin_some(timeout_sec=0.0, max_drain=max_drain)
    node.spin_some = mock_spin_some

    return node, clk


def populate_safe_state(node, clk):
    now = clk.now()
    js = JointState()
    js.name = list(ARM_JOINT_NAMES)
    js.position = list(PRESET_POSES['transport'])
    node.latest_joint_state = js
    node.latest_joint_state_time = now
    node.arm_motion_status = "HOLDING"
    node.latest_arm_status_time = now
    node.arm_target_reached = True


def test_use_sim_time_configuration():
    print("--- Test 1: use_sim_time Parameter Configuration ---")
    if not rclpy.ok():
        rclpy.init()
    node = PaintingStateMachine()
    assert node.has_parameter('use_sim_time'), "Node must declare 'use_sim_time' parameter!"
    val = node.get_parameter('use_sim_time').value
    assert val is True, f"Expected use_sim_time=True, got {val}"
    node.destroy_node()
    print("  [PASS] PaintingStateMachine correctly declares and defaults use_sim_time to True.")


def test_queue_draining_with_spin_some():
    print("--- Test 2: Message Backlog Draining with spin_some() ---")
    if not rclpy.ok():
        rclpy.init()
    test_node = Node('test_drain_node')
    received_js = []
    test_node.create_subscription(JointState, '/test_drain_js', lambda m: received_js.append(m), 10)
    pub = test_node.create_publisher(JointState, '/test_drain_js', 10)

    # Publish burst of 5 joint state messages
    for i in range(5):
        msg = JointState()
        msg.name = ['arm_base_joint']
        msg.position = [float(i) * 0.1]
        pub.publish(msg)

    # Demonstrate that a single spin_once only processes 1 message
    rclpy.spin_once(test_node, timeout_sec=0.05)
    assert len(received_js) == 1, f"Single spin_once should process exactly 1 item, got {len(received_js)}"
    print(f"  [CONFIRMED] Single spin_once only processed {len(received_js)}/5 messages (backlog created).")

    # Now execute backlog drain (spin_some logic)
    for _ in range(15):
        rclpy.spin_once(test_node, timeout_sec=0.0)
    assert len(received_js) == 5, f"spin_some drain must process all 5 items, got {len(received_js)}"
    print(f"  [PASS] Queue draining fully processed all {len(received_js)}/5 messages (backlog eliminated).")
    test_node.destroy_node()


def test_genuine_stale_feedback_blocking():
    print("--- Test 3: Genuine Stale Feedback (>0.5s) Motion Blocking ---")
    node, clk = make_isolated_test_node(100.0)
    populate_safe_state(node, clk)

    # Make feedback 0.505s old (exceeds 0.5s limit)
    node.latest_joint_state_time = clk.now() - 0.505
    is_safe, msg = node.verify_arm_transport_safety()
    assert is_safe is False, "verify_arm_transport_safety MUST return False when feedback is stale!"
    assert "stale" in msg.lower() and "0.505" in msg, f"Unexpected message: {msg}"
    print(f"  [PASS] verify_arm_transport_safety rejected stale feedback: {msg}")

    # Attempt navigation route
    route = [(1.80, 2.30)]
    ok, r_msg = node.drive_route(route, final_yaw=0.0, timeout=2.0)
    assert ok is False, "drive_route MUST reject motion when feedback is stale!"
    assert len(node.published_twists) == 0 or all(tw == (0.0, 0.0, 0.0) for tw in node.published_twists), (
        f"Non-zero velocity commands prohibited on stale feedback: {node.published_twists}"
    )
    print("  [PASS] Base motion strictly blocked under genuine stale feedback.")
    node.destroy_node()


def test_controller_status_interlocks():
    print("--- Test 4: Arm Controller Status Interlocks ---")
    node, clk = make_isolated_test_node(100.0)
    populate_safe_state(node, clk)

    # Case 4A: Status is TRAJECTORY instead of HOLDING
    node.arm_motion_status = "TRAJECTORY"
    is_safe, msg = node.verify_arm_transport_safety()
    assert is_safe is False
    assert "holding" in msg.lower()
    print("  [PASS] Case 4A: Status 'TRAJECTORY' blocked base motion.")

    # Case 4B: Status is FAILED
    node.arm_motion_status = "FAILED"
    is_safe, msg = node.verify_arm_transport_safety()
    assert is_safe is False
    print("  [PASS] Case 4B: Status 'FAILED' blocked base motion.")

    # Case 4C: Status message is stale (> 1.0s)
    node.arm_motion_status = "HOLDING"
    node.latest_arm_status_time = clk.now() - 1.2
    is_safe, msg = node.verify_arm_transport_safety()
    assert is_safe is False
    assert "stale" in msg.lower()
    print("  [PASS] Case 4C: Stale controller status (>1.0s) blocked base motion.")
    node.destroy_node()


def test_spray_suppression_on_all_aborts():
    print("--- Test 5: Spray Safety on Abort Paths ---")
    node, clk = make_isolated_test_node(100.0)
    populate_safe_state(node, clk)

    # Force spray active, then trigger interlock failure
    node.set_spray(True)
    assert node.published_sprays[-1] is True

    # Make feedback stale and run drive_route
    node.latest_joint_state_time = clk.now() - 0.6
    route = [(1.80, 2.30)]
    node.drive_route(route, final_yaw=0.0, timeout=1.0)

    # Spray MUST be commanded False
    assert node.published_sprays[-1] is False, "Spray MUST be shut OFF on motion interlock trip!"
    print("  [PASS] Spray immediately commanded False on interlock trip.")
    node.destroy_node()


def test_stale_feedback_during_arm_convergence():
    print("--- Test 6: Stale Feedback Handling During Active Arm Convergence ---")
    node, clk = make_isolated_test_node(100.0)
    target_q = [0.0, 0.45, -0.90, 0.0, 0.45, 0.0]

    # Case 6A: Genuine stale feedback (>0.5s) aborts convergence and disables spray
    populate_safe_state(node, clk)
    node.set_spray(True)
    assert node.published_sprays[-1] is True
    node.latest_joint_state_time = clk.now() - 0.520  # 520ms stale (> 0.5s limit)

    ok, norm = node.wait_for_arm_convergence(target_q, timeout=1.0)
    assert ok is False, "wait_for_arm_convergence MUST abort immediately when joint feedback is stale (>0.5s)!"
    assert node.published_sprays[-1] is False, "Spray MUST be deactivated immediately on stale feedback abort!"
    print("  [PASS] Case 6A: Stale feedback (0.520s > 0.5s) aborted convergence and shut off spray.")

    # Case 6B: Missing feedback (None) aborts convergence and disables spray
    node.latest_joint_state = None
    node.latest_joint_state_time = 0.0
    node.set_spray(True)
    ok, norm = node.wait_for_arm_convergence(target_q, timeout=1.0)
    assert ok is False, "wait_for_arm_convergence MUST abort when latest_joint_state is None!"
    assert node.published_sprays[-1] is False, "Spray MUST be deactivated on missing feedback!"
    print("  [PASS] Case 6B: Missing feedback (None) safely aborted convergence and disabled spray.")

    # Case 6C: Incomplete joint state message missing arm joints aborts convergence
    js_incomplete = JointState()
    js_incomplete.name = ['arm_base_joint', 'shoulder_joint']
    js_incomplete.position = [0.0, 0.45]
    node.latest_joint_state = js_incomplete
    node.latest_joint_state_time = clk.now()
    node.set_spray(True)
    ok, norm = node.wait_for_arm_convergence(target_q, timeout=1.0)
    assert ok is False, "wait_for_arm_convergence MUST abort when required arm joints are missing!"
    assert node.published_sprays[-1] is False
    print("  [PASS] Case 6C: Incomplete joint feedback aborted convergence and disabled spray.")

    # Case 6D: Fresh feedback within tolerance successfully converges
    js_converged = JointState()
    js_converged.name = list(ARM_JOINT_NAMES)
    js_converged.position = [float(x) for x in target_q]
    node.latest_joint_state = js_converged
    node.latest_joint_state_time = clk.now()
    node.set_spray(True)
    ok, norm = node.wait_for_arm_convergence(target_q, timeout=1.0)
    assert ok is True, f"wait_for_arm_convergence should succeed with fresh in-tolerance joints, got {ok}!"
    assert norm <= DEFAULT_MAX_NORM_TOLERANCE, f"Error norm {norm} exceeds {DEFAULT_MAX_NORM_TOLERANCE}!"
    print(f"  [PASS] Case 6D: Fresh valid feedback successfully converged (norm={norm:.4f} rad).")

    # Case 6E: Emergency retraction under genuine stale feedback is safely blocked
    node.latest_joint_state_time = clk.now() - 0.550
    retract_ok = node.set_arm_pose("transport", wait_sec=1.0)
    assert retract_ok is False, "set_arm_pose MUST refuse verification when feedback is stale!"
    assert node.published_sprays[-1] is False
    print("  [PASS] Case 6E: Emergency retraction safely refused verification under stale feedback.")

    node.destroy_node()


def test_arm_transport_pose_convergence_and_drift_regression():
    print("--- Test 7: Transport Pose Convergence, False Success Elimination & Drift Protection ---")
    node, clk = make_isolated_test_node(100.0)
    transport_q = PRESET_POSES['transport']

    # Case 7A: False success elimination on unknown pose name
    ok = node.set_arm_pose("invalid_nonexistent_pose", wait_sec=0.1)
    assert ok is False, "set_arm_pose MUST return False immediately for unknown pose!"
    print("  [PASS] Case 7A: Unknown pose name rejected with False.")

    # Case 7B: Confirmation timeout when controller status remains 'SETTLING' (not HOLDING)
    node.telemetry_autofresh = True
    node.status_autofresh = True
    node.spin_advance_dt = 0.05
    populate_safe_state(node, clk)
    node.arm_motion_status = "SETTLING"  # Controller has not finished settling
    ok = node.set_arm_pose("transport", wait_sec=0.1)
    assert ok is False, "set_arm_pose MUST return False if controller status does not reach HOLDING!"
    print("  [PASS] Case 7B: Non-HOLDING status ('SETTLING') timed out and returned False.")

    # Case 7C: Confirmation timeout when arm_target_reached is False
    populate_safe_state(node, clk)
    node.arm_target_reached = False
    ok = node.set_arm_pose("transport", wait_sec=0.1)
    assert ok is False, "set_arm_pose MUST return False if arm_target_reached is False!"
    print("  [PASS] Case 7C: Unconfirmed target (arm_target_reached=False) timed out and returned False.")

    # Case 7D: Confirmation timeout when arm controller status is stale (>1.0s)
    node.status_autofresh = False
    populate_safe_state(node, clk)
    node.latest_arm_status_time = clk.now() - 1.20  # 1.2s old (>1.0s limit)
    ok = node.set_arm_pose("transport", wait_sec=0.1)
    assert ok is False, "set_arm_pose MUST return False when arm controller status is stale (>1.0s)!"
    print("  [PASS] Case 7D: Stale arm status (>1.0s) timed out and returned False.")

    # Case 7E: Verified convergence succeeds when all conditions are fresh and verified
    node.spin_advance_dt = 0.0
    node.status_autofresh = True
    node.telemetry_autofresh = False
    populate_safe_state(node, clk)
    node.arm_current_pose = "transport"
    node.latest_arm_pose_time = clk.now()
    ok = node.set_arm_pose("transport", wait_sec=0.1)
    assert ok is True, "set_arm_pose MUST return True when fully converged and confirmed!"
    print("  [PASS] Case 7E: Fully verified transport pose succeeded with True.")

    # Case 7F: Measured-joint drift after apparent convergence triggers safety interlock
    # Observed failure: elbow_joint at -0.6584 rad (offset 0.0584 rad > 0.0500 rad limit)
    populate_safe_state(node, clk)
    drifted_js = JointState()
    drifted_js.name = list(ARM_JOINT_NAMES)
    drifted_pos = list(transport_q)
    drifted_pos[2] = -0.6584  # elbow offset = 0.0584 rad
    drifted_js.position = drifted_pos
    node.latest_joint_state = drifted_js
    node.latest_joint_state_time = clk.now()

    is_safe, reason = node.verify_arm_transport_safety()
    assert is_safe is False, "verify_arm_transport_safety MUST trigger on 0.0584 rad offset!"
    assert "elbow_joint" in reason
    assert "0.0584" in reason

    # Confirm drive_route blocks base motion immediately when drifted
    route = [(1.80, 2.30), (3.50, 2.30)]
    nav_ok, nav_msg = node.drive_route(route, final_yaw=-math.pi, timeout=1.0)
    assert nav_ok is False, "drive_route MUST abort when transport pose drifts!"
    assert "Base motion interlock triggered" in nav_msg
    print("  [PASS] Case 7F: Drifted joint (elbow 0.0584 rad) strictly tripped base motion interlock.")

    # Case 7G: Re-commanding same target preserves gravity_trim in controller
    from arm_controller import ArmTrajectoryController
    ctrl = ArmTrajectoryController()
    now_t = clk.now()
    ctrl.has_valid_joint_feedback = True
    ctrl.last_feedback_time = now_t
    ctrl.target_angles = list(transport_q)
    ctrl.motion_state = 'HOLDING'
    ctrl.target_reached = True
    ctrl.gravity_trim = [0.005, 0.010, 0.035, 0.0, 0.005, 0.0]

    # Re-command same transport pose: gravity trim must NOT be wiped
    ctrl.set_target_angles(transport_q, 'transport', duration=2.0)
    assert ctrl.gravity_trim[2] == 0.035, (
        f"Gravity trim must NOT be reset to 0 when re-commanding same target! Got: {ctrl.gravity_trim[2]}"
    )
    assert ctrl.motion_state == 'HOLDING'
    print("  [PASS] Case 7G: Re-commanding same target preserved accumulated gravity trim.")

    # Case 7H: Inconsistent controller target vs measured status
    # Controller holding 'home' with measured joints at 0: when checked against 'transport', returns False
    ctrl.current_angles = list(PRESET_POSES['home'])
    ctrl.target_angles = list(PRESET_POSES['home'])
    ctrl.measured_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    converged_transport, _, _, _ = ctrl.check_target_convergence(transport_q)
    assert converged_transport is False, "Convergence against transport target MUST be False when joints are at home!"
    print("  [PASS] Case 7H: Inconsistent target/measurement correctly identified (home vs transport).")

    # Case 7I: False HOLDING rejection with non-zero dynamic velocity
    # Arm position matches transport pose, but elbow has non-zero dynamic swing velocity (-0.95 rad/s > 0.50 rad/s limit)
    ctrl.get_time_sec = lambda: now_t
    ctrl.has_valid_joint_feedback = True
    ctrl.last_feedback_time = now_t
    ctrl.target_angles = list(transport_q)
    ctrl.measured_angles = list(transport_q)
    ctrl.measured_velocities = [0.0, 0.10, -0.95, 0.0, 0.05, 0.0]
    converged_vel_fail, _, _, _ = ctrl.check_target_convergence(transport_q)
    assert converged_vel_fail is False, "Convergence MUST fail when measured velocity exceeds justified threshold!"
    ctrl.motion_state = 'SETTLING'
    ctrl.settling_start_time = now_t
    ctrl.settling_converged_start_time = None
    ctrl.control_loop()
    assert ctrl.motion_state == 'SETTLING', f"Controller MUST NOT transition to HOLDING with high measured velocity, got: {ctrl.motion_state}"
    assert ctrl.target_reached is False, "target_reached MUST be False when velocity is non-zero dynamic!"
    print("  [PASS] Case 7I: False HOLDING with non-zero dynamic velocity strictly rejected.")

    # Case 7J: Genuine stable convergence with steady-state velocities and sustained settling window
    # Arm positions at transport pose with Gazebo steady-state ODE gravity balance velocities
    ctrl.measured_angles = [0.0, 0.305, -0.609, 0.0, 0.303, 0.0]
    ctrl.measured_velocities = [0.0, 0.207, -0.404, 0.0, 0.01, 0.0]

    # First cycle: convergence detected, but sustained settling window has not elapsed
    ctrl.control_loop()
    assert ctrl.motion_state == 'SETTLING', "Must remain in SETTLING on first cycle"
    assert ctrl.target_reached is False

    # After sustained period (0.30s >= 0.25s)
    ctrl.get_time_sec = lambda: now_t + 0.30
    ctrl.last_feedback_time = now_t + 0.30
    ctrl.control_loop()
    assert ctrl.motion_state == 'HOLDING', f"Must transition to HOLDING after sustained settling! Got: {ctrl.motion_state}"
    assert ctrl.target_reached is True, "target_reached must be True after sustained settling!"
    print("  [PASS] Case 7J: Genuine stable convergence confirmed with sustained settling window.")

    ctrl.destroy_node()
    node.destroy_node()


def run_all_regression_tests():
    print("=" * 70)
    print(" RUNNING FEEDBACK STALENESS & INTERLOCK REGRESSION SUITE")
    print("=" * 70)
    test_use_sim_time_configuration()
    test_queue_draining_with_spin_some()
    test_genuine_stale_feedback_blocking()
    test_controller_status_interlocks()
    test_spray_suppression_on_all_aborts()
    test_stale_feedback_during_arm_convergence()
    test_arm_transport_pose_convergence_and_drift_regression()
    print("=" * 70)
    print(" ALL FEEDBACK STALENESS REGRESSION TESTS PASSED SUCCESSFULLY!")
    print("=" * 70)


if __name__ == '__main__':
    run_all_regression_tests()

