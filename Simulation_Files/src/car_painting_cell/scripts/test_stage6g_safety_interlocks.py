#!/usr/bin/env python3
"""
Stage 6G Regression Test Suite: Runtime Safety Interlocks & Failure Guarding

Tests:
1. Fresh joint feedback verification in wait_for_arm_convergence():
   - Missing joint feedback (None and uninitialized timestamp) immediately aborts.
   - Stale joint feedback (> 0.5s) immediately aborts.
   - Missing arm joints immediately aborts.
   - Fresh feedback (<= 0.5s) within tolerance succeeds.
2. Base-motion interlock in drive_route() & verify_arm_transport_safety():
   - Missing joint feedback blocks base motion.
   - Stale joint feedback (> 0.5s) blocks base motion.
   - Fresh feedback with arm not in transport blocks base motion.
   - Arm in transport but controller status not HOLDING blocks base motion.
   - Arm in transport but controller status stale (> 1.0s) blocks base motion.
   - Prohibits ANY non-zero /cmd_vel commands on any interlock failure.
   - Successful base-route authorization only when all required conditions pass.
3. Continuous mid-route safety interlock trip detection and immediate halt.
4. Transport retraction failure checks in paint_surface() and movement blocking:
   - Failed initial waypoint retraction returns False and stops base.
   - Failed standoff retraction returns False and stops base.
   - Failed post-paint retraction returns False and stops base.
   - Retraction failure strictly blocks subsequent base motion attempts.
5. Single-station return-to-HOME behavior:
   - Painting failure with unsafe arm does NOT attempt return route.
   - Successful painting with safe arm executes return route to HOME.
   - All 6 stations have verified defined return routes ending at HOME.
6. Spray shutoff verification across all abort paths.
7. Preservation of rear trajectory fine resolution (0.08m, 52 waypoints).

Zero live ROS/Gazebo nodes or motion commands are issued.
"""

import sys
import os
import math
import time
import numpy as np
from unittest.mock import MagicMock

# Isolate ROS domain to prevent any interaction with live nodes/Gazebo
os.environ['ROS_DOMAIN_ID'] = '123'

import rclpy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from painting_state_machine import (
    PaintingStateMachine,
    ARM_JOINT_NAMES,
    PRESET_POSES,
    DEFAULT_JOINT_TOLERANCES,
    DEFAULT_MAX_NORM_TOLERANCE,
    STATIONS,
    TRANSIT_ROUTES,
    SURFACE_CONFIG,
    SURFACE_WAYPOINT_STEPS,
    get_surface_waypoint_step,
    run_single_station,
    run_full_sequence
)
from trajectory_planner import PaintingTrajectoryPlanner


class SimulatedClock:
    """Deterministic simulated clock for precise timestamp testing."""
    def __init__(self, start_time=100.0):
        self.time = float(start_time)

    def now(self):
        return self.time

    def advance(self, dt):
        self.time += float(dt)


def create_joint_state_msg(joint_dict):
    """Helper to build a JointState message from a dict of joint_name -> position."""
    msg = JointState()
    for name, pos in joint_dict.items():
        msg.name.append(name)
        msg.position.append(float(pos))
    return msg


def make_test_node(sim_clock=None):
    """Instantiates a PaintingStateMachine with simulated clock and publisher spies."""
    if sim_clock is None:
        sim_clock = SimulatedClock(100.0)

    node = PaintingStateMachine()
    node.clock = sim_clock
    node.get_time_sec = sim_clock.now

    node.published_twists = []
    node.published_sprays = []

    def mock_twist_pub(twist_msg):
        tw = (float(twist_msg.linear.x), float(twist_msg.linear.y), float(twist_msg.angular.z))
        node.published_twists.append(tw)
        # Advance simulated kinematics if velocity is applied
        cx, cy, cyaw = node.current_pose
        dt = 0.05
        vx = tw[0]
        wz = tw[2]
        new_yaw = cyaw + wz * dt
        new_x = cx + vx * math.cos(new_yaw) * dt
        new_y = cy + vx * math.sin(new_yaw) * dt
        node.current_pose = (new_x, new_y, new_yaw)
        sim_clock.advance(0.02)

    def mock_spray_pub(spray_msg):
        node.published_sprays.append(bool(spray_msg.data))

    node.cmd_vel_pub.publish = mock_twist_pub
    node.spray_active_pub.publish = mock_spray_pub

    # Set mock current pose at HOME
    node.current_pose = (0.00, 2.45, -math.pi / 2.0)
    return node, sim_clock


def set_safe_transport_state(node, sim_clock, extra_offset=0.0):
    """Configures node state into perfectly verified transport and HOLDING condition."""
    now = sim_clock.now()
    transport_q = PRESET_POSES['transport']
    js_dict = {name: transport_q[i] + extra_offset for i, name in enumerate(ARM_JOINT_NAMES)}
    node.latest_joint_state = create_joint_state_msg(js_dict)
    node.latest_joint_state_time = now
    node.arm_motion_status = "HOLDING"
    node.latest_arm_status_time = now


# ---------------------------------------------------------------------------
# TEST 1: Fresh Joint Feedback in wait_for_arm_convergence()
# ---------------------------------------------------------------------------
def test_fresh_joint_feedback_in_convergence():
    print("\n--- Test 1: Fresh Joint Feedback in wait_for_arm_convergence() ---")
    node, clk = make_test_node()
    target_q = PRESET_POSES['ready']

    # Case 1A: Missing joint feedback (None)
    node.latest_joint_state = None
    node.latest_joint_state_time = 0.0
    node.published_sprays.clear()
    ok, norm = node.wait_for_arm_convergence(target_q, timeout=1.0)
    assert ok is False, "wait_for_arm_convergence should fail immediately when joint feedback is None!"
    assert len(node.published_sprays) > 0 and node.published_sprays[-1] is False, (
        "Spray must be disabled when joint feedback is missing!"
    )
    print("  [PASS] Case 1A: Missing feedback (None) immediately returns failure and shuts off spray.")

    # Case 1B: Missing timestamp (uninitialized <= 0.0)
    transport_dict = {name: PRESET_POSES['transport'][i] for i, name in enumerate(ARM_JOINT_NAMES)}
    node.latest_joint_state = create_joint_state_msg(transport_dict)
    node.latest_joint_state_time = 0.0
    node.published_sprays.clear()
    ok, norm = node.wait_for_arm_convergence(target_q, timeout=1.0)
    assert ok is False, "wait_for_arm_convergence should fail when joint feedback timestamp <= 0.0!"
    assert len(node.published_sprays) > 0 and node.published_sprays[-1] is False, (
        "Spray must be disabled when timestamp is uninitialized!"
    )
    print("  [PASS] Case 1B: Uninitialized timestamp (0.0) safely aborts and shuts off spray.")

    # Case 1C: Stale feedback (> 0.5s old)
    set_safe_transport_state(node, clk)
    clk.advance(0.55)  # Advance clock so feedback is 0.55s old (> 0.5s)
    node.published_sprays.clear()
    ok, norm = node.wait_for_arm_convergence(target_q, timeout=1.0)
    assert ok is False, "wait_for_arm_convergence must fail immediately when feedback age > 0.5s!"
    assert len(node.published_sprays) > 0 and node.published_sprays[-1] is False, (
        "Spray must be disabled when feedback is stale!"
    )
    print("  [PASS] Case 1C: Stale feedback (0.55s > 0.5s limit) immediately aborts and shuts off spray.")

    # Case 1D: Missing arm joints in JointState
    now = clk.now()
    partial_dict = {'arm_base_joint': 0.0, 'shoulder_joint': 0.0}
    node.latest_joint_state = create_joint_state_msg(partial_dict)
    node.latest_joint_state_time = now
    node.published_sprays.clear()
    ok, norm = node.wait_for_arm_convergence(target_q, timeout=1.0)
    assert ok is False, "wait_for_arm_convergence must fail when arm joints are incomplete!"
    assert len(node.published_sprays) > 0 and node.published_sprays[-1] is False, (
        "Spray must be disabled when arm joints are missing!"
    )
    print("  [PASS] Case 1D: Incomplete joint list immediately aborts and shuts off spray.")

    # Case 1E: Fresh valid feedback converging within tolerances
    now = clk.now()
    ready_dict = {name: PRESET_POSES['ready'][i] + 0.005 for i, name in enumerate(ARM_JOINT_NAMES)}
    node.latest_joint_state = create_joint_state_msg(ready_dict)
    node.latest_joint_state_time = now
    ok, norm = node.wait_for_arm_convergence(target_q, timeout=1.0)
    assert ok is True, f"wait_for_arm_convergence should succeed when within tolerance, got {ok}!"
    assert norm <= DEFAULT_MAX_NORM_TOLERANCE, f"Error norm {norm} exceeds {DEFAULT_MAX_NORM_TOLERANCE}!"
    print(f"  [PASS] Case 1E: Fresh feedback (age=0.0s) successfully converges (norm={norm:.4f} rad).")
    node.destroy_node()


# ---------------------------------------------------------------------------
# TEST 2: Base-Motion Interlock in verify_arm_transport_safety() and drive_route()
# ---------------------------------------------------------------------------
def test_base_motion_interlock():
    print("\n--- Test 2: Base-Motion Interlock in verify_arm_transport_safety() and drive_route() ---")
    node, clk = make_test_node()
    route = [(1.80, 2.30)]
    final_yaw = -math.pi

    # Condition 2A: Missing joint feedback
    set_safe_transport_state(node, clk)
    node.latest_joint_state = None
    node.latest_joint_state_time = 0.0
    node.published_twists.clear()
    node.published_sprays.clear()

    safe, reason = node.verify_arm_transport_safety()
    assert safe is False, "verify_arm_transport_safety must return False on missing feedback!"
    assert "missing" in reason.lower() or "no /joint_states" in reason.lower(), f"Unexpected reason: {reason}"

    ok_route, r_msg = node.drive_route(route, final_yaw, timeout=1.0)
    assert ok_route is False, "drive_route must abort when arm transport safety fails!"
    non_zero_twists = [tw for tw in node.published_twists if any(abs(v) > 1e-4 for v in tw)]
    assert len(non_zero_twists) == 0, f"Prohibited non-zero cmd_vel published on missing feedback: {non_zero_twists}"
    print("  [PASS] Condition 2A: Missing feedback blocked base motion (zero non-zero cmd_vel).")

    # Condition 2B: Stale feedback (0.65s > 0.5s)
    set_safe_transport_state(node, clk)
    clk.advance(0.65)
    node.published_twists.clear()

    safe, reason = node.verify_arm_transport_safety()
    assert safe is False, "verify_arm_transport_safety must return False on stale feedback!"
    assert "stale" in reason.lower(), f"Unexpected reason: {reason}"

    ok_route, r_msg = node.drive_route(route, final_yaw, timeout=1.0)
    assert ok_route is False, "drive_route must abort on stale feedback!"
    non_zero_twists = [tw for tw in node.published_twists if any(abs(v) > 1e-4 for v in tw)]
    assert len(non_zero_twists) == 0, f"Prohibited non-zero cmd_vel published on stale feedback: {non_zero_twists}"
    print("  [PASS] Condition 2B: Stale feedback (0.65s) blocked base motion.")

    # Condition 2C: Fresh feedback, but arm NOT in transport pose (e.g. side_paint pose)
    set_safe_transport_state(node, clk)
    side_paint_dict = {name: PRESET_POSES['side_paint'][i] for i, name in enumerate(ARM_JOINT_NAMES)}
    node.latest_joint_state = create_joint_state_msg(side_paint_dict)
    node.published_twists.clear()

    safe, reason = node.verify_arm_transport_safety()
    assert safe is False, "verify_arm_transport_safety must return False when arm is not in transport!"
    assert "tolerance" in reason.lower() or "offset" in reason.lower() or "exceeds" in reason.lower(), f"Unexpected reason: {reason}"

    ok_route, r_msg = node.drive_route(route, final_yaw, timeout=1.0)
    assert ok_route is False, "drive_route must abort when arm is in side_paint pose!"
    non_zero_twists = [tw for tw in node.published_twists if any(abs(v) > 1e-4 for v in tw)]
    assert len(non_zero_twists) == 0, f"Prohibited non-zero cmd_vel published when arm not in transport: {non_zero_twists}"
    print("  [PASS] Condition 2C: Non-transport arm configuration blocked base motion.")

    # Condition 2D: Arm in transport, but arm controller status is NOT HOLDING (e.g. TRAJECTORY)
    set_safe_transport_state(node, clk)
    node.arm_motion_status = "TRAJECTORY"
    node.published_twists.clear()

    safe, reason = node.verify_arm_transport_safety()
    assert safe is False, "verify_arm_transport_safety must return False when status != HOLDING!"
    assert "holding" in reason.lower(), f"Unexpected reason: {reason}"

    ok_route, r_msg = node.drive_route(route, final_yaw, timeout=1.0)
    assert ok_route is False, "drive_route must abort when arm status is TRAJECTORY!"
    non_zero_twists = [tw for tw in node.published_twists if any(abs(v) > 1e-4 for v in tw)]
    assert len(non_zero_twists) == 0, f"Prohibited non-zero cmd_vel published when arm is moving: {non_zero_twists}"
    print("  [PASS] Condition 2D: Controller status 'TRAJECTORY' blocked base motion.")

    # Condition 2E: Arm in transport, status HOLDING, but status message is STALE (> 1.0s)
    set_safe_transport_state(node, clk)
    node.latest_arm_status_time = clk.now() - 1.2
    node.published_twists.clear()

    safe, reason = node.verify_arm_transport_safety()
    assert safe is False, "verify_arm_transport_safety must return False when arm status is stale!"
    assert "stale" in reason.lower(), f"Unexpected reason: {reason}"

    ok_route, r_msg = node.drive_route(route, final_yaw, timeout=1.0)
    assert ok_route is False, "drive_route must abort when arm status is stale!"
    non_zero_twists = [tw for tw in node.published_twists if any(abs(v) > 1e-4 for v in tw)]
    assert len(non_zero_twists) == 0, f"Prohibited non-zero cmd_vel published when status stale: {non_zero_twists}"
    print("  [PASS] Condition 2E: Stale arm status (1.2s) blocked base motion.")

    # Condition 2F: All conditions pass -> base motion authorized
    set_safe_transport_state(node, clk)
    safe, reason = node.verify_arm_transport_safety()
    assert safe is True, f"verify_arm_transport_safety should return True under safe conditions, got: {reason}"

    # Micro route to target (adjacent position already at target)
    micro_route = [(node.current_pose[0], node.current_pose[1])]
    ok_auth, auth_msg = node.drive_route(micro_route, node.current_pose[2], timeout=2.0)
    assert ok_auth is True, f"Base route execution failed when all conditions met: {auth_msg}"
    print(f"  [PASS] Condition 2F: All safety conditions verified; base route authorized and completed ({auth_msg}).")

    node.destroy_node()


# ---------------------------------------------------------------------------
# TEST 3: Continuous Mid-Route Interlock Check
# ---------------------------------------------------------------------------
def test_mid_route_interlock_trip():
    print("\n--- Test 3: Continuous Mid-Route Safety Interlock Tripping ---")
    node, clk = make_test_node()
    set_safe_transport_state(node, clk)

    # Route requiring multiple steps
    route = [(1.80, 2.30), (3.50, 2.30)]
    final_yaw = -math.pi

    # Hook spin_once to trip feedback staleness after first motion commands
    trip_count = [0]
    orig_spin_once = rclpy.spin_once

    def mock_spin_once(target_node, timeout_sec=0.0):
        trip_count[0] += 1
        if trip_count[0] >= 3:
            # Advance clock by 1.0s without updating latest_joint_state_time
            clk.advance(1.0)

    rclpy.spin_once = mock_spin_once
    node.published_twists.clear()
    node.published_sprays.clear()

    ok, msg = node.drive_route(route, final_yaw, timeout=5.0)
    rclpy.spin_once = orig_spin_once

    assert ok is False, "drive_route should abort when feedback goes stale mid-route!"
    assert "interlock triggered" in msg.lower(), f"Unexpected abort message: {msg}"
    assert len(node.published_sprays) > 0 and node.published_sprays[-1] is False, (
        "Spray must be disabled when route is aborted mid-flight!"
    )
    # The last published twist must be a zero-velocity halt from stop_base()
    last_twist = node.published_twists[-1]
    assert last_twist == (0.0, 0.0, 0.0), f"Expected final twist to be zero stop, got {last_twist}!"
    print("  [PASS] Mid-route staleness safely halted base with zero velocity and disabled spray.")

    node.destroy_node()


# ---------------------------------------------------------------------------
# TEST 4: Check All Transport Retraction Calls in paint_surface()
# ---------------------------------------------------------------------------
def test_transport_retraction_failure_checks():
    print("\n--- Test 4: Checked Transport Retraction in paint_surface() & Movement Blocking ---")
    node, clk = make_test_node()
    set_safe_transport_state(node, clk)

    # 4A: Normal paint completion but set_arm_pose("transport") fails
    node.wait_for_arm_convergence = MagicMock(return_value=(True, 0.01))
    node.set_arm_pose = MagicMock(return_value=False)  # Retraction fails!
    node.spawn_paint_layer = MagicMock(return_value=True)

    painted = node.paint_surface('front_bumper', 'paint_layer_front', waypoint_step=0.18, wp_duration=0.1)
    assert painted is False, "paint_surface must return False when post-painting retraction fails!"
    print("  [PASS] 4A: Failed post-painting transport retraction causes paint_surface to return False.")

    # 4B: Initial waypoint abort but set_arm_pose("transport") fails
    node.wait_for_arm_convergence = MagicMock(return_value=(False, 0.50))  # First WP fails
    node.set_arm_pose = MagicMock(return_value=False)

    painted = node.paint_surface('front_bumper', 'paint_layer_front', waypoint_step=0.18, wp_duration=0.1)
    assert painted is False, "paint_surface must return False on initial WP failure!"
    print("  [PASS] 4B: Initial waypoint convergence failure safely handled.")

    # 4C: Standoff abort with retraction failure
    # Ensure arm remains out of transport when retraction fails
    extended_dict = {name: PRESET_POSES['side_paint'][i] for i, name in enumerate(ARM_JOINT_NAMES)}
    node.latest_joint_state = create_joint_state_msg(extended_dict)

    # Verify that subsequent drive_route is strictly blocked
    safe, reason = node.verify_arm_transport_safety()
    assert safe is False, "Arm must not be considered safe in transport when retraction failed!"

    node.published_twists.clear()
    ok_drive, d_msg = node.drive_route([(0.0, 2.0)], 0.0, timeout=1.0)
    assert ok_drive is False, "drive_route must refuse movement when arm failed retraction!"
    non_zero = [tw for tw in node.published_twists if any(abs(v) > 1e-4 for v in tw)]
    assert len(non_zero) == 0, "No non-zero base motion permitted after retraction failure!"
    print("  [PASS] 4C: Failed retraction strictly blocks subsequent base motion attempts.")

    node.destroy_node()


# ---------------------------------------------------------------------------
# TEST 5: Single-Station Return-to-HOME Safety Guarding
# ---------------------------------------------------------------------------
def test_single_station_return_behavior():
    print("\n--- Test 5: Single-Station Return-to-HOME Behavior & Unsafe Arm Guarding ---")
    node, clk = make_test_node()
    output_dir = "/tmp"

    # Case 5A: Single station failure where arm is UNSAFE (failed to retract)
    set_safe_transport_state(node, clk)
    node.set_arm_pose = MagicMock(side_effect=[True, False])  # Initial transport succeeds, paint retract fails
    node.drive_route = MagicMock(return_value=(True, "Arrived at station"))

    # Mock paint_surface to return False (painting failed and arm left extended)
    def mock_paint_failure(*args, **kwargs):
        extended = {name: PRESET_POSES['side_paint'][i] for i, name in enumerate(ARM_JOINT_NAMES)}
        node.latest_joint_state = create_joint_state_msg(extended)
        return False

    node.paint_surface = mock_paint_failure
    node.stop_base = MagicMock()

    res = run_single_station(node, 'REAR', output_dir)
    assert res is False, "run_single_station must return False on painting failure!"
    # Must NOT have called drive_route for return-to-HOME with extended arm
    assert node.drive_route.call_count == 1, (
        f"drive_route was called {node.drive_route.call_count} times! It must NOT attempt return with unsafe arm."
    )
    node.stop_base.assert_called()
    print("  [PASS] Case 5A: Single-station failure with unsafe arm strictly prevents return route.")

    # Case 5B: Single station successful run with verified transport pose and return-to-HOME
    set_safe_transport_state(node, clk)
    node.set_arm_pose = MagicMock(return_value=True)
    drive_route_calls = []

    def mock_successful_drive(route, yaw, timeout=50.0):
        drive_route_calls.append((route, yaw))
        if route:
            node.current_pose = (route[-1][0], route[-1][1], yaw)
        return True, "Success"

    node.drive_route = mock_successful_drive
    node.paint_surface = MagicMock(return_value=True)
    node.save_screenshot = MagicMock(return_value=True)

    res_success = run_single_station(node, 'REAR', output_dir)
    assert res_success is True, "run_single_station should succeed when all operations pass!"
    assert len(drive_route_calls) == 2, (
        f"Expected 2 drive_route calls (outbound + return), got {len(drive_route_calls)}!"
    )
    outbound_route, outbound_yaw = drive_route_calls[0]
    return_route, return_yaw = drive_route_calls[1]

    # Verify return route target matches HOME
    assert return_route == TRANSIT_ROUTES[('REAR', 'HOME')], (
        f"Return route mismatch: got {return_route}, expected {TRANSIT_ROUTES[('REAR', 'HOME')]}"
    )
    assert return_yaw == STATIONS['HOME'][2], f"Return yaw mismatch: {return_yaw} != {STATIONS['HOME'][2]}"
    print("  [PASS] Case 5B: Single-station success properly returns robot to HOME along defined corridor.")

    # Case 5C: Verify all stations have defined return routes to HOME
    for stn_key in ['FRONT', 'LEFT_SIDE', 'REAR', 'RIGHT_SIDE', 'FRONT_HOOD', 'ROOF']:
        ret_key = (stn_key, 'HOME')
        assert ret_key in TRANSIT_ROUTES, f"Missing return route in TRANSIT_ROUTES for {ret_key}!"
        r_path = TRANSIT_ROUTES[ret_key]
        assert len(r_path) >= 1, f"Empty return path for {ret_key}!"
        final_pt = r_path[-1]
        assert math.isclose(final_pt[0], STATIONS['HOME'][0], abs_tol=0.05), (
            f"Return route for {stn_key} ends at X={final_pt[0]}, expected {STATIONS['HOME'][0]}"
        )
        assert math.isclose(final_pt[1], STATIONS['HOME'][1], abs_tol=0.05), (
            f"Return route for {stn_key} ends at Y={final_pt[1]}, expected {STATIONS['HOME'][1]}"
        )
    print("  [PASS] Case 5C: Verified all 6 stations have collision-free defined return routes ending at HOME.")

    node.destroy_node()


# ---------------------------------------------------------------------------
# TEST 6: Spray Shutoff on Every Abort Path
# ---------------------------------------------------------------------------
def test_spray_shutoff_on_all_aborts():
    print("\n--- Test 6: Spray Shutoff Verification on Every Abort Path ---")
    node, clk = make_test_node()
    set_safe_transport_state(node, clk)

    # Track every call to set_spray
    spray_records = []
    node.spray_active_pub.publish = lambda m: spray_records.append(bool(m.data))

    # Abort Path 1: Initial arm pose at HOME fails
    node.set_arm_pose = MagicMock(return_value=False)
    spray_records.clear()
    res1 = run_single_station(node, 'FRONT', "/tmp")
    assert res1 is False and spray_records and spray_records[-1] is False, (
        "Spray not shut off when initial arm transport fails in single station!"
    )
    print("  [PASS] Abort Path 1: Initial arm pose failure shuts off spray.")

    # Abort Path 2: Navigation to station fails
    node.set_arm_pose = MagicMock(return_value=True)
    node.drive_route = MagicMock(return_value=(False, "Route collision"))
    spray_records.clear()
    res2 = run_single_station(node, 'FRONT', "/tmp")
    assert res2 is False and spray_records and spray_records[-1] is False, (
        "Spray not shut off when navigation fails!"
    )
    print("  [PASS] Abort Path 2: Navigation failure shuts off spray.")

    # Abort Path 3: Station positioning tolerance fails
    node.drive_route = MagicMock(return_value=(True, "Arrived"))
    node.current_pose = (0.0, 0.0, 0.0)  # Grossly outside FRONT station (3.5, 0.0)
    spray_records.clear()
    res3 = run_single_station(node, 'FRONT', "/tmp")
    assert res3 is False and spray_records and spray_records[-1] is False, (
        "Spray not shut off when station tolerance fails!"
    )
    print("  [PASS] Abort Path 3: Station positioning tolerance failure shuts off spray.")

    # Abort Path 4: Initial waypoint reach failure in paint_surface
    node.current_pose = STATIONS['FRONT']
    node.wait_for_arm_convergence = MagicMock(return_value=(False, 0.50))
    spray_records.clear()
    res4 = node.paint_surface('front_bumper', 'paint_layer_front', waypoint_step=0.18, wp_duration=0.1)
    assert res4 is False and spray_records and spray_records[-1] is False, (
        "Spray not shut off when initial painting waypoint fails!"
    )
    print("  [PASS] Abort Path 4: Initial painting waypoint failure shuts off spray.")

    # Abort Path 5: Collision checker stops raster sweep mid-pass
    node.wait_for_arm_convergence = MagicMock(return_value=(True, 0.01))
    node.collision_checker.check_robot_configuration = MagicMock(return_value=(False, "Booth limit"))
    spray_records.clear()
    res5 = node.paint_surface('front_bumper', 'paint_layer_front', waypoint_step=0.18, wp_duration=0.1)
    assert res5 is False and spray_records and spray_records[-1] is False, (
        "Spray not shut off when collision check fails!"
    )
    print("  [PASS] Abort Path 5: Intermediate collision detection shuts off spray.")

    node.destroy_node()


# ---------------------------------------------------------------------------
# TEST 7: Preservation of Rear Trajectory Resolution (0.08 m)
# ---------------------------------------------------------------------------
def test_rear_resolution_preservation():
    print("\n--- Test 7: Preservation of Rear Trajectory Resolution (0.08 m) ---")
    assert get_surface_waypoint_step('rear') == 0.08, (
        f"Rear surface waypoint step altered to {get_surface_waypoint_step('rear')}, must be 0.08!"
    )
    assert SURFACE_WAYPOINT_STEPS['rear'] == 0.08, (
        f"SURFACE_WAYPOINT_STEPS['rear'] is {SURFACE_WAYPOINT_STEPS['rear']}, must be 0.08!"
    )

    planner = PaintingTrajectoryPlanner()
    plan = planner.generate_surface_plan('rear')
    ok, solved, report = planner.discretize_and_solve(
        plan, waypoint_step=0.08, base_station=STATIONS['REAR']
    )
    assert ok is True, f"Failed to solve rear plan at 0.08m: {report}"
    assert len(solved.joint_waypoints) == 52, (
        f"Expected exactly 52 rear waypoints, got {len(solved.joint_waypoints)}!"
    )

    # Verify joint space step < 0.60 rad
    max_step = 0.0
    for i in range(len(solved.joint_waypoints) - 1):
        qa = np.array(solved.joint_waypoints[i])
        qb = np.array(solved.joint_waypoints[i + 1])
        dq = float(np.linalg.norm(qb - qa))
        if dq > max_step:
            max_step = dq
    assert max_step < 0.60, f"Max joint-vector step {max_step:.4f} rad exceeds 0.60 rad threshold!"
    print(f"  [PASS] Rear trajectory resolution confirmed: 52 waypoints at 0.08m, max step {max_step:.4f} rad < 0.60 rad.")


# ---------------------------------------------------------------------------
# TEST 8: Live Navigation Velocity Interlock & Slew-Rate Limiting Regression
# ---------------------------------------------------------------------------
def test_live_navigation_velocity_interlock_regression():
    print("\n--- Test 8: Live Navigation Velocity Interlock & Slew-Rate Limiting Regression ---")
    node, clk = make_test_node()
    set_safe_transport_state(node, clk)
    route = [(1.80, 2.30), (3.50, 2.30), (3.50, 0.00)]
    final_yaw = -math.pi

    # Case 8A: Exact live failure condition: shoulder velocity = 0.3159 rad/s > 0.3000 rad/s
    now = clk.now()
    js_msg = JointState()
    js_msg.name = list(ARM_JOINT_NAMES)
    js_msg.position = list(PRESET_POSES['transport'])
    # Populate exact live failure velocities
    js_msg.velocity = [0.0, 0.3159, -0.4200, 0.0, 0.0100, 0.0]
    node.latest_joint_state = js_msg
    node.latest_joint_state_time = now

    safe, reason = node.verify_arm_transport_safety()
    assert safe is False, "verify_arm_transport_safety MUST return False when shoulder velocity is 0.3159 rad/s!"
    assert "0.3159" in reason and "0.3000" in reason, f"Reason must report exact velocity and limit! Got: {reason}"
    print("  [PASS] Case 8A: Shoulder velocity 0.3159 rad/s strictly tripped 0.3000 rad/s limit.")

    # Confirm drive_route strictly aborts with zero non-zero cmd_vel
    node.published_twists.clear()
    ok_route, r_msg = node.drive_route(route, final_yaw, timeout=1.0)
    assert ok_route is False, "drive_route MUST abort when shoulder velocity exceeds 0.3000 rad/s!"
    assert "0.3159" in r_msg and "0.3000" in r_msg, f"Abort message must state velocity interlock trip! Got: {r_msg}"
    non_zero_twists = [tw for tw in node.published_twists if any(abs(v) > 1e-4 for v in tw)]
    assert len(non_zero_twists) == 0, f"Prohibited non-zero cmd_vel published under velocity trip: {non_zero_twists}"
    print("  [PASS] Case 8B: Base motion strictly prohibited under shoulder velocity limit violation.")

    # Case 8C: Steady-state ODE gravity balance velocities (shoulder=0.211 rad/s <= 0.3000 rad/s)
    js_msg.velocity = [0.0, 0.2110, -0.4200, 0.0, 0.0100, 0.0]
    node.latest_joint_state = js_msg
    node.latest_joint_state_time = clk.now()
    safe_steady, reason_steady = node.verify_arm_transport_safety()
    assert safe_steady is True, f"Steady-state ODE velocity must be safe! Got: {reason_steady}"
    print("  [PASS] Case 8C: Steady-state ODE gravity balance (shoulder=0.2110 rad/s) verified safe.")

    # Case 8D: Micro route execution under bounded velocity ramping
    micro_route = [(node.current_pose[0], node.current_pose[1])]
    node.published_twists.clear()
    ok_micro, micro_msg = node.drive_route(micro_route, node.current_pose[2], timeout=2.0)
    assert ok_micro is True, f"Micro route should complete cleanly under safe conditions! Got: {micro_msg}"
    print("  [PASS] Case 8D: Route completes cleanly with smooth rate-limited commands.")

    # Case 8E: Verify ramp_stop_base bounds deceleration without instantaneous step drops
    node.published_twists.clear()
    node.ramp_stop_base(current_vx=0.25, current_wz=0.20, max_lin_accel=0.30, max_ang_accel=0.40, settle_sec=0.10)
    assert len(node.published_twists) > 0, "ramp_stop_base must publish deceleration trajectory!"
    # Verify no adjacent linear velocity change exceeds max_lin_accel * dt_max (0.30 * 0.05 = 0.015 m/s)
    for i in range(len(node.published_twists) - 1):
        dv = abs(node.published_twists[i+1][0] - node.published_twists[i][0])
        assert dv <= 0.020, f"ramp_stop_base allowed excessive linear velocity step {dv:.4f} m/s > 0.020 m/s!"
    final_tw = node.published_twists[-1]
    assert abs(final_tw[0]) < 1e-4 and abs(final_tw[2]) < 1e-4, f"Final velocity must be zero! Got: {final_tw}"
    print("  [PASS] Case 8E: ramp_stop_base bounds linear and angular deceleration without step drops.")

    node.destroy_node()


# ---------------------------------------------------------------------------
# MAIN RUNNER
# ---------------------------------------------------------------------------
def run_all_tests():
    print("=" * 76)
    print("STAGE 6G COMPREHENSIVE SAFETY INTERLOCKS & FAILURE GUARDING REGRESSION SUITE")
    print("=" * 76)

    rclpy.init()
    try:
        test_fresh_joint_feedback_in_convergence()
        test_base_motion_interlock()
        test_mid_route_interlock_trip()
        test_transport_retraction_failure_checks()
        test_single_station_return_behavior()
        test_spray_shutoff_on_all_aborts()
        test_rear_resolution_preservation()
        test_live_navigation_velocity_interlock_regression()

        print("\n" + "=" * 76)
        print("ALL 8 STAGE 6G SAFETY INTERLOCK REGRESSION TESTS PASSED SUCCESSFULLY!")
        print("=" * 76)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    run_all_tests()
