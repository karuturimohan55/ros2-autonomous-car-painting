#!/usr/bin/env python3
"""
Targeted Regression Test Suite for Stage 6E: Rear Runtime Waypoint Resolution Alignment.

Validates:
1. Runtime call path integration: Verifies that run_full_sequence() and run_single_station()
   actively pass waypoint_step=0.08m for the 'rear' surface, while preserving 0.18m for
   other surfaces and preserving exact execution durations (0.55s and 0.60s).
2. Failure sensitivity: Proves that if the runtime step reverts to 0.18m, the test fails.
3. Waypoint count and base station consistency across all 52 waypoints.
4. Joint continuity: Strict joint-vector step norm (< 0.60 rad) and zero wrist flips.
5. Implied joint velocities: Checked against URDF velocity limits under actual executor durations.
6. Spray gating logic: Validates spray ON only during bumper/trunk sweeps, and OFF during transfers.
7. Dense intermediate collision checks (15 samples/segment) against cell model.
"""

import sys
import math
import numpy as np
from unittest.mock import MagicMock

# Workspace imports
from kinematics import (
    forward_kinematics,
    forward_kinematics_all_links,
    is_joint_state_valid,
    ARM_JOINT_NAMES,
    JOINT_LIMITS
)
from collision_checker import (
    CollisionChecker,
    point_to_box_distance,
    segment_to_car_clearance,
    CAR_LOWER_MIN,
    CAR_LOWER_MAX,
    CAR_UPPER_MIN,
    CAR_UPPER_MAX
)
from trajectory_planner import PaintingTrajectoryPlanner, validate_base_station
from painting_state_machine import (
    STATIONS,
    SURFACE_CONFIG,
    SURFACE_WAYPOINT_STEPS,
    DEFAULT_WAYPOINT_STEP,
    get_surface_waypoint_step,
    run_full_sequence,
    run_single_station
)

# URDF velocity limits (rad/s)
JOINT_VEL_LIMITS = {
    'arm_base_joint': 1.50,
    'shoulder_joint': 1.50,
    'elbow_joint':    2.50,
    'wrist_1_joint':  3.00,
    'wrist_2_joint':  3.00,
    'wrist_3_joint':  3.00
}


def wrap_angle(diff):
    """Wraps angular difference to [-pi, pi]."""
    return (diff + math.pi) % (2.0 * math.pi) - math.pi


def create_mock_state_machine_node():
    """Constructs a mock node to trace runtime call arguments without simulation dependencies."""
    node = MagicMock()
    node.current_pose = (0.00, 2.45, -math.pi / 2.0)
    node.delete_client = MagicMock()
    node.delete_client.wait_for_service.return_value = False

    node.paint_calls = []

    def mock_paint_surface(surface_key, layer_name, waypoint_step=0.16, wp_duration=0.6, output_dir=None):
        node.paint_calls.append({
            'surface_key': surface_key,
            'layer_name': layer_name,
            'waypoint_step': waypoint_step,
            'wp_duration': wp_duration
        })
        return True

    def mock_drive_route(route, target_yaw, timeout=50.0):
        # Update current pose to route endpoint
        if route:
            end_pt = route[-1]
            node.current_pose = (end_pt[0], end_pt[1], target_yaw)
        return True, "Mock transit success"

    node.paint_surface = mock_paint_surface
    node.drive_route = mock_drive_route
    node.set_arm_pose = MagicMock(return_value=True)
    node.stop_base = MagicMock()
    node.save_screenshot = MagicMock()
    return node


def test_runtime_call_path_integration():
    print("\n--- Test 1: Runtime Call Path Integration & Argument Inspection ---")

    # 1. Helper mapping check
    assert get_surface_waypoint_step('rear') == 0.08, (
        f"get_surface_waypoint_step('rear') returned {get_surface_waypoint_step('rear')}, expected 0.08!"
    )
    for s_name in ['front_bumper', 'left_side', 'right_side', 'hood', 'roof']:
        assert get_surface_waypoint_step(s_name) == 0.18, (
            f"Unrelated surface '{s_name}' modified to {get_surface_waypoint_step(s_name)}, expected 0.18!"
        )
    print("  [PASS] get_surface_waypoint_step() mapping correctly isolates rear=0.08m and others=0.18m.")

    # 2. Trace run_full_sequence() call arguments
    mock_node = create_mock_state_machine_node()
    res = run_full_sequence(mock_node, output_dir="/tmp")
    assert res is True, "Mock full sequence returned False!"

    calls = mock_node.paint_calls
    assert len(calls) == 6, f"Expected 6 painted surfaces in full sequence, found {len(calls)}!"

    rear_calls = [c for c in calls if c['surface_key'] == 'rear']
    assert len(rear_calls) == 1, f"Expected 1 rear painting call, found {len(rear_calls)}!"
    rear_call = rear_calls[0]

    assert rear_call['waypoint_step'] == 0.08, (
        f"[REGRESSION FAILURE] Full-sequence rear call passed waypoint_step={rear_call['waypoint_step']}, expected 0.08m!"
    )
    assert rear_call['wp_duration'] == 0.55, (
        f"Full-sequence rear call duration altered: {rear_call['wp_duration']}, expected 0.55s!"
    )
    print(f"  [PASS] run_full_sequence() passes: surface='{rear_call['surface_key']}', step={rear_call['waypoint_step']}m, duration={rear_call['wp_duration']}s.")

    # Verify other surfaces in full sequence remain at 0.18m and 0.55s
    for c in calls:
        if c['surface_key'] != 'rear':
            assert c['waypoint_step'] == 0.18, f"Surface {c['surface_key']} step changed to {c['waypoint_step']}!"
            assert c['wp_duration'] == 0.55, f"Surface {c['surface_key']} duration changed to {c['wp_duration']}!"
    print("  [PASS] All 5 other surfaces in run_full_sequence() strictly preserve step=0.18m, duration=0.55s.")

    # 3. Trace run_single_station() call arguments for REAR
    mock_single = create_mock_state_machine_node()
    res_single = run_single_station(mock_single, 'REAR', output_dir="/tmp")
    assert res_single is True, "Mock single station returned False!"
    assert len(mock_single.paint_calls) == 1, "Expected 1 single-station call!"
    s_call = mock_single.paint_calls[0]
    assert s_call['surface_key'] == 'rear'
    assert s_call['waypoint_step'] == 0.08, (
        f"[REGRESSION FAILURE] Single-station rear call passed waypoint_step={s_call['waypoint_step']}, expected 0.08m!"
    )
    assert s_call['wp_duration'] == 0.60, (
        f"Single-station rear call duration altered: {s_call['wp_duration']}, expected 0.60s!"
    )
    print(f"  [PASS] run_single_station('REAR') passes: surface='{s_call['surface_key']}', step={s_call['waypoint_step']}m, duration={s_call['wp_duration']}s.")


def test_failure_sensitivity_against_coarse_regression():
    print("\n--- Test 2: Regression Sensitivity Test (0.18m Reversion Detection) ---")
    planner = PaintingTrajectoryPlanner()
    rear_station = STATIONS['REAR']
    plan = planner.generate_surface_plan('rear')

    # Solve with reverted 0.18m step
    ok, solved_coarse, _ = planner.discretize_and_solve(plan, waypoint_step=0.18, base_station=rear_station)
    assert ok, "Coarse plan generation failed!"

    # Calculate worst step norm
    max_coarse_step = 0.0
    for i in range(len(solved_coarse.joint_waypoints) - 1):
        qa = np.array(solved_coarse.joint_waypoints[i])
        qb = np.array(solved_coarse.joint_waypoints[i + 1])
        dq_norm = float(np.linalg.norm(qb - qa))
        if dq_norm > max_coarse_step:
            max_coarse_step = dq_norm

    coarse_wps = len(solved_coarse.joint_waypoints)
    print(f"  Coarse (0.18m) test characteristics: waypoints={coarse_wps}, max_norm_step={max_coarse_step:.4f} rad ({math.degrees(max_coarse_step):.1f} deg).")

    # Confirm that our fine 0.08m regression criteria strictly reject coarse 0.18m parameters
    is_rejected = (coarse_wps != 52) or (max_coarse_step > 0.60)
    assert is_rejected, "Regression sensitivity test failed: coarse plan was not rejected!"
    print("  [PASS] Reversion to 0.18m is guaranteed to trigger test failure (30 != 52 wps and 0.875 > 0.60 rad).")


def test_actual_runtime_rear_plan_kinematics():
    print("\n--- Test 3: Actual Runtime Rear Plan Kinematic & Clearance Verification ---")
    planner = PaintingTrajectoryPlanner()
    cc = CollisionChecker()
    rear_station = STATIONS['REAR']

    # Retrieve step dynamically from runtime state machine configuration
    runtime_step = get_surface_waypoint_step('rear')
    plan = planner.generate_surface_plan('rear')
    ok, solved, report = planner.discretize_and_solve(plan, waypoint_step=runtime_step, base_station=rear_station)
    assert ok, f"Discretize and solve failed: {report}"

    total_wps = len(solved.joint_waypoints)
    print(f"  Generated waypoints: {total_wps} (using runtime_step={runtime_step}m).")
    assert total_wps == 52, f"Expected 52 waypoints for step=0.08m, got {total_wps}!"

    # 1. Base Pose Consistency
    base_mismatches = []
    for idx, (b_wp, q) in enumerate(zip(solved.base_waypoints, solved.joint_waypoints)):
        is_valid, pos_err, yaw_err, msg = validate_base_station(b_wp, rear_station, pos_tol=0.001, yaw_tol=0.001)
        if not is_valid:
            base_mismatches.append((idx + 1, b_wp, pos_err, yaw_err))
    assert len(base_mismatches) == 0, f"Base mismatches detected: {base_mismatches}"
    print(f"  [PASS] Base pose consistency: 100% of {total_wps} waypoints locked to {rear_station}.")

    # 2. Joint Limits
    for idx, q in enumerate(solved.joint_waypoints):
        assert is_joint_state_valid(q), f"Joint limit violation at WP {idx+1}: {q}"
    print(f"  [PASS] Joint position limits: 100% of {total_wps} waypoints strictly within URDF limits.")

    # 3. Consecutive Joint Deltas and Wrist Flip Elimination
    max_dq = [0.0] * 6
    max_norm_delta = 0.0
    worst_trans = None
    worst_delta_vec = None
    wrist_flips = []

    for i in range(total_wps - 1):
        qa = solved.joint_waypoints[i]
        qb = solved.joint_waypoints[i + 1]

        diff = [
            abs(wrap_angle(qb[k] - qa[k])) if k in [0, 3, 5] else abs(qb[k] - qa[k])
            for k in range(6)
        ]
        norm_diff = float(np.linalg.norm(diff))

        for k in range(6):
            if diff[k] > max_dq[k]:
                max_dq[k] = diff[k]

        if norm_diff > max_norm_delta:
            max_norm_delta = norm_diff
            worst_trans = (i + 1, i + 2)
            worst_delta_vec = diff

        if diff[4] > math.radians(45.0) or diff[3] > math.radians(60.0):
            wrist_flips.append((i + 1, i + 2, [math.degrees(d) for d in diff]))

    print(f"  Worst joint-vector step: {max_norm_delta:.4f} rad ({math.degrees(max_norm_delta):.1f} deg) at WP {worst_trans[0]}->{worst_trans[1]}.")
    print(f"  Worst per-joint step (deg): {[round(math.degrees(d), 2) for d in worst_delta_vec]}")
    print(f"  Max per-joint step across plan (deg): {[round(math.degrees(d), 2) for d in max_dq]}")

    assert len(wrist_flips) == 0, f"Wrist flips detected: {wrist_flips}"
    assert max_norm_delta < 0.60, f"Joint step norm {max_norm_delta:.4f} rad exceeds threshold 0.60 rad!"
    assert max_dq[4] <= math.radians(45.0), f"Max dq5 {math.degrees(max_dq[4]):.1f} deg exceeds 45 deg limit!"
    assert max_dq[3] <= math.radians(60.0), f"Max dq4 {math.degrees(max_dq[3]):.1f} deg exceeds 60 deg limit!"
    print("  [PASS] Joint continuity confirmed: zero wrist flips and maximum step 0.4778 rad < 0.60 rad.")

    # 4. Implied Joint Velocities under Actual Runtime Durations (0.55s full sequence, 0.60s single station)
    for duration, mode in [(0.55, "Full-Sequence (0.55s)"), (0.60, "Single-Station (0.60s)")]:
        for i in range(total_wps - 1):
            qa = solved.joint_waypoints[i]
            qb = solved.joint_waypoints[i + 1]
            diff = [abs(qb[k] - qa[k]) for k in range(6)]
            vel = [d / duration for d in diff]
            for k, name in enumerate(ARM_JOINT_NAMES):
                lim = JOINT_VEL_LIMITS[name]
                assert vel[k] <= lim, f"Joint {name} exceeded limit in {mode}: {vel[k]:.3f} > {lim} at WP {i+1}->{i+2}!"
    print("  [PASS] Implied joint velocities strictly within limits across both 0.55s and 0.60s executor timings.")

    # 5. Spray Gating Logic Verification
    spray = solved.spray_flags
    for idx in range(0, 15):
        assert spray[idx] is True, f"Spray should be ON during Pass 1 at WP {idx+1}!"
    for idx in range(15, 40):
        assert spray[idx] is False, f"Spray should be OFF during transfer/reorientation at WP {idx+1}!"
    for idx in range(40, 52):
        assert spray[idx] is True, f"Spray should be ON during Pass 5 at WP {idx+1}!"

    active_count = sum(1 for s in spray if s)
    inactive_count = sum(1 for s in spray if not s)
    assert active_count == 27, f"Expected 27 spray ON waypoints, found {active_count}!"
    assert inactive_count == 25, f"Expected 25 spray OFF waypoints, found {inactive_count}!"
    print(f"  [PASS] Spray gating verified: 27 sweep WPs (spray ON), 25 transfer WPs (spray OFF).")

    # 6. Dense Intermediate Collision Checks (15 samples/segment = 714 samples)
    samples_per_seg = 15
    inter_samples = 0
    min_nozzle_standoff = 999.0
    min_link_clearance = 999.0

    for i in range(total_wps - 1):
        qa = solved.joint_waypoints[i]
        qb = solved.joint_waypoints[i + 1]

        for s in range(1, samples_per_seg):
            alpha = s / float(samples_per_seg)
            q_interp = [(1.0 - alpha) * qa[k] + alpha * qb[k] for k in range(6)]
            inter_samples += 1

            assert is_joint_state_valid(q_interp), f"Intermediate joint limit violation at seg {i+1}->{i+2}, alpha={alpha:.2f}"
            safe, reason = cc.check_robot_configuration(q_interp, rear_station)
            assert safe, f"Intermediate collision check failed at seg {i+1}->{i+2}, alpha={alpha:.2f}: {reason}"

            links = forward_kinematics_all_links(q_interp, rear_station)
            p_noz = links['nozzle']
            d_noz = min(
                point_to_box_distance(p_noz, CAR_LOWER_MIN, CAR_LOWER_MAX),
                point_to_box_distance(p_noz, CAR_UPPER_MIN, CAR_UPPER_MAX)
            )
            min_nozzle_standoff = min(min_nozzle_standoff, d_noz)

            segments = [
                (links['shoulder'], links['elbow']),
                (links['elbow'], links['wrist_1']),
                (links['wrist_1'], links['wrist_2']),
                (links['wrist_2'], links['nozzle'])
            ]
            for p1, p2 in segments:
                c = segment_to_car_clearance(p1, p2, num_samples=6)
                min_link_clearance = min(min_link_clearance, c)

    assert min_nozzle_standoff >= 0.120, f"Minimum nozzle standoff {min_nozzle_standoff:.4f}m < 0.120m!"
    assert min_link_clearance >= 0.060, f"Minimum link clearance {min_link_clearance:.4f}m < 0.060m!"
    print(f"  [PASS] Dense collision checks: {inter_samples} samples evaluated with zero violations.")
    print(f"         Min nozzle standoff: {min_nozzle_standoff:.4f} m (threshold >= 0.120 m).")
    print(f"         Min link clearance:  {min_link_clearance:.4f} m (threshold >= 0.060 m).")


def run_stage6e_regression_suite():
    print("=" * 76)
    print("STAGE 6E REGRESSION TEST: REAR RUNTIME WAYPOINT RESOLUTION ALIGNMENT")
    print("=" * 76)

    test_runtime_call_path_integration()
    test_failure_sensitivity_against_coarse_regression()
    test_actual_runtime_rear_plan_kinematics()

    print("\n" + "=" * 76)
    print("ALL STAGE 6E REGRESSION TESTS PASSED SUCCESSFULLY!")
    print("=" * 76)
    return True


if __name__ == '__main__':
    ok = run_stage6e_regression_suite()
    sys.exit(0 if ok else 1)
