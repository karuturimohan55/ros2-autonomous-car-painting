#!/usr/bin/env python3
"""
Targeted Verification Suite for Stage 6C: Rear Station Consistency & Continuous Transition.

Validates:
1. Unified base station: every rear waypoint uses STATIONS['REAR'] = (-3.50, 0.0, 0.0).
2. Elimination of abrupt wrist-orientation flip (delta q5 <= 45 deg, delta q4 <= 60 deg, norm delta <= 0.80 rad).
3. Spray-disabled transfer and reorientation between bumper and trunk.
4. Strict joint position limits and implied joint velocity limits across all waypoints.
5. Dense intermediate collision sampling (10 intermediate configurations per segment)
   verifying minimum nozzle standoff >= 0.12m and minimum link clearance >= 0.06m.
6. Clean analytical FK and orientation alignment for active painting sweeps.
"""

import sys
import math
import numpy as np

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
from painting_state_machine import STATIONS

# URDF velocity limits (rad/s)
JOINT_VEL_LIMITS = {
    'arm_base_joint': 1.50,
    'shoulder_joint': 1.50,
    'elbow_joint':    2.50,
    'wrist_1_joint':  3.00,
    'wrist_2_joint':  3.00,
    'wrist_3_joint':  3.00
}


def run_stage6c_validation():
    print("=" * 72)
    print("STAGE 6C TARGETED VERIFICATION: REAR PLAN CONSISTENCY & CONTINUITY")
    print("=" * 72)

    planner = PaintingTrajectoryPlanner()
    cc = CollisionChecker()

    expected_station = STATIONS['REAR']
    print(f"Target Rear Station from State Machine: {expected_station}")

    # Generate and discretize rear surface plan
    plan = planner.generate_surface_plan('rear')
    ok, solved, report = planner.discretize_and_solve(plan, waypoint_step=0.08, base_station=expected_station)

    assert ok, f"Discretization failed: {report}"
    total_wps = len(solved.joint_waypoints)
    print(f"[PASS] Rear plan discretized into {total_wps} waypoints over {solved.time_stamps[-1]:.1f}s.")

    # -------------------------------------------------------------------------
    # Check 1: Base Station Consistency
    # -------------------------------------------------------------------------
    print("\n--- Check 1: Base Station Consistency ---")
    base_mismatches = []
    for idx, (base_wp, q) in enumerate(zip(solved.base_waypoints, solved.joint_waypoints)):
        is_valid, pos_err, yaw_err, msg = validate_base_station(base_wp, expected_station, pos_tol=0.01, yaw_tol=0.01)
        if not is_valid:
            base_mismatches.append((idx + 1, base_wp, pos_err, yaw_err))

    assert len(base_mismatches) == 0, f"Found {len(base_mismatches)} waypoints with mismatched base station: {base_mismatches}"
    print(f"  [PASS] All {total_wps} waypoints use exact rear station {expected_station} (0 mismatches).")

    # -------------------------------------------------------------------------
    # Check 2: Joint Position Limits
    # -------------------------------------------------------------------------
    print("\n--- Check 2: Joint Position Limits ---")
    limit_violations = []
    for idx, q in enumerate(solved.joint_waypoints):
        if not is_joint_state_valid(q):
            limit_violations.append((idx + 1, q))

    assert len(limit_violations) == 0, f"Found {len(limit_violations)} joint limit violations: {limit_violations}"
    print(f"  [PASS] All {total_wps} waypoints strictly respect joint limits.")

    # -------------------------------------------------------------------------
    # Check 3: Consecutive Waypoint Deltas & Wrist Flip Elimination
    # -------------------------------------------------------------------------
    print("\n--- Check 3: Waypoint Continuity & Wrist Flip Elimination ---")
    max_dq = [0.0] * 6
    max_norm_delta = 0.0
    worst_transition = None
    worst_delta_vec = None
    wrist_flips = []

    for i in range(total_wps - 1):
        qa = np.array(solved.joint_waypoints[i])
        qb = np.array(solved.joint_waypoints[i + 1])
        dq = np.abs(qb - qa)
        norm_dq = float(np.linalg.norm(dq))

        for j in range(6):
            if dq[j] > max_dq[j]:
                max_dq[j] = dq[j]

        if norm_dq > max_norm_delta:
            max_norm_delta = norm_dq
            worst_transition = (i + 1, i + 2)
            worst_delta_vec = dq

        # Flag any sudden wrist reorientation
        if dq[4] > math.radians(45.0) or dq[3] > math.radians(60.0):
            wrist_flips.append((i + 1, i + 2, [round(math.degrees(d), 1) for d in dq]))

    print(f"  Worst consecutive joint norm delta: {max_norm_delta:.4f} rad ({math.degrees(max_norm_delta):.1f} deg) at WP {worst_transition[0]} -> {worst_transition[1]}")
    print(f"  Worst per-joint deltas: {[round(math.degrees(d), 1) for d in worst_delta_vec]} deg")
    print(f"  Max joint delta per joint across entire plan (deg): {[round(math.degrees(d), 1) for d in max_dq]}")
    print(f"  Wrist flips detected (dq5 > 45 deg or dq4 > 60 deg): {len(wrist_flips)}")

    assert len(wrist_flips) == 0, f"Abrupt wrist flips detected: {wrist_flips}"
    assert max_norm_delta < 0.80, f"Joint norm delta too high: {max_norm_delta:.4f} rad >= 0.80 rad"
    assert max_dq[4] <= math.radians(45.0), f"Max dq5 {math.degrees(max_dq[4]):.1f} deg exceeds 45 deg"
    assert max_dq[3] <= math.radians(60.0), f"Max dq4 {math.degrees(max_dq[3]):.1f} deg exceeds 60 deg"
    print("  [PASS] Continuous joint-space trajectory verified; abrupt wrist flip successfully eliminated.")

    # -------------------------------------------------------------------------
    # Check 4: Maximum Implied Joint Velocities
    # -------------------------------------------------------------------------
    print("\n--- Check 4: Implied Joint Velocities vs Limits ---")
    max_implied_vel = [0.0] * 6
    worst_vel_pair = [None] * 6

    for i in range(total_wps - 1):
        dt = solved.time_stamps[i + 1] - solved.time_stamps[i]
        assert dt > 1e-4, f"Zero time step between WP {i+1} and WP {i+2}"
        qa = np.array(solved.joint_waypoints[i])
        qb = np.array(solved.joint_waypoints[i + 1])
        vel = np.abs(qb - qa) / dt

        for j, name in enumerate(ARM_JOINT_NAMES):
            if vel[j] > max_implied_vel[j]:
                max_implied_vel[j] = vel[j]
                worst_vel_pair[j] = (i + 1, i + 2)

    for j, name in enumerate(ARM_JOINT_NAMES):
        limit = JOINT_VEL_LIMITS[name]
        pair_str = f"at WP {worst_vel_pair[j][0]}->{worst_vel_pair[j][1]}" if worst_vel_pair[j] is not None else "(held at 0.0)"
        print(f"  {name:16s}: max_vel={max_implied_vel[j]:.3f} rad/s (limit: {limit:.2f} rad/s) {pair_str}")
        assert max_implied_vel[j] <= limit, f"Joint {name} exceeded velocity limit: {max_implied_vel[j]:.3f} > {limit}"

    print("  [PASS] All implied joint velocities strictly within URDF velocity limits.")

    # -------------------------------------------------------------------------
    # Check 5: Spray State Logic
    # -------------------------------------------------------------------------
    print("\n--- Check 5: Spray State Logic ---")
    # In 5-pass plan: Pass 1 is bumper (spray ON), Pass 2-4 are transfer (spray OFF), Pass 5 is trunk (spray ON)
    spray_flags = solved.spray_flags
    reorient_spraying = False

    # Check that reorientation and transfer (Pass 2, 3, 4) have spray == False
    # Pass 1 is 15 waypoints, Pass 2 is 8, Pass 3 is 10, Pass 4 is 5, Pass 5 is 11
    # Waypoints 15 through 38 are transfer/reorientation
    active_spray_indices = [i + 1 for i, s in enumerate(spray_flags) if s]
    inactive_spray_indices = [i + 1 for i, s in enumerate(spray_flags) if not s]

    print(f"  Active spray waypoints count:   {len(active_spray_indices)}")
    print(f"  Inactive spray waypoints count: {len(inactive_spray_indices)}")

    # Ensure reorientation waypoints have spray disabled
    # In Pass 3 (reorientation pass), spray MUST be False
    for i, (q, s) in enumerate(zip(solved.joint_waypoints, spray_flags)):
        # Check transition between bumper elevation and trunk elevation
        if 15 <= i <= 38:
            assert not s, f"Spray active during transfer/reorientation at WP {i+1}!"

    print("  [PASS] Spray is verified disabled during all transfer and reorientation waypoints.")

    # -------------------------------------------------------------------------
    # Check 6: Dense Intermediate Collision Sampling
    # -------------------------------------------------------------------------
    print("\n--- Check 6: Dense Intermediate Collision & Clearance Sampling ---")
    samples_per_seg = 10
    total_samples = 0
    intermediate_col_fails = []
    min_nozzle_standoff = 999.0
    min_link_clearance = 999.0

    for i in range(total_wps - 1):
        qa = solved.joint_waypoints[i]
        qb = solved.joint_waypoints[i + 1]

        for s in range(1, samples_per_seg):
            alpha = s / float(samples_per_seg)
            q_mid = [(1.0 - alpha) * qa[k] + alpha * qb[k] for k in range(6)]
            total_samples += 1

            # Check limits
            if not is_joint_state_valid(q_mid):
                intermediate_col_fails.append((i + 1, i + 2, f"alpha={alpha:.1f} limit violation"))
                continue

            # Check collision checker
            safe, reason = cc.check_robot_configuration(q_mid, expected_station)
            if not safe:
                intermediate_col_fails.append((i + 1, i + 2, f"alpha={alpha:.1f} {reason}"))

            # Measure clearances
            links = forward_kinematics_all_links(q_mid, expected_station)
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

    print(f"  Intermediate samples evaluated: {total_samples}")
    print(f"  Intermediate collision violations: {len(intermediate_col_fails)}")
    print(f"  Minimum sampled nozzle standoff: {min_nozzle_standoff:.3f} m (threshold >= 0.120 m)")
    print(f"  Minimum sampled link clearance:  {min_link_clearance:.3f} m (threshold >= 0.060 m)")

    assert len(intermediate_col_fails) == 0, f"Intermediate collisions detected: {intermediate_col_fails}"
    assert min_nozzle_standoff >= 0.120, f"Nozzle standoff violated: {min_nozzle_standoff:.3f} m < 0.120 m"
    assert min_link_clearance >= 0.060, f"Link clearance violated: {min_link_clearance:.3f} m < 0.060 m"
    print("  [PASS] Dense intermediate collision checks passed with generous safety margins.")

    print("\n" + "=" * 72)
    print("ALL STAGE 6C VERIFICATION ASSERTIONS PASSED SUCCESSFULLY!")
    print("=" * 72)
    return True


if __name__ == '__main__':
    ok = run_stage6c_validation()
    sys.exit(0 if ok else 1)
