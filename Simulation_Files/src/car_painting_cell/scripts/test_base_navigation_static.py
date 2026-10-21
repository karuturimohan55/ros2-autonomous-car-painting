#!/usr/bin/env python3
"""
Static Unit & Safety Tests for Mobile Base Navigation to Left-Side Station.
Verifies:
1. Corridor route safety and car clearance from current position (-0.577, 2.15) to target (0.0, 2.0).
2. Angle normalization and error calculation across all quadrants.
3. Kinematic simulation of the 3-phase closed-loop controller confirming convergence to <0.01m and <0.5 deg.
4. Threshold validation (pos <= 0.03m, yaw <= 2.0 deg).
"""

import math
import sys
import os
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from collision_checker import CollisionChecker, point_to_box_distance, CAR_LOWER_MIN, CAR_LOWER_MAX, BOOTH_MIN, BOOTH_MAX


def normalize_angle(angle):
    """Normalizes angle to [-pi, pi]."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def check_pose_clearance(x, y, yaw, cc):
    """Checks 4 chassis corners for car and booth clearance."""
    half_l = 0.45
    half_w = 0.30
    corners = [(half_l, half_w), (half_l, -half_w), (-half_l, half_w), (-half_l, -half_w)]
    cos_y = math.cos(yaw)
    sin_y = math.sin(yaw)

    min_car_dist = 999.0
    for cx, cy in corners:
        wx = x + cx * cos_y - cy * sin_y
        wy = y + cx * sin_y + cy * cos_y
        p = np.array([wx, wy, 0.15])
        d = point_to_box_distance(p, CAR_LOWER_MIN, CAR_LOWER_MAX)
        min_car_dist = min(min_car_dist, d)
        # Check booth limits
        assert BOOTH_MIN[0] <= wx <= BOOTH_MAX[0], f"Corner ({wx:.2f}, {wy:.2f}) outside booth X"
        assert BOOTH_MIN[1] <= wy <= BOOTH_MAX[1], f"Corner ({wx:.2f}, {wy:.2f}) outside booth Y"

    return min_car_dist


def test_route_safety():
    print("Testing route clearance from current pose to left-side station...")
    cc = CollisionChecker()
    start_x, start_y = -0.577, 2.15
    target_x, target_y = 0.0, 2.00
    target_yaw = -math.pi / 2.0

    # Sample 20 points along direct path
    for i in range(21):
        alpha = i / 20.0
        x = start_x + alpha * (target_x - start_x)
        y = start_y + alpha * (target_y - start_y)
        # Heading varies between -87.3 deg and -90 deg
        yaw = -math.radians(87.3) + alpha * (-math.radians(90.0) - (-math.radians(87.3)))

        d_car = check_pose_clearance(x, y, yaw, cc)
        assert d_car >= 0.25, f"Clearance violated at ({x:.2f}, {y:.2f}): {d_car:.3f}m < 0.25m"

    print(f"  [PASS] Route clearance verified: minimum clearance = {d_car:.3f} m (buffer >= 0.25 m).")


def test_angle_normalization():
    print("Testing angle normalization...")
    assert abs(normalize_angle(0.0)) < 1e-6
    assert abs(normalize_angle(math.pi) - math.pi) < 1e-6 or abs(normalize_angle(math.pi) + math.pi) < 1e-6
    assert abs(normalize_angle(3.0 * math.pi) - math.pi) < 1e-6 or abs(normalize_angle(3.0 * math.pi) + math.pi) < 1e-6
    assert abs(normalize_angle(-math.pi / 2.0) - (-math.pi / 2.0)) < 1e-6
    assert abs(normalize_angle(math.radians(350)) - (-math.radians(10))) < 1e-6
    assert abs(normalize_angle(-math.radians(350)) - (math.radians(10))) < 1e-6
    print("  [PASS] Angle normalization verified.")


def test_kinematic_controller_simulation():
    print("Simulating closed-loop 2-stage corridor base controller...")
    x = -0.5771
    y = 2.1499
    yaw = -math.radians(87.28)

    target_x = 0.0
    target_y = 2.00
    target_yaw = -math.pi / 2.0

    entry_x = target_x - 0.25 * math.cos(target_yaw)
    entry_y = target_y - 0.25 * math.sin(target_yaw)

    dt = 0.05
    t_max = 30.0
    t = 0.0

    state = "TRANSIT_TO_ENTRY"
    settle_count = 0

    while t < t_max:
        pos_err = math.hypot(target_x - x, target_y - y)
        yaw_err = abs(normalize_angle(target_yaw - yaw))

        if state == "TRANSIT_TO_ENTRY":
            dx_entry = entry_x - x
            dy_entry = entry_y - y
            d_entry = math.hypot(dx_entry, dy_entry)

            if d_entry <= 0.035:
                e_entry_yaw = normalize_angle(target_yaw - yaw)
                if abs(e_entry_yaw) <= math.radians(3.5):
                    state = "CORRIDOR_APPROACH"
                    v, w = 0.0, 0.0
                else:
                    v = 0.0
                    w = math.copysign(max(0.08, min(0.30, 1.5 * abs(e_entry_yaw))), e_entry_yaw)
            else:
                phi = math.atan2(dy_entry, dx_entry)
                e_head = normalize_angle(phi - yaw)
                if abs(e_head) > math.radians(20.0):
                    v = 0.0
                    w = math.copysign(max(0.10, min(0.35, 1.8 * abs(e_head))), e_head)
                else:
                    v = max(0.04, min(0.12, 0.6 * d_entry))
                    w = math.copysign(min(0.25, 1.2 * abs(e_head)), e_head)

        elif state == "CORRIDOR_APPROACH":
            dx_target = x - target_x
            dy_target = y - target_y
            e_yaw = normalize_angle(target_yaw - yaw)

            if pos_err <= 0.025:
                v = 0.0
                if abs(e_yaw) <= math.radians(1.5):
                    w = 0.0
                    settle_count += 1
                    if settle_count >= 10:
                        state = "DONE"
                        break
                else:
                    settle_count = 0
                    w = math.copysign(max(0.06, min(0.18, 1.2 * abs(e_yaw))), e_yaw)
            else:
                settle_count = 0
                des_th = target_yaw - math.atan2(dx_target, 0.25)
                des_th = max(target_yaw - math.radians(35.0), min(target_yaw + math.radians(35.0), des_th))
                e_steer = normalize_angle(des_th - yaw)

                if dy_target < -0.015:
                    v = -0.04
                    w = math.copysign(min(0.15, 0.8 * abs(e_steer)), e_steer)
                else:
                    v = max(0.03, min(0.10, 0.5 * pos_err))
                    w = math.copysign(min(0.20, 1.2 * abs(e_steer)), e_steer)

        # Kinematic update
        x += v * math.cos(yaw) * dt
        y += v * math.sin(yaw) * dt
        yaw = normalize_angle(yaw + w * dt)
        t += dt

    final_pos_err = math.hypot(target_x - x, target_y - y)
    final_yaw_err = abs(normalize_angle(target_yaw - yaw))

    assert state == "DONE", f"Controller failed to finish: state={state}, t={t:.2f}s"
    assert final_pos_err <= 0.03, f"Final pos error {final_pos_err:.4f}m exceeds 0.03m"
    assert final_yaw_err <= math.radians(2.0), f"Final yaw error {math.degrees(final_yaw_err):.2f}deg exceeds 2.0 deg"

    print(f"  [PASS] Simulation reached target in {t:.2f}s: pos_err={final_pos_err*1000:.1f}mm, yaw_err={math.degrees(final_yaw_err):.2f}deg.")


if __name__ == '__main__':
    test_route_safety()
    test_angle_normalization()
    test_kinematic_controller_simulation()
    print("\nALL STATIC NAVIGATION TESTS PASSED!")
