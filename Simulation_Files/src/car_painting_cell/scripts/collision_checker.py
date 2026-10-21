#!/usr/bin/env python3
"""
Collision Checker for Yuktii AI Labs Painting Robot (Stage 3).
Validates:
1. Mobile base clearance against vehicle body and painting booth enclosure.
2. 3D line-segment clearance for all articulated arm links (shoulder, elbow, forearm, wrist, gun).
3. Minimum standoff distance enforcement for spray gun nozzle tip.
4. Rejection of unsafe or colliding trajectories before execution.
"""

import math
import numpy as np
from kinematics import forward_kinematics_all_links, is_joint_state_valid

# Vehicle Bounding Volumes (meters, centered at 0,0,0)
# Lower chassis & bumper envelope
CAR_LOWER_MIN = np.array([-2.42, -0.92, 0.00])
CAR_LOWER_MAX = np.array([+2.42, +0.92, 0.74])

# Cabin, windshield, windows & roof envelope
CAR_UPPER_MIN = np.array([-1.20, -0.80, 0.74])
CAR_UPPER_MAX = np.array([+1.05, +0.80, 1.62])

# Booth Interior Working Boundaries
BOOTH_MIN = np.array([-6.50, -4.50, 0.00])
BOOTH_MAX = np.array([+6.50, +4.50, 3.80])

# Safety Clearance Thresholds (meters)
MIN_BASE_TO_CAR_CLEARANCE = 0.16   # Base footprint buffer (16cm)
MIN_LINK_TO_CAR_CLEARANCE = 0.06   # Arm link buffer
MIN_NOZZLE_STANDOFF = 0.12         # Nozzle must not touch car


def point_to_box_distance(p, box_min, box_max):
    """
    Computes signed/Euclidean distance from point p to axis-aligned bounding box.
    Returns 0.0 if p is inside box, otherwise positive Euclidean distance to boundary.
    """
    dx = max(box_min[0] - p[0], 0.0, p[0] - box_max[0])
    dy = max(box_min[1] - p[1], 0.0, p[1] - box_max[1])
    dz = max(box_min[2] - p[2], 0.0, p[2] - box_max[2])
    return math.sqrt(dx**2 + dy**2 + dz**2)


def is_point_inside_car(p, buffer=0.0):
    """Checks if point p is inside the car bounding volumes (with buffer)."""
    # Check lower body
    if (CAR_LOWER_MIN[0] - buffer <= p[0] <= CAR_LOWER_MAX[0] + buffer and
        CAR_LOWER_MIN[1] - buffer <= p[1] <= CAR_LOWER_MAX[1] + buffer and
        CAR_LOWER_MIN[2] - buffer <= p[2] <= CAR_LOWER_MAX[2] + buffer):
        return True

    # Check upper cabin
    if (CAR_UPPER_MIN[0] - buffer <= p[0] <= CAR_UPPER_MAX[0] + buffer and
        CAR_UPPER_MIN[1] - buffer <= p[1] <= CAR_UPPER_MAX[1] + buffer and
        CAR_UPPER_MIN[2] - buffer <= p[2] <= CAR_UPPER_MAX[2] + buffer):
        return True

    return False


def segment_to_car_clearance(p1, p2, num_samples=8):
    """Computes minimum clearance between 3D line segment p1-p2 and car bounding volume."""
    min_dist = 999.0
    for s in range(num_samples + 1):
        alpha = s / float(num_samples)
        p = p1 + alpha * (p2 - p1)
        d_lower = point_to_box_distance(p, CAR_LOWER_MIN, CAR_LOWER_MAX)
        d_upper = point_to_box_distance(p, CAR_UPPER_MIN, CAR_UPPER_MAX)
        d = min(d_lower, d_upper)
        if d < min_dist:
            min_dist = d
            if min_dist < 0.01:
                break
    return min_dist


class CollisionChecker:
    def __init__(self):
        pass

    def check_base_pose(self, base_pose):
        """
        Validates mobile base chassis footprint at base_pose = (xb, yb, yaw).
        Returns: (is_safe: bool, reason: str)
        """
        xb, yb, yaw = base_pose

        # Chassis half-extents
        half_l = 0.45
        half_w = 0.30

        # Check 4 chassis corners
        corners = [
            (half_l, half_w),
            (half_l, -half_w),
            (-half_l, half_w),
            (-half_l, -half_w)
        ]

        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)

        for cx, cy in corners:
            wx = xb + cx * cos_y - cy * sin_y
            wy = yb + cx * sin_y + cy * cos_y

            # Check booth boundaries
            if not (BOOTH_MIN[0] <= wx <= BOOTH_MAX[0] and BOOTH_MIN[1] <= wy <= BOOTH_MAX[1]):
                return False, f"Base corner ({wx:.2f}, {wy:.2f}) outside booth safety boundary"

            # Check car boundary clearance
            p_corner = np.array([wx, wy, 0.15])
            d_car = point_to_box_distance(p_corner, CAR_LOWER_MIN, CAR_LOWER_MAX)
            if d_car < MIN_BASE_TO_CAR_CLEARANCE:
                return False, f"Base corner too close to car: clearance {d_car:.2f}m < {MIN_BASE_TO_CAR_CLEARANCE}m"

        return True, "Base pose clear"

    def check_robot_configuration(self, q, base_pose):
        """
        Validates full robot kinematic state (base + all 6 arm links).
        Returns: (is_safe: bool, reason: str)
        """
        # 1. Joint limits check
        if not is_joint_state_valid(q):
            return False, "One or more joint angles violate joint limits"

        # 2. Base check
        base_ok, base_msg = self.check_base_pose(base_pose)
        if not base_ok:
            return False, base_msg

        # 3. Compute 3D link positions
        links = forward_kinematics_all_links(q, base_pose)
        p_base = links['base']
        p_shoulder = links['shoulder']
        p_elbow = links['elbow']
        p_wrist1 = links['wrist_1']
        p_wrist2 = links['wrist_2']
        p_nozzle = links['nozzle']

        # 4. Check booth height limits
        for name, p in links.items():
            if name == 'spray_direction':
                continue
            if p[2] < 0.0:
                return False, f"Link {name} penetrates floor (z={p[2]:.3f}m)"
            if p[2] > BOOTH_MAX[2]:
                return False, f"Link {name} exceeds ceiling clearance (z={p[2]:.3f}m)"

        # 5. Check Nozzle Standoff Distance
        d_nozzle_lower = point_to_box_distance(p_nozzle, CAR_LOWER_MIN, CAR_LOWER_MAX)
        d_nozzle_upper = point_to_box_distance(p_nozzle, CAR_UPPER_MIN, CAR_UPPER_MAX)
        d_nozzle = min(d_nozzle_lower, d_nozzle_upper)

        if is_point_inside_car(p_nozzle):
            return False, f"Nozzle tip intersects car body! Position: {p_nozzle.round(2)}"

        if d_nozzle < MIN_NOZZLE_STANDOFF:
            return False, f"Nozzle tip violates minimum standoff: {d_nozzle:.3f}m < {MIN_NOZZLE_STANDOFF}m"

        # 6. Check Arm Link Segment Clearances
        segments = [
            ('shoulder-elbow', p_shoulder, p_elbow),
            ('elbow-wrist1', p_elbow, p_wrist1),
            ('wrist1-wrist2', p_wrist1, p_wrist2),
            ('wrist2-nozzle', p_wrist2, p_nozzle)
        ]

        for seg_name, p1, p2 in segments:
            c = segment_to_car_clearance(p1, p2, num_samples=6)
            if c < MIN_LINK_TO_CAR_CLEARANCE:
                return False, f"Arm segment '{seg_name}' violates car buffer: {c:.3f}m < {MIN_LINK_TO_CAR_CLEARANCE}m"

        return True, "Configuration collision-free"

    def check_trajectory(self, trajectory_q, base_pose):
        """
        Validates an entire sequence of arm waypoints for collisions and limit violations.
        Returns: (is_safe: bool, first_violating_index: int, reason: str)
        """
        for idx, q in enumerate(trajectory_q):
            ok, msg = self.check_robot_configuration(q, base_pose)
            if not ok:
                return False, idx, f"Waypoint {idx}: {msg}"
        return True, -1, "Trajectory collision-free"
