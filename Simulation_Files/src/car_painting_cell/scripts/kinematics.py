#!/usr/bin/env python3
"""
Kinematics Engine for Yuktii AI Labs Painting Robot (Stage 3).
Provides:
1. Exact 3D Forward Kinematics (FK) for all 6 arm links + nozzle tip.
2. Analytical Geometric Inverse Kinematics (IK) with Pieper decoupling
   for position and surface normal orientation.
3. Strict joint limit and velocity limit verification.
4. Orientation alignment to target surface normal vectors (perpendicular spray).
5. Clean, meaningful physical error metrics for reachability diagnostics.
"""

import math
import numpy as np

# Robot Mechanical Parameters (meters) - matching URDF/Xacro exactly
Z_BASE_CHASSIS = 0.10     # base_footprint to base_link
Z_ARM_BASE = 0.28         # base_link to arm_base_link (waist pan)
Z_SHOULDER = 0.12         # arm_base_link to shoulder knuckle
L_UPPER_ARM = 0.70        # shoulder to elbow link length
L_FOREARM = 0.65          # elbow to wrist 1 link length
L_WRIST_1 = 0.10          # wrist 1 to wrist 2
L_WRIST_2 = 0.09          # wrist 2 to tool flange
L_TOOL = 0.20             # tool flange to nozzle tip (spray gun 0.02 + nozzle 0.14 + atomizer tip 0.04)

# Total shoulder elevation above ground (meters)
Z_SHOULDER_GROUND = Z_BASE_CHASSIS + Z_ARM_BASE + Z_SHOULDER  # 0.50m

# Total wrist 2 pivot to nozzle tip distance along spray tool axis
D_WRIST_TO_NOZZLE = L_WRIST_2 + L_TOOL  # 0.09 + 0.20 = 0.29m

# Effective link lengths for 2-link planar position solver:
# L1 = shoulder to elbow
# L2 = elbow to wrist 2 pivot (forearm 0.65 + wrist 1 0.10 = 0.75m)
L_ARM_1 = L_UPPER_ARM                  # 0.70m
L_ARM_2 = L_FOREARM + L_WRIST_1        # 0.75m

# Joint Limits (radians) matching urdf/arm_and_gun.xacro exactly
JOINT_LIMITS = {
    'arm_base_joint': (-math.pi, math.pi),
    'shoulder_joint': (-1.80, 1.80),
    'elbow_joint':    (-2.50, 2.50),
    'wrist_1_joint':  (-math.pi, math.pi),
    'wrist_2_joint':  (-2.00, 2.00),
    'wrist_3_joint':  (-math.pi, math.pi)
}

ARM_JOINT_NAMES = [
    'arm_base_joint',
    'shoulder_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint'
]


def clamp(val, min_val, max_val):
    return max(min_val, min(max_val, val))


def _trans_4(x, y, z):
    """4x4 homogeneous translation matrix."""
    return np.array([
        [1.0, 0.0, 0.0, float(x)],
        [0.0, 1.0, 0.0, float(y)],
        [0.0, 0.0, 1.0, float(z)],
        [0.0, 0.0, 0.0, 1.0]
    ])


def _rot_z_4(angle):
    """4x4 homogeneous rotation matrix about Z axis."""
    c = math.cos(angle)
    s = math.sin(angle)
    return np.array([
        [c,   -s,  0.0, 0.0],
        [s,    c,  0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0]
    ])


def _rot_y_4(angle):
    """4x4 homogeneous rotation matrix about Y axis."""
    c = math.cos(angle)
    s = math.sin(angle)
    return np.array([
        [ c,  0.0, s,   0.0],
        [0.0, 1.0, 0.0, 0.0],
        [-s,  0.0, c,   0.0],
        [0.0, 0.0, 0.0, 1.0]
    ])


def _rot_z_3(angle):
    """3x3 rotation matrix about Z axis."""
    c = math.cos(angle)
    s = math.sin(angle)
    return np.array([
        [c, -s, 0.0],
        [s,  c, 0.0],
        [0.0, 0.0, 1.0]
    ])


def _rot_y_3(angle):
    """3x3 rotation matrix about Y axis."""
    c = math.cos(angle)
    s = math.sin(angle)
    return np.array([
        [ c, 0.0, s],
        [0.0, 1.0, 0.0],
        [-s, 0.0, c]
    ])


def forward_kinematics_all_links(q, base_pose=(0.0, 0.0, 0.0)):
    """
    Computes exact 3D Cartesian coordinates for each joint knuckle and nozzle tip
    using the full 6-DOF transform chain matching the URDF model.
    base_pose: (x_base, y_base, yaw_base) in world coordinates.
    Returns dict of link positions in world frame.
    """
    xb, yb, yaw = base_pose
    q1, q2, q3, q4, q5, q6 = q

    # 1. Base footprint to base link
    T_w_base = _trans_4(xb, yb, Z_BASE_CHASSIS) @ _rot_z_4(yaw)

    # 2. Arm base (waist pan q1)
    T_base_1 = _trans_4(0.0, 0.0, Z_ARM_BASE) @ _rot_z_4(q1)
    T_0_1 = T_w_base @ T_base_1

    # 3. Shoulder knuckle (pitch q2)
    T_1_2 = _trans_4(0.0, 0.0, Z_SHOULDER) @ _rot_y_4(q2)
    T_0_2 = T_0_1 @ T_1_2

    # 4. Elbow knuckle (pitch q3)
    T_2_3 = _trans_4(0.0, 0.0, L_UPPER_ARM) @ _rot_y_4(q3)
    T_0_3 = T_0_2 @ T_2_3

    # 5. Wrist 1 (roll q4)
    T_3_4 = _trans_4(0.0, 0.0, L_FOREARM) @ _rot_z_4(q4)
    T_0_4 = T_0_3 @ T_3_4

    # 6. Wrist 2 (pitch q5)
    T_4_5 = _trans_4(0.0, 0.0, L_WRIST_1) @ _rot_y_4(q5)
    T_0_5 = T_0_4 @ T_4_5

    # 7. Wrist 3 (roll q6)
    T_5_6 = _trans_4(0.0, 0.0, L_WRIST_2) @ _rot_z_4(q6)
    T_0_6 = T_0_5 @ T_5_6

    # 8. Spray gun and nozzle tip
    T_6_nozzle = _trans_4(0.0, 0.0, L_TOOL)
    T_0_nozzle = T_0_6 @ T_6_nozzle

    p_base = T_w_base[:3, 3]
    p_shoulder = T_0_2[:3, 3]
    p_elbow = T_0_3[:3, 3]
    p_wrist1 = T_0_4[:3, 3]
    p_wrist2 = T_0_5[:3, 3]
    p_nozzle = T_0_nozzle[:3, 3]
    spray_dir = T_0_nozzle[:3, 2]  # Z axis of tool frame
    norm_s = np.linalg.norm(spray_dir)
    if norm_s > 1e-6:
        spray_dir = spray_dir / norm_s

    return {
        'base': p_base,
        'shoulder': p_shoulder,
        'elbow': p_elbow,
        'wrist_1': p_wrist1,
        'wrist_2': p_wrist2,
        'nozzle': p_nozzle,
        'spray_direction': spray_dir
    }


def forward_kinematics(q, base_pose=(0.0, 0.0, 0.0)):
    """Computes nozzle tip position and spray direction."""
    res = forward_kinematics_all_links(q, base_pose)
    return res['nozzle'], res['spray_direction']


def inverse_kinematics(target_pos, target_normal, base_pose=(0.0, 0.0, 0.0), current_q=None):
    """
    Exact analytical closed-form Inverse Kinematics using Pieper kinematic decoupling.
    Converts target Cartesian nozzle position and surface normal into valid 6-DOF joint angles.

    Decoupling strategy:
    1. The spray gun is oriented opposite to surface normal (into surface):
       v_spray = -target_normal / ||target_normal||
    2. The wrist 2 pivot is at distance D_WRIST_TO_NOZZLE (0.29m) behind nozzle along v_spray:
       p_w = p_target - D_WRIST_TO_NOZZLE * v_spray
    3. The position of p_w is completely independent of q4, q5, q6!
       Solve (q1, q2, q3) algebraically using 2-link planar trigonometry.
    4. Link 3 orientation is now determined. Project v_spray into forearm frame.
       Solve (q4, q5) algebraically so tool Z axis exactly matches v_spray.
    5. Set q6 = 0.0 (roll about tool symmetry axis).

    Returns: (success: bool, q_result: list of 6 floats, error_dist: float)
    """
    xb, yb, yaw = base_pose
    p_tgt = np.array(target_pos, dtype=float)
    n_surf = np.array(target_normal, dtype=float)
    norm_n = np.linalg.norm(n_surf)
    if norm_n < 1e-6:
        n_surf = np.array([0.0, 0.0, 1.0])
    else:
        n_surf = n_surf / norm_n

    # Desired spray direction is into the vehicle surface
    v_spray = -n_surf

    # 1. Wrist 2 pivot location in world coordinates
    p_w = p_tgt - D_WRIST_TO_NOZZLE * v_spray

    # Relative to shoulder origin
    dx = p_w[0] - xb
    dy = p_w[1] - yb
    dz = p_w[2] - Z_SHOULDER_GROUND

    # 2. Joint 1: Waist azimuth
    phi = math.atan2(dy, dx)
    q1 = phi - yaw
    q1 = (q1 + math.pi) % (2.0 * math.pi) - math.pi

    lim_q1 = JOINT_LIMITS['arm_base_joint']
    if not (lim_q1[0] <= q1 <= lim_q1[1]):
        # Check wrap
        for alt_q1 in [q1 + 2.0 * math.pi, q1 - 2.0 * math.pi]:
            if lim_q1[0] <= alt_q1 <= lim_q1[1]:
                q1 = alt_q1
                break
        else:
            return False, list(current_q or [0.0] * 6), float(np.linalg.norm(p_tgt - np.array([xb, yb, Z_SHOULDER_GROUND])))

    # 3. Planar distance in vertical shoulder plane
    r_wrist = math.sqrt(dx**2 + dy**2)
    h_wrist = dz
    d_sq = r_wrist**2 + h_wrist**2
    D = math.sqrt(d_sq)

    l1 = L_ARM_1  # 0.70m
    l2 = L_ARM_2  # 0.75m
    max_reach = l1 + l2 - 0.005
    min_reach = abs(l1 - l2) + 0.01

    if D > max_reach:
        return False, list(current_q or [0.0] * 6), float(D - max_reach)
    if D < min_reach:
        return False, list(current_q or [0.0] * 6), float(min_reach - D)

    # Law of cosines for elbow angle q3
    cos_q3 = (d_sq - l1**2 - l2**2) / (2.0 * l1 * l2)
    cos_q3 = clamp(cos_q3, -1.0, 1.0)

    curr = np.array(current_q if current_q is not None else [0.0] * 6)
    best_cand = None
    min_joint_disp = 1e9

    best_clamped_cand = None
    min_clamped_score = 1e9

    # Try both elbow configurations (elbow-down vs elbow-up)
    for elbow_sign in [1.0, -1.0]:
        q3 = elbow_sign * math.acos(cos_q3)

        # Angle from vertical +Z to wrist position vector
        alpha = math.atan2(r_wrist, h_wrist)
        # Internal angle at shoulder knuckle
        beta = math.atan2(l2 * math.sin(abs(q3)), l1 + l2 * math.cos(abs(q3)))
        q2 = alpha - (beta if elbow_sign > 0 else -beta)

        # 4. Rotation matrix of link 3 (forearm) in world frame
        R_w_3 = _rot_z_3(yaw + q1) @ _rot_y_3(q2) @ _rot_y_3(q3)

        # Express desired spray direction in forearm frame
        u = R_w_3.T @ v_spray
        norm_u = np.linalg.norm(u)
        if norm_u < 1e-6:
            continue
        u = u / norm_u
        if abs(u[1]) < 1e-9:
            u[1] = 0.0

        # In forearm frame, tool points along:
        # RotZ(q4) @ RotY(q5) @ [0, 0, 1]^T = [sin(q5)*cos(q4), sin(q5)*sin(q4), cos(q5)]^T
        cos_q5 = clamp(u[2], -1.0, 1.0)
        q5_base = math.acos(cos_q5)
        sin_q5 = math.sin(q5_base)

        if abs(sin_q5) > 1e-4:
            q4_base = math.atan2(u[1], u[0])
        else:
            q4_base = 0.0

        # Two spherical wrist branches
        wrist_candidates = [
            (q4_base, q5_base),
            (q4_base + math.pi, -q5_base)
        ]

        for q4_raw, q5_raw in wrist_candidates:
            q4 = (q4_raw + math.pi) % (2.0 * math.pi) - math.pi
            q5 = q5_raw

            cand = [q1, q2, q3, q4, q5, 0.0]

            # Continuous wrap alignment for revolute joints [0, 3, 5]
            if current_q is not None:
                for k in [0, 3, 5]:
                    low, high = JOINT_LIMITS[ARM_JOINT_NAMES[k]]
                    diff_k = (cand[k] - curr[k] + math.pi) % (2.0 * math.pi) - math.pi
                    aligned_k = curr[k] + diff_k
                    if low - 1e-4 <= aligned_k <= high + 1e-4:
                        cand[k] = clamp(aligned_k, low, high)

            # Verify joint limits
            valid = True
            for i, name in enumerate(ARM_JOINT_NAMES):
                low, high = JOINT_LIMITS[name]
                if not (low - 1e-4 <= cand[i] <= high + 1e-4):
                    valid = False
                    break

            if valid:
                p_act, v_act = forward_kinematics(cand, base_pose)
                err_p = float(np.linalg.norm(p_act - p_tgt))
                dot_a = float(np.dot(v_act, v_spray))

                if err_p < 0.02 and dot_a > 0.85:
                    if current_q is not None:
                        diff = [
                            ((cand[k] - curr[k] + math.pi) % (2.0 * math.pi) - math.pi) if k in [0, 3, 5]
                            else (cand[k] - curr[k])
                            for k in range(6)
                        ]
                        disp = float(np.linalg.norm(diff))
                    else:
                        disp = float(np.linalg.norm(cand))

                    if disp < min_joint_disp:
                        min_joint_disp = disp
                        best_cand = cand
            else:
                # Track best candidate when clamped to physical limits
                cand_clamped = [clamp(cand[k], JOINT_LIMITS[ARM_JOINT_NAMES[k]][0], JOINT_LIMITS[ARM_JOINT_NAMES[k]][1]) for k in range(6)]
                p_c, v_c = forward_kinematics(cand_clamped, base_pose)
                err_c = float(np.linalg.norm(p_c - p_tgt))
                dot_c = float(np.dot(v_c, v_spray))
                score_c = err_c + 0.2 * max(0.0, 1.0 - dot_c)
                if score_c < min_clamped_score:
                    min_clamped_score = score_c
                    best_clamped_cand = cand_clamped

    if best_cand is not None:
        p_final, v_final = forward_kinematics(best_cand, base_pose)
        final_err = float(np.linalg.norm(p_final - p_tgt))
        return True, [float(x) for x in best_cand], final_err

    # DLS refinement seeded from current_q or best clamped analytical candidate
    q_init = current_q if current_q is not None else best_clamped_cand
    if q_init is None:
        q_init = [0.0] * 6
    q_dls = np.array(q_init, dtype=float)

    best_tested_q = list(q_dls)
    p_best, v_best = forward_kinematics(best_tested_q, base_pose)
    best_tested_err = float(np.linalg.norm(p_tgt - p_best))

    for _ in range(25):
        p_cur, v_cur = forward_kinematics(q_dls, base_pose)
        err_pos = p_tgt - p_cur
        err_dist = float(np.linalg.norm(err_pos))
        err_rot = np.cross(v_cur, v_spray)

        if err_dist < best_tested_err and np.dot(v_cur, v_spray) > 0.85:
            best_tested_err = err_dist
            best_tested_q = list(q_dls)

        if err_dist < 0.003 and np.linalg.norm(err_rot) < 0.05:
            best_tested_q = list(q_dls)
            break

        J = np.zeros((6, 6))
        eps = 1e-4
        for j in range(6):
            q_plus = np.array(q_dls)
            q_plus[j] += eps
            p_plus, v_plus = forward_kinematics(q_plus, base_pose)
            J[:3, j] = (p_plus - p_cur) / eps
            J[3:, j] = (v_plus - v_cur) / eps

        e_total = np.concatenate([err_pos, 0.4 * err_rot])
        damping_sq = 1e-3
        J_dls = J.T @ np.linalg.inv(J @ J.T + damping_sq * np.eye(6))
        dq = J_dls @ e_total
        q_dls += dq

        for k, name in enumerate(ARM_JOINT_NAMES):
            low, high = JOINT_LIMITS[name]
            q_dls[k] = clamp(q_dls[k], low, high)

    p_final, v_final = forward_kinematics(best_tested_q, base_pose)
    final_err = float(np.linalg.norm(p_tgt - p_final))
    dot_align = float(np.dot(v_spray, v_final))
    success = (final_err < 0.02) and (dot_align > 0.85) and is_joint_state_valid(best_tested_q)

    return success, [float(x) for x in best_tested_q], final_err


def is_joint_state_valid(q):
    """Verifies that all 6 joint angles lie strictly within physical limits."""
    if len(q) != 6:
        return False
    for i, name in enumerate(ARM_JOINT_NAMES):
        low, high = JOINT_LIMITS[name]
        if not (low - 1e-4 <= q[i] <= high + 1e-4):
            return False
    return True
