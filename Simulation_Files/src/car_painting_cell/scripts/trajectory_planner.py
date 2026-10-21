#!/usr/bin/env python3
"""
Autonomous Trajectory Planner for Yuktii AI Labs Painting Robot (Stage 3).
Generates smooth, parallel raster spray passes for:
1. Front Hood (Transverse sweeps with downward perpendicular spray)
2. Left Flank Doors & Fenders (Longitudinal sweeps with inward perpendicular spray)
3. Roof Panel & Cant Rail (Longitudinal sweeps with inward-downward spray)
4. Rear Trunk & Bumper (Trunk top and rear bumper fascia sweeps)
5. Right Flank Doors & Fenders (Longitudinal sweeps with inward perpendicular spray)
6. Front Bumper & Grille (Transverse bumper sweeps)

Key Capabilities:
- Configurable spray speed, pass spacing, and nozzle-to-surface standoff distance.
- Pre-computation and validation of all Cartesian points into joint-space trajectories via IK.
- Automated collision checking for base, arm links, and nozzle standoff.
- Automatic spray trigger flags for each waypoint (SPRAY ON during active sweeps, OFF during re-positioning).
- Coordinated mobile base positioning matching designated cell corridors and safety buffers.
"""

import math
import numpy as np
from kinematics import inverse_kinematics, forward_kinematics, is_joint_state_valid
from collision_checker import CollisionChecker


class TrajectoryPass:
    def __init__(self, p_start, p_end, normal, is_spray_active, speed=0.20, base_station=None,
                 normal_end=None, q_target=None):
        self.p_start = np.array(p_start, dtype=float)
        self.p_end = np.array(p_end, dtype=float)
        self.normal = np.array(normal, dtype=float) / (np.linalg.norm(normal) + 1e-9)
        if normal_end is not None:
            self.normal_end = np.array(normal_end, dtype=float) / (np.linalg.norm(normal_end) + 1e-9)
        else:
            self.normal_end = np.array(self.normal, copy=True)
        self.is_spray_active = is_spray_active
        self.speed = speed
        self.base_station = base_station  # Optional per-pass base station (x, y, yaw)
        self.q_target = list(q_target) if q_target is not None else None


def validate_base_station(actual_pose, required_station, pos_tol=0.12, yaw_tol=math.radians(6.0)):
    """
    Validates whether the measured base pose matches the designated station within tolerances.

    Parameters:
        actual_pose: (x, y, yaw) measured odometry pose in World/Odom frame.
        required_station: (x, y, yaw) target base station in World/Odom frame.
        pos_tol: maximum allowable position error (meters, default 0.08m = 80mm).
        yaw_tol: maximum allowable yaw error (radians, default 5.0 deg).

    Returns:
        (is_valid: bool, pos_err: float, yaw_err: float, message: str)
    """
    dx = actual_pose[0] - required_station[0]
    dy = actual_pose[1] - required_station[1]
    pos_err = math.hypot(dx, dy)

    # Angular error normalized to [-pi, pi]
    yaw_diff = actual_pose[2] - required_station[2]
    yaw_err = abs(math.atan2(math.sin(yaw_diff), math.cos(yaw_diff)))

    is_valid = (pos_err <= pos_tol) and (yaw_err <= yaw_tol)
    if is_valid:
        msg = (f"Base at station: pos_err={pos_err*1000.0:.1f}mm <= {pos_tol*1000.0:.1f}mm, "
               f"yaw_err={math.degrees(yaw_err):.2f}deg <= {math.degrees(yaw_tol):.2f}deg")
    else:
        errs = []
        if pos_err > pos_tol:
            errs.append(f"position error {pos_err*1000.0:.1f}mm exceeds {pos_tol*1000.0:.1f}mm")
        if yaw_err > yaw_tol:
            errs.append(f"yaw error {math.degrees(yaw_err):.2f}deg exceeds {math.degrees(yaw_tol):.2f}deg")
        msg = f"Station mismatch: {'; '.join(errs)}"

    return is_valid, pos_err, yaw_err, msg


class SurfaceTrajectoryPlan:
    def __init__(self, surface_name, base_station, passes, approach_pose=None, retract_pose=None):
        self.surface_name = surface_name
        self.base_station = base_station  # Default base station (x, y, yaw)
        self.passes = passes              # list of TrajectoryPass
        self.approach_pose = approach_pose
        self.retract_pose = retract_pose

        # Computed waypoints
        self.joint_waypoints = []       # list of list[6] joint angles
        self.spray_flags = []           # list of bool
        self.time_stamps = []           # list of float (cumulative seconds)
        self.cartesian_waypoints = []   # list of np.array([x, y, z])
        self.base_waypoints = []        # list of (x, y, yaw)


class PaintingTrajectoryPlanner:
    def __init__(self, standoff_dist=0.26, pass_speed=0.22):
        self.standoff_dist = standoff_dist
        self.pass_speed = pass_speed
        self.collision_checker = CollisionChecker()

    def generate_surface_plan(self, surface_name, base_station=None):
        """Generates the surface trajectory specification for the requested region."""
        s = surface_name.lower()
        if s in ['hood', 'front_hood']:
            plan = self._plan_hood()
        elif s in ['left', 'left_side', 'left_flank']:
            plan = self._plan_left_side()
        elif s in ['roof', 'car_roof']:
            plan = self._plan_roof()
        elif s in ['rear', 'trunk', 'rear_bumper']:
            plan = self._plan_rear()
        elif s in ['right', 'right_side', 'right_flank']:
            plan = self._plan_right_side()
        elif s in ['front', 'front_bumper']:
            plan = self._plan_front_bumper()
        else:
            raise ValueError(f"Unknown surface name: {surface_name}")

        if base_station is not None:
            plan.base_station = base_station
        return plan

    def _plan_hood(self):
        # Base stationed across front of car at X=3.00m (yaw = -pi/2)
        # Half-width is 0.30m, clearance to bumper (2.42m) is 3.00 - 0.30 - 2.42 = 0.28m >= 0.25m
        base_station = (3.00, 0.0, -math.pi / 2.0)
        normal = [0.0, 0.0, 1.0]  # Pointing up, spray points down
        z_nozzle = 0.78 + self.standoff_dist  # 0.78m + 0.26m = 1.04m

        passes = [
            # Pass 1: Front edge of hood (X = 2.20m)
            TrajectoryPass([2.20, -0.55, z_nozzle], [2.20, +0.55, z_nozzle], normal, is_spray_active=True, speed=self.pass_speed),
            # Transition to mid hood
            TrajectoryPass([2.20, +0.55, z_nozzle], [2.07, +0.50, z_nozzle], normal, is_spray_active=False, speed=self.pass_speed * 1.5),
            # Pass 2: Middle of hood (X = 2.07m)
            TrajectoryPass([2.07, +0.50, z_nozzle], [2.07, -0.50, z_nozzle], normal, is_spray_active=True, speed=self.pass_speed),
            # Transition to upper hood
            TrajectoryPass([2.07, -0.50, z_nozzle], [1.95, -0.45, z_nozzle], normal, is_spray_active=False, speed=self.pass_speed * 1.5),
            # Pass 3: Upper hood (X = 1.95m)
            TrajectoryPass([1.95, -0.45, z_nozzle], [1.95, +0.45, z_nozzle], normal, is_spray_active=True, speed=self.pass_speed),
        ]
        return SurfaceTrajectoryPlan('hood', base_station, passes)

    def _plan_left_side(self):
        # Base stationed on left cyan corridor facing car doors: Y = 2.00m, yaw = -pi/2
        # Clearance to car flank (0.92m): 2.00 - 0.45 - 0.92 = 0.63m >= 0.25m
        base_station = (0.0, 2.00, -math.pi / 2.0)
        normal = [0.0, 1.0, 0.0]  # Outward +Y, spray points -Y
        y_nozzle = 0.90 + self.standoff_dist + 0.02  # 0.90m + 0.28m = 1.18m

        passes = [
            # Pass 1: Lower doors / rocker panel (Z = 0.55m)
            TrajectoryPass([+0.80, y_nozzle, 0.55], [-0.80, y_nozzle, 0.55], normal, is_spray_active=True, speed=self.pass_speed),
            # Transition up to beltline
            TrajectoryPass([-0.80, y_nozzle, 0.55], [-0.80, y_nozzle + 0.04, 0.85], normal, is_spray_active=False, speed=self.pass_speed * 1.5),
            # Pass 2: Upper doors / beltline (Z = 0.85m)
            TrajectoryPass([-0.80, y_nozzle, 0.85], [+0.80, y_nozzle, 0.85], normal, is_spray_active=True, speed=self.pass_speed),
        ]
        return SurfaceTrajectoryPlan('left_side', base_station, passes)

    def _plan_roof(self):
        # Base stationed on left corridor: Y = 1.85m, yaw = -pi/2
        # Clearance: 1.85 - 0.45 - 0.92 = 0.48m >= 0.25m
        base_station = (0.0, 1.85, -math.pi / 2.0)
        # Inward and downward spray angle (20 deg from horizontal) pointing at upper roofline / cant rail
        tilt = math.radians(20)
        normal = [0.0, math.cos(tilt), math.sin(tilt)]
        y_nozzle = 1.05
        z_nozzle = 1.45

        passes = [
            # Pass 1: Upper roofline / cant rail (X from +0.65m to -0.65m)
            TrajectoryPass([+0.65, y_nozzle, z_nozzle], [-0.65, y_nozzle, z_nozzle], normal, is_spray_active=True, speed=self.pass_speed),
            # Transition slightly inward
            TrajectoryPass([-0.65, y_nozzle, z_nozzle], [-0.65, y_nozzle - 0.05, z_nozzle + 0.05], normal, is_spray_active=False, speed=self.pass_speed * 1.5),
            # Pass 2: Upper roofline return pass
            TrajectoryPass([-0.65, y_nozzle - 0.05, z_nozzle + 0.05], [+0.65, y_nozzle - 0.05, z_nozzle + 0.05], normal, is_spray_active=True, speed=self.pass_speed),
        ]
        return SurfaceTrajectoryPlan('roof', base_station, passes)

    def _plan_rear(self):
        # Base stationed at standard cell rear station matching state machine STATIONS['REAR']:
        # Base: (-3.50, 0.0, 0.0) -> clearance to bumper (2.42m): 3.50 - 0.45 - 2.42 = 0.63m >= 0.25m
        base_station = (-3.50, 0.0, 0.0)

        normal_bumper = [-1.0, 0.0, 0.0]  # Outward rearward, spray points +X
        tilt = math.radians(25.0)          # 25 deg forward-downward spray angle matching roof cant rail convention
        normal_trunk = [-math.sin(tilt), 0.0, math.cos(tilt)]

        x_bumper = -2.43 - self.standoff_dist  # -2.43m - 0.26m = -2.69m
        z_trunk = 0.79 + self.standoff_dist    # 0.79m + 0.26m = 1.05m
        x_trunk = -2.28                        # Over trunk lid panel with safe forward reach

        # Target entry point for trunk approach in safe buffer behind vehicle
        p_reorient_end = [-2.70, +0.45, z_trunk]

        # Seed IK with Candidate 1 downward branch (q5 > 0, q4 near 0) for smooth trunk coverage
        ok, q_trunk_entry, err = inverse_kinematics(
            p_reorient_end, normal_trunk, base_station, current_q=[0.0, 0.5, 1.0, 0.0, 1.4, 0.0]
        )
        if not ok:
            q_trunk_entry = None

        passes = [
            # Pass 1: Rear Bumper Fascia Sweep (spray ON)
            TrajectoryPass([x_bumper, -0.60, 0.45], [x_bumper, +0.60, 0.45], normal_bumper,
                           is_spray_active=True, speed=self.pass_speed, base_station=base_station),
            # Pass 2: Vertical Retract / Raise to Trunk Height in Safe Rear Air Buffer (spray OFF)
            TrajectoryPass([x_bumper, +0.60, 0.45], [x_bumper - 0.03, +0.45, z_trunk], normal_bumper,
                           is_spray_active=False, speed=self.pass_speed * 1.5, base_station=base_station),
            # Pass 3: Deliberate, Spray-Disabled Reorientation Transition in Open Air (spray OFF)
            TrajectoryPass([x_bumper - 0.03, +0.45, z_trunk], p_reorient_end, normal_bumper,
                           is_spray_active=False, speed=self.pass_speed * 1.5, base_station=base_station,
                           normal_end=normal_trunk, q_target=q_trunk_entry),
            # Pass 4: Forward Approach over Trunk Lid (spray OFF)
            TrajectoryPass(p_reorient_end, [x_trunk, +0.45, z_trunk], normal_trunk,
                           is_spray_active=False, speed=self.pass_speed * 1.5, base_station=base_station),
            # Pass 5: Trunk Lid Sweep (spray ON)
            TrajectoryPass([x_trunk, +0.45, z_trunk], [x_trunk, -0.45, z_trunk], normal_trunk,
                           is_spray_active=True, speed=self.pass_speed, base_station=base_station),
        ]
        return SurfaceTrajectoryPlan('rear', base_station, passes)

    def _plan_right_side(self):
        # Base stationed on right cyan corridor facing car doors: Y = -2.00m, yaw = pi/2
        # Clearance: 2.00 - 0.45 - 0.92 = 0.63m >= 0.25m
        base_station = (0.0, -2.00, math.pi / 2.0)
        normal = [0.0, -1.0, 0.0]  # Outward -Y, spray points +Y
        y_nozzle = -0.90 - self.standoff_dist - 0.02  # -1.18m

        passes = [
            # Pass 1: Lower doors / rocker (Z = 0.55m)
            TrajectoryPass([-0.80, y_nozzle, 0.55], [+0.80, y_nozzle, 0.55], normal, is_spray_active=True, speed=self.pass_speed),
            # Transition up to beltline
            TrajectoryPass([+0.80, y_nozzle, 0.55], [+0.80, y_nozzle - 0.04, 0.85], normal, is_spray_active=False, speed=self.pass_speed * 1.5),
            # Pass 2: Upper doors / beltline (Z = 0.85m)
            TrajectoryPass([+0.80, y_nozzle, 0.85], [-0.80, y_nozzle, 0.85], normal, is_spray_active=True, speed=self.pass_speed),
        ]
        return SurfaceTrajectoryPlan('right_side', base_station, passes)

    def _plan_front_bumper(self):
        # Base stationed in front facing rear at X = 3.50m (yaw = -pi)
        # Clearance: 3.50 - 0.45 - 2.42 = 0.63m >= 0.25m
        base_station = (3.50, 0.0, -math.pi)
        normal_bumper = [1.0, 0.0, 0.0]  # Outward +X, spray points -X
        x_bumper = 2.43 + self.standoff_dist  # 2.69m

        passes = [
            # Pass 1: Front Bumper / Grille Sweep (Y from -0.65m to +0.65m)
            TrajectoryPass([x_bumper, -0.65, 0.45], [x_bumper, +0.65, 0.45], normal_bumper,
                           is_spray_active=True, speed=self.pass_speed),
        ]
        return SurfaceTrajectoryPlan('front_bumper', base_station, passes)

    def discretize_and_solve(self, plan, waypoint_step=0.08, base_station=None):
        """
        Discretizes Cartesian passes into fine waypoints, solves IK for each,
        and verifies collision-free motion.
        Supports per-pass base station coordination.
        If base_station is provided, passes without an explicit station will use it.
        Returns: (success: bool, discretized_plan: SurfaceTrajectoryPlan, report: str)
        """
        all_q = []
        all_spray = []
        all_times = []
        all_cartesian = []
        all_base = []

        curr_time = 0.0
        last_q = [0.0] * 6

        for p_idx, p in enumerate(plan.passes):
            if base_station is not None and p.base_station is None:
                active_base = base_station
            else:
                active_base = p.base_station if p.base_station is not None else plan.base_station

            # Verify base pose clearance
            ok_base, base_msg = self.collision_checker.check_base_pose(active_base)
            if not ok_base:
                return False, plan, f"Base clearance check failed at Pass {p_idx+1}: {base_msg}"

            if getattr(p, 'q_target', None) is not None:
                # Deliberate, spray-disabled reorientation transition in open safety buffer
                q_tgt = p.q_target
                max_diff = max(abs(q_tgt[k] - last_q[k]) for k in range(6))
                num_pts = max(5, int(math.ceil(max_diff / math.radians(12.0))))
                start_q = list(last_q)

                for i in range(num_pts):
                    alpha = i / float(num_pts - 1) if num_pts > 1 else 1.0
                    q_interp = [(1.0 - alpha) * start_q[k] + alpha * q_tgt[k] for k in range(6)]

                    if not is_joint_state_valid(q_interp):
                        return False, plan, f"Joint limit violated during reorientation at Pass {p_idx+1}, pt {i+1}"

                    ok_col, reason = self.collision_checker.check_robot_configuration(q_interp, active_base)
                    if not ok_col:
                        return False, plan, f"Collision check failed during reorientation at Pass {p_idx+1}, pt {i+1}: {reason}"

                    p_act, v_act = forward_kinematics(q_interp, active_base)
                    dt = 0.20
                    curr_time += dt

                    all_q.append(q_interp)
                    all_spray.append(p.is_spray_active)
                    all_times.append(curr_time)
                    all_cartesian.append(p_act)
                    all_base.append(active_base)

                last_q = list(q_tgt)
            else:
                dist = float(np.linalg.norm(p.p_end - p.p_start))
                num_pts = max(3, int(math.ceil(dist / waypoint_step)))

                for i in range(num_pts):
                    alpha = i / float(num_pts - 1) if num_pts > 1 else 0.0
                    p_interp = p.p_start + alpha * (p.p_end - p.p_start)
                    norm_interp = (1.0 - alpha) * p.normal + alpha * p.normal_end
                    norm_len = float(np.linalg.norm(norm_interp))
                    if norm_len > 1e-6:
                        norm_interp = norm_interp / norm_len
                    else:
                        norm_interp = p.normal

                    ok_ik, q_sol, err_ik = inverse_kinematics(
                        p_interp, norm_interp, active_base, current_q=last_q
                    )

                    if not ok_ik:
                        return False, plan, f"IK failed at Pass {p_idx+1}, pt {i+1}/{num_pts}: target={p_interp.round(2)}, err={err_ik:.4f}m"

                    # Collision check
                    ok_col, reason = self.collision_checker.check_robot_configuration(q_sol, active_base)
                    if not ok_col:
                        return False, plan, f"Collision check failed at Pass {p_idx+1}, pt {i+1}: {reason}"

                    dt = (dist / num_pts) / p.speed if num_pts > 0 else 0.1
                    curr_time += dt

                    all_q.append(q_sol)
                    all_spray.append(p.is_spray_active)
                    all_times.append(curr_time)
                    all_cartesian.append(p_interp)
                    all_base.append(active_base)
                    last_q = list(q_sol)

        plan.joint_waypoints = all_q
        plan.spray_flags = all_spray
        plan.time_stamps = all_times
        plan.cartesian_waypoints = all_cartesian
        plan.base_waypoints = all_base

        return True, plan, f"Successfully planned {len(all_q)} collision-free waypoints over {curr_time:.1f}s."
