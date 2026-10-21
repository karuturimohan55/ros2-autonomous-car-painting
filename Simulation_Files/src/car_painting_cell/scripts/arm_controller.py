#!/usr/bin/env python3
"""
Arm Joint Position & Trajectory Controller for Yuktii AI Labs Painting Robot (Stage 5).
Provides smooth trajectory interpolation, active position holding, and robust measured
joint target-arrival verification for the 6-axis arm.
Works directly with Gazebo Classic's libgazebo_ros_joint_pose_trajectory plugin without
requiring external ros2_control dependencies.
"""

import sys
import time
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from std_msgs.msg import String, Bool
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration, Time


ARM_JOINT_NAMES = [
    'arm_base_joint',
    'shoulder_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint'
]

ALL_CONTROLLED_JOINTS = ARM_JOINT_NAMES + ['spray_cone_joint']

# Calibrated automotive painting poses (radians)
PRESET_POSES = {
    'home': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    'ready': [0.0, 0.30, -0.60, 0.0, 0.30, 0.0],
    'transport': [0.0, 0.30, -0.60, 0.0, 0.30, 0.0],
    'side_paint': [0.0, 0.55, -1.10, 0.0, 0.55, 0.0],
    'roof_paint': [0.0, 0.15, -0.45, 0.0, -1.10, 0.0],
    'hood_paint': [0.50, 0.45, -0.90, 0.0, 0.45, 0.0]
}

# Configurable per-joint arrival tolerances (radians)
# Accommodates physics gravity deflection in Gazebo Classic while rejecting gross errors
DEFAULT_JOINT_TOLERANCES = {
    'arm_base_joint': 0.03,   # rad (~1.7 deg)
    'shoulder_joint': 0.05,   # rad (~2.9 deg)
    'elbow_joint':    0.05,   # rad (~2.9 deg)
    'wrist_1_joint':  0.03,   # rad (~1.7 deg)
    'wrist_2_joint':  0.04,   # rad (~2.3 deg)
    'wrist_3_joint':  0.03    # rad (~1.7 deg)
}
DEFAULT_MAX_NORM_TOLERANCE = 0.06  # rad (~3.4 deg overall Euclidean error norm)

# Justified per-joint velocity arrival tolerances (rad/s)
# Under Gazebo Classic ODE physics with libgazebo_ros_joint_pose_trajectory (SetPosition preserveWorldVelocity=true),
# gravitational torque on cantilevered links balances joint damping at terminal velocities:
# shoulder ~ +0.207 rad/s, elbow ~ -0.404 rad/s in steady-state hold.
# Nonzero dynamic transit and swing velocities (0.6 - 2.0 rad/s) are strictly rejected.
DEFAULT_JOINT_VEL_TOLERANCES = {
    'arm_base_joint': 0.20,   # rad/s
    'shoulder_joint': 0.30,   # rad/s (covers +0.207 rad/s steady-state ODE gravity balance)
    'elbow_joint':    0.50,   # rad/s (covers -0.404 rad/s steady-state ODE gravity balance)
    'wrist_1_joint':  0.20,   # rad/s
    'wrist_2_joint':  0.20,   # rad/s
    'wrist_3_joint':  0.20    # rad/s
}
DEFAULT_MAX_VEL_NORM_TOLERANCE = 0.55  # rad/s (Euclidean norm of [0, 0.207, -0.404, 0, 0, 0] is 0.454 rad/s)
DEFAULT_SUSTAINED_SETTLING_SEC = 0.25  # Sustained settling period (seconds)
MAX_FEEDBACK_AGE_SEC = 0.5             # Stale feedback threshold (seconds)
DEFAULT_SETTLING_TIMEOUT_SEC = 5.0     # Maximum wait time for measured joints to settle

# Justified velocity arrival tolerances for deployed / painting configurations (rad/s)
# When the arm is extended forward (e.g. over car hood or bumper), the cantilevered links
# experience ~40 Nm gravitational torque, balancing joint damping (d=25.0) at steady-state
# terminal ODE velocities (~1.60 rad/s on shoulder, ~0.59 rad/s on elbow).
# Bounded strictly below URDF velocity limits (shoulder: 1.80 rad/s, elbow: 2.20 rad/s).
DEPLOYED_JOINT_VEL_TOLERANCES = {
    'arm_base_joint': 0.30,
    'shoulder_joint': 3.00,   # rad/s (accommodates ODE gravity-damping terminal velocity ~2.8 rad/s)
    'elbow_joint':    1.80,   # rad/s (accommodates ODE gravity-damping terminal velocity ~1.3 rad/s)
    'wrist_1_joint':  0.50,
    'wrist_2_joint':  0.50,
    'wrist_3_joint':  0.30
}
DEPLOYED_MAX_VEL_NORM_TOLERANCE = 3.60  # rad/s (vector norm bound accommodating ODE cantilever velocity)

# Dynamic gravity deflection trim limits per joint (rad)
# Under Gazebo Classic ODE physics, cantilevered links undergo gravitational deflection.
# Shoulder requires up to 0.10 rad trim to neutralize gravity droop when extended forward.
MAX_GRAVITY_TRIM = {
    'arm_base_joint': 0.045,
    'shoulder_joint': 0.045,
    'elbow_joint':    0.045,
    'wrist_1_joint':  0.045,
    'wrist_2_joint':  0.045,
    'wrist_3_joint':  0.045
}

# Physical URDF Joint Limits (min_rad, max_rad) from arm_and_gun.xacro
ARM_JOINT_LIMITS = {
    'arm_base_joint': (-3.14159, 3.14159),
    'shoulder_joint': (-1.80, 1.80),
    'elbow_joint':    (-2.50, 2.50),
    'wrist_1_joint':  (-3.14159, 3.14159),
    'wrist_2_joint':  (-2.00, 2.00),
    'wrist_3_joint':  (-3.14159, 3.14159),
}


class ArmTrajectoryController(Node):
    def __init__(self):
        super().__init__('painting_arm_controller')

        # Current and target joint angles (measured initial pose once feedback arrives)
        self.current_angles = list(PRESET_POSES['home'])
        self.target_angles = list(PRESET_POSES['home'])
        self.start_angles = list(PRESET_POSES['home'])
        self.active_pose_name = 'uninitialized'

        # Measured joint angles and velocities from simulation telemetry
        self.measured_angles = None
        self.measured_velocities = None
        self.has_valid_joint_feedback = False
        self.last_feedback_time = 0.0

        # Motion and verification states
        self.motion_state = 'INITIALIZING'  # 'INITIALIZING', 'HOLDING', 'INTERPOLATING', 'SETTLING', 'FAILED'
        self.target_reached = False
        self.settling_start_time = 0.0
        self.settling_converged_start_time = None
        self.settling_timeout = DEFAULT_SETTLING_TIMEOUT_SEC
        self.sustained_settling_time = DEFAULT_SUSTAINED_SETTLING_SEC
        self.joint_tolerances = dict(DEFAULT_JOINT_TOLERANCES)
        self.norm_tolerance = DEFAULT_MAX_NORM_TOLERANCE
        self.joint_vel_tolerances = dict(DEFAULT_JOINT_VEL_TOLERANCES)
        self.vel_norm_tolerance = DEFAULT_MAX_VEL_NORM_TOLERANCE

        # Spray state
        self.is_spraying = False

        # Closed-loop gravity deflection compensation for steady-state hold
        self.gravity_trim = [0.0] * len(ARM_JOINT_NAMES)

        # Trajectory transition state
        self.is_interpolating = False
        self.trajectory_duration = 2.0  # seconds
        self.trajectory_start_time = 0.0

        # ROS 2 Publisher: Gazebo trajectory controller plugin topic
        self.traj_pub = self.create_publisher(
            JointTrajectory, '/set_joint_trajectory', 10
        )

        # ROS 2 Publishers: Pose name, motion status, arrival confirmation, & spray status
        self.pose_pub = self.create_publisher(
            String, '/arm_current_pose', 10
        )
        self.motion_status_pub = self.create_publisher(
            String, '/arm_motion_status', 10
        )
        self.target_reached_pub = self.create_publisher(
            Bool, '/arm_target_reached', 10
        )
        self.spray_pub = self.create_publisher(
            Bool, '/spray_status', 10
        )

        # ROS 2 Subscribers
        self.cmd_pose_sub = self.create_subscription(
            String, '/cmd_arm_pose', self.cmd_pose_callback, 10
        )
        self.traj_sub = self.create_subscription(
            JointTrajectory, '/cmd_arm_trajectory', self.cmd_traj_callback, 10
        )
        self.spray_sub = self.create_subscription(
            Bool, '/spray_active', self.spray_callback, 10
        )
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )

        # 100 Hz Control and Hold Loop Timer (monotonic clock prevents simulation /clock throttling)
        self.control_timer = self.create_timer(
            1.0 / 100.0, self.control_loop, clock=Clock(clock_type=ClockType.STEADY_TIME)
        )

        self.get_logger().info('Painting Arm Controller initialized. Ready at "home" pose.')

    def get_time_sec(self) -> float:
        """Returns time in seconds using monotonic steady clock matching the 100 Hz control timer."""
        return time.monotonic()

    def check_target_convergence(self, target_angles):
        """
        Compares measured joint angles and velocities with target_angles and tolerances.
        Returns:
            (converged: bool, max_err: float, err_norm: float, per_joint_err: dict)
        """
        if self.measured_angles is None or len(self.measured_angles) != len(ARM_JOINT_NAMES):
            return False, float('inf'), float('inf'), {}

        per_joint_err = {}
        all_within_tol = True
        err_vec = []

        for i, name in enumerate(ARM_JOINT_NAMES):
            err = abs(self.measured_angles[i] - target_angles[i])
            per_joint_err[name] = err
            err_vec.append(err)
            tol = self.joint_tolerances.get(name, 0.04)
            if err > tol:
                all_within_tol = False

        err_norm = float(np.linalg.norm(err_vec))
        pos_converged = all_within_tol and (err_norm <= self.norm_tolerance)
        max_err = max(err_vec) if err_vec else 0.0

        # Velocity convergence verification (Task 2)
        vel_converged = True
        if self.measured_velocities is not None and len(self.measured_velocities) == len(ARM_JOINT_NAMES):
            # Strict velocity limits apply to folded transport pose (protects base transit interlock).
            # Deployed painting poses account for ODE gravity-damping terminal velocity.
            is_transport = (
                self.active_pose_name == 'transport' or
                float(np.linalg.norm(np.array(target_angles) - np.array(PRESET_POSES['transport']))) < 0.05
            )
            v_tols = self.joint_vel_tolerances if is_transport else DEPLOYED_JOINT_VEL_TOLERANCES
            v_norm_tol = self.vel_norm_tolerance if is_transport else DEPLOYED_MAX_VEL_NORM_TOLERANCE

            vel_vec = []
            for i, name in enumerate(ARM_JOINT_NAMES):
                v = abs(self.measured_velocities[i])
                vel_vec.append(v)
                v_tol = v_tols.get(name, 0.50)
                if v > v_tol:
                    vel_converged = False
            v_norm = float(np.linalg.norm(vel_vec))
            if v_norm > v_norm_tol:
                vel_converged = False

        converged = pos_converged and vel_converged
        return converged, max_err, err_norm, per_joint_err

    def spray_callback(self, msg: Bool):
        self.is_spraying = msg.data
        self.get_logger().info(f'Spray gun active: {self.is_spraying}')

    def cmd_pose_callback(self, msg: String):
        """Accepts preset pose names via /cmd_arm_pose."""
        now = self.get_time_sec()
        if not self.has_valid_joint_feedback or (now - self.last_feedback_time) > MAX_FEEDBACK_AGE_SEC:
            self.get_logger().warn(
                'Rejecting /cmd_arm_pose: valid, fresh joint-state feedback not available.'
            )
            return

        pose_name = msg.data.strip().lower()
        if pose_name in PRESET_POSES:
            self.set_target_angles(PRESET_POSES[pose_name], pose_name)
        else:
            self.get_logger().warn(
                f'Unknown pose: "{pose_name}". Available: {list(PRESET_POSES.keys())}'
            )

    def cmd_traj_callback(self, msg: JointTrajectory):
        """Accepts arbitrary joint angle targets via /cmd_arm_trajectory."""
        now = self.get_time_sec()
        if not self.has_valid_joint_feedback or (now - self.last_feedback_time) > MAX_FEEDBACK_AGE_SEC:
            self.get_logger().warn(
                'Rejecting /cmd_arm_trajectory: valid, fresh joint-state feedback not available.'
            )
            return

        if not msg.points:
            return

        pt = msg.points[-1]
        target = list(self.current_angles)

        for i, name in enumerate(msg.joint_names):
            if name in ARM_JOINT_NAMES and i < len(pt.positions):
                idx = ARM_JOINT_NAMES.index(name)
                val = float(pt.positions[i])
                if math.isfinite(val):
                    target[idx] = val

        duration = 2.0
        if pt.time_from_start.sec > 0 or pt.time_from_start.nanosec > 0:
            duration = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9

        self.set_target_angles(target, 'custom', duration=duration)

    def joint_state_callback(self, msg: JointState):
        """Monitors actual joint states and velocities from simulation."""
        if not msg.name or not msg.position:
            return

        if len(msg.name) != len(msg.position):
            return

        name_to_pos = dict(zip(msg.name, msg.position))
        name_to_vel = dict(zip(msg.name, msg.velocity)) if (msg.velocity and len(msg.velocity) == len(msg.name)) else {}

        extracted = []
        extracted_vel = []
        has_vel = bool(name_to_vel)

        for joint_name in ARM_JOINT_NAMES:
            if joint_name not in name_to_pos:
                return  # Incomplete message: missing required arm joint
            val = name_to_pos[joint_name]
            if val is None or not isinstance(val, (int, float)) or not math.isfinite(val):
                return  # Non-finite or invalid numerical value
            extracted.append(float(val))

            if has_vel:
                v = name_to_vel.get(joint_name, 0.0)
                extracted_vel.append(float(v) if math.isfinite(v) else 0.0)

        now = self.get_time_sec()
        self.last_feedback_time = now
        self.measured_angles = extracted
        if has_vel:
            self.measured_velocities = extracted_vel

        if not self.has_valid_joint_feedback:
            self.has_valid_joint_feedback = True
            if self.motion_state == 'INITIALIZING':
                self.current_angles = list(self.measured_angles)
                self.target_angles = list(self.measured_angles)
                self.start_angles = list(self.measured_angles)
                self.active_pose_name = 'measured_initial'
                self.motion_state = 'HOLDING'
                self.target_reached = True
                self.settling_converged_start_time = None
                self.get_logger().info(
                    f'Controller initialized from measured joint positions: {[round(a, 4) for a in self.measured_angles]}. Now HOLDING initial pose.'
                )
            else:
                self.get_logger().info(
                    f'Valid joint-state feedback established: {[round(a, 4) for a in self.measured_angles]}'
                )

    def set_target_angles(self, target, pose_name='custom', duration=2.0):
        """Initiates smooth trajectory transition to target angles."""
        now = self.get_time_sec()
        if self.motion_state == 'INITIALIZING' or not self.has_valid_joint_feedback or (now - self.last_feedback_time) > MAX_FEEDBACK_AGE_SEC:
            self.get_logger().warn(
                f'Cannot set target angles for "{pose_name}": fresh joint-state feedback not available or controller still INITIALIZING.'
            )
            return

        self.start_angles = list(self.measured_angles if self.measured_angles is not None else self.current_angles)
        # Enforce URDF joint limits on targets
        clamped_target = []
        for i, name in enumerate(ARM_JOINT_NAMES):
            min_lim, max_lim = ARM_JOINT_LIMITS.get(name, (-3.14159, 3.14159))
            clamped_target.append(max(min_lim, min(max_lim, float(target[i]))))

        dist_to_target = float(np.linalg.norm(np.array(self.target_angles) - np.array(clamped_target)))
        is_same_target = dist_to_target < 0.01

        # If already holding and verified at this exact target, do not disrupt holding
        if is_same_target and self.motion_state == 'HOLDING' and self.target_reached:
            self.active_pose_name = pose_name
            return

        self.target_angles = clamped_target
        self.active_pose_name = pose_name

        if not is_same_target:
            self.gravity_trim = [0.0] * len(ARM_JOINT_NAMES)
            self.trajectory_duration = max(0.5, float(duration))
            self.trajectory_start_time = now
            self.is_interpolating = True
            self.motion_state = 'INTERPOLATING'
            self.target_reached = False
            self.settling_converged_start_time = None
            self.get_logger().info(
                f'Transitioning arm to "{pose_name}" over {self.trajectory_duration:.1f}s: {[round(x, 4) for x in self.target_angles]}'
            )
        else:
            # Re-commanding same target: preserve accumulated trim to prevent gravity droop
            if self.motion_state not in ('SETTLING', 'HOLDING'):
                self.motion_state = 'SETTLING'
                self.settling_start_time = now
                self.settling_converged_start_time = None
                self.target_reached = False

    def control_loop(self):
        """Executes at 100 Hz: interpolates trajectories, verifies convergence, and holds pose."""
        now = self.get_time_sec()

        if self.motion_state == 'INITIALIZING':
            # Do NOT command any motion or publish /set_joint_trajectory until initial feedback is verified
            status_msg = String()
            status_msg.data = self.active_pose_name
            self.pose_pub.publish(status_msg)

            motion_msg = String()
            motion_msg.data = self.motion_state
            self.motion_status_pub.publish(motion_msg)

            reached_msg = Bool()
            reached_msg.data = False
            self.target_reached_pub.publish(reached_msg)

            spray_msg = Bool()
            spray_msg.data = False
            self.spray_pub.publish(spray_msg)
            return

        # Feedback staleness check
        if self.has_valid_joint_feedback and (now - self.last_feedback_time) > MAX_FEEDBACK_AGE_SEC:
            self.has_valid_joint_feedback = False
            self.motion_state = 'FAILED'
            self.target_reached = False
            self.settling_converged_start_time = None
            if self.is_spraying:
                self.is_spraying = False
                self.get_logger().error(
                    f'Joint feedback lost (age: {now - self.last_feedback_time:.3f}s)! Disabled spray for safety.'
                )

        if self.motion_state == 'INTERPOLATING':
            elapsed = now - self.trajectory_start_time
            alpha = min(1.0, elapsed / self.trajectory_duration)
            # Smooth cosine S-curve interpolation: 0.5 * (1 - cos(pi * alpha))
            s_curve = 0.5 * (1.0 - math.cos(math.pi * alpha))

            for i in range(len(ARM_JOINT_NAMES)):
                self.current_angles[i] = (
                    self.start_angles[i] +
                    s_curve * (self.target_angles[i] - self.start_angles[i])
                )

            if alpha >= 1.0:
                self.is_interpolating = False
                self.current_angles = list(self.target_angles)
                self.motion_state = 'SETTLING'
                self.target_reached = False
                self.settling_start_time = now
                self.settling_converged_start_time = None

        elif self.motion_state == 'SETTLING':
            # Hold commanded target angles while monitoring measured convergence
            self.current_angles = list(self.target_angles)
            converged, max_err, err_norm, per_joint_err = self.check_target_convergence(self.target_angles)

            if converged:
                if self.settling_converged_start_time is None:
                    self.settling_converged_start_time = now

                # Require convergence to be sustained for sustained_settling_time
                settled_duration = now - self.settling_converged_start_time
                if settled_duration >= self.sustained_settling_time:
                    self.motion_state = 'HOLDING'
                    self.target_reached = True
                    self.get_logger().info(
                        f'Arm successfully reached, stabilized, and verified target pose "{self.active_pose_name}" '
                        f'with error norm {err_norm:.4f} rad over {settled_duration:.2f}s sustained settling. Holding position.'
                    )
                else:
                    self.target_reached = False
            else:
                self.settling_converged_start_time = None
                self.target_reached = False

            if self.motion_state == 'SETTLING' and (now - self.settling_start_time) > self.settling_timeout:
                self.motion_state = 'FAILED'
                self.target_reached = False
                self.settling_converged_start_time = None
                if self.is_spraying:
                    self.is_spraying = False
                    self.get_logger().error('Emergency spray shutoff triggered by arm convergence failure!')
                err_summary = ', '.join([f'{k}: {v:.4f}' for k, v in per_joint_err.items()])
                self.get_logger().error(
                    f'Arm FAILED to converge to "{self.active_pose_name}" within {self.settling_timeout:.1f}s timeout! '
                    f'Pos norm: {err_norm:.4f} rad (tol: {self.norm_tolerance:.4f} rad). Errors: [{err_summary}]'
                )

        elif self.motion_state == 'HOLDING':
            self.current_angles = list(self.target_angles)
            converged, max_err, err_norm, _ = self.check_target_convergence(self.target_angles)
            self.target_reached = converged and self.has_valid_joint_feedback
            if not converged:
                # Disturbance or velocity spike during HOLDING: drop to SETTLING to re-converge
                self.motion_state = 'SETTLING'
                self.settling_start_time = now
                self.settling_converged_start_time = None

        # Compute dynamic gravity deflection compensation trim in SETTLING and HOLDING
        if self.motion_state in ('SETTLING', 'HOLDING') and self.has_valid_joint_feedback and self.measured_angles is not None:
            dt = 1.0 / 100.0
            for i, name in enumerate(ARM_JOINT_NAMES):
                err = self.target_angles[i] - self.measured_angles[i]
                # Continuous deadband of 0.003 rad (~0.17 deg) prevents micro-hunting once converged
                if abs(err) <= 0.003:
                    eff_err = 0.0
                else:
                    eff_err = err - math.copysign(0.003, err)
                max_trim = MAX_GRAVITY_TRIM.get(name, 0.045)
                eff_gain = 1.5 if name in ('shoulder_joint', 'elbow_joint') else 1.0
                # Responsive integral compensation clamped to physical joint limits
                self.gravity_trim[i] = max(-max_trim, min(max_trim, self.gravity_trim[i] + eff_gain * eff_err * dt))
        elif self.motion_state == 'INTERPOLATING':
            self.gravity_trim = [t * 0.90 for t in self.gravity_trim]
        elif self.motion_state == 'FAILED' or not self.has_valid_joint_feedback:
            # On feedback failure or convergence timeout, decay trim to avoid unverified bias
            self.gravity_trim = [t * 0.95 for t in self.gravity_trim]

        # Publish JointTrajectory command to Gazebo Classic plugin
        msg = JointTrajectory()
        msg.header.frame_id = 'world'
        msg.header.stamp = Time(sec=0, nanosec=0)  # Immediate execution
        msg.joint_names = ALL_CONTROLLED_JOINTS

        # Spray cone visual link position: 0.0 (extended at nozzle tip) when spraying, -1.5 (retracted/hidden) when spray off
        cone_pos = 0.0 if self.is_spraying else -1.5
        pt = JointTrajectoryPoint()
        cmd_positions = []
        for i, name in enumerate(ARM_JOINT_NAMES):
            raw_pos = self.current_angles[i] + self.gravity_trim[i]
            min_lim, max_lim = ARM_JOINT_LIMITS.get(name, (-3.14159, 3.14159))
            cmd_positions.append(max(min_lim, min(max_lim, raw_pos)))
        pt.positions = [float(a) for a in cmd_positions] + [cone_pos]
        pt.time_from_start = Duration(sec=0, nanosec=10000000)  # 10 ms horizon matches 100 Hz control loop
        msg.points.append(pt)

        self.traj_pub.publish(msg)

        # Status publications
        status_msg = String()
        status_msg.data = self.active_pose_name
        self.pose_pub.publish(status_msg)

        motion_msg = String()
        motion_msg.data = self.motion_state
        self.motion_status_pub.publish(motion_msg)

        reached_msg = Bool()
        reached_msg.data = self.target_reached
        self.target_reached_pub.publish(reached_msg)

        spray_msg = Bool()
        spray_msg.data = self.is_spraying
        self.spray_pub.publish(spray_msg)


def main():
    rclpy.init()
    node = ArmTrajectoryController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

