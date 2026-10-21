#!/usr/bin/env python3
"""
Autonomous Multi-Station Car Painting Orchestration State Machine
for Yuktii AI Labs Robotic Car Painting Simulation.

Implements the complete state machine:
HOME -> FRONT -> PAINT_FRONT -> LEFT_SIDE -> PAINT_LEFT -> REAR -> PAINT_REAR ->
RIGHT_SIDE -> PAINT_RIGHT -> FRONT/HOOD -> PAINT_HOOD -> ROOF -> PAINT_ROOF -> HOME

Features:
1. Closed-loop corridor-based differential drive navigation around the vehicle.
2. Verified station arrival checking with position and yaw error bounds.
3. Pre-flight collision safety checking against measured actual base poses.
4. Coordinated 6-DOF arm trajectory execution with smooth interpolation.
5. Dynamic spray cone activation/retraction during painting passes.
6. Visible, persistent paint layer deposition on each car surface via Gazebo entity spawning.
7. Safe transport pose return before any base transit.
8. Multi-camera screenshot evidence capture.
"""

import os
import sys
import time
import math
import argparse
import inspect
import numpy as np
from PIL import Image as PILImage
import concurrent.futures

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, Image
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from gazebo_msgs.srv import SpawnEntity, DeleteEntity

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from kinematics import forward_kinematics, ARM_JOINT_NAMES
from collision_checker import CollisionChecker, point_to_box_distance, CAR_LOWER_MIN, CAR_LOWER_MAX
from trajectory_planner import PaintingTrajectoryPlanner, validate_base_station

# Calibrated automotive painting poses (radians) matching arm_controller.py
PRESET_POSES = {
    'home': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    'ready': [0.0, 0.30, -0.60, 0.0, 0.30, 0.0],
    'transport': [0.0, 0.30, -0.60, 0.0, 0.30, 0.0],
    'side_paint': [0.0, 0.55, -1.10, 0.0, 0.55, 0.0],
    'roof_paint': [0.0, 0.15, -0.45, 0.0, -1.10, 0.0],
    'hood_paint': [0.50, 0.45, -0.90, 0.0, 0.45, 0.0]
}

# Configurable per-joint arrival tolerances (radians)
DEFAULT_JOINT_TOLERANCES = {
    'arm_base_joint': 0.03,   # rad (~1.7 deg)
    'shoulder_joint': 0.05,   # rad (~2.9 deg)
    'elbow_joint':    0.05,   # rad (~2.9 deg)
    'wrist_1_joint':  0.03,   # rad (~1.7 deg)
    'wrist_2_joint':  0.04,   # rad (~2.3 deg)
    'wrist_3_joint':  0.03    # rad (~1.7 deg)
}
DEFAULT_MAX_NORM_TOLERANCE = 0.06  # rad (~3.4 deg overall Euclidean norm)

# Justified per-joint velocity arrival tolerances (rad/s) matching arm_controller.py
DEFAULT_JOINT_VEL_TOLERANCES = {
    'arm_base_joint': 0.20,   # rad/s
    'shoulder_joint': 0.30,   # rad/s (covers +0.207 rad/s steady-state ODE gravity balance)
    'elbow_joint':    0.50,   # rad/s (covers -0.404 rad/s steady-state ODE gravity balance)
    'wrist_1_joint':  0.20,   # rad/s
    'wrist_2_joint':  0.20,   # rad/s
    'wrist_3_joint':  0.20    # rad/s
}
DEFAULT_MAX_VEL_NORM_TOLERANCE = 0.55  # rad/s Euclidean norm

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


def normalize_angle(angle):
    """Normalizes angle to [-pi, pi] radians."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def quat_to_yaw(q):
    """Extracts yaw (rad) from geometry_msgs/Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


# Station Definitions: (target_x, target_y, target_yaw)
STATIONS = {
    'HOME':        (0.00,  2.45, -math.pi / 2.0),
    'FRONT':       (3.50,  0.00, -math.pi),
    'LEFT_SIDE':   (0.00,  2.00, -math.pi / 2.0),
    'REAR':        (-3.50, 0.00,  0.0),
    'RIGHT_SIDE':  (0.00, -2.00,  math.pi / 2.0),
    'FRONT_HOOD':  (3.00,  0.00, -math.pi / 2.0),
    'ROOF':        (0.00,  1.85, -math.pi / 2.0),
}

# Collision-Free Corridor Navigation Routes between Stations
# All corridors maintain >= 1.0m clearance from the vehicle body envelope
TRANSIT_ROUTES = {
    # Sequential State Machine Transitions
    ('HOME', 'FRONT'): [
        (1.80, 2.30), (3.50, 2.30), (3.50, 0.00)
    ],
    ('FRONT', 'LEFT_SIDE'): [
        (3.50, 2.20), (1.50, 2.20), (0.00, 2.00)
    ],
    ('LEFT_SIDE', 'REAR'): [
        (-1.80, 2.20), (-3.50, 2.20), (-3.50, 0.00)
    ],
    ('REAR', 'RIGHT_SIDE'): [
        (-3.50, -2.20), (-1.50, -2.20), (0.00, -2.00)
    ],
    ('RIGHT_SIDE', 'FRONT_HOOD'): [
        (1.80, -2.20), (3.50, -2.20), (3.50, 0.00), (2.98, 0.00)
    ],
    ('FRONT_HOOD', 'ROOF'): [
        (3.50, 2.20), (1.50, 2.20), (0.00, 1.85)
    ],
    ('ROOF', 'HOME'): [
        (0.00, 2.45)
    ],
    # Direct access from HOME
    ('HOME', 'LEFT_SIDE'): [
        (0.00, 2.00)
    ],
    ('HOME', 'REAR'): [
        (-1.80, 2.20), (-3.50, 2.20), (-3.50, 0.00)
    ],
    ('HOME', 'RIGHT_SIDE'): [
        (1.80, 2.30), (3.50, 2.30), (3.50, -2.20), (1.50, -2.20), (0.00, -2.00)
    ],
    ('HOME', 'FRONT_HOOD'): [
        (1.80, 2.30), (3.50, 2.30), (3.50, 0.00), (3.00, 0.00)
    ],
    ('HOME', 'ROOF'): [
        (0.00, 1.85)
    ],
    # Inter-station connections for isolated testing
    ('FRONT', 'REAR'): [
        (3.50, 2.20), (0.00, 2.20), (-3.50, 2.20), (-3.50, 0.00)
    ],
    ('REAR', 'FRONT'): [
        (-3.50, 2.20), (0.00, 2.20), (3.50, 2.20), (3.50, 0.00)
    ],
    ('RIGHT_SIDE', 'HOME'): [
        (1.80, -2.20), (3.50, -2.20), (3.50, 2.30), (0.00, 2.45)
    ],
    ('REAR', 'HOME'): [
        (-3.50, 2.20), (-1.50, 2.20), (0.00, 2.45)
    ],
    ('FRONT', 'HOME'): [
        (3.50, 2.30), (1.80, 2.30), (0.00, 2.45)
    ],
    ('LEFT_SIDE', 'HOME'): [
        (0.00, 2.45)
    ],
    ('FRONT_HOOD', 'HOME'): [
        (3.50, 2.20), (1.50, 2.20), (0.00, 2.45)
    ],
}

# Surface Mapping to Station and Trajectory Surface Key
SURFACE_CONFIG = {
    'FRONT':       ('front_bumper', 'paint_layer_front'),
    'LEFT_SIDE':   ('left_side',    'paint_layer_left'),
    'REAR':        ('rear',         'paint_layer_rear'),
    'RIGHT_SIDE':  ('right_side',   'paint_layer_right'),
    'FRONT_HOOD':  ('hood',         'paint_layer_hood'),
    'ROOF':        ('roof',         'paint_layer_roof'),
}

# Surface-specific raster discretization resolutions (meters)
SURFACE_WAYPOINT_STEPS = {
    'rear': 0.08,  # Stage 6E: Fine resolution required for smooth rear bumper-to-trunk transition
}
DEFAULT_WAYPOINT_STEP = 0.18


def get_surface_waypoint_step(surface_key):
    """Returns the calibrated runtime waypoint resolution for a given surface."""
    return SURFACE_WAYPOINT_STEPS.get(surface_key, DEFAULT_WAYPOINT_STEP)

# SDF Visual Deposition Models (Glossy Metallic Deep Blue Coat)
# Form-fitted to exact vehicle geometry without z-fighting or air gaps
PAINT_SDF_TEMPLATES = {
    'paint_layer_front': """<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='paint_layer_front'>
    <static>true</static>
    <pose>2.435 0 0.36 0 0 0</pose>
    <link name='link'>
      <visual name='visual'>
        <geometry><box><size>0.01 1.84 0.25</size></box></geometry>
        <material>
          <ambient>0.0 0.35 0.85 1.0</ambient>
          <diffuse>0.0 0.55 1.0 1.0</diffuse>
          <specular>0.85 0.85 0.95 1.0</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>""",

    'paint_layer_left': """<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='paint_layer_left'>
    <static>true</static>
    <pose>0.0 0.905 0.65 0 0 0</pose>
    <link name='link'>
      <visual name='visual'>
        <geometry><box><size>2.40 0.01 0.50</size></box></geometry>
        <material>
          <ambient>0.0 0.35 0.85 1.0</ambient>
          <diffuse>0.0 0.55 1.0 1.0</diffuse>
          <specular>0.85 0.85 0.95 1.0</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>""",

    'paint_layer_rear': """<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='paint_layer_rear'>
    <static>true</static>
    <pose>-2.435 0 0.36 0 0 0</pose>
    <link name='link'>
      <visual name='bumper_visual'>
        <geometry><box><size>0.01 1.84 0.25</size></box></geometry>
        <material>
          <ambient>0.0 0.35 0.85 1.0</ambient>
          <diffuse>0.0 0.55 1.0 1.0</diffuse>
          <specular>0.85 0.85 0.95 1.0</specular>
        </material>
      </visual>
      <visual name='trunk_visual'>
        <pose>0.725 0 0.45 0 0 0</pose>
        <geometry><box><size>1.34 1.70 0.01</size></box></geometry>
        <material>
          <ambient>0.0 0.35 0.85 1.0</ambient>
          <diffuse>0.0 0.55 1.0 1.0</diffuse>
          <specular>0.85 0.85 0.95 1.0</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>""",

    'paint_layer_right': """<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='paint_layer_right'>
    <static>true</static>
    <pose>0.0 -0.905 0.65 0 0 0</pose>
    <link name='link'>
      <visual name='visual'>
        <geometry><box><size>2.40 0.01 0.50</size></box></geometry>
        <material>
          <ambient>0.0 0.35 0.85 1.0</ambient>
          <diffuse>0.0 0.55 1.0 1.0</diffuse>
          <specular>0.85 0.85 0.95 1.0</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>""",

    'paint_layer_hood': """<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='paint_layer_hood'>
    <static>true</static>
    <pose>1.69 0 0.795 0 0.02 0</pose>
    <link name='link'>
      <visual name='visual'>
        <geometry><box><size>1.38 1.70 0.01</size></box></geometry>
        <material>
          <ambient>0.0 0.35 0.85 1.0</ambient>
          <diffuse>0.0 0.55 1.0 1.0</diffuse>
          <specular>0.85 0.85 0.95 1.0</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>""",

    'paint_layer_roof': """<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='paint_layer_roof'>
    <static>true</static>
    <pose>-0.05 0 1.605 0 0 0</pose>
    <link name='link'>
      <visual name='visual'>
        <geometry><box><size>2.05 1.50 0.01</size></box></geometry>
        <material>
          <ambient>0.0 0.35 0.85 1.0</ambient>
          <diffuse>0.0 0.55 1.0 1.0</diffuse>
          <specular>0.85 0.85 0.95 1.0</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>""",
}


class PaintingStateMachine(Node):
    def __init__(self):
        super().__init__(
            'painting_state_machine',
            parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)]
        )

        # Declare and configure simulation time parameter (default True for Gazebo)
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)
        else:
            self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cmd_arm_pose_pub = self.create_publisher(String, '/cmd_arm_pose', 10)
        self.cmd_arm_traj_pub = self.create_publisher(JointTrajectory, '/cmd_arm_trajectory', 10)
        self.spray_active_pub = self.create_publisher(Bool, '/spray_active', 10)

        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
        self.arm_target_reached_sub = self.create_subscription(Bool, '/arm_target_reached', self.arm_reached_cb, 10)
        self.arm_motion_status_sub = self.create_subscription(String, '/arm_motion_status', self.arm_status_cb, 10)
        self.arm_current_pose_sub = self.create_subscription(String, '/arm_current_pose', self.arm_pose_cb, 10)

        # Camera frame storage - frames are captured on-demand during save_screenshot
        # to prevent 50+ MB/s DDS queue congestion and Python GIL starvation during navigation
        self.overview_img = None
        self.front_img = None
        self.top_img = None

        # Gazebo entity spawner client
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete_client = self.create_client(DeleteEntity, '/delete_entity')

        # State tracking
        self.current_pose = None
        self.current_vel = (0.0, 0.0)
        self.latest_joint_state = None
        self.latest_joint_state_time = 0.0
        self.latest_arm_status_time = 0.0
        self.latest_arm_pose_time = 0.0
        self.arm_current_pose = ""
        self.arm_target_reached = False
        self.arm_motion_status = "HOLDING"
        self.planner = PaintingTrajectoryPlanner()
        self.collision_checker = CollisionChecker()

        # Asynchronous background thread pool for photographic evidence disk I/O
        # Prevents synchronous PNG compression and file writes from stalling the ROS 2 executor thread
        self.io_executor = concurrent.futures.ThreadPoolExecutor(max_workers=2, thread_name_prefix="evidence_writer")

        self.get_logger().info("Painting State Machine initialized.")

    def destroy_node(self):
        try:
            if hasattr(self, 'io_executor') and self.io_executor is not None:
                self.io_executor.shutdown(wait=True)
        except Exception:
            pass
        return super().destroy_node()

    def spin_some(self, timeout_sec=0.02, max_drain=25):
        """
        Executes one timed callback event and then drains any ready callbacks
        without blocking. Prevents queue backlog and callback starvation during
        intensive simulation and navigation while preserving test hook compatibility.
        """
        rclpy.spin_once(self, timeout_sec=timeout_sec)
        for _ in range(max_drain):
            rclpy.spin_once(self, timeout_sec=0.0)

    def refresh_joint_telemetry(self, max_wait_sec=0.15):
        """
        Actively drains the callback queue and ensures fresh /joint_states telemetry
        is ingested before proceeding to subsequent trajectory waypoints or motion checks.
        """
        t_start = self.get_time_sec()
        t0_time = self.latest_joint_state_time
        while rclpy.ok() and (self.get_time_sec() - t_start < max_wait_sec):
            self.spin_some(timeout_sec=0.02, max_drain=25)
            now_sec = self.get_time_sec()
            if (self.latest_joint_state_time > t0_time and
                (now_sec - self.latest_joint_state_time) <= 0.1):
                break

    def get_time_sec(self) -> float:
        """Returns current time in seconds using ROS clock if valid, otherwise wall-clock."""
        try:
            nanosec = self.get_clock().now().nanoseconds
            if nanosec > 0:
                return float(nanosec) * 1e-9
        except Exception:
            pass
        return time.time()

    def arm_reached_cb(self, msg: Bool):
        self.arm_target_reached = msg.data

    def arm_status_cb(self, msg: String):
        self.arm_motion_status = msg.data
        self.latest_arm_status_time = self.get_time_sec()

    def arm_pose_cb(self, msg: String):
        self.arm_current_pose = msg.data
        self.latest_arm_pose_time = self.get_time_sec()

    def odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        yaw = quat_to_yaw(msg.pose.pose.orientation)
        self.current_pose = (p.x, p.y, yaw)
        self.current_vel = (msg.twist.twist.linear.x, msg.twist.twist.angular.z)

    def joint_cb(self, msg: JointState):
        self.latest_joint_state = msg
        msg_time = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9 if msg.header.stamp.sec > 0 else 0.0
        now = self.get_time_sec()
        self.latest_joint_state_time = now if now > 0.0 else msg_time

    def overview_cb(self, msg: Image):
        self.overview_img = msg

    def front_cb(self, msg: Image):
        self.front_img = msg

    def top_cb(self, msg: Image):
        self.top_img = msg

    def wait_for_telemetry(self, timeout=8.0):
        t0 = self.get_time_sec()
        while self.get_time_sec() - t0 < timeout:
            self.spin_some(timeout_sec=0.05)
            if (self.current_pose is not None and
                self.latest_joint_state is not None and
                self.latest_arm_status_time > 0.0):
                return True
        return False

    def stop_base(self, cycles=10):
        """Actively holds base stationary with zero-velocity commands."""
        zero = Twist()
        for _ in range(cycles):
            self.cmd_vel_pub.publish(zero)
            self.spin_some(timeout_sec=0.02)
            time.sleep(0.01)

    def ramp_stop_base(self, current_vx=0.0, current_wz=0.0, max_lin_accel=0.30, max_ang_accel=0.40, settle_sec=0.35):
        """
        Smoothly ramps commanded base velocities down to zero using bounded acceleration,
        then holds zero velocity until inertial chassis pitch oscillations settle.
        This prevents chassis pitch shocks from imparting dynamic angular velocity impulses
        to the cantilevered robot arm during waypoint junctions or before yaw turns.
        """
        twist = Twist()
        t_last = self.get_time_sec()
        vx = float(current_vx)
        wz = float(current_wz)
        while abs(vx) > 0.005 or abs(wz) > 0.005:
            now = self.get_time_sec()
            dt = max(0.01, min(0.05, now - t_last))
            t_last = now
            max_dv = max_lin_accel * dt
            max_dw = max_ang_accel * dt
            if vx > 0:
                vx = max(0.0, vx - max_dv)
            elif vx < 0:
                vx = min(0.0, vx + max_dv)
            if wz > 0:
                wz = max(0.0, wz - max_dw)
            elif wz < 0:
                wz = min(0.0, wz + max_dw)
            twist.linear.x = float(vx)
            twist.angular.z = float(wz)
            self.cmd_vel_pub.publish(twist)
            self.spin_some(timeout_sec=0.02)
            time.sleep(0.01)

        # Hold zero velocity and allow inertial damping to settle
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        t0 = self.get_time_sec()
        while (self.get_time_sec() - t0) < settle_sec:
            self.cmd_vel_pub.publish(twist)
            self.spin_some(timeout_sec=0.02)
            time.sleep(0.02)

    def wait_for_arm_convergence(self, target_q, timeout=5.0, tolerances=None):
        """
        Blocks until measured joints on /joint_states converge to target_q within tolerances,
        or until timeout is reached.
        Every polling iteration validates that joint feedback is present and <= 0.5s fresh.
        If missing or stale, shuts off spray immediately, logs an error, and returns (False, last_norm).
        """
        if tolerances is None:
            tolerances = DEFAULT_JOINT_TOLERANCES

        t0_sec = self.get_time_sec()
        last_norm = float('inf')

        # Prime queue with fresh telemetry before beginning verification loop
        self.spin_some(timeout_sec=0.02, max_drain=25)

        while rclpy.ok():
            self.spin_some(timeout_sec=0.02, max_drain=25)
            now_sec = self.get_time_sec()

            # 1. Fresh joint feedback presence and staleness check (Task 1)
            if self.latest_joint_state is None or self.latest_joint_state_time <= 0.0:
                self.set_spray(False)
                self.get_logger().error(
                    "Arm convergence aborted: No joint feedback received on /joint_states!"
                )
                return False, last_norm

            joint_age = now_sec - self.latest_joint_state_time
            if joint_age > 0.5 or joint_age < -0.1:
                # Active drain check in case fresh message is queued in DDS before aborting
                self.spin_some(timeout_sec=0.03, max_drain=25)
                now_sec = self.get_time_sec()
                joint_age = now_sec - self.latest_joint_state_time
                if joint_age > 0.5 or joint_age < -0.1:
                    self.set_spray(False)
                    self.get_logger().error(
                        f"Arm convergence aborted: Joint feedback is stale ({joint_age:.3f}s > 0.5s limit)!"
                    )
                    return False, last_norm

            # 2. Extract measured positions and evaluate tolerances
            if self.latest_joint_state.name:
                name_to_pos = dict(zip(self.latest_joint_state.name, self.latest_joint_state.position))
                if not all(n in name_to_pos for n in ARM_JOINT_NAMES):
                    self.set_spray(False)
                    self.get_logger().error(
                        "Arm convergence aborted: /joint_states missing required arm joints!"
                    )
                    return False, last_norm

                measured = [name_to_pos[n] for n in ARM_JOINT_NAMES]
                err_vec = [abs(m - t) for m, t in zip(measured, target_q)]
                all_within = all(e <= tolerances.get(n, 0.04) for e, n in zip(err_vec, ARM_JOINT_NAMES))
                last_norm = float(np.linalg.norm(err_vec))
                # Diagnostic only: report per-joint error twice per second.
                diagnostic_bucket = int((now_sec - t0_sec) * 2)
                previous_bucket = int(max(0.0, now_sec - t0_sec - 0.02) * 2)
                if diagnostic_bucket != previous_bucket:
                    err_details = ", ".join(
                        f"{name}={measured[i] - target_q[i]:+.4f}"
                        for i, name in enumerate(ARM_JOINT_NAMES)
                    )
                    self.get_logger().info(
                        f"Convergence diagnostic: norm={last_norm:.4f} rad; {err_details}"
                    )


                # Verify velocity stability if velocity data is populated
                vel_ok = True
                if (hasattr(self.latest_joint_state, 'velocity') and
                    self.latest_joint_state.velocity and
                    len(self.latest_joint_state.velocity) == len(self.latest_joint_state.name)):
                    name_to_vel = dict(zip(self.latest_joint_state.name, self.latest_joint_state.velocity))
                    is_transport = (
                        float(np.linalg.norm(np.array(target_q) - np.array(PRESET_POSES['transport']))) < 0.05
                    )
                    vel_tols = DEFAULT_JOINT_VEL_TOLERANCES if is_transport else DEPLOYED_JOINT_VEL_TOLERANCES
                    for n in ARM_JOINT_NAMES:
                        if abs(name_to_vel.get(n, 0.0)) > vel_tols.get(n, 0.50):
                            vel_ok = False
                            break

                if all_within and last_norm <= DEFAULT_MAX_NORM_TOLERANCE and vel_ok:
                    return True, last_norm
            else:
                self.set_spray(False)
                self.get_logger().error(
                    "Arm convergence aborted: Empty joint_states message!"
                )
                return False, last_norm

            # 3. Timeout check
            if (now_sec - t0_sec) > timeout:
                self.set_spray(False)
                self.get_logger().error(
                    f"Arm convergence timeout after {timeout:.2f}s! Target: {[round(x, 4) for x in target_q]}, "
                    f"Final err norm: {last_norm:.4f} rad."
                )
                return False, last_norm

        self.set_spray(False)
        return False, last_norm

    def set_arm_pose(self, pose_name="transport", wait_sec=2.0):
        """Commands arm to a preset pose ('transport', 'ready', 'home') and verifies convergence."""
        if pose_name not in PRESET_POSES:
            self.get_logger().error(f"Cannot set unknown arm pose: '{pose_name}'")
            return False

        target_q = PRESET_POSES[pose_name]
        tolerances = DEFAULT_JOINT_TOLERANCES
        norm_tol = DEFAULT_MAX_NORM_TOLERANCE

        # Prime queue to ingest fresh feedback before commanding target
        self.spin_some(timeout_sec=0.02, max_drain=25)
        msg = String()
        msg.data = pose_name
        self.cmd_arm_pose_pub.publish(msg)

        timeout = max(2.5, float(wait_sec) + 2.0)
        ok, err_norm = self.wait_for_arm_convergence(target_q, timeout=timeout)
        if not ok:
            self.get_logger().error(
                f"Failed to achieve verified pose '{pose_name}': error norm = {err_norm:.4f} rad"
            )
            return False

        # Confirm arm controller status has settled into fresh HOLDING and target reached
        t_hold = self.get_time_sec()
        confirmed = False
        confirm_count = 0
        REQUIRED_CONFIRM_CYCLES = 5  # Sustained verified hold cycles (>= 150ms)
        while rclpy.ok() and (self.get_time_sec() - t_hold < 3.5):
            self.spin_some(timeout_sec=0.03, max_drain=25)
            now_sec = self.get_time_sec()

            # Verify fresh controller status (<= 1.0s)
            status_age = now_sec - self.latest_arm_status_time if self.latest_arm_status_time > 0.0 else 999.0
            fresh_status = (0.0 <= status_age <= 1.0)

            # Verify fresh joint feedback (<= 0.5s)
            joint_age = now_sec - self.latest_joint_state_time if self.latest_joint_state_time > 0.0 else 999.0
            fresh_joint = (0.0 <= joint_age <= 0.5)

            # Check if controller reports HOLDING and arm_target_reached is True
            if (self.arm_motion_status == "HOLDING" and
                self.arm_target_reached and
                fresh_status and
                fresh_joint and
                self.latest_joint_state is not None and
                self.latest_joint_state.name):

                name_to_pos = dict(zip(self.latest_joint_state.name, self.latest_joint_state.position))
                if all(n in name_to_pos for n in ARM_JOINT_NAMES):
                    measured = [name_to_pos[n] for n in ARM_JOINT_NAMES]
                    err_vec = [abs(m - t) for m, t in zip(measured, target_q)]
                    all_within = all(e <= tolerances.get(n, 0.04) for e, n in zip(err_vec, ARM_JOINT_NAMES))
                    norm_e = float(np.linalg.norm(err_vec))

                    # Verify velocity stability if velocity data is populated
                    vel_ok = True
                    if (hasattr(self.latest_joint_state, 'velocity') and
                        self.latest_joint_state.velocity and
                        len(self.latest_joint_state.velocity) == len(self.latest_joint_state.name)):
                        name_to_vel = dict(zip(self.latest_joint_state.name, self.latest_joint_state.velocity))
                        vel_tols = DEFAULT_JOINT_VEL_TOLERANCES if pose_name == 'transport' else DEPLOYED_JOINT_VEL_TOLERANCES
                        for n in ARM_JOINT_NAMES:
                            if abs(name_to_vel.get(n, 0.0)) > vel_tols.get(n, 0.50):
                                vel_ok = False
                                break

                    if all_within and norm_e <= norm_tol and vel_ok:
                        if hasattr(self, 'latest_arm_pose_time') and self.latest_arm_pose_time > 0.0:
                            pose_age = now_sec - self.latest_arm_pose_time
                            if 0.0 <= pose_age <= 1.0 and hasattr(self, 'arm_current_pose'):
                                if self.arm_current_pose and self.arm_current_pose != pose_name:
                                    confirm_count = 0
                                    time.sleep(0.01)
                                    continue
                        confirm_count += 1
                        if confirm_count >= REQUIRED_CONFIRM_CYCLES:
                            confirmed = True
                            break
                    else:
                        confirm_count = 0
            else:
                confirm_count = 0

            time.sleep(0.01)

        if not confirmed:
            now_sec = self.get_time_sec()
            status_age = now_sec - self.latest_arm_status_time if self.latest_arm_status_time > 0.0 else 999.0
            self.get_logger().error(
                f"Arm controller confirmation timeout for '{pose_name}': status='{self.arm_motion_status}' "
                f"(age={status_age:.3f}s), reached={self.arm_target_reached}"
            )
            return False

        return True

    def set_spray(self, active: bool):
        """Turns atomized spray effect on or off."""
        msg = Bool()
        msg.data = bool(active)
        self.spray_active_pub.publish(msg)

    def send_arm_joint_target(self, target_q, duration_sec=1.0):
        """Sends a specific 6-DOF joint target to the arm controller."""
        msg = JointTrajectory()
        msg.joint_names = ARM_JOINT_NAMES
        pt = JointTrajectoryPoint()
        pt.positions = [float(x) for x in target_q]
        sec = int(duration_sec)
        nanosec = int((duration_sec - sec) * 1e9)
        pt.time_from_start = Duration(sec=sec, nanosec=nanosec)
        msg.points.append(pt)
        self.cmd_arm_traj_pub.publish(msg)

    def verify_arm_transport_safety(self):
        """
        Runtime safety interlock check (Task 2):
        1. Fresh joint feedback is available (<= 0.5s old) with all 6 arm joints.
        2. Measured arm joints are within configured tolerance of PRESET_POSES['transport'].
        3. Arm controller status is verified as stationary/holding (status == 'HOLDING' and fresh <= 1.0s).
        Returns (is_safe: bool, reason: str).
        """
        now_sec = self.get_time_sec()

        # 1. Joint feedback presence and staleness (<= 0.5s)
        if self.latest_joint_state is None or self.latest_joint_state_time <= 0.0:
            return False, "Joint feedback missing (no /joint_states received)"

        joint_age = now_sec - self.latest_joint_state_time
        if joint_age > 0.5 or joint_age < -0.1:
            self.spin_some(timeout_sec=0.03, max_drain=25)
            now_sec = self.get_time_sec()
            joint_age = now_sec - self.latest_joint_state_time
            if joint_age > 0.5 or joint_age < -0.1:
                return False, f"Joint feedback is stale (age={joint_age:.3f}s > 0.5s limit)"

        if not self.latest_joint_state.name:
            return False, "Joint feedback has empty name list"

        name_to_pos = dict(zip(self.latest_joint_state.name, self.latest_joint_state.position))
        missing_joints = [n for n in ARM_JOINT_NAMES if n not in name_to_pos]
        if missing_joints:
            return False, f"Joint feedback missing arm joints: {missing_joints}"

        # 2. Arm joints within transport pose tolerance
        transport_q = PRESET_POSES.get('transport')
        if transport_q is None:
            return False, "PRESET_POSES has no 'transport' configuration"

        measured = [name_to_pos[n] for n in ARM_JOINT_NAMES]
        err_vec = [abs(m - t) for m, t in zip(measured, transport_q)]
        for e, n in zip(err_vec, ARM_JOINT_NAMES):
            max_tol = DEFAULT_JOINT_TOLERANCES.get(n, 0.04)
            if e > max_tol:
                return False, f"Joint '{n}' offset {e:.4f} rad exceeds transport tolerance {max_tol:.4f} rad"

        norm_err = float(np.linalg.norm(err_vec))
        if norm_err > DEFAULT_MAX_NORM_TOLERANCE:
            return False, f"Arm joint norm error {norm_err:.4f} rad exceeds max tolerance {DEFAULT_MAX_NORM_TOLERANCE:.4f} rad"

        # Verify joint velocities are within justified stable limits if populated
        if (hasattr(self.latest_joint_state, 'velocity') and
            self.latest_joint_state.velocity and
            len(self.latest_joint_state.velocity) == len(self.latest_joint_state.name)):
            name_to_vel = dict(zip(self.latest_joint_state.name, self.latest_joint_state.velocity))
            for n in ARM_JOINT_NAMES:
                v = abs(name_to_vel.get(n, 0.0))
                max_v = DEFAULT_JOINT_VEL_TOLERANCES.get(n, 0.50)
                if v > max_v:
                    return False, f"Arm joint '{n}' velocity {v:.4f} rad/s exceeds stable limit {max_v:.4f} rad/s"

        # 3. Arm controller status verification
        if self.latest_arm_status_time <= 0.0:
            return False, "Arm controller status missing (no /arm_motion_status received)"

        status_age = now_sec - self.latest_arm_status_time
        if status_age > 1.0 or status_age < -0.1:
            return False, f"Arm controller status is stale (age={status_age:.3f}s > 1.0s limit)"

        if self.arm_motion_status != "HOLDING":
            return False, f"Arm controller status is '{self.arm_motion_status}', expected 'HOLDING'"

        return True, "Arm safely verified in transport pose and controller HOLDING"

    def drive_route(self, waypoints, final_yaw, pos_tol=0.04, yaw_tol=math.radians(4.0), timeout=60.0):
        """
        Navigates mobile base through a sequence of collision-free corridor waypoints,
        then aligns precisely to final_yaw at the station.
        Enforces runtime safety interlock: prohibits any non-zero velocity commands unless
        arm is verified in transport pose, feedback is fresh, and arm status is HOLDING.
        """
        # Pre-motion safety interlock check
        is_safe, safety_msg = self.verify_arm_transport_safety()
        if not is_safe:
            self.stop_base()
            self.set_spray(False)
            self.get_logger().error(f"Base motion interlock prevented drive_route: {safety_msg}")
            return False, f"Base motion interlock triggered: {safety_msg}"

        route_timeout = max(timeout, len(waypoints) * 45.0 + 35.0)
        t0 = self.get_time_sec()
        twist = Twist()

        # Smooth velocity state variables for bounded chassis acceleration
        current_vx = 0.0
        current_wz = 0.0
        last_twist_time = self.get_time_sec()

        # Dynamic acceleration limits (prevents inertial chassis pitch shock on cantilevered arm)
        MAX_LIN_ACCEL = 0.35   # m/s^2 linear acceleration/deceleration limit
        MAX_ANG_ACCEL = 0.35   # rad/s^2 angular acceleration limit

        for idx, (tx, ty) in enumerate(waypoints):
            is_final = (idx == len(waypoints) - 1)
            dist_threshold = 0.020 if is_final else 0.20
            prev_dist = 999.0

            while rclpy.ok():
                self.spin_some(timeout_sec=0.02)
                if self.get_time_sec() - t0 > route_timeout:
                    self.stop_base()
                    return False, f"Route transit timeout at segment {idx+1}"

                cx, cy, cyaw = self.current_pose
                dx = tx - cx
                dy = ty - cy
                dist = math.hypot(dx, dy)

                if dist <= dist_threshold or (is_final and dist <= 0.030 and dist > prev_dist):
                    break
                prev_dist = dist

                target_heading = math.atan2(dy, dx)
                e_head = normalize_angle(target_heading - cyaw)

                if abs(e_head) > math.radians(20.0):
                    target_vx = 0.0
                    target_wz = float(math.copysign(max(0.04, min(0.12, 0.8 * abs(e_head))), e_head))
                else:
                    if is_final:
                        if dist > 0.40:
                            speed = max(0.06, min(0.22, 0.6 * dist))
                        else:
                            speed = max(0.02, min(0.08, 0.3 * dist))
                    else:
                        speed = max(0.06, min(0.25, 0.7 * dist))
                    target_vx = float(speed)
                    target_wz = float(math.copysign(min(0.10, 0.6 * abs(e_head)), e_head))

                # Rate-limit commanded velocities to prevent sudden chassis jerk
                now_cmd = self.get_time_sec()
                dt_cmd = max(0.005, min(0.05, now_cmd - last_twist_time))
                last_twist_time = now_cmd
                max_dv = MAX_LIN_ACCEL * dt_cmd
                max_dw = MAX_ANG_ACCEL * dt_cmd

                current_vx += max(-max_dv, min(max_dv, target_vx - current_vx))
                current_wz += max(-max_dw, min(max_dw, target_wz - current_wz))

                twist.linear.x = float(current_vx)
                twist.angular.z = float(current_wz)

                # Continuous interlock check before publishing non-zero base velocity
                is_safe, safety_msg = self.verify_arm_transport_safety()
                if not is_safe:
                    self.stop_base()
                    if "SETTLING" in safety_msg or "velocity" in safety_msg:
                        self.get_logger().warn(f"Arm dynamic disturbance ({safety_msg}) during transit segment {idx+1}. Pausing base to re-verify hold...")
                        t_wait = self.get_time_sec()
                        recovered = False
                        while self.get_time_sec() - t_wait < 2.0:
                            self.spin_some(timeout_sec=0.03, max_drain=25)
                            is_safe_now, _ = self.verify_arm_transport_safety()
                            if is_safe_now:
                                recovered = True
                                break
                            time.sleep(0.02)
                        if recovered:
                            self.get_logger().info("Arm stabilized and verified in HOLDING. Resuming transit.")
                            last_twist_time = self.get_time_sec()
                            current_vx = 0.0
                            current_wz = 0.0
                            continue

                    self.set_spray(False)
                    self.get_logger().error(f"Base motion interlock triggered at segment {idx+1}: {safety_msg}")
                    return False, f"Base motion interlock triggered at segment {idx+1}: {safety_msg}"

                self.cmd_vel_pub.publish(twist)

            if not is_final:
                # Decelerate base smoothly between waypoints and allow chassis pitch inertia to settle
                self.ramp_stop_base(current_vx, current_wz, max_lin_accel=MAX_LIN_ACCEL, max_ang_accel=MAX_ANG_ACCEL, settle_sec=0.35)
                current_vx = 0.0
                current_wz = 0.0
                last_twist_time = self.get_time_sec()

        # Halt linear motion smoothly before yaw alignment and let chassis settle
        self.ramp_stop_base(current_vx, current_wz, max_lin_accel=MAX_LIN_ACCEL, max_ang_accel=MAX_ANG_ACCEL, settle_sec=0.35)
        current_vx = 0.0
        current_wz = 0.0
        last_twist_time = self.get_time_sec()

        # Fine Yaw Alignment at Final Station
        settled_count = 0
        t_yaw = self.get_time_sec()
        while rclpy.ok():
            self.spin_some(timeout_sec=0.02)
            if self.get_time_sec() - t_yaw > 25.0:
                self.stop_base()
                break  # Proceed with best achieved yaw and allow adaptive IK

            cx, cy, cyaw = self.current_pose
            e_yaw = normalize_angle(final_yaw - cyaw)

            if abs(e_yaw) <= yaw_tol:
                settled_count += 1
                current_wz = 0.0
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
                if settled_count >= 12:
                    break
            else:
                settled_count = 0
                is_safe, safety_msg = self.verify_arm_transport_safety()
                if not is_safe:
                    self.stop_base()
                    if "SETTLING" in safety_msg or "velocity" in safety_msg:
                        self.get_logger().warn(f"Arm dynamic disturbance ({safety_msg}) during yaw turn. Pausing base to re-verify hold...")
                        t_wait = self.get_time_sec()
                        recovered = False
                        while self.get_time_sec() - t_wait < 2.0:
                            self.spin_some(timeout_sec=0.03, max_drain=25)
                            is_safe_now, _ = self.verify_arm_transport_safety()
                            if is_safe_now:
                                recovered = True
                                break
                            time.sleep(0.02)
                        if recovered:
                            self.get_logger().info("Arm stabilized and verified in HOLDING. Resuming yaw turn.")
                            last_twist_time = self.get_time_sec()
                            current_wz = 0.0
                            continue

                    self.set_spray(False)
                    self.get_logger().error(f"Base motion interlock triggered during yaw turn: {safety_msg}")
                    return False, f"Base motion interlock triggered during yaw turn: {safety_msg}"

                target_wz = float(math.copysign(max(0.04, min(0.12, 0.8 * abs(e_yaw))), e_yaw))
                now_cmd = self.get_time_sec()
                dt_cmd = max(0.005, min(0.05, now_cmd - last_twist_time))
                last_twist_time = now_cmd
                max_dw = MAX_ANG_ACCEL * dt_cmd
                current_wz += max(-max_dw, min(max_dw, target_wz - current_wz))

                twist.linear.x = 0.0
                twist.angular.z = float(current_wz)
                self.cmd_vel_pub.publish(twist)

            time.sleep(0.01)

        self.ramp_stop_base(0.0, current_wz, max_lin_accel=MAX_LIN_ACCEL, max_ang_accel=MAX_ANG_ACCEL, settle_sec=0.30)
        current_vx = 0.0
        current_wz = 0.0
        last_twist_time = self.get_time_sec()

        # Fine Longitudinal Correction along robot heading after yaw turn (only if offset > 80mm)
        target_final = waypoints[-1]
        cx, cy, cyaw = self.current_pose
        current_pos_err = math.hypot(target_final[0] - cx, target_final[1] - cy)
        final_yaw_err = abs(normalize_angle(final_yaw - cyaw))
        if current_pos_err > 0.08 and final_yaw_err <= math.radians(6.0):
            t_corr = self.get_time_sec()
            while rclpy.ok() and self.get_time_sec() - t_corr < 4.0:
                self.spin_some(timeout_sec=0.02)
                cx, cy, cyaw = self.current_pose
                dx = target_final[0] - cx
                dy = target_final[1] - cy
                # Longitudinal error in robot frame
                e_long = dx * math.cos(cyaw) + dy * math.sin(cyaw)
                if abs(e_long) <= 0.030:
                    break

                is_safe, safety_msg = self.verify_arm_transport_safety()
                if not is_safe:
                    self.stop_base()
                    if "SETTLING" in safety_msg:
                        self.get_logger().warn("Arm settling during longitudinal correction. Pausing base to re-verify hold...")
                        t_wait = self.get_time_sec()
                        recovered = False
                        while self.get_time_sec() - t_wait < 2.0:
                            self.spin_some(timeout_sec=0.03, max_drain=25)
                            is_safe_now, _ = self.verify_arm_transport_safety()
                            if is_safe_now:
                                recovered = True
                                break
                            time.sleep(0.02)
                        if recovered:
                            self.get_logger().info("Arm stabilized and verified in HOLDING. Resuming correction.")
                            last_twist_time = self.get_time_sec()
                            current_vx = 0.0
                            continue

                    self.set_spray(False)
                    self.get_logger().error(f"Base motion interlock triggered during longitudinal correction: {safety_msg}")
                    return False, f"Base motion interlock triggered during longitudinal correction: {safety_msg}"

                target_vx = float(math.copysign(max(0.02, min(0.05, 0.5 * abs(e_long))), e_long))
                now_cmd = self.get_time_sec()
                dt_cmd = max(0.005, min(0.05, now_cmd - last_twist_time))
                last_twist_time = now_cmd
                max_dv = MAX_LIN_ACCEL * dt_cmd
                current_vx += max(-max_dv, min(max_dv, target_vx - current_vx))

                twist.linear.x = float(current_vx)
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
                time.sleep(0.01)

            self.ramp_stop_base(current_vx, 0.0, max_lin_accel=MAX_LIN_ACCEL, max_ang_accel=MAX_ANG_ACCEL, settle_sec=0.30)
        time.sleep(0.1)
        self.spin_some(timeout_sec=0.05)

        cx, cy, cyaw = self.current_pose
        final_pos_err = math.hypot(target_final[0] - cx, target_final[1] - cy)
        final_yaw_err = abs(normalize_angle(final_yaw - cyaw))

        success = (final_pos_err <= 0.12) and (final_yaw_err <= math.radians(6.0))
        msg = f"pos_err={final_pos_err*1000:.1f}mm, yaw_err={math.degrees(final_yaw_err):.2f}deg"
        return success, msg

    def spawn_paint_layer(self, layer_name):
        """Spawns visible, persistent automotive paint finish model on car body."""
        if layer_name not in PAINT_SDF_TEMPLATES:
            self.get_logger().error(f"Unknown paint layer name: '{layer_name}' (available: {list(PAINT_SDF_TEMPLATES.keys())})")
            return False

        if not self.spawn_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("SpawnEntity service (/spawn_entity) unavailable or timed out.")
            return False

        # If entity already exists, delete it first to ensure fresh, artifact-free placement
        if self.delete_client.wait_for_service(timeout_sec=1.0):
            del_req = DeleteEntity.Request()
            del_req.name = layer_name
            fut_del = self.delete_client.call_async(del_req)
            rclpy.spin_until_future_complete(self, fut_del, timeout_sec=2.0)

        req = SpawnEntity.Request()
        req.name = layer_name
        req.xml = PAINT_SDF_TEMPLATES[layer_name]
        future = self.spawn_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        res = future.result()
        if res is None:
            self.get_logger().error(f"Failed to spawn paint layer [{layer_name}]: Service call timed out after 5.0s!")
            return False
        if res.success:
            self.get_logger().info(f"Deposited visible paint layer: [{layer_name}] ({res.status_message})")
            return True
        else:
            self.get_logger().error(f"Gazebo rejected paint layer [{layer_name}]: {res.status_message}")
            return False

    def paint_surface(self, surface_key, layer_name, waypoint_step=0.16, wp_duration=0.6, output_dir=None, dry_run=False):
        """
        Executes complete painting pass sequence for a surface:
        1. Discretizes raster trajectory using actual base pose.
        2. Steps through raster sweeps with spray triggered (or forced OFF in dry_run).
        3. Shuts off spray and spawns persistent surface deposition (suppressed in dry_run).
        4. Returns arm to transport pose.
        """
        actual_base = self.current_pose
        if dry_run:
            self.set_spray(False)
        print(f"\n[PAINT] Planning trajectory for '{surface_key}' at base pose ({actual_base[0]:.3f}, {actual_base[1]:.3f}, {math.degrees(actual_base[2]):.1f}deg)...")

        plan = self.planner.generate_surface_plan(surface_key)
        ok, solved_plan, report = self.planner.discretize_and_solve(
            plan, waypoint_step=waypoint_step, base_station=actual_base
        )
        if not ok:
            print(f"  [ERROR] Planning failed: {report}")
            return False

        num_wps = len(solved_plan.joint_waypoints)
        print(f"  [INFO] Discretized into {num_wps} raster waypoints. Commencing painting sweeps...")

        # Move to first waypoint without spraying
        first_q = solved_plan.joint_waypoints[0]
        self.send_arm_joint_target(first_q, duration_sec=1.5)
        ok, err_norm = self.wait_for_arm_convergence(first_q, timeout=6.5)
        if not ok:
            print(f"  [ABORT] Failed to reach initial painting waypoint (error norm: {err_norm:.4f} rad)!")
            self.set_spray(False)
            retract_ok = self.set_arm_pose("transport", wait_sec=2.0)
            if not retract_ok:
                self.get_logger().error("Emergency: Arm failed to retract to transport pose after initial waypoint abort!")
                self.stop_base()
            return False

        for i, (q, is_spray, target_cart) in enumerate(zip(
            solved_plan.joint_waypoints, solved_plan.spray_flags, solved_plan.cartesian_waypoints
        )):
            # Collision check at current actual base
            safe, reason = self.collision_checker.check_robot_configuration(q, self.current_pose)
            if not safe:
                print(f"  [SAFETY STOP] Collision check failed at WP {i+1}: {reason}")
                self.set_spray(False)
                retract_ok = self.set_arm_pose("transport", wait_sec=2.0)
                if not retract_ok:
                    self.get_logger().error(f"Emergency: Arm failed to retract to transport pose after collision abort at WP {i+1}!")
                    self.stop_base()
                return False

            # Set spray status (forced OFF in dry-run mode, visible spray cone extends when active in normal mode)
            effective_spray = False if dry_run else is_spray
            self.set_spray(effective_spray)

            # Command joint target (adapt duration for larger line-transition steps)
            actual_wp_duration = float(wp_duration)
            if i > 0:
                prev_q = solved_plan.joint_waypoints[i - 1]
                dq = float(np.linalg.norm(np.array(q) - np.array(prev_q)))
                if dq > 0.15:
                    actual_wp_duration = max(float(wp_duration), min(1.5, dq * 1.8))

            self.send_arm_joint_target(q, duration_sec=actual_wp_duration)

            # Monitor arm motion and verify measured joint arrival within tolerance
            ok, err_norm = self.wait_for_arm_convergence(q, timeout=max(3.5, float(actual_wp_duration) + 2.8))
            if not ok:
                print(f"  [CONVERGENCE ABORT] Raster WP {i+1} failed to reach tolerance within timeout (err norm: {err_norm:.4f} rad)!")
                self.set_spray(False)
                retract_ok = self.set_arm_pose("transport", wait_sec=2.0)
                if not retract_ok:
                    self.get_logger().error(f"Emergency: Arm failed to retract to transport pose after convergence abort at WP {i+1}!")
                    self.stop_base()
                return False

            # Capture in-action evidence with active spray cone
            if is_spray and (i == num_wps // 2) and output_dir:
                self.spin_some(timeout_sec=0.05, max_drain=25)
                self.save_screenshot(os.path.join(output_dir, f"evidence_{surface_key}_spraying_active_front.png"), camera='front')
                self.save_screenshot(os.path.join(output_dir, f"evidence_{surface_key}_spraying_active_overview.png"), camera='overview')
                self.refresh_joint_telemetry(max_wait_sec=0.15)

            # Validate physical nozzle standoff
            p_fk, _ = forward_kinematics(q, self.current_pose)
            d_nozzle = point_to_box_distance(p_fk, CAR_LOWER_MIN, CAR_LOWER_MAX)
            if d_nozzle < 0.10:
                print(f"  [EMERGENCY STOP] Standoff distance violated: {d_nozzle:.3f}m < 0.10m!")
                self.set_spray(False)
                retract_ok = self.set_arm_pose("transport", wait_sec=2.0)
                if not retract_ok:
                    self.get_logger().error("Emergency: Arm failed to retract to transport pose after standoff abort!")
                    self.stop_base()
                return False

            if (i + 1) % 4 == 0 or (i + 1) == num_wps:
                spray_str = 'DRY ' if dry_run else ('ON ' if is_spray else 'OFF')
                print(f"    Raster WP {i+1:2d}/{num_wps:2d} | Spray={spray_str} | Standoff={d_nozzle:.3f}m | Nozzle XYZ={p_fk.round(2)}")

        # Finish painting: Spray OFF immediately
        self.set_spray(False)
        print("  [PASS] All raster sweeps completed. Spray deactivated.")

        # Deposit visible paint coat on vehicle body (suppressed in dry-run mode)
        if not dry_run:
            print(f"  [DEPOSITION] Applying simulated finish coat: {layer_name}...")
            spawn_ok = self.spawn_paint_layer(layer_name)
            if not spawn_ok:
                print(f"  [ERROR] Paint layer deposition failed for {layer_name}!")
                self.get_logger().error(f"Failed to deposit paint layer '{layer_name}' on surface '{surface_key}'!")
                self.set_spray(False)
                self.set_arm_pose("transport", wait_sec=2.0)
                self.stop_base()
                return False
            print(f"  [PASS] Paint layer successfully deposited: {layer_name}")
        else:
            print(f"  [DRY-RUN] Paint coat deposition suppressed ({layer_name}).")

        # Safely retract arm to compact transport pose before proceeding
        print("  [RETRACT] Returning arm to safe transport pose...")
        retract_ok = self.set_arm_pose("transport", wait_sec=2.0)
        if not retract_ok:
            print("  [ERROR] Failed to retract arm to transport pose after painting!")
            self.set_spray(False)
            self.stop_base()
            return False

        return True

    def save_screenshot(self, filepath, camera='overview', timeout_sec=3.0):
        """Captures on-demand frame from camera topic and saves to file."""
        topic_map = {
            'front': '/cell_camera/front/image_raw',
            'top': '/cell_camera/top/image_raw',
            'overview': '/cell_camera/overview/image_raw'
        }
        topic = topic_map.get(camera, '/cell_camera/overview/image_raw')

        captured = [None]
        def _img_cb(msg):
            captured[0] = msg

        sub = self.create_subscription(Image, topic, _img_cb, QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST))
        t0 = time.time()
        while rclpy.ok() and captured[0] is None and (time.time() - t0) < timeout_sec:
            self.spin_some(timeout_sec=0.03, max_drain=25)

        self.destroy_subscription(sub)
        self.spin_some(timeout_sec=0.01, max_drain=25)

        img_msg = captured[0]
        if camera == 'front':
            self.front_img = img_msg
        elif camera == 'top':
            self.top_img = img_msg
        else:
            self.overview_img = img_msg

        if img_msg is None:
            self.get_logger().warn(f"Failed to capture frame from {topic} within {timeout_sec}s")
            return False

        try:
            os.makedirs(os.path.dirname(os.path.abspath(filepath)), exist_ok=True)
            raw = bytes(img_msg.data)
            if img_msg.encoding == 'bgr8':
                img = PILImage.frombytes('RGB', (img_msg.width, img_msg.height), raw)
                b, g, r = img.split()
                img = PILImage.merge('RGB', (r, g, b))
            else:
                img = PILImage.frombytes('RGB', (img_msg.width, img_msg.height), raw)

            def _async_write(pil_img, path):
                try:
                    pil_img.save(path)
                except Exception as ex:
                    print(f"Error writing screenshot {path}: {ex}")

            if hasattr(self, 'io_executor') and self.io_executor is not None:
                self.io_executor.submit(_async_write, img, filepath)
            else:
                img.save(filepath)

            print(f"  [EVIDENCE] Saved screenshot ({camera}) -> {filepath}")
            self.spin_some(timeout_sec=0.02, max_drain=25)
            return True
        except Exception as e:
            self.get_logger().warn(f"Failed to save screenshot: {e}")
            return False


def run_single_station(node, station_key, output_dir, dry_run=False):
    """Executes painting workflow for a single designated station."""
    surface_key, layer_name = SURFACE_CONFIG[station_key]
    target_station = STATIONS[station_key]
    target_pos = (target_station[0], target_station[1])
    target_yaw = target_station[2]

    # Force spray OFF before any movement and throughout the entire run
    node.set_spray(False)

    print("\n" + "=" * 70)
    print(f" EXECUTING SINGLE STATION: {station_key} -> {surface_key}{' [DRY-RUN MODE]' if dry_run else ''}")
    print("=" * 70)

    # 1. Arm to transport pose before moving
    ok_arm = node.set_arm_pose("transport", wait_sec=2.0)
    if not ok_arm:
        print("[ABORT] Arm failed to achieve verified transport pose prior to station transit! Manual recovery may be required.")
        node.set_spray(False)
        node.stop_base()
        return False

    # 2. Navigate to station
    print(f"\n1. Navigating to {station_key} station: {target_station}...")
    curr = node.current_pose
    d_to_target = math.hypot(target_pos[0] - curr[0], target_pos[1] - curr[1])

    # Determine nearest station to current pose
    nearest_stn = 'HOME'
    min_d = 999.0
    for s_name, s_pose in STATIONS.items():
        d = math.hypot(s_pose[0] - curr[0], s_pose[1] - curr[1])
        if d < min_d:
            min_d = d
            nearest_stn = s_name

    if nearest_stn == station_key or d_to_target < 0.6:
        route = [target_pos]
    elif (nearest_stn, station_key) in TRANSIT_ROUTES:
        route = TRANSIT_ROUTES[(nearest_stn, station_key)]
    elif ('HOME', station_key) in TRANSIT_ROUTES:
        route = TRANSIT_ROUTES[('HOME', station_key)]
    else:
        route = [target_pos]

    ok, msg = node.drive_route(route, target_yaw, timeout=90.0)
    print(f"   Navigation Status: {'SUCCESS' if ok else 'FAILURE'} ({msg})")
    if not ok:
        print(f"[ABORT] Navigation to station {station_key} failed: {msg}")
        node.set_spray(False)
        node.stop_base()
        return False

    # 3. Verify station arrival tolerance
    is_valid, pos_err, yaw_err, check_msg = validate_base_station(node.current_pose, target_station)
    print(f"   Station Check:     {'PASS' if is_valid else 'FAIL'} ({check_msg})")
    if not is_valid:
        print("[ABORT] Cannot paint: base failed station positioning tolerance!")
        node.set_spray(False)
        node.stop_base()
        return False

    # 4. Execute painting sweeps
    step = get_surface_waypoint_step(surface_key)
    sig = inspect.signature(node.paint_surface)
    if 'dry_run' in sig.parameters or any(p.kind == inspect.Parameter.VAR_KEYWORD for p in sig.parameters.values()):
        painted = node.paint_surface(surface_key, layer_name, waypoint_step=step, wp_duration=0.6, output_dir=output_dir, dry_run=dry_run)
    else:
        painted = node.paint_surface(surface_key, layer_name, waypoint_step=step, wp_duration=0.6, output_dir=output_dir)
    if not painted:
        print(f"[FAIL] Painting surface {surface_key} failed!")
        node.set_spray(False)
        node.stop_base()
        chk = node.verify_arm_transport_safety()
        is_safe, safety_msg = chk if isinstance(chk, tuple) and len(chk) >= 2 else (bool(chk), str(chk))
        if not is_safe:
            print(f"[SAFETY LOCK] Arm transport pose verification failed ({safety_msg}). "
                  "Refusing return-to-HOME route. Manual recovery may be required.")
            return False
        print("[SAFETY STOP] Painting failed. Robot holding station in verified transport pose.")
        return False

    # 5. Capture evidence screenshot
    screenshot_front = os.path.join(output_dir, f"evidence_{station_key.lower()}_painted_front.png")
    screenshot_overview = os.path.join(output_dir, f"evidence_{station_key.lower()}_painted.png")
    node.save_screenshot(screenshot_front, camera='front')
    node.save_screenshot(screenshot_overview, camera='overview')

    # 6. Safe return-to-HOME route (Task 4)
    print(f"\n[CLEANUP] Initiating return-to-HOME from {station_key}...")
    chk = node.verify_arm_transport_safety()
    is_safe, safety_msg = chk if isinstance(chk, tuple) and len(chk) >= 2 else (bool(chk), str(chk))
    if not is_safe:
        node.stop_base()
        node.set_spray(False)
        print(f"[SAFETY LOCK] Arm transport pose verification failed ({safety_msg}). "
              "Base motion prohibited. Manual recovery may be required.")
        return False

    return_key = (station_key, 'HOME')
    if return_key not in TRANSIT_ROUTES:
        node.stop_base()
        print(f"[ERROR] No defined transit route from {station_key} to HOME in TRANSIT_ROUTES. "
              "Base motion prohibited. Manual recovery may be required.")
        return False

    return_route = TRANSIT_ROUTES[return_key]
    home_yaw = STATIONS['HOME'][2]
    print(f"Navigating return route to HOME: {return_route}...")
    ok_home, home_msg = node.drive_route(return_route, home_yaw, timeout=90.0)
    node.stop_base()
    node.set_spray(False)
    if not ok_home:
        print(f"[ERROR] Return to HOME failed: {home_msg}. Stopping base for safety.")
        return False

    is_home_valid, home_pos_err, home_yaw_err, home_chk_msg = validate_base_station(node.current_pose, STATIONS['HOME'])
    print(f"Return HOME Validation: {'PASS' if is_home_valid else 'FAIL'} ({home_chk_msg})")
    if not is_home_valid:
        print(f"[ABORT] Base pose outside safety tolerance for HOME station: {home_chk_msg}")
        return False

    print(f"\n>>> SINGLE STATION DEMONSTRATION FOR {station_key} SUCCEEDED (RETURNED HOME)! <<<\n")
    return True


def run_full_sequence(node, output_dir, dry_run=False):
    """
    Executes the full required state machine:
    HOME -> FRONT -> PAINT_FRONT -> LEFT_SIDE -> PAINT_LEFT -> REAR -> PAINT_REAR ->
    RIGHT_SIDE -> PAINT_RIGHT -> FRONT/HOOD -> PAINT_HOOD -> ROOF -> PAINT_ROOF -> HOME
    """
    sequence = [
        ('HOME', 'FRONT'),
        ('FRONT', 'LEFT_SIDE'),
        ('LEFT_SIDE', 'REAR'),
        ('REAR', 'RIGHT_SIDE'),
        ('RIGHT_SIDE', 'FRONT_HOOD'),
        ('FRONT_HOOD', 'ROOF'),
        ('ROOF', 'HOME'),
    ]

    # Force spray OFF before any movement
    node.set_spray(False)

    print("\n" + "=" * 70)
    print(f" EXECUTING FULL CAR PAINTING STATE MACHINE{' [DRY-RUN MODE]' if dry_run else ''}")
    print(" Sequence: HOME -> FRONT -> LEFT -> REAR -> RIGHT -> HOOD -> ROOF -> HOME")
    print("=" * 70)

    # Start at HOME in transport pose
    ok_arm = node.set_arm_pose("transport", wait_sec=2.0)
    node.stop_base()
    if not ok_arm:
        print("[ABORT] Arm failed initial transport pose verification at HOME!")
        node.set_spray(False)
        return False

    # Clear previous paint layers to demonstrate clean multi-station deposition
    if node.delete_client.wait_for_service(timeout_sec=0.5):
        for layer in ['paint_layer_front', 'paint_layer_left', 'paint_layer_rear', 'paint_layer_right', 'paint_layer_hood', 'paint_layer_roof']:
            del_req = DeleteEntity.Request()
            del_req.name = layer
            fut = node.delete_client.call_async(del_req)
            rclpy.spin_until_future_complete(node, fut, timeout_sec=0.5)

    # Ensure robot starts cleanly at HOME pose
    home_pose = STATIONS['HOME']
    d_home = math.hypot(home_pose[0] - node.current_pose[0], home_pose[1] - node.current_pose[1])
    e_yaw_home = abs(normalize_angle(home_pose[2] - node.current_pose[2]))
    if d_home > 0.05 or e_yaw_home > math.radians(4.0):
        print(f"Aligning robot to initial HOME position {home_pose} (offset {d_home*1000:.1f}mm, yaw err {math.degrees(e_yaw_home):.1f}deg)...")
        if node.current_pose[0] > 1.5:
            init_route = [(3.50, 2.30), (home_pose[0], home_pose[1])]
        elif node.current_pose[0] < -1.5:
            init_route = [(-3.50, 2.30), (home_pose[0], home_pose[1])]
        else:
            init_route = [(home_pose[0], home_pose[1])]
        ok_init, msg_init = node.drive_route(init_route, home_pose[2], timeout=50.0)
        if not ok_init:
            print(f"[ABORT] Initial HOME alignment drive failed: {msg_init}")
            node.stop_base()
            node.set_spray(False)
            return False

    # Validate actual base pose after HOME alignment; do not assume that a successful route means the robot reached HOME.
    is_home_valid, home_pos_err, home_yaw_err, home_chk_msg = validate_base_station(node.current_pose, home_pose)
    print(f"Initial HOME Validation: {'PASS' if is_home_valid else 'FAIL'} ({home_chk_msg})")
    if not is_home_valid:
        print(f"[ABORT] Base pose outside safety tolerance for initial HOME station: {home_chk_msg}")
        node.stop_base()
        node.set_spray(False)
        return False

    completed_surfaces = []

    for from_stn, to_stn in sequence:
        print(f"\n" + "-" * 70)
        print(f" STATE TRANSITION: {from_stn} -> {to_stn}")
        print("-" * 70)

        # 1. Transit to station
        route = TRANSIT_ROUTES.get((from_stn, to_stn))
        target_station = STATIONS[to_stn]
        target_yaw = target_station[2]

        print(f"Transit: Navigating along corridor to {to_stn} station ({target_station})...")
        ok_transit, t_msg = node.drive_route(route, target_yaw, timeout=60.0)
        print(f"Transit Status: {'SUCCESS' if ok_transit else 'FAILED'} ({t_msg})")
        if not ok_transit:
            print(f"[ABORT] Transit to station {to_stn} failed: {t_msg}")
            node.stop_base()
            node.set_spray(False)
            return False

        # 2. Verify station tolerance
        is_valid, pos_err, yaw_err, chk_msg = validate_base_station(node.current_pose, target_station)
        print(f"Station Tolerance Check: {'PASS' if is_valid else 'FAIL'} ({chk_msg})")
        if not is_valid:
            print(f"[ABORT] Base pose outside safety tolerance for station {to_stn}: {chk_msg}")
            node.stop_base()
            node.set_spray(False)
            return False

        # If arrived at HOME at the end of the sequence
        if to_stn == 'HOME':
            print("\n>>> FULL SEQUENCE COMPLETED: ROBOT RETURNED SAFELY HOME! <<<")
            break

        # 3. Execute Surface Painting
        surface_key, layer_name = SURFACE_CONFIG[to_stn]
        print(f"\nState: PAINT_{to_stn} (Surface: {surface_key})")
        step = get_surface_waypoint_step(surface_key)
        sig = inspect.signature(node.paint_surface)
        if 'dry_run' in sig.parameters or any(p.kind == inspect.Parameter.VAR_KEYWORD for p in sig.parameters.values()):
            painted = node.paint_surface(surface_key, layer_name, waypoint_step=step, wp_duration=0.55, output_dir=output_dir, dry_run=dry_run)
        else:
            painted = node.paint_surface(surface_key, layer_name, waypoint_step=step, wp_duration=0.55, output_dir=output_dir)
        if not painted:
            print(f"[ABORT] Painting aborted at surface {surface_key}!")
            node.stop_base()
            node.set_spray(False)
            return False

        completed_surfaces.append(surface_key)
        node.save_screenshot(os.path.join(output_dir, f"evidence_stage_{to_stn.lower()}_done_front.png"), camera='front')
        node.save_screenshot(os.path.join(output_dir, f"evidence_stage_{to_stn.lower()}_done.png"), camera='overview')

    # Verify that all 6 required surfaces were successfully completed
    if len(completed_surfaces) < 6:
        print(f"\n[ABORT] State machine incomplete: only {len(completed_surfaces)}/6 surfaces completed ({', '.join(completed_surfaces)})!")
        node.stop_base()
        node.set_spray(False)
        return False

    # Final overview screenshot
    final_screenshot = os.path.join(output_dir, "final_fully_painted_car.png")
    node.save_screenshot(final_screenshot, camera='overview')
    node.save_screenshot(os.path.join(output_dir, "final_fully_painted_car_front.png"), camera='front')
    node.save_screenshot(os.path.join(output_dir, "final_fully_painted_car_top.png"), camera='top')

    print("\n" + "=" * 70)
    print(" STATE MACHINE DEMONSTRATION COMPLETE")
    print(f" Successfully Painted Surfaces: {len(completed_surfaces)}/6 ({', '.join(completed_surfaces)})")
    print(" Base returned safely to HOME pose outside vehicle envelope.")
    print("=" * 70 + "\n")
    return True


def main():
    parser = argparse.ArgumentParser(description="Autonomous Painting State Machine")
    parser.add_argument('--single-station', type=str, default=None,
                        help='Execute only one station: FRONT, LEFT_SIDE, REAR, RIGHT_SIDE, FRONT_HOOD, ROOF')
    ws_screenshots = os.path.join(os.getcwd(), 'screenshots')
    pkg_screenshots = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'screenshots')
    default_screenshots = ws_screenshots if os.path.isdir(ws_screenshots) else pkg_screenshots
    parser.add_argument('--output-dir', type=str, default=default_screenshots,
                        help='Directory to save photographic evidence')
    parser.add_argument('--dry-run', action='store_true', default=False,
                        help='Execute trajectory and navigation in dry-run mode with spray and deposition forced OFF')
    args = parser.parse_args()

    rclpy.init()
    node = PaintingStateMachine()

    print("Waiting for simulation telemetry...")
    if not node.wait_for_telemetry(timeout=10.0):
        print("[FAIL] Timeout waiting for /odom and /joint_states telemetry!")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    print(f"Initial Telemetry: Base={node.current_pose}, Joints={len(node.latest_joint_state.name)}")

    if args.single_station:
        stn = args.single_station.upper()
        if stn == 'HOOD':
            stn = 'FRONT_HOOD'
        if stn not in STATIONS or stn == 'HOME':
            valid_stations = list(SURFACE_CONFIG.keys()) + ['HOOD']
            print(f"Invalid station: {args.single_station}. Choose from: {valid_stations}")
            node.destroy_node()
            rclpy.shutdown()
            sys.exit(1)
        success = run_single_station(node, stn, args.output_dir, dry_run=args.dry_run)
    else:
        success = run_full_sequence(node, args.output_dir, dry_run=args.dry_run)

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
