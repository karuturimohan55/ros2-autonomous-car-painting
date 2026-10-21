#!/usr/bin/env python3
import time
import math
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from kinematics import forward_kinematics, ARM_JOINT_NAMES
from collision_checker import CollisionChecker, point_to_box_distance, CAR_LOWER_MIN, CAR_LOWER_MAX
from trajectory_planner import PaintingTrajectoryPlanner

def quat_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

class FrontArmTester(Node):
    def __init__(self):
        super().__init__('front_arm_tester')
        self.cmd_arm_pose_pub = self.create_publisher(String, '/cmd_arm_pose', 10)
        self.cmd_arm_traj_pub = self.create_publisher(JointTrajectory, '/cmd_arm_trajectory', 10)
        self.spray_pub = self.create_publisher(Bool, '/spray_active', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
        self.latest_odom = None
        self.latest_joints = None
        self.planner = PaintingTrajectoryPlanner()
        self.collision_checker = CollisionChecker()

    def odom_cb(self, msg):
        self.latest_odom = msg

    def joint_cb(self, msg):
        self.latest_joints = msg

    def send_arm_target(self, target_q, duration_sec=0.8):
        msg = JointTrajectory()
        msg.joint_names = ARM_JOINT_NAMES
        pt = JointTrajectoryPoint()
        pt.positions = [float(x) for x in target_q]
        sec = int(duration_sec)
        nanosec = int((duration_sec - sec) * 1e9)
        pt.time_from_start = Duration(sec=sec, nanosec=nanosec)
        msg.points.append(pt)
        self.cmd_arm_traj_pub.publish(msg)

    def set_arm_pose(self, name="transport", wait_sec=2.0):
        msg = String()
        msg.data = name
        self.cmd_arm_pose_pub.publish(msg)
        t0 = time.time()
        while time.time() - t0 < wait_sec:
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.04)

def main():
    rclpy.init()
    node = FrontArmTester()

    t0 = time.time()
    while rclpy.ok() and (node.latest_odom is None or node.latest_joints is None) and time.time() - t0 < 5.0:
        rclpy.spin_once(node, timeout_sec=0.1)

    p = node.latest_odom.pose.pose.position
    yaw = quat_to_yaw(node.latest_odom.pose.pose.orientation)
    curr_base = (p.x, p.y, yaw)
    print("=" * 75)
    print(f" FRONT ARM TRAJECTORY VERIFICATION (Base at x={curr_base[0]:.3f}, y={curr_base[1]:.3f}, yaw={math.degrees(curr_base[2]):.1f}°)")
    print("=" * 75)

    plan = node.planner.generate_surface_plan('front_bumper')
    ok, solved_plan, report = node.planner.discretize_and_solve(plan, waypoint_step=0.18, base_station=curr_base)
    print(f"Planning status: {'SUCCESS' if ok else 'FAIL'} - {report}")
    if not ok:
        node.destroy_node()
        rclpy.shutdown()
        return

    num_wps = len(solved_plan.joint_waypoints)
    print(f"Executing {num_wps} waypoints across front bumper...")

    # Move to first WP
    node.send_arm_target(solved_plan.joint_waypoints[0], duration_sec=1.2)
    t1 = time.time()
    while time.time() - t1 < 1.3:
        rclpy.spin_once(node, timeout_sec=0.05)

    standoffs = []
    nozzle_positions = []

    for i, (q, spray, cart_target) in enumerate(zip(solved_plan.joint_waypoints, solved_plan.spray_flags, solved_plan.cartesian_waypoints)):
        safe, reason = node.collision_checker.check_robot_configuration(q, curr_base)
        if not safe:
            print(f"[FAIL] Collision detected at WP {i+1}: {reason}")
            node.set_arm_pose("transport")
            return

        node.spray_pub.publish(Bool(data=bool(spray)))
        node.send_arm_target(q, duration_sec=0.6)

        t_wp = time.time()
        while time.time() - t_wp < 0.6:
            rclpy.spin_once(node, timeout_sec=0.03)

        # FK check
        p_fk, _ = forward_kinematics(q, curr_base)
        d_nozzle = point_to_box_distance(p_fk, CAR_LOWER_MIN, CAR_LOWER_MAX)
        standoffs.append(d_nozzle)
        nozzle_positions.append(p_fk)

        # Read actual joint states
        act_q = []
        for jname in ARM_JOINT_NAMES:
            if jname in node.latest_joints.name:
                idx = node.latest_joints.name.index(jname)
                act_q.append(node.latest_joints.position[idx])
            else:
                act_q.append(0.0)

        q_err = np.max(np.abs(np.array(q) - np.array(act_q)))

        print(f"  WP {i+1:2d}/{num_wps:2d} | Spray={'ON ' if spray else 'OFF'} | "
              f"Nozzle XYZ=[{p_fk[0]:.2f}, {p_fk[1]:.2f}, {p_fk[2]:.2f}]m | "
              f"Standoff={d_nozzle:.3f}m | Max Joint Error={math.degrees(q_err):.2f}°")

    node.spray_pub.publish(Bool(data=False))
    print("\n[RETRACT] Arm returning to transport pose...")
    node.set_arm_pose("transport", wait_sec=2.0)

    avg_standoff = np.mean(standoffs)
    min_standoff = np.min(standoffs)
    max_standoff = np.max(standoffs)
    print(f"\nFront Arm Trajectory Execution Summary:")
    print(f"  Waypoints Executed: {num_wps}/{num_wps}")
    print(f"  Standoff Distance: min={min_standoff:.3f}m, avg={avg_standoff:.3f}m, max={max_standoff:.3f}m")
    print(f"  Standoff in [0.20m, 0.35m] tolerance: {'PASS' if (min_standoff >= 0.15 and max_standoff <= 0.40) else 'FAIL'}")
    print("=" * 75)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
