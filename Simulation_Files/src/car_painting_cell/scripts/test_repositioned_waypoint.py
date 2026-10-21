#!/usr/bin/env python3
"""
Test Waypoint 1 after base repositioning to Y ~ 1.98m.
Calculates IK using the actual measured /odom base pose.
Commands arm via painting_arm_controller.
Evaluates Cartesian error against planned target [0.80, 1.18, 0.55].
"""

import sys
import os
import time
import math
import numpy as np

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from builtin_interfaces.msg import Duration
import tf2_ros

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from kinematics import forward_kinematics, inverse_kinematics, ARM_JOINT_NAMES
from collision_checker import CollisionChecker, point_to_box_distance, CAR_LOWER_MIN, CAR_LOWER_MAX

def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def main():
    rclpy.init()
    node = Node('waypoint_test')

    joints = {}
    odom_pose = [0.0, 0.0, 0.0]
    
    def joint_cb(m):
        for name, pos in zip(m.name, m.position):
            joints[name] = pos
            
    def odom_cb(m):
        p = m.pose.pose.position
        yaw = quaternion_to_yaw(m.pose.pose.orientation)
        odom_pose[0] = p.x
        odom_pose[1] = p.y
        odom_pose[2] = yaw

    node.create_subscription(JointState, '/joint_states', joint_cb, 10)
    node.create_subscription(Odometry, '/odom', odom_cb, 10)
    pub = node.create_publisher(JointTrajectory, '/cmd_arm_trajectory', 10)
    
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer, node)

    t0 = time.time()
    while time.time() - t0 < 1.5:
        rclpy.spin_once(node, timeout_sec=0.05)

    actual_base = list(odom_pose)
    print(f"Measured Base Pose from /odom: X={actual_base[0]:.4f}m, Y={actual_base[1]:.4f}m, Yaw={math.degrees(actual_base[2]):.2f} deg")

    # Target Cartesian waypoint 1 for left side
    planned_cart = np.array([0.80, 1.18, 0.55])
    target_normal = np.array([0.0, 1.0, 0.0])

    # Compute IK using the MEASURED base pose!
    ok_ik, target_q, ik_err = inverse_kinematics(planned_cart, target_normal, base_pose=actual_base)
    print(f"IK with Measured Base: ok={ok_ik}, err={ik_err*1000:.2f} mm")
    print(f"Target Joint Angles: {[round(x, 4) for x in target_q]}")

    # Pre-check FK with target joints
    p_fk_target, _ = forward_kinematics(target_q, actual_base)
    print(f"FK with Target Q: {p_fk_target.round(4)} (diff to target: {np.linalg.norm(planned_cart - p_fk_target)*1000:.2f} mm)")

    # Send command via /cmd_arm_trajectory
    duration = 3.0
    msg = JointTrajectory()
    msg.joint_names = ARM_JOINT_NAMES
    pt = JointTrajectoryPoint()
    pt.positions = [float(x) for x in target_q]
    sec = int(duration)
    nanosec = int((duration - sec) * 1e9)
    pt.time_from_start = Duration(sec=sec, nanosec=nanosec)
    msg.points.append(pt)

    print(f"\nCommanding arm to Waypoint 1 (duration={duration}s)...")
    pub.publish(msg)

    # Wait 5.0 seconds for arm to settle
    t_start = time.time()
    while time.time() - t_start < 5.0:
        rclpy.spin_once(node, timeout_sec=0.05)

    actual_q = [joints.get(k, 0.0) for k in ARM_JOINT_NAMES]
    joint_err = np.array(actual_q) - np.array(target_q)
    joint_err_norm = np.linalg.norm(joint_err)

    print("\n--- RESULTS AFTER SETTLING ---")
    print(f"Target Joint Angles:  {[round(x, 4) for x in target_q]}")
    print(f"Actual Joint Angles:  {[round(x, 4) for x in actual_q]}")
    print(f"Joint Error Norm:     {joint_err_norm:.4f} rad ({math.degrees(joint_err_norm):.2f} deg)")
    print("Individual Joint Errors:")
    for name, e in zip(ARM_JOINT_NAMES, joint_err):
        print(f"  {name:16s}: {e:+.4f} rad ({math.degrees(e):+.2f} deg)")

    # Calculate FK from actual joints and actual base
    p_act_fk, v_act = forward_kinematics(actual_q, actual_base)
    cart_err_fk = np.linalg.norm(planned_cart - p_act_fk)

    # TF lookup
    tf_cone_pos = None
    try:
        t_cone = tf_buffer.lookup_transform('odom', 'spray_cone_link', rclpy.time.Time())
        p = t_cone.transform.translation
        tf_cone_pos = np.array([p.x, p.y, p.z])
    except Exception as e:
        print(f"TF lookup error: {e}")

    print(f"\n--- CARTESIAN TRACKING VERIFICATION ---")
    print(f"Planned Target:                   {planned_cart.round(4)}")
    print(f"Measured FK Nozzle:               {p_act_fk.round(4)}")
    print(f"Cartesian Error (Target vs FK):   {cart_err_fk:.4f} m ({cart_err_fk*1000:.2f} mm)")

    if tf_cone_pos is not None:
        print(f"Actual TF Nozzle (spray_cone):    {tf_cone_pos.round(4)}")
        cart_err_tf = np.linalg.norm(planned_cart - tf_cone_pos)
        diff_fk_tf = np.linalg.norm(p_act_fk - tf_cone_pos)
        print(f"Cartesian Error (Target vs TF):   {cart_err_tf:.4f} m ({cart_err_tf*1000:.2f} mm)")
        print(f"FK vs TF Discrepancy:             {diff_fk_tf:.4f} m ({diff_fk_tf*1000:.2f} mm)")

    # Check standoff
    d_nozzle = point_to_box_distance(p_act_fk, CAR_LOWER_MIN, CAR_LOWER_MAX)
    print(f"Actual Standoff to Vehicle:       {d_nozzle:.4f} m (threshold >= 0.12 m)")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
