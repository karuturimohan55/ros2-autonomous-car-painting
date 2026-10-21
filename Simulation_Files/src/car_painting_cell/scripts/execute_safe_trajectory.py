#!/usr/bin/env python3
"""
Live Trajectory Execution & Telemetry Verification for Stage 3/4 Painting Cell.
Validates:
1. Base station positioning verification against required surface station before execution.
2. Low-speed non-spraying trajectory tracking on live Gazebo robot.
3. Real-time comparison between planned joint waypoints and actual /joint_states.
4. Forward kinematics verification of physical nozzle tip position from /joint_states using measured /odom base pose.
5. Dynamic standoff and collision monitoring against vehicle envelope using actual base pose.
6. Discrepancy comparison between FK nozzle tip and live Gazebo TF (spray_cone_link).
"""

import time
import math
import sys
import os
import argparse
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import tf2_ros

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from kinematics import forward_kinematics, forward_kinematics_all_links, ARM_JOINT_NAMES
from collision_checker import CollisionChecker, point_to_box_distance, CAR_LOWER_MIN, CAR_LOWER_MAX
from trajectory_planner import PaintingTrajectoryPlanner, validate_base_station


def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class TrajectoryValidatorNode(Node):
    def __init__(self):
        super().__init__('trajectory_validator_node')

        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        self.traj_pub = self.create_publisher(
            JointTrajectory, '/cmd_arm_trajectory', 10
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.latest_joints = {}
        self.latest_base_pose = None
        self.latest_base_vel = (0.0, 0.0)
        self.collision_checker = CollisionChecker()

    def joint_callback(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            self.latest_joints[name] = pos

    def odom_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        self.latest_base_pose = (p.x, p.y, yaw)
        self.latest_base_vel = (msg.twist.twist.linear.x, msg.twist.twist.angular.z)

    def get_arm_angles(self):
        if not all(k in self.latest_joints for k in ARM_JOINT_NAMES):
            return None
        return [self.latest_joints[k] for k in ARM_JOINT_NAMES]

    def send_arm_target(self, target_q, duration_sec=1.5):
        msg = JointTrajectory()
        msg.joint_names = ARM_JOINT_NAMES
        pt = JointTrajectoryPoint()
        pt.positions = [float(x) for x in target_q]
        sec = int(duration_sec)
        nanosec = int((duration_sec - sec) * 1e9)
        pt.time_from_start = Duration(sec=sec, nanosec=nanosec)
        msg.points.append(pt)
        self.traj_pub.publish(msg)

    def get_tf_nozzle_pos(self):
        """Looks up odom -> spray_cone_link in TF."""
        try:
            t = self.tf_buffer.lookup_transform('odom', 'spray_cone_link', rclpy.time.Time())
            return np.array([
                t.transform.translation.x,
                t.transform.translation.y,
                t.transform.translation.z
            ])
        except Exception:
            return None


def main():
    parser = argparse.ArgumentParser(description="Painting Cell Trajectory Execution & Safety Validator")
    parser.add_argument('--surface', type=str, default='left_side',
                        help='Surface to paint: hood, left_side, roof, rear, right_side, front_bumper')
    parser.add_argument('--validate-only', action='store_true',
                        help='Perform telemetry, station validation, collision, and FK checks without sending live commands.')
    parser.add_argument('--pos-tol', type=float, default=0.08,
                        help='Position tolerance for base station verification (meters, default 0.08m).')
    parser.add_argument('--yaw-tol', type=float, default=5.0,
                        help='Heading tolerance for base station verification (degrees, default 5.0 deg).')
    parser.add_argument('--num-waypoints', type=int, default=6,
                        help='Number of waypoints to execute (default 6).')
    args = parser.parse_args()

    rclpy.init()
    node = TrajectoryValidatorNode()

    print("=" * 70)
    print(" LIVE GAZEBO TRAJECTORY EXECUTION & SAFETY VALIDATION")
    print("=" * 70)
    print(f"Target Surface:     '{args.surface}'")
    print(f"Mode:               {'VALIDATE-ONLY (No Arm Dispatch)' if args.validate_only else 'LIVE EXECUTION'}")
    print(f"Base Station Tol:   pos={args.pos_tol*1000:.1f} mm, yaw={args.yaw_tol:.1f} deg")

    # Wait for telemetry
    print("\nWaiting for /odom and /joint_states telemetry...")
    start_wait = time.time()
    while rclpy.ok() and (node.latest_base_pose is None or node.get_arm_angles() is None):
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.time() - start_wait > 10.0:
            print("[FAIL] Timeout waiting for simulation telemetry!")
            sys.exit(1)

    # Allow TF buffer to accumulate transforms
    spin_tf_start = time.time()
    while time.time() - spin_tf_start < 1.0:
        rclpy.spin_once(node, timeout_sec=0.05)

    base_pose = node.latest_base_pose
    base_vel = node.latest_base_vel
    init_q = node.get_arm_angles()
    print(f"Base Telemetry:     X={base_pose[0]:.4f}m, Y={base_pose[1]:.4f}m, Yaw={math.degrees(base_pose[2]):.2f}deg")
    print(f"Base Velocity:      vx={base_vel[0]:.4f}m/s, wz={base_vel[1]:.4f}rad/s")
    print(f"Arm Joint Angles:   {[round(x, 4) for x in init_q]}")

    # Generate Surface Trajectory Plan
    planner = PaintingTrajectoryPlanner()
    try:
        plan = planner.generate_surface_plan(args.surface)
    except ValueError as e:
        print(f"[ERROR] {e}")
        sys.exit(1)

    # 1. Base Station Verification
    yaw_tol_rad = math.radians(args.yaw_tol)
    is_valid_station, pos_err, yaw_err, station_msg = validate_base_station(
        base_pose, plan.base_station, pos_tol=args.pos_tol, yaw_tol=yaw_tol_rad
    )

    print("\n" + "-" * 70)
    print(" BASE STATION VERIFICATION")
    print("-" * 70)
    print(f"  Required Station: X={plan.base_station[0]:.4f}m, Y={plan.base_station[1]:.4f}m, Yaw={math.degrees(plan.base_station[2]):.2f}deg")
    print(f"  Measured Pose:    X={base_pose[0]:.4f}m, Y={base_pose[1]:.4f}m, Yaw={math.degrees(base_pose[2]):.2f}deg")
    print(f"  Position Offset:  {pos_err*1000.0:.1f} mm (tolerance: {args.pos_tol*1000.0:.1f} mm)")
    print(f"  Heading Offset:   {math.degrees(yaw_err):.2f} deg (tolerance: {args.yaw_tol:.2f} deg)")
    print(f"  Station Status:   {'PASSED' if is_valid_station else 'FAILED'}")
    print("-" * 70)

    if not is_valid_station:
        print(f"\n[ABORT] {station_msg}")
        print("  The robot mobile base is NOT at the required station for this surface.")
        print("  Trajectory execution is aborted to prevent coordinate-frame mismatch and collision.")
        print("  Reposition the base to the required station before executing this trajectory.")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    # 2. Plan discretization and IK solving
    ok, solved_plan, msg = planner.discretize_and_solve(plan)
    assert ok, f"Planning failed: {msg}"
    print(f"\nTrajectory Plan Generated: {len(solved_plan.joint_waypoints)} waypoints.")

    # Select test waypoints
    num_pts = min(args.num_waypoints, len(solved_plan.joint_waypoints))
    test_waypoints = solved_plan.joint_waypoints[:num_pts]
    test_cartesian = solved_plan.cartesian_waypoints[:num_pts]
    test_base = solved_plan.base_waypoints[:num_pts]

    if args.validate_only:
        print("\n--- Validating Waypoints (Validate-Only Mode, No Arm Motion) ---")
        for idx, (target_q, target_cart, target_b) in enumerate(zip(test_waypoints, test_cartesian, test_base)):
            # Always check collision against current actual base pose
            actual_base = node.latest_base_pose
            safe, reason = node.collision_checker.check_robot_configuration(target_q, actual_base)
            p_fk, _ = forward_kinematics(target_q, actual_base)
            d_nozzle = point_to_box_distance(p_fk, CAR_LOWER_MIN, CAR_LOWER_MAX)
            cart_err = np.linalg.norm(target_cart - p_fk)
            print(f"  WP {idx+1}/{num_pts}: target={target_cart.round(3)} | FK={p_fk.round(3)} | err={cart_err*1000:.1f}mm | standoff={d_nozzle:.3f}m | safe={'YES' if safe else 'NO: '+reason}")
            assert safe, f"Collision check failed at WP {idx+1}: {reason}"

        print("\nAll waypoints validated successfully for station and collision safety.")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    print("\n--- Executing Low-Speed Test Segment (No Spray) ---")
    all_tracking_errors = []
    max_tracking_err = 0.0

    for idx, (target_q, target_cart, target_b) in enumerate(zip(test_waypoints, test_cartesian, test_base)):
        print(f"\n[Waypoint {idx+1}/{num_pts}] Commanding joint target: {[round(a, 3) for a in target_q]}")

        # Always check collision against measured actual base pose
        actual_base = node.latest_base_pose
        safe, reason = node.collision_checker.check_robot_configuration(target_q, actual_base)
        if not safe:
            print(f"  [ABORT] Waypoint failed collision check at actual base pose: {reason}")
            sys.exit(1)

        # Send trajectory command with 2.0s duration for smooth motion
        node.send_arm_target(target_q, duration_sec=2.0)

        # Monitor motion until reached
        start_move = time.time()
        while time.time() - start_move < 2.5:
            rclpy.spin_once(node, timeout_sec=0.05)

        # Check telemetry convergence using measured base pose
        actual_base = node.latest_base_pose
        actual_q = node.get_arm_angles()
        joint_err = np.linalg.norm(np.array(actual_q) - np.array(target_q))
        p_act, v_act = forward_kinematics(actual_q, actual_base)
        cart_err = np.linalg.norm(target_cart - p_act)
        all_tracking_errors.append(cart_err)
        max_tracking_err = max(max_tracking_err, cart_err)

        # Verify standoff distance to car body
        d_nozzle = point_to_box_distance(p_act, CAR_LOWER_MIN, CAR_LOWER_MAX)

        # Compare FK with TF nozzle position
        p_tf = node.get_tf_nozzle_pos()
        fk_tf_diff_str = "N/A"
        if p_tf is not None:
            fk_tf_diff = np.linalg.norm(p_act - p_tf)
            fk_tf_diff_str = f"{fk_tf_diff*1000.0:.2f} mm"

        print(f"  Actual Base Pose:   X={actual_base[0]:.3f}m, Y={actual_base[1]:.3f}m, Yaw={math.degrees(actual_base[2]):.1f}deg")
        print(f"  Actual Joint State: {[round(a, 3) for a in actual_q]}")
        print(f"  Joint Error Norm:   {joint_err:.4f} rad")
        print(f"  FK Nozzle Pos:      {p_act.round(3)} (Target: {target_cart.round(3)})")
        print(f"  Cartesian FK Error: {cart_err*1000.0:.2f} mm")
        print(f"  FK-to-TF Discrep.:  {fk_tf_diff_str}")
        print(f"  Nozzle Standoff:    {d_nozzle:.3f} m (Min required: 0.12 m)")

        if d_nozzle < 0.12:
            print(f"  [EMERGENCY STOP] Nozzle standoff violated: {d_nozzle:.3f}m < 0.12m!")
            sys.exit(1)

    print("\n-------------------------------------------------------")
    print(f" >>> LIVE TRAJECTORY EXECUTION COMPLETED SUCCESSFULLY <<<")
    print(f" Mean Cartesian Tracking Error: {np.mean(all_tracking_errors)*1000.0:.2f} mm")
    print(f" Max Cartesian Tracking Error:  {max_tracking_err*1000.0:.2f} mm")
    print(" All waypoints maintained safe standoff with zero collisions.")
    print("-------------------------------------------------------")

    # Return arm to home pose
    print("\nReturning arm to home pose...")
    node.send_arm_target([0.0]*6, duration_sec=2.0)
    time.sleep(2.5)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
