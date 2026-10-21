#!/usr/bin/env python3
"""
Comprehensive Control and Test Utility for Mobile Painting Robot (Stage 2).
Provides automated verification for:
1. Mobile base drive controller (/cmd_vel): Forward, Reverse, Left Turn, Right Turn, Stop.
   Quantitatively verifies displacement and heading change via /odom.
2. 6-Axis articulated arm (/cmd_arm_pose & /set_joint_trajectory):
   Cycles through key automotive painting poses ('home', 'ready', 'side_paint', 'roof_paint', 'hood_paint').
   Quantitatively verifies physical joint movement via /joint_states.
3. Live telemetry reporting (Base position, linear/angular velocity, all 10 joint angles).
"""

import sys
import time
import math
import argparse
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
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

WHEEL_JOINT_NAMES = [
    'front_left_wheel_joint',
    'front_right_wheel_joint',
    'rear_left_wheel_joint',
    'rear_right_wheel_joint'
]

# Calibrated automotive painting poses (radians)
PRESET_POSES = {
    'home': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    'ready': [0.0, 0.30, -0.60, 0.0, 0.30, 0.0],
    'side_paint': [0.0, 0.55, -1.10, 0.0, 0.55, 0.0],
    'roof_paint': [0.0, 0.15, -0.45, 0.0, -1.10, 0.0],
    'hood_paint': [0.50, 0.45, -0.90, 0.0, 0.45, 0.0]
}


def quaternion_to_yaw(q):
    """Computes yaw (rad) from geometry_msgs/Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class RobotTestController(Node):
    def __init__(self):
        super().__init__('robot_test_controller')

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cmd_arm_pose_pub = self.create_publisher(String, '/cmd_arm_pose', 10)
        self.direct_traj_pub = self.create_publisher(JointTrajectory, '/set_joint_trajectory', 10)

        # Subscribers
        self.latest_odom = None
        self.latest_joint_state = None
        self.active_pose_name = 'unknown'

        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10
        )
        self.pose_sub = self.create_subscription(
            String, '/arm_current_pose', self.pose_callback, 10
        )

        self.get_logger().info('Robot Test Controller initialized.')

    def odom_callback(self, msg: Odometry):
        self.latest_odom = msg

    def joint_callback(self, msg: JointState):
        self.latest_joint_state = msg

    def pose_callback(self, msg: String):
        self.active_pose_name = msg.data

    def wait_for_data(self, timeout=5.0):
        """Waits until initial /odom and /joint_states messages arrive."""
        start = time.time()
        while time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.latest_odom is not None and self.latest_joint_state is not None:
                return True
        return False

    def get_arm_joint_positions(self):
        """Returns dict of current arm joint positions."""
        rclpy.spin_once(self, timeout_sec=0.1)
        if not self.latest_joint_state:
            return {}
        pos_dict = {}
        for name, pos in zip(self.latest_joint_state.name, self.latest_joint_state.position):
            if name in ARM_JOINT_NAMES:
                pos_dict[name] = pos
        return pos_dict

    def send_base_velocity(self, linear_x=0.0, angular_z=0.0, duration=1.0):
        """Sends velocity to mobile base for a specified duration then stops."""
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.angular.z = float(angular_z)

        start = time.time()
        while time.time() - start < duration:
            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.05)

        # Stop
        stop_twist = Twist()
        self.cmd_vel_pub.publish(stop_twist)
        rclpy.spin_once(self, timeout_sec=0.1)

    def test_base_movement(self):
        """Tests all base motion primitives and quantitatively validates displacement."""
        print("\n=======================================================")
        print(" [TEST] MOBILE BASE DRIVE CONTROLLER VERIFICATION")
        print("=======================================================")

        if not self.wait_for_data():
            print("\033[91m[FAIL] No /odom telemetry received. Is Gazebo running?\033[0m")
            return False

        all_passed = True

        # Initial state
        p0 = self.latest_odom.pose.pose.position
        yaw0 = quaternion_to_yaw(self.latest_odom.pose.pose.orientation)
        print(f" Initial Pose: x={p0.x:.3f}m, y={p0.y:.3f}m, yaw={yaw0*180/math.pi:.1f}deg")

        # 1. Forward motion test
        print("\n 1. Forward Drive Test (v = +0.35 m/s for 1.2s)...")
        self.send_base_velocity(linear_x=0.35, angular_z=0.0, duration=1.2)
        time.sleep(0.4)
        rclpy.spin_once(self, timeout_sec=0.2)
        p1 = self.latest_odom.pose.pose.position
        dist_fwd = math.sqrt((p1.x - p0.x)**2 + (p1.y - p0.y)**2)
        fwd_ok = dist_fwd >= 0.15
        all_passed = all_passed and fwd_ok
        status = "\033[92m[PASS]\033[0m" if fwd_ok else "\033[91m[FAIL]\033[0m"
        print(f"    {status} Forward displacement: {dist_fwd:.3f} m (threshold >= 0.15 m)")

        # 2. Reverse motion test
        print("\n 2. Reverse Drive Test (v = -0.35 m/s for 1.2s)...")
        self.send_base_velocity(linear_x=-0.35, angular_z=0.0, duration=1.2)
        time.sleep(0.4)
        rclpy.spin_once(self, timeout_sec=0.2)
        p2 = self.latest_odom.pose.pose.position
        dist_rev = math.sqrt((p2.x - p1.x)**2 + (p2.y - p1.y)**2)
        rev_ok = dist_rev >= 0.15
        all_passed = all_passed and rev_ok
        status = "\033[92m[PASS]\033[0m" if rev_ok else "\033[91m[FAIL]\033[0m"
        print(f"    {status} Reverse displacement: {dist_rev:.3f} m (threshold >= 0.15 m)")

        # 3. Counter-Clockwise (Left) turn test
        print("\n 3. Turn Left (CCW) Test (w = +0.50 rad/s for 1.5s)...")
        yaw_before_left = quaternion_to_yaw(self.latest_odom.pose.pose.orientation)
        self.send_base_velocity(linear_x=0.0, angular_z=0.50, duration=1.5)
        time.sleep(0.4)
        rclpy.spin_once(self, timeout_sec=0.2)
        yaw_after_left = quaternion_to_yaw(self.latest_odom.pose.pose.orientation)
        yaw_diff_left = yaw_after_left - yaw_before_left
        # Normalize
        yaw_diff_left = (yaw_diff_left + math.pi) % (2 * math.pi) - math.pi
        left_ok = yaw_diff_left >= 0.20
        all_passed = all_passed and left_ok
        status = "\033[92m[PASS]\033[0m" if left_ok else "\033[91m[FAIL]\033[0m"
        print(f"    {status} CCW Rotation: {yaw_diff_left*180/math.pi:+.1f} deg ({yaw_diff_left:+.3f} rad, threshold >= +0.20 rad)")

        # 4. Clockwise (Right) turn test
        print("\n 4. Turn Right (CW) Test (w = -0.50 rad/s for 1.5s)...")
        yaw_before_right = quaternion_to_yaw(self.latest_odom.pose.pose.orientation)
        self.send_base_velocity(linear_x=0.0, angular_z=-0.50, duration=1.5)
        time.sleep(0.4)
        rclpy.spin_once(self, timeout_sec=0.2)
        yaw_after_right = quaternion_to_yaw(self.latest_odom.pose.pose.orientation)
        yaw_diff_right = yaw_after_right - yaw_before_right
        yaw_diff_right = (yaw_diff_right + math.pi) % (2 * math.pi) - math.pi
        right_ok = yaw_diff_right <= -0.20
        all_passed = all_passed and right_ok
        status = "\033[92m[PASS]\033[0m" if right_ok else "\033[91m[FAIL]\033[0m"
        print(f"    {status} CW Rotation: {yaw_diff_right*180/math.pi:+.1f} deg ({yaw_diff_right:+.3f} rad, threshold <= -0.20 rad)")

        # 5. Stop test
        print("\n 5. Full Stop Verification...")
        self.send_base_velocity(linear_x=0.0, angular_z=0.0, duration=0.5)
        time.sleep(0.3)
        rclpy.spin_once(self, timeout_sec=0.2)
        v_lin = self.latest_odom.twist.twist.linear.x
        v_ang = self.latest_odom.twist.twist.angular.z
        stop_ok = abs(v_lin) < 0.05 and abs(v_ang) < 0.05
        all_passed = all_passed and stop_ok
        status = "\033[92m[PASS]\033[0m" if stop_ok else "\033[91m[FAIL]\033[0m"
        print(f"    {status} Base Stationary: vx={v_lin:.3f} m/s, wz={v_ang:.3f} rad/s")

        print("\n-------------------------------------------------------")
        if all_passed:
            print("\033[92m >>> BASE CONTROL TEST: ALL MOTIONS VERIFIED (PASS) <<<\033[0m")
        else:
            print("\033[91m >>> BASE CONTROL TEST: ONE OR MORE CHECKS FAILED <<<\033[0m")
        print("-------------------------------------------------------\n")
        return all_passed

    def move_arm_to_pose(self, target_angles, pose_name="target", duration=2.0, hold_time=1.0):
        """Commands arm to target angles and actively holds position."""
        init_pos = self.get_arm_joint_positions()

        # Publish via controller topic if active
        pose_msg = String()
        pose_msg.data = pose_name
        self.cmd_arm_pose_pub.publish(pose_msg)

        # Also actively stream trajectory to guarantee direct Gazebo response
        traj = JointTrajectory()
        traj.header.frame_id = 'world'
        traj.header.stamp = Time(sec=0, nanosec=0)
        traj.joint_names = ARM_JOINT_NAMES

        # Generate trajectory interpolation
        steps = int(duration * 25)
        dt = duration / steps
        for s in range(1, steps + 1):
            alpha = s / steps
            s_curve = 0.5 * (1.0 - math.cos(math.pi * alpha))
            pt = JointTrajectoryPoint()
            interpolated = []
            for i, name in enumerate(ARM_JOINT_NAMES):
                start_val = init_pos.get(name, 0.0)
                target_val = target_angles[i]
                interpolated.append(start_val + s_curve * (target_val - start_val))
            pt.positions = interpolated
            pt.time_from_start = Duration(sec=int(s * dt), nanosec=int(((s * dt) % 1) * 1e9))
            traj.points.append(pt)

        self.direct_traj_pub.publish(traj)

        # Hold loop
        hold_start = time.time()
        hold_traj = JointTrajectory()
        hold_traj.header.frame_id = 'world'
        hold_traj.header.stamp = Time(sec=0, nanosec=0)
        hold_traj.joint_names = ARM_JOINT_NAMES
        hold_pt = JointTrajectoryPoint()
        hold_pt.positions = [float(a) for a in target_angles]
        hold_pt.time_from_start = Duration(sec=0, nanosec=40000000)
        hold_traj.points.append(hold_pt)

        while time.time() - hold_start < (duration + hold_time):
            self.direct_traj_pub.publish(hold_traj)
            rclpy.spin_once(self, timeout_sec=0.03)
            time.sleep(0.03)

        # Return final joint states
        final_pos = self.get_arm_joint_positions()
        return init_pos, final_pos

    def test_arm_articulation(self):
        """Cycles through automotive painting poses and verifies joint articulation."""
        print("\n=======================================================")
        print(" [TEST] ARTICULATED 6-AXIS ARM KINEMATIC VERIFICATION")
        print("=======================================================")

        if not self.wait_for_data():
            print("\033[91m[FAIL] No /joint_states telemetry received. Is Gazebo running?\033[0m")
            return False

        all_passed = True
        test_sequence = ['ready', 'side_paint', 'roof_paint', 'hood_paint', 'home']

        for pose_name in test_sequence:
            targets = PRESET_POSES[pose_name]
            print(f"\n >>> Commanding Arm to '{pose_name.upper()}' Pose...")
            print(f"     Target angles: {targets}")

            init_pos, final_pos = self.move_arm_to_pose(
                targets, pose_name=pose_name, duration=1.8, hold_time=0.8
            )

            # Check each joint
            pose_passed = True
            for i, jname in enumerate(ARM_JOINT_NAMES):
                t_val = targets[i]
                f_val = final_pos.get(jname, 0.0)
                error = abs(f_val - t_val)
                # Verification tolerance: within 0.15 rad of target
                j_ok = error <= 0.15
                pose_passed = pose_passed and j_ok
                status = "\033[92mPASS\033[0m" if j_ok else "\033[91mFAIL\033[0m"
                print(f"     [{status}] {jname:18s}: Target={t_val:+.3f} rad | Actual={f_val:+.3f} rad | Error={error:.3f} rad")

            all_passed = all_passed and pose_passed

        print("\n-------------------------------------------------------")
        if all_passed:
            print("\033[92m >>> ARM ARTICULATION TEST: ALL 6 JOINTS VERIFIED (PASS) <<<\033[0m")
        else:
            print("\033[91m >>> ARM ARTICULATION TEST: ONE OR MORE POSES OUT OF TOLERANCE <<<\033[0m")
        print("-------------------------------------------------------\n")
        return all_passed

    def print_telemetry(self):
        """Displays formatted telemetry for base, wheels, and arm joints."""
        self.wait_for_data(timeout=2.0)
        rclpy.spin_once(self, timeout_sec=0.3)

        print("\n" + "=" * 65)
        print(" ROBOT SIMULATION TELEMETRY")
        print("=" * 65)

        if self.latest_odom:
            pos = self.latest_odom.pose.pose.position
            yaw = quaternion_to_yaw(self.latest_odom.pose.pose.orientation)
            lin = self.latest_odom.twist.twist.linear
            ang = self.latest_odom.twist.twist.angular
            print(f" Base Position   : X={pos.x:+.3f} m, Y={pos.y:+.3f} m, Z={pos.z:+.3f} m")
            print(f" Base Orientation: Yaw={yaw*180/math.pi:+.1f} deg ({yaw:+.3f} rad)")
            print(f" Base Velocities : vx={lin.x:+.3f} m/s, wz={ang.z:+.3f} rad/s")
        else:
            print(" Base Odometry: (No /odom message received)")

        if self.latest_joint_state:
            print("\n Arm Joint States:")
            for jname in ARM_JOINT_NAMES:
                if jname in self.latest_joint_state.name:
                    idx = self.latest_joint_state.name.index(jname)
                    p = self.latest_joint_state.position[idx]
                    v = self.latest_joint_state.velocity[idx] if idx < len(self.latest_joint_state.velocity) else 0.0
                    print(f"   {jname:22s}: {p:+.3f} rad ({p*180/math.pi:+.1f} deg) [vel: {v:+.3f}]")

            print("\n Wheel Joint States:")
            for wname in WHEEL_JOINT_NAMES:
                if wname in self.latest_joint_state.name:
                    idx = self.latest_joint_state.name.index(wname)
                    p = self.latest_joint_state.position[idx]
                    v = self.latest_joint_state.velocity[idx] if idx < len(self.latest_joint_state.velocity) else 0.0
                    print(f"   {wname:24s}: {p:+.3f} rad [vel: {v:+.3f}]")
        else:
            print("\n Joint States: (No /joint_states message received)")

        print("=" * 65 + "\n")


def main():
    parser = argparse.ArgumentParser(
        description="Comprehensive Test and Control Utility for Mobile Painting Robot."
    )
    parser.add_argument('--test-base', action='store_true', help='Test mobile base driving (fwd, rev, turns, stop)')
    parser.add_argument('--test-arm', action='store_true', help='Test arm articulation through all key poses')
    parser.add_argument('--pose', choices=list(PRESET_POSES.keys()), help='Command arm to a specific preset pose')
    parser.add_argument('--status', action='store_true', help='Display current odometry and joint telemetry')
    parser.add_argument('--all', action='store_true', help='Execute all base and arm verification tests')
    args = parser.parse_args()

    rclpy.init()
    controller = RobotTestController()

    success = True

    if args.status or (not args.test_base and not args.test_arm and not args.pose and not args.all):
        controller.print_telemetry()

    if args.test_base or args.all:
        b_ok = controller.test_base_movement()
        success = success and b_ok

    if args.test_arm or args.all:
        a_ok = controller.test_arm_articulation()
        success = success and a_ok

    if args.pose:
        print(f"\nMoving arm to '{args.pose}' pose: {PRESET_POSES[args.pose]}...")
        controller.move_arm_to_pose(PRESET_POSES[args.pose], pose_name=args.pose, duration=2.0, hold_time=2.0)
        controller.print_telemetry()

    controller.destroy_node()
    rclpy.shutdown()

    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
