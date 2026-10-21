#!/usr/bin/env python3
import time
import math
import subprocess
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def quat_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def get_gazebo_ground_truth():
    try:
        out = subprocess.check_output(
            ["gz", "model", "-m", "mobile_painting_robot", "-p"],
            text=True, stderr=subprocess.DEVNULL
        ).strip().split()
        if len(out) >= 6:
            x = float(out[0])
            y = float(out[1])
            yaw = float(out[5])
            return (x, y, yaw)
    except Exception:
        pass
    return None


class MovementTester(Node):
    def __init__(self):
        super().__init__('movement_tester')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.latest_odom = None

    def odom_cb(self, msg: Odometry):
        self.latest_odom = msg

    def get_current_odom(self):
        rclpy.spin_once(self, timeout_sec=0.1)
        if self.latest_odom is None:
            return None
        p = self.latest_odom.pose.pose.position
        yaw = quat_to_yaw(self.latest_odom.pose.pose.orientation)
        return (p.x, p.y, yaw)

    def stop_robot(self):
        zero = Twist()
        for _ in range(8):
            self.cmd_pub.publish(zero)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.04)

    def run_command(self, vx, wz, duration=2.0):
        t0 = time.time()
        cmd = Twist()
        cmd.linear.x = float(vx)
        cmd.angular.z = float(wz)
        while time.time() - t0 < duration:
            self.cmd_pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.04)
            time.sleep(0.04)
        self.stop_robot()
        time.sleep(0.5)
        rclpy.spin_once(self, timeout_sec=0.1)


def main():
    rclpy.init()
    node = MovementTester()

    # Wait for odom
    t0 = time.time()
    while rclpy.ok() and node.latest_odom is None and time.time() - t0 < 5.0:
        rclpy.spin_once(node, timeout_sec=0.1)

    if node.latest_odom is None:
        print("[ERROR] No /odom received!")
        return

    print("=" * 70)
    print(" POST-FIX LIVE MOVEMENT TESTS (WITH CORRECTED WHEEL PARAMETERS)")
    print("=" * 70)

    tests = [
        ("FORWARD TEST", 0.30, 0.0, 2.0),
        ("REVERSE TEST", -0.30, 0.0, 2.0),
        ("IN-PLACE ROTATION TEST (CCW)", 0.0, 0.50, 2.0),
        ("IN-PLACE ROTATION TEST (CW)", 0.0, -0.50, 2.0),
    ]

    for name, vx, wz, dur in tests:
        node.stop_robot()
        time.sleep(0.4)
        odom_start = node.get_current_odom()
        gt_start = get_gazebo_ground_truth()

        print(f"\n--- {name} ---")
        print(f"Commanded: vx={vx:+.2f} m/s, wz={wz:+.2f} rad/s for {dur:.1f} s")
        print(f"Start Odom:      x={odom_start[0]:+.4f}m, y={odom_start[1]:+.4f}m, yaw={math.degrees(odom_start[2]):+.2f}deg")
        if gt_start:
            print(f"Start Gazebo GT: x={gt_start[0]:+.4f}m, y={gt_start[1]:+.4f}m, yaw={math.degrees(gt_start[2]):+.2f}deg")

        node.run_command(vx, wz, duration=dur)

        odom_end = node.get_current_odom()
        gt_end = get_gazebo_ground_truth()

        dx_odom = odom_end[0] - odom_start[0]
        dy_odom = odom_end[1] - odom_start[1]
        dist_odom = math.hypot(dx_odom, dy_odom)
        dyaw_odom = (odom_end[2] - odom_start[2] + math.pi) % (2 * math.pi) - math.pi

        print(f"End Odom:        x={odom_end[0]:+.4f}m, y={odom_end[1]:+.4f}m, yaw={math.degrees(odom_end[2]):+.2f}deg")
        print(f"Measured Odom:   dist={dist_odom:.4f}m, dyaw={math.degrees(dyaw_odom):+.2f}deg ({dyaw_odom:+.4f} rad)")

        if gt_end and gt_start:
            dx_gt = gt_end[0] - gt_start[0]
            dy_gt = gt_end[1] - gt_start[1]
            dist_gt = math.hypot(dx_gt, dy_gt)
            dyaw_gt = (gt_end[2] - gt_start[2] + math.pi) % (2 * math.pi) - math.pi
            print(f"End Gazebo GT:   x={gt_end[0]:+.4f}m, y={gt_end[1]:+.4f}m, yaw={math.degrees(gt_end[2]):+.2f}deg")
            print(f"Measured GT:     dist={dist_gt:.4f}m, dyaw={math.degrees(dyaw_gt):+.2f}deg ({dyaw_gt:+.4f} rad)")

    node.stop_robot()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
