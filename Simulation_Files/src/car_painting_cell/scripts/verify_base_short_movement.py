#!/usr/bin/env python3
import time
import math
import subprocess
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

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
            return float(out[0]), float(out[1]), float(out[5])
    except Exception:
        pass
    return None

class BaseMovementVerifier(Node):
    def __init__(self):
        super().__init__('base_movement_verifier')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
        self.latest_odom = None
        self.latest_joints = None

    def odom_cb(self, msg: Odometry):
        self.latest_odom = msg

    def joint_cb(self, msg: JointState):
        self.latest_joints = msg

    def stop_robot(self):
        zero = Twist()
        for _ in range(8):
            self.cmd_pub.publish(zero)
            rclpy.spin_once(self, timeout_sec=0.03)
            time.sleep(0.02)

    def get_wheel_positions(self):
        if not self.latest_joints:
            return {}
        wheels = ['front_left_wheel_joint', 'front_right_wheel_joint', 'rear_left_wheel_joint', 'rear_right_wheel_joint']
        res = {}
        for w in wheels:
            if w in self.latest_joints.name:
                idx = self.latest_joints.name.index(w)
                res[w] = self.latest_joints.position[idx]
        return res

    def run_cmd(self, vx, wz, duration=1.5):
        t0 = time.time()
        cmd = Twist()
        cmd.linear.x = float(vx)
        cmd.angular.z = float(wz)
        while time.time() - t0 < duration:
            self.cmd_pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.04)
            time.sleep(0.03)
        self.stop_robot()
        time.sleep(0.4)
        rclpy.spin_once(self, timeout_sec=0.1)

def main():
    rclpy.init()
    node = BaseMovementVerifier()

    t0 = time.time()
    while rclpy.ok() and (node.latest_odom is None or node.latest_joints is None) and time.time() - t0 < 5.0:
        rclpy.spin_once(node, timeout_sec=0.1)

    if node.latest_odom is None:
        print("[FAIL] No odom received!")
        return

    print("=" * 75)
    print(" BASE SHORT MOVEMENT & WHEEL VERIFICATION")
    print("=" * 75)

    tests = [
        ("SHORT FORWARD", 0.20, 0.0, 1.5),
        ("SHORT REVERSE", -0.20, 0.0, 1.5),
        ("SHORT ROTATION (CCW)", 0.0, 0.40, 1.5),
        ("SHORT ROTATION (CW)", 0.0, -0.40, 1.5),
    ]

    for name, vx, wz, dur in tests:
        node.stop_robot()
        time.sleep(0.3)
        rclpy.spin_once(node, timeout_sec=0.1)
        
        p = node.latest_odom.pose.pose.position
        yaw = quat_to_yaw(node.latest_odom.pose.pose.orientation)
        start_odom = (p.x, p.y, yaw)
        start_gt = get_gazebo_ground_truth()
        start_wheels = node.get_wheel_positions()

        node.run_cmd(vx, wz, duration=dur)

        p2 = node.latest_odom.pose.pose.position
        yaw2 = quat_to_yaw(node.latest_odom.pose.pose.orientation)
        end_odom = (p2.x, p2.y, yaw2)
        end_gt = get_gazebo_ground_truth()
        end_wheels = node.get_wheel_positions()

        d_odom = math.hypot(end_odom[0] - start_odom[0], end_odom[1] - start_odom[1])
        dyaw_odom = math.degrees((end_odom[2] - start_odom[2] + math.pi) % (2*math.pi) - math.pi)

        d_gt = math.hypot(end_gt[0] - start_gt[0], end_gt[1] - start_gt[1]) if (end_gt and start_gt) else 0.0
        dyaw_gt = math.degrees((end_gt[2] - start_gt[2] + math.pi) % (2*math.pi) - math.pi) if (end_gt and start_gt) else 0.0

        wheel_deltas = {k: end_wheels.get(k, 0.0) - start_wheels.get(k, 0.0) for k in start_wheels}

        print(f"\n--- {name} (vx={vx:+.2f} m/s, wz={wz:+.2f} rad/s, t={dur:.1f}s) ---")
        print(f"  Start Odom: x={start_odom[0]:+.4f}m, y={start_odom[1]:+.4f}m, yaw={math.degrees(start_odom[2]):+.2f}°")
        print(f"  End Odom:   x={end_odom[0]:+.4f}m, y={end_odom[1]:+.4f}m, yaw={math.degrees(end_odom[2]):+.2f}°")
        print(f"  Measured Odom: dist={d_odom:.4f}m, dyaw={dyaw_odom:+.2f}°")
        if start_gt and end_gt:
            print(f"  Start GT:   x={start_gt[0]:+.4f}m, y={start_gt[1]:+.4f}m, yaw={math.degrees(start_gt[2]):+.2f}°")
            print(f"  End GT:     x={end_gt[0]:+.4f}m, y={end_gt[1]:+.4f}m, yaw={math.degrees(end_gt[2]):+.2f}°")
            print(f"  Measured GT:   dist={d_gt:.4f}m, dyaw={dyaw_gt:+.2f}°")
            print(f"  Odom vs GT Diff: dist_err={abs(d_odom - d_gt)*1000:.2f}mm, yaw_err={abs(dyaw_odom - dyaw_gt):.2f}°")
        print(f"  Wheel Deltas (rad):")
        for wk, d_th in wheel_deltas.items():
            print(f"    {wk}: {d_th:+.3f} rad ({(d_th*0.1):+.3f}m wheel arc)")

    node.stop_robot()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
