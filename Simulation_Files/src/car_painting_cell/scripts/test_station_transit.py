#!/usr/bin/env python3
import time
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np


def normalize_angle(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def quat_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class PreciseStationNavigator(Node):
    def __init__(self):
        super().__init__('precise_station_navigator')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.current_pose = None
        self.current_vel = (0.0, 0.0)

    def odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        yaw = quat_to_yaw(msg.pose.pose.orientation)
        self.current_pose = (p.x, p.y, yaw)
        self.current_vel = (msg.twist.twist.linear.x, msg.twist.twist.angular.z)

    def stop_robot(self, cycles=10):
        zero = Twist()
        for _ in range(cycles):
            self.cmd_pub.publish(zero)
            rclpy.spin_once(self, timeout_sec=0.03)
            time.sleep(0.02)

    def drive_route(self, waypoints, final_yaw, pos_tol=0.06, yaw_tol=math.radians(4.0), timeout=60.0):
        t0 = time.time()
        twist = Twist()

        for idx, (tx, ty) in enumerate(waypoints):
            is_final = (idx == len(waypoints) - 1)
            dist_threshold = pos_tol if is_final else 0.22

            print(f"  -> Segment {idx+1}/{len(waypoints)} to ({tx:.2f}, {ty:.2f}) [tol={dist_threshold:.2f}m]")

            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.02)
                if time.time() - t0 > timeout:
                    self.stop_robot()
                    return False, "Route timeout"

                cx, cy, cyaw = self.current_pose
                dx = tx - cx
                dy = ty - cy
                dist = math.hypot(dx, dy)

                if dist <= dist_threshold:
                    break

                target_heading = math.atan2(dy, dx)
                e_head = normalize_angle(target_heading - cyaw)

                if abs(e_head) > math.radians(35.0):
                    twist.linear.x = 0.0
                    twist.angular.z = float(math.copysign(max(0.12, min(0.45, 1.4 * abs(e_head))), e_head))
                else:
                    speed = max(0.06, min(0.30, 0.8 * dist)) if is_final else max(0.12, min(0.35, 0.9 * dist))
                    twist.linear.x = float(speed)
                    twist.angular.z = float(math.copysign(min(0.35, 1.2 * abs(e_head)), e_head))

                self.cmd_pub.publish(twist)
                time.sleep(0.02)

        # Yaw alignment at final station
        print(f"  -> Aligning to target yaw {math.degrees(final_yaw):.1f} deg...")
        settled_count = 0
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)
            if time.time() - t0 > timeout:
                self.stop_robot()
                return False, "Yaw alignment timeout"

            cx, cy, cyaw = self.current_pose
            e_yaw = normalize_angle(final_yaw - cyaw)

            if abs(e_yaw) <= yaw_tol:
                settled_count += 1
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_pub.publish(twist)
                if settled_count >= 10:
                    break
            else:
                settled_count = 0
                twist.linear.x = 0.0
                twist.angular.z = float(math.copysign(max(0.06, min(0.28, 1.2 * abs(e_yaw))), e_yaw))
                self.cmd_pub.publish(twist)

            time.sleep(0.02)

        self.stop_robot(cycles=15)
        time.sleep(0.2)
        rclpy.spin_once(self, timeout_sec=0.1)

        cx, cy, cyaw = self.current_pose
        target_final = waypoints[-1]
        final_pos_err = math.hypot(target_final[0] - cx, target_final[1] - cy)
        final_yaw_err = abs(normalize_angle(final_yaw - cyaw))

        success = (final_pos_err <= 0.08) and (final_yaw_err <= math.radians(5.0))
        msg = f"Pose: ({cx:.4f}, {cy:.4f}, {math.degrees(cyaw):.2f}deg) | Err: pos={final_pos_err*1000:.1f}mm, yaw={math.degrees(final_yaw_err):.2f}deg"
        return success, msg


def main():
    rclpy.init()
    node = PreciseStationNavigator()
    while rclpy.ok() and node.current_pose is None:
        rclpy.spin_once(node, timeout_sec=0.1)

    print("Initial Pose:", node.current_pose)

    # Test Transit 1: From current pose to FRONT station (3.50, 0.0, -pi)
    print("\n--- Transit 1: To FRONT Bumper Station (3.50, 0.0, -pi) ---")
    route_front = [(2.00, 2.30), (3.50, 2.30), (3.50, 0.0)]
    ok1, msg1 = node.drive_route(route_front, -math.pi, timeout=40.0)
    print("Transit 1 Result:", "PASS" if ok1 else "FAIL", msg1)

    time.sleep(1.0)

    # Test Transit 2: From FRONT to LEFT_SIDE station (0.0, 2.00, -pi/2)
    print("\n--- Transit 2: To LEFT_SIDE Station (0.0, 2.00, -pi/2) ---")
    route_left = [(3.50, 2.20), (1.50, 2.20), (0.0, 2.00)]
    ok2, msg2 = node.drive_route(route_left, -math.pi / 2.0, timeout=40.0)
    print("Transit 2 Result:", "PASS" if ok2 else "FAIL", msg2)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
