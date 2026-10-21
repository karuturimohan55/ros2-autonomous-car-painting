#!/usr/bin/env python3
import time
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def normalize_angle(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def quat_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.current_pose = None

    def odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        yaw = quat_to_yaw(msg.pose.pose.orientation)
        self.current_pose = (p.x, p.y, yaw)

    def stop_robot(self):
        zero = Twist()
        for _ in range(5):
            self.cmd_pub.publish(zero)
            rclpy.spin_once(self, timeout_sec=0.03)
            time.sleep(0.02)

    def navigate_waypoints(self, waypoints, final_yaw=None, pos_tol=0.04, yaw_tol=math.radians(3.0), timeout=45.0):
        t0 = time.time()
        for wp_idx, wp in enumerate(waypoints):
            tx, ty = wp[0], wp[1]
            is_final = (wp_idx == len(waypoints) - 1)
            target_tol = pos_tol if is_final else 0.20

            print(f"Heading to waypoint {wp_idx+1}/{len(waypoints)}: ({tx:.3f}, {ty:.3f})...")

            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.02)
                if time.time() - t0 > timeout:
                    self.stop_robot()
                    return False, "Timeout"

                cx, cy, cyaw = self.current_pose
                dx = tx - cx
                dy = ty - cy
                dist = math.hypot(dx, dy)

                if dist <= target_tol:
                    break

                target_heading = math.atan2(dy, dx)
                e_head = normalize_angle(target_heading - cyaw)

                twist = Twist()
                if abs(e_head) > math.radians(30.0):
                    # Rotate towards target
                    twist.linear.x = 0.0
                    twist.angular.z = float(math.copysign(max(0.15, min(0.50, 1.5 * abs(e_head))), e_head))
                else:
                    # Drive forward and steer
                    twist.linear.x = float(max(0.08, min(0.28, 0.6 * dist)))
                    twist.angular.z = float(math.copysign(min(0.40, 1.2 * abs(e_head)), e_head))

                self.cmd_pub.publish(twist)
                time.sleep(0.02)

        # Final yaw alignment
        if final_yaw is not None:
            print(f"Aligning to final yaw: {math.degrees(final_yaw):.1f} deg...")
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.02)
                if time.time() - t0 > timeout:
                    self.stop_robot()
                    return False, "Timeout during yaw alignment"

                cx, cy, cyaw = self.current_pose
                e_yaw = normalize_angle(final_yaw - cyaw)

                if abs(e_yaw) <= yaw_tol:
                    break

                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = float(math.copysign(max(0.08, min(0.35, 1.5 * abs(e_yaw))), e_yaw))
                self.cmd_pub.publish(twist)
                time.sleep(0.02)

        self.stop_robot()
        time.sleep(0.3)
        rclpy.spin_once(self, timeout_sec=0.1)

        cx, cy, cyaw = self.current_pose
        final_target = waypoints[-1]
        pos_err = math.hypot(final_target[0] - cx, final_target[1] - cy)
        yaw_err = abs(normalize_angle(final_yaw - cyaw)) if final_yaw is not None else 0.0
        print(f"Reached: pos=({cx:.4f}, {cy:.4f}), yaw={math.degrees(cyaw):.2f}deg | err: pos={pos_err*1000:.1f}mm, yaw={math.degrees(yaw_err):.2f}deg")
        return (pos_err <= pos_tol and yaw_err <= yaw_tol), f"pos_err={pos_err*1000:.1f}mm, yaw_err={math.degrees(yaw_err):.2f}deg"


def main():
    rclpy.init()
    node = WaypointNavigator()
    while rclpy.ok() and node.current_pose is None:
        rclpy.spin_once(node, timeout_sec=0.1)

    print("Initial Pose:", node.current_pose)
    target_station = (0.0, 2.00)
    target_yaw = -math.pi / 2.0

    ok, msg = node.navigate_waypoints([target_station], final_yaw=target_yaw, pos_tol=0.05, yaw_tol=math.radians(3.5))
    print("Navigation Result:", ok, msg)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
