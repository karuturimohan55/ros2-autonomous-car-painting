#!/usr/bin/env python3
"""
Captures comprehensive Stage 3 visual evidence from Gazebo Classic simulation:
1. complete_booth_car_robot.png: Overview showing booth, car, base, and arm.
2. robot_at_hood.png: Robot arm positioned over the front hood.
3. robot_side_paint.png: Robot arm positioned in side-painting configuration facing doors.
4. robot_roof_paint.png: Robot arm positioned in roof-painting configuration.
"""

import os
import sys
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from PIL import Image as PILImage

class EvidenceCapture(Node):
    def __init__(self):
        super().__init__('evidence_capture_node')

        self.overview_sub = self.create_subscription(
            Image, '/cell_camera/overview/image_raw', self.overview_cb, 10
        )
        self.front_sub = self.create_subscription(
            Image, '/cell_camera/front/image_raw', self.front_cb, 10
        )
        self.pose_pub = self.create_publisher(
            String, '/cmd_arm_pose', 10
        )

        self.latest_overview = None
        self.latest_front = None

    def overview_cb(self, msg: Image):
        self.latest_overview = msg

    def front_cb(self, msg: Image):
        self.latest_front = msg

    def save_image(self, msg: Image, path: str):
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        raw_bytes = bytes(msg.data)
        if msg.encoding == 'bgr8':
            img = PILImage.frombytes('RGB', (msg.width, msg.height), raw_bytes)
            b, g, r = img.split()
            img = PILImage.merge('RGB', (r, g, b))
        else:
            img = PILImage.frombytes('RGB', (msg.width, msg.height), raw_bytes)
        img.save(path)
        print(f"  [SAVED] {path}")

    def command_pose_and_capture(self, pose_name, out_overview_path, out_front_path=None, wait_sec=3.0):
        print(f"\nCommanding arm to '{pose_name}' pose...")
        msg = String()
        msg.data = pose_name
        self.pose_pub.publish(msg)

        # Allow arm to interpolate and settle
        start = time.time()
        while time.time() - start < wait_sec:
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.latest_overview is not None:
            self.save_image(self.latest_overview, out_overview_path)
        if out_front_path is not None and self.latest_front is not None:
            self.save_image(self.latest_front, out_front_path)


def main():
    rclpy.init()
    node = EvidenceCapture()

    screenshots_dir = os.path.abspath(
        os.path.join(os.path.dirname(__file__), '..', 'screenshots')
    )

    # Wait for camera feeds
    print("Connecting to Gazebo camera feeds...")
    start_wait = time.time()
    while rclpy.ok() and (node.latest_overview is None or node.latest_front is None):
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.time() - start_wait > 10.0:
            print("[FAIL] Timeout waiting for camera topics!")
            sys.exit(1)

    print("Camera feeds active.")

    # 1. Complete booth, car, mobile base, arm (Overview)
    node.command_pose_and_capture(
        'ready',
        os.path.join(screenshots_dir, 'complete_booth_car_robot.png'),
        wait_sec=2.5
    )

    # 2. Robot positioned at hood
    node.command_pose_and_capture(
        'hood_paint',
        os.path.join(screenshots_dir, 'robot_at_hood_overview.png'),
        os.path.join(screenshots_dir, 'robot_at_hood_front.png'),
        wait_sec=3.0
    )

    # 3. Side-painting configuration
    node.command_pose_and_capture(
        'side_paint',
        os.path.join(screenshots_dir, 'robot_side_paint_overview.png'),
        os.path.join(screenshots_dir, 'robot_side_paint_front.png'),
        wait_sec=3.0
    )

    # 4. Roof-painting configuration
    node.command_pose_and_capture(
        'roof_paint',
        os.path.join(screenshots_dir, 'robot_roof_paint_overview.png'),
        os.path.join(screenshots_dir, 'robot_roof_paint_front.png'),
        wait_sec=3.0
    )

    # Return to home
    node.command_pose_and_capture('home', os.path.join(screenshots_dir, 'robot_home.png'), wait_sec=2.0)

    node.destroy_node()
    rclpy.shutdown()
    print("\nEvidence capture completed successfully.")


if __name__ == '__main__':
    main()
