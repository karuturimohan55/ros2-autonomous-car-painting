#!/usr/bin/env python3
"""
Screenshot Capture Script for Yuktii AI Labs - Robotic Car Painting Cell Simulation.
Captures high-resolution frames from the Gazebo simulation camera feeds:
- /cell_camera/overview/image_raw (Rear-Left Isometric Perspective)
- /cell_camera/front/image_raw (Front-Right Isometric Perspective)
- /cell_camera/top/image_raw (Top-Down Orthographic Layout View)
and saves them directly to the screenshots directory.
"""

import os
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from PIL import Image as PILImage

class MultiCameraCapture(Node):
    def __init__(self, targets):
        super().__init__('cell_multi_screenshot_capture')
        self.targets = targets  # list of (topic, output_path)
        self.received = {topic: False for topic, _ in targets}
        self.subscriptions_list = []

        for topic, path in targets:
            sub = self.create_subscription(
                Image,
                topic,
                lambda msg, t=topic, p=path: self.image_callback(msg, t, p),
                10
            )
            self.subscriptions_list.append(sub)
            self.get_logger().info(f'Subscribed to {topic} -> {os.path.basename(path)}')

    def image_callback(self, msg: Image, topic: str, path: str):
        if self.received[topic]:
            return
        self.received[topic] = True
        self.get_logger().info(f'[{topic}] Received frame: {msg.width}x{msg.height}, encoding: {msg.encoding}')

        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)

        if msg.encoding in ['rgb8', 'bgr8']:
            raw_bytes = bytes(msg.data)
            if msg.encoding == 'bgr8':
                img = PILImage.frombytes('RGB', (msg.width, msg.height), raw_bytes)
                b, g, r = img.split()
                img = PILImage.merge('RGB', (r, g, b))
            else:
                img = PILImage.frombytes('RGB', (msg.width, msg.height), raw_bytes)
        else:
            self.get_logger().error(f'[{topic}] Unsupported encoding: {msg.encoding}')
            return

        img.save(path)
        self.get_logger().info(f'[{topic}] Saved screenshot -> {path}')

    def all_done(self):
        return all(self.received.values())

def main():
    rclpy.init()

    screenshots_dir = os.path.abspath(
        os.path.join(os.path.dirname(__file__), '..', 'screenshots')
    )

    if len(sys.argv) > 2:
        # User specified topic and path
        targets = [(sys.argv[1], sys.argv[2])]
    else:
        # Default: Capture all 3 cell perspectives
        targets = [
            ('/cell_camera/overview/image_raw', os.path.join(screenshots_dir, 'car_painting_cell_overview.png')),
            ('/cell_camera/front/image_raw', os.path.join(screenshots_dir, 'car_painting_cell_front.png')),
            ('/cell_camera/top/image_raw', os.path.join(screenshots_dir, 'car_painting_cell_top.png')),
        ]

    node = MultiCameraCapture(targets)

    timeout_sec = 15.0
    start_time = node.get_clock().now()

    while rclpy.ok() and not node.all_done():
        rclpy.spin_once(node, timeout_sec=0.5)
        elapsed = (node.get_clock().now() - start_time).nanoseconds / 1e9
        if elapsed > timeout_sec:
            node.get_logger().warn(f'Timeout reached after {timeout_sec}s. Completed: {node.received}')
            break

    node.destroy_node()
    rclpy.shutdown()

    print("\nCapture Summary:")
    for topic, path in targets:
        status = "SAVED" if node.received.get(topic, False) else "FAILED"
        print(f" - {topic}: [{status}] -> {path}")

if __name__ == '__main__':
    main()
