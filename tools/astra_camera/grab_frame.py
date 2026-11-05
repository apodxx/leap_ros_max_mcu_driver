#!/usr/bin/env python3
"""Grab one color frame and one depth frame from the Astra Pro and save PNGs."""
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class Grabber(Node):
    def __init__(self):
        super().__init__("astra_grab")
        self.bridge = CvBridge()
        self.color = None
        self.depth = None
        self.create_subscription(Image, "/camera/color/image_raw", self.on_color, qos_profile_sensor_data)
        self.create_subscription(Image, "/camera/depth/image_raw", self.on_depth, qos_profile_sensor_data)

    def on_color(self, msg):
        if self.color is None:
            self.color = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def on_depth(self, msg):
        if self.depth is None:
            self.depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")


def main():
    rclpy.init()
    node = Grabber()
    end = time.time() + 15
    while time.time() < end and (node.color is None or node.depth is None):
        rclpy.spin_once(node, timeout_sec=0.5)
    if node.color is not None:
        cv2.imwrite("/tmp/astra_color.png", node.color)
        print("color", node.color.shape, "mean", float(node.color.mean()))
    else:
        print("color: none")
    if node.depth is not None:
        d = node.depth.astype(np.float32)
        valid = d[d > 0]
        print("depth", node.depth.shape, node.depth.dtype,
              "min", float(d.min()), "max", float(d.max()),
              "valid%%", 100.0 * valid.size / d.size,
              "median_valid", float(np.median(valid)) if valid.size else -1)
        vis = np.clip(d / 3000.0 * 255, 0, 255).astype(np.uint8)
        vis = cv2.applyColorMap(vis, cv2.COLORMAP_JET)
        vis[d == 0] = 0
        cv2.imwrite("/tmp/astra_depth.png", vis)
    else:
        print("depth: none")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
