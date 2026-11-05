#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS 2 节点：把创乐博 AI 语音模块的识别结果变成 /cmd_vel 速度指令。

  ros2 run <pkg> voice_cmd_node --ros-args -p port:=/dev/ttyUSB0

参数：
  port            串口，默认自动查找
  linear_speed    前进/后退线速度 m/s，默认 0.15
  angular_speed   转向角速度 rad/s，默认 0.5
  hold            每条指令持续秒数，默认 1.2
话题：
  /cmd_vel        geometry_msgs/Twist
  /voice_command  std_msgs/String（wake / sleep / forward / back / left / right / cmdXX）
"""
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

from ai_voice_module import VoiceModule


class VoiceCmdNode(Node):
    def __init__(self):
        super().__init__("voice_cmd_node")
        self.declare_parameter("port", "")
        self.declare_parameter("linear_speed", 0.15)
        self.declare_parameter("angular_speed", 0.5)
        self.declare_parameter("hold", 1.2)

        port = self.get_parameter("port").value or None
        self.linear = float(self.get_parameter("linear_speed").value)
        self.angular = float(self.get_parameter("angular_speed").value)
        self.hold = float(self.get_parameter("hold").value)

        self.module = VoiceModule(port)
        self.get_logger().info(f"语音模块已连接：{self.module.port} @115200")

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.evt_pub = self.create_publisher(String, "/voice_command", 10)

        self.stop_at = 0.0
        self.create_timer(0.02, self.poll_serial)
        self.create_timer(0.1, self.watchdog)
        self.get_logger().info("等待唤醒词 / 指令 ……")

    def publish_twist(self, lin: float, ang: float) -> None:
        msg = Twist()
        msg.linear.x = lin
        msg.angular.z = ang
        self.cmd_pub.publish(msg)

    def publish_event(self, name: str) -> None:
        msg = String()
        msg.data = name
        self.evt_pub.publish(msg)

    def handle_event(self, cmd: int, data: int) -> None:
        if cmd == 0x03:
            self.publish_event("wake")
            self.get_logger().info("唤醒")
            return
        if cmd == 0x02:
            self.publish_event("sleep")
            self.get_logger().info("休眠")
            return
        if cmd != 0x00:
            return

        table = {
            0x01: ("forward", self.linear, 0.0),
            0x02: ("back", -self.linear, 0.0),
            0x03: ("left", 0.0, self.angular),
            0x04: ("right", 0.0, -self.angular),
            0x09: ("stop", 0.0, 0.0),
        }
        if data not in table:
            self.publish_event(f"cmd{data:02X}")
            self.get_logger().info(f"收到未收录命令 0x{data:02X}")
            return

        name, lin, ang = table[data]
        self.publish_event(name)
        self.publish_twist(lin, ang)
        self.stop_at = 0.0 if name == "stop" else time.time() + self.hold
        self.get_logger().info(f"{name}: linear={lin:.2f} angular={ang:.2f}")

    def poll_serial(self) -> None:
        for _ in range(8):
            ev = self.module.read_event()
            if ev is None:
                break
            self.handle_event(*ev)

    def watchdog(self) -> None:
        if self.stop_at and time.time() >= self.stop_at:
            self.publish_twist(0.0, 0.0)
            self.stop_at = 0.0

    def destroy_node(self):
        self.publish_twist(0.0, 0.0)
        self.module.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VoiceCmdNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
