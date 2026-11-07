#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS 2 节点：把创乐博 AI 语音模块的识别结果变成 /cmd_vel 速度指令。

  python3 voice_cmd_node.py --ros-args -p port:=/dev/ttyUSB0

参数：
  port                 串口，默认自动查找
  linear_speed         前进/后退线速度 m/s，默认 0.50（比键盘慢）
  angular_speed        左转/右转角速度 rad/s，默认 1.50
  hold                 每条指令持续秒数，默认 0.5；到时间自动补零
  max_linear_speed     线速度硬上限 m/s，默认 0.70
  max_angular_speed    角速度硬上限 rad/s，默认 2.0
话题：
  /cmd_vel        geometry_msgs/Twist
  /voice_command  std_msgs/String（wake / sleep / forward / back / left / right / stop / cmdXX）
"""
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

from ai_voice_module import VoiceModule


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class VoiceCmdNode(Node):
    def __init__(self):
        super().__init__("voice_cmd_node")
        self.declare_parameter("port", "")
        self.declare_parameter("linear_speed", 0.50)
        self.declare_parameter("angular_speed", 1.50)
        self.declare_parameter("uturn_angular_speed", 1.00)
        self.declare_parameter("hold", 0.5)
        self.declare_parameter("max_linear_speed", 0.70)
        self.declare_parameter("max_angular_speed", 2.0)

        port = self.get_parameter("port").value or None
        max_linear = float(self.get_parameter("max_linear_speed").value)
        max_angular = float(self.get_parameter("max_angular_speed").value)

        # 语音控制默认慢速，且参数本身再被硬上限限制，避免误配置后乱跑。
        self.linear = clamp(float(self.get_parameter("linear_speed").value), 0.01, max_linear)
        self.angular = clamp(float(self.get_parameter("angular_speed").value), 0.01, max_angular)
        self.uturn_angular = clamp(
            float(self.get_parameter("uturn_angular_speed").value), 0.1, max_angular
        )
        self.hold = clamp(float(self.get_parameter("hold").value), 0.2, 2.0)

        self.port = port
        self.module = None
        self.next_reconnect = 0.0
        self.connect_module()
        self.get_logger().info(
            f"语音运动速度：linear={self.linear:.2f} m/s, "
            f"angular={self.angular:.2f} rad/s, "
            f"uturn={self.uturn_angular:.2f} rad/s, hold={self.hold:.2f}s"
        )

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.evt_pub = self.create_publisher(String, "/voice_command", 10)

        self.stop_at = 0.0
        self.current_lin = 0.0
        self.current_lat = 0.0
        self.current_ang = 0.0
        self.create_timer(0.02, self.poll_serial)
        self.create_timer(0.1, self.watchdog)
        self.get_logger().info("等待唤醒词 / 指令 ……")

    def connect_module(self) -> bool:
        try:
            self.module = VoiceModule(self.port)
            self.get_logger().info(f"语音模块已连接：{self.module.port} @115200")
            return True
        except Exception as exc:  # module unplugged / busy
            self.module = None
            self.get_logger().warn(f"语音模块连接失败：{exc}")
            return False

    def stop_motion(self) -> None:
        self.current_lin = 0.0
        self.current_lat = 0.0
        self.current_ang = 0.0
        self.stop_at = 0.0
        self.publish_twist(0.0, 0.0, 0.0)

    def publish_twist(self, lin_x: float, lin_y: float, ang_z: float) -> None:
        msg = Twist()
        msg.linear.x = lin_x
        msg.linear.y = lin_y
        msg.angular.z = ang_z
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
            # 模块休眠后不应继续保持速度；立即停车，防止无人看管时乱跑。
            self.publish_event("sleep")
            self.current_lin = 0.0
            self.current_lat = 0.0
            self.current_ang = 0.0
            self.publish_twist(0.0, 0.0, 0.0)
            self.stop_at = 0.0
            self.get_logger().info("休眠，已停车")
            return
        if cmd != 0x00:
            return

        # 这台底盘的 angular.z 实际表现为横移，linear.y 实际表现为转向，
        # 与标准 ROS 坐标不一致；因此转向命令走 linear.y，横移走 angular.z。
        table = {
            0x01: ("forward", self.linear, 0.0, 0.0, self.hold),
            0x02: ("back", -self.linear, 0.0, 0.0, self.hold),
            0x03: ("left", 0.0, self.linear, 0.0, self.hold),
            0x04: ("right", 0.0, -self.linear, 0.0, self.hold),
            # 0x1E = 掉头；用转向上限约 3.2 秒，降低电流/USB 跌落。
            0x1E: ("uturn", 0.0, self.linear, 0.0, 3.2),
            0x09: ("stop", 0.0, 0.0, 0.0, 0.0),
        }
        if data not in table:
            self.publish_event(f"cmd{data:02X}")
            self.get_logger().info(f"收到未收录命令 0x{data:02X}")
            return

        name, lin, lat, ang, duration = table[data]
        self.publish_event(name)
        self.current_lin = lin
        self.current_lat = lat
        self.current_ang = ang
        self.publish_twist(lin, lat, ang)
        self.stop_at = 0.0 if name == "stop" else time.monotonic() + duration
        self.get_logger().info(
            f"{name}: x={lin:.2f} y={lat:.2f} z={ang:.2f} duration={duration:.2f}s"
        )

    def poll_serial(self) -> None:
        if self.module is None:
            now = time.monotonic()
            if now < self.next_reconnect:
                return
            self.next_reconnect = now + 2.0
            self.connect_module()
            return

        try:
            for _ in range(8):
                ev = self.module.read_event()
                if ev is None:
                    break
                self.handle_event(*ev)
        except Exception as exc:
            # USB 模块可能因电机干扰/供电跌落而掉线；重连但不要立刻打断动作，
            # 让 watchdog 在本次动作时长结束后正常停车（掉头需要约 3.2 秒）。
            self.get_logger().error(f"语音串口异常，稍后自动重连：{exc}")
            try:
                self.module.close()
            except Exception:
                pass
            self.module = None
            self.next_reconnect = time.monotonic() + 2.0

    def watchdog(self) -> None:
        if not self.stop_at:
            return
        now = time.monotonic()
        if now >= self.stop_at:
            self.stop_motion()
        else:
            # 连续补发，和网页键盘一样维持速度；避免单帧被其他零速消息覆盖。
            self.publish_twist(self.current_lin, self.current_lat, self.current_ang)

    def destroy_node(self):
        self.publish_twist(0.0, 0.0, 0.0)
        if self.module is not None:
            try:
                self.module.close()
            except Exception:
                pass
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
