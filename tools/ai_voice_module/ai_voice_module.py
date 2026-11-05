#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""创乐博 CI1302 AI 语音识别模块 —— 串口协议驱动 + 命令行工具。

模块在 USB 上是一个复合设备（内部带一个 4 口 HUB）：
  * CH340 USB 转串口  (1a86:7523) -> /dev/ttyCH341USB0（Jetson/老内核）或 /dev/ttyUSB0
  * USB 声卡          (0c76:161f) -> arecord / aplay 里的 "USB PnP Audio Device"

串口：115200 8N1，帧固定 5 字节：AA 55 CMD DATA FB

  AA 55 03 00 FB   唤醒（听到“小创小创 / 你好小创”后上报）
  AA 55 02 00 FB   休眠（约 15 秒没有指令后上报）
  AA 55 00 01 FB   前进
  AA 55 00 02 FB   后退
  AA 55 00 03 FB   左转
  AA 55 00 04 FB   右转
  AA 55 00 09 FB   停止
  AA 55 01 00 FB   上电/打开串口后的就绪帧
  AA 55 FF 01 FB   主机 -> 模块：播报（“叮”一声）

用法：
  python3 ai_voice_module.py listen              # 监听并打印识别结果
  python3 ai_voice_module.py listen --seconds 30
  python3 ai_voice_module.py say                 # 让模块播报一声
  python3 ai_voice_module.py raw AA55FF01FB      # 发送任意帧
  python3 ai_voice_module.py ports               # 查看找到的串口/声卡
"""
from __future__ import annotations

import argparse
import glob
import os
import sys
import time

try:
    import serial
except ImportError:  # pragma: no cover
    serial = None

HEAD = b"\xaa\x55"
TAIL = 0xFB
FRAME_LEN = 5

WAKEUP = b"\xaa\x55\x03\x00\xfb"
SLEEP = b"\xaa\x55\x02\x00\xfb"
READY = b"\xaa\x55\x01\x00\xfb"
SAY = b"\xaa\x55\xff\x01\xfb"

COMMANDS = {
    0x01: "前进",
    0x02: "后退",
    0x03: "左转",
    0x04: "右转",
    0x09: "停止",
}

EVENT_NAMES = {
    0x01: "就绪",
    0x02: "休眠",
    0x03: "唤醒",
}


def find_ports() -> list[str]:
    """返回所有看起来像 CH340 的串口设备。"""
    candidates = []
    for pattern in ("/dev/ttyCH341USB*", "/dev/ttyUSB*"):
        candidates.extend(sorted(glob.glob(pattern)))
    return candidates


def usb_ids(dev: str) -> list[str]:
    """沿 sysfs 往上找，返回这条 USB 链路上各设备的 VID:PID。

    模块内部带一个 1a86:8091 的 HUB，所以“祖先里有 1a86:8091”就是模块的串口，
    可以把它和机器人板载的其他 CH340 区分开。
    """
    name = os.path.basename(dev)
    base = f"/sys/class/tty/{name}/device"
    ids = []
    for up in ("..", "../..", "../../..", "../../../.."):
        try:
            with open(os.path.join(base, up, "idVendor")) as f:
                vid = f.read().strip()
            with open(os.path.join(base, up, "idProduct")) as f:
                pid = f.read().strip()
            ids.append(f"{vid}:{pid}")
        except OSError:
            continue
    return ids


def describe_port(dev: str) -> str:
    ids = usb_ids(dev)
    if not ids:
        return "?"
    text = ids[0]
    if "1a86:8091" in ids:
        text += "（模块内部 HUB）"
    return text


def is_module_port(dev: str) -> bool:
    ids = usb_ids(dev)
    return "1a86:7523" in ids and "1a86:8091" in ids


def pick_port(explicit: str | None = None) -> str | None:
    if explicit:
        return explicit
    ports = find_ports()
    if not ports:
        return None
    # 1) 挂在模块内部 HUB 下面的 CH340；2) 任意 CH340；3) 第一个
    for dev in ports:
        if is_module_port(dev):
            return dev
    for dev in ports:
        if "1a86:7523" in usb_ids(dev):
            return dev
    return ports[0]


class VoiceModule:
    """最小可用的模块串口驱动。"""

    def __init__(self, port: str | None = None, baud: int = 115200):
        if serial is None:
            raise RuntimeError("缺少 pyserial：pip3 install pyserial")
        self.port = pick_port(port)
        if not self.port:
            raise FileNotFoundError(
                "没有找到 /dev/ttyCH341USB* 或 /dev/ttyUSB*，"
                "请确认模块已插好（lsusb 里应有 1a86:7523）"
            )
        self.ser = serial.Serial()
        self.ser.port = self.port
        self.ser.baudrate = baud
        self.ser.bytesize = serial.EIGHTBITS
        self.ser.parity = serial.PARITY_NONE
        self.ser.stopbits = serial.STOPBITS_ONE
        self.ser.timeout = 0.05
        # 关键：打开前先把 DTR/RTS 拉低，否则 CH340 会把模块复位
        self.ser.rts = False
        self.ser.dtr = False
        self.ser.open()
        self._buf = bytearray()
        self.ser.reset_input_buffer()

    # ---- 底层 -------------------------------------------------------
    def close(self) -> None:
        try:
            self.ser.close()
        except Exception:
            pass

    def write(self, frame: bytes) -> None:
        self.ser.write(frame)
        self.ser.flush()

    def say(self) -> None:
        """让模块播报（对应例程里的 bobao）。"""
        self.write(SAY)

    def _poll(self) -> bytes:
        data = self.ser.read(self.ser.in_waiting or 1)
        if data:
            self._buf.extend(data)
        return data

    def read_frame(self) -> bytes | None:
        """从缓冲区里切出一个完整的 5 字节帧，找不到返回 None。"""
        buf = self._buf
        while True:
            idx = buf.find(HEAD)
            if idx < 0:
                # 保留最后一个可能是帧头的字节
                if len(buf) > 1:
                    del buf[:-1]
                return None
            if idx > 0:
                del buf[:idx]
            if len(buf) < FRAME_LEN:
                return None
            frame = bytes(buf[:FRAME_LEN])
            if frame[4] != TAIL:
                del buf[:1]  # 假帧头，跳过继续找
                continue
            del buf[:FRAME_LEN]
            return frame

    def read_event(self) -> tuple[int, int] | None:
        """返回 (cmd, data)；没有完整帧返回 None。"""
        self._poll()
        frame = self.read_frame()
        if frame is None:
            return None
        return frame[2], frame[3]

    def events(self, seconds: float):
        """持续产出 (cmd, data, 时间戳)。"""
        end = time.time() + seconds
        while time.time() < end:
            ev = self.read_event()
            if ev is not None:
                yield ev[0], ev[1], time.time()
            else:
                time.sleep(0.01)


def frame_text(cmd: int, data: int) -> str:
    if cmd == 0x00:
        return f"命令 0x{data:02X} {COMMANDS.get(data, '（未收录）')}"
    if cmd == 0xFF:
        return f"播报/设置 0x{data:02X}"
    return f"事件 0x{cmd:02X} {EVENT_NAMES.get(cmd, '')} 参数 0x{data:02X}"


# ---- 命令行 ---------------------------------------------------------
def cmd_ports(_args) -> int:
    print("串口：")
    ports = find_ports()
    if not ports:
        print("  （无）")
    for dev in ports:
        print(f"  {dev}  USB {describe_port(dev)}")
    print("声卡：")
    sys.stdout.flush()
    os.system("arecord -l 2>/dev/null | sed 's/^/  /'")
    return 0


def cmd_listen(args) -> int:
    mod = VoiceModule(args.port)
    print(f"串口 {mod.port} @115200，监听 {args.seconds:g} 秒，Ctrl-C 结束")
    try:
        for cmd, data, ts in mod.events(args.seconds):
            print(f"{time.strftime('%H:%M:%S', time.localtime(ts))}  "
                  f"AA 55 {cmd:02X} {data:02X} FB   {frame_text(cmd, data)}")
    except KeyboardInterrupt:
        pass
    finally:
        mod.close()
    return 0


def cmd_say(args) -> int:
    mod = VoiceModule(args.port)
    try:
        mod.say()
        print(f"已向 {mod.port} 发送播报帧 AA 55 FF 01 FB")
    finally:
        mod.close()
    return 0


def cmd_raw(args) -> int:
    hexstr = args.hex.replace(" ", "").replace(",", "")
    if len(hexstr) % 2:
        raise SystemExit("十六进制长度必须是偶数")
    frame = bytes.fromhex(hexstr)
    mod = VoiceModule(args.port)
    try:
        mod.write(frame)
        print(f"已发送 {frame.hex(' ').upper()}")
    finally:
        mod.close()
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="创乐博 CI1302 AI 语音模块工具")
    parser.add_argument("--port", help="串口设备，默认自动查找")
    sub = parser.add_subparsers(dest="cmd", required=True)

    p_listen = sub.add_parser("listen", help="监听识别结果")
    p_listen.add_argument("--seconds", type=float, default=60)
    p_listen.set_defaults(func=cmd_listen)

    p_say = sub.add_parser("say", help="让模块播报")
    p_say.set_defaults(func=cmd_say)

    p_raw = sub.add_parser("raw", help="发送任意帧")
    p_raw.add_argument("hex")
    p_raw.set_defaults(func=cmd_raw)

    p_ports = sub.add_parser("ports", help="列出串口和声卡")
    p_ports.set_defaults(func=cmd_ports)

    args = parser.parse_args(argv)
    try:
        return args.func(args)
    except (FileNotFoundError, RuntimeError) as exc:
        print(f"错误：{exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
