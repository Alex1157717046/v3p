#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
E0.2 上位机波形链路 —— 二进制帧接收 + 实时波形 + 命令下发

帧格式（与 lib/v3plab/v3p_telemetry.cpp 一致）：
    0xAA 0x55 | nch(u8) | id(u8) | float32 × nch (小端) | crc16(u16 小端)
    CRC16-CCITT (poly 0x1021, init 0xFFFF)，覆盖 nch..payload

安装依赖：
    python3 -m pip install -r tools/requirements.txt

用法：
    # 列出串口
    python3 tools/host_scope.py --list

    # 打开波形（端口名按你机器上的实际名字改）
    python3 tools/host_scope.py -p /dev/cu.usbserial-1420

    # 同时记录 CSV
    python3 tools/host_scope.py -p /dev/cu.usbserial-1420 --csv run1.csv

界面：
    上方波形，下方命令输入框（回车发送，和串口命令行一致）
    例：在输入框敲  start   就能让电机开环转起来
"""

from __future__ import annotations

import argparse
import struct
import sys
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    print("缺少 pyserial：python3 -m pip install -r tools/requirements.txt")
    sys.exit(1)

BAUD_DEFAULT = 921600
SYNC = b"\xAA\x55"
MAX_CH = 16


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
    return crc


class FrameParser:
    """把字节流切成一帧帧 float 数组。"""

    def __init__(self) -> None:
        self.buf = bytearray()
        self.frames_ok = 0
        self.frames_bad = 0

    def feed(self, data: bytes):
        self.buf.extend(data)
        out = []
        while True:
            i = self.buf.find(SYNC)
            if i < 0:
                # 没有同步头，保留最后 1 字节（可能是半个同步头）
                if len(self.buf) > 1:
                    del self.buf[:-1]
                break
            if i > 0:
                del self.buf[:i]
            if len(self.buf) < 4:
                break
            nch = self.buf[2]
            if nch == 0 or nch > MAX_CH:
                del self.buf[:2]
                self.frames_bad += 1
                continue
            need = 4 + 4 * nch + 2
            if len(self.buf) < need:
                break
            frame = bytes(self.buf[:need])
            n, fid = frame[2], frame[3]
            payload = frame[4 : 4 + 4 * n]
            crc_rx = frame[4 + 4 * n] | (frame[4 + 4 * n + 1] << 8)
            if crc16(frame[2 : 4 + 4 * n]) != crc_rx:
                del self.buf[:2]
                self.frames_bad += 1
                continue
            vals = struct.unpack("<%df" % n, payload)
            out.append((fid, vals))
            self.frames_ok += 1
            del self.buf[:need]
        return out


DEFAULT_NAMES = [
    "i_a", "i_b", "i_c", "theta", "vbus", "agc",
    "d1", "d2", "d3", "d4", "d5", "d6", "d7", "d8", "d9", "d10",
]


def list_ports_cli():
    print("可用串口：")
    for p in list_ports.comports():
        print(f"  {p.device:32s} {p.description}")
    print("\n提示：macOS 上选 cu.* 那个（例如 /dev/cu.usbserial-1420），不要选 tty.*")


def run_gui(port: str, baud: int, csv_path: str | None, window_s: float):
    try:
        import numpy as np
        import pyqtgraph as pg
        from pyqtgraph.Qt import QtCore, QtWidgets
    except ImportError as e:
        print("缺少绘图依赖：", e)
        print("请执行：python3 -m pip install -r tools/requirements.txt")
        print("（只想看文本的话可以用：pio device monitor -b %d）" % baud)
        sys.exit(1)

    ser = serial.Serial(port, baud, timeout=0)
    parser = FrameParser()

    app = pg.mkQApp("V3P Scope")
    win = QtWidgets.QWidget()
    win.setWindowTitle(f"V3P 波形 — {port} @ {baud}")
    layout = QtWidgets.QVBoxLayout(win)

    plot = pg.PlotWidget()
    plot.showGrid(x=True, y=True, alpha=0.3)
    plot.setLabel("bottom", "时间", units="s")
    plot.addLegend()
    layout.addWidget(plot, stretch=1)

    cmd = QtWidgets.QLineEdit()
    cmd.setPlaceholderText("输入串口命令后回车（如 start / stop / mode svpwm7 / m 0.9）")
    layout.addWidget(cmd)

    status = QtWidgets.QLabel("...")
    layout.addWidget(status)

    win.resize(1100, 620)
    win.show()

    curves = {}
    buffers: dict[int, list] = {}
    t0 = time.time()

    csv_f = None
    if csv_path:
        csv_f = open(csv_path, "w", buffering=1)
        csv_f.write("t," + ",".join(DEFAULT_NAMES) + "\n")

    def send_cmd():
        text = cmd.text().strip()
        if text:
            ser.write((text + "\n").encode())
            print(">>>", text)
        cmd.clear()

    cmd.returnPressed.connect(send_cmd)

    def update():
        nonlocal t0
        data = ser.read(65536)
        if data:
            for _fid, vals in parser.feed(data):
                t = time.time() - t0
                n = len(vals)
                if not buffers:
                    for i in range(n):
                        buffers[i] = []
                        name = DEFAULT_NAMES[i] if i < len(DEFAULT_NAMES) else f"ch{i}"
                        curves[i] = plot.plot(pen=pg.intColor(i, hues=max(6, n)), name=name)
                for i in range(min(n, len(buffers))):
                    buffers[i].append((t, vals[i]))
                if csv_f:
                    csv_f.write(f"{t:.6f}," + ",".join(f"{v:.6g}" for v in vals) + "\n")

        # 只保留时间窗内的数据
        tnow = time.time() - t0
        for i, buf in buffers.items():
            while buf and buf[0][0] < tnow - window_s:
                buf.pop(0)
            if buf:
                xs = [p[0] for p in buf]
                ys = [p[1] for p in buf]
                curves[i].setData(xs, ys)

        status.setText(
            f"帧: OK={parser.frames_ok}  CRC错={parser.frames_bad}   "
            f"通道数={len(buffers)}   缓冲={len(data)} B/次"
        )

    timer = QtCore.QTimer()
    timer.timeout.connect(update)
    timer.start(20)   # 50 Hz 刷新

    def on_close():
        try:
            ser.close()
        finally:
            if csv_f:
                csv_f.close()
        app.quit()

    win.closeEvent = lambda ev: on_close()
    app.exec()


def main():
    ap = argparse.ArgumentParser(description="V3P 二进制波形上位机")
    ap.add_argument("-p", "--port", help="串口设备，如 /dev/cu.usbserial-1420")
    ap.add_argument("-b", "--baud", type=int, default=BAUD_DEFAULT)
    ap.add_argument("--list", action="store_true", help="列出串口后退出")
    ap.add_argument("--csv", help="把收到的数据同时写 CSV")
    ap.add_argument("--window", type=float, default=5.0, help="波形显示时间窗（秒）")
    args = ap.parse_args()

    if args.list or not args.port:
        list_ports_cli()
        if not args.port:
            return
    run_gui(args.port, args.baud, args.csv, args.window)


if __name__ == "__main__":
    main()
