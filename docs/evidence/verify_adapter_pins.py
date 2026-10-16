#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
STM32_V3P转接板原理图 —— 连接器引脚/网名 自动核对脚本（可复现）

原理：该 PDF 由嘉立创EDA导出，是"矢量图"。每个连接器引脚 = 一段暗红色水平线(器件引脚)
      + 一段绿色水平线(导线) + 该导线正上方 6pt 处的蓝色网名文字。
      没有引出导线的引脚 = 只有暗红色引脚段，若为"禁止连接"会画一个绿色 X。
      本脚本直接解析 PDF 内容流(线条/颜色) 与 pdftotext -bbox (文字坐标)，
      按"导线y坐标 == 引脚号文字y坐标"配对引脚号，再按"文字底边贴在导线上方"配对网名。

用法：
    python3 verify_adapter_pins.py "/path/to/STM32转接板原理图.pdf"
"""
import re, subprocess, sys, tempfile, os

PDF_DEFAULT = "/Users/Zhuanz/v3p/4、原理图与PCB/STM32转接板原理图.pdf"


def pdf_words(pdf):
    with tempfile.TemporaryDirectory() as d:
        xml = os.path.join(d, "b.xml")
        subprocess.run(["pdftotext", "-bbox", pdf, xml], check=True)
        txt = open(xml, encoding="utf-8").read()
    ws = re.findall(r'<word xMin="([\d.]+)" yMin="([\d.]+)" xMax="([\d.]+)" yMax="([\d.]+)">(.*?)</word>', txt)
    return [(float(a), float(b), float(c), float(d), e) for a, b, c, d, e in ws]


def pdf_segments(pdf):
    """返回 (x1,y1pdf,x2,y2pdf,strokeRGB) 线段列表；y 为 PDF 坐标(自下而上)。"""
    data = open(pdf, "rb").read()
    s = data.find(b"stream\n") + 7
    e = data.find(b"endstream", s)
    content = data[s:e].decode("latin-1")
    stroke = (0, 0, 0)
    cur = None
    segs = []

    def flush():
        nonlocal cur
        if cur and len(cur) >= 2:
            for i in range(len(cur) - 1):
                segs.append((cur[i][0], cur[i][1], cur[i + 1][0], cur[i + 1][1], stroke))
        cur = None

    for ln in content.split("\n"):
        ln = ln.strip()
        m = re.fullmatch(r"([\d.]+) ([\d.]+) ([\d.]+) RG", ln)
        if m:
            stroke = tuple(round(float(x), 2) for x in m.groups()); continue
        m = re.fullmatch(r"([\d.-]+) ([\d.-]+) m", ln)
        if m:
            cur = [(float(m.group(1)), float(m.group(2)))]; continue
        m = re.fullmatch(r"([\d.-]+) ([\d.-]+) l", ln)
        if m:
            if cur is None:
                cur = []
            cur.append((float(m.group(1)), float(m.group(2)))); continue
        if ln in ("S", "B", "b", "s", "f", "F", "f*"):
            flush(); continue
        if ln in ("n", "W", "W*"):
            cur = None; continue
    return segs


# 连接器几何参数（单位: pt，坐标系 y 自页面顶部向下）
#  name: (引脚号文字x范围, 器件引脚暗红段x范围, 页高)
CONN = {
    "S1": dict(num_x=(174.0, 183.5), stub=(186.0, 194.0), lab_x=(190.0, 245.0)),
    "S2": dict(num_x=(80.5, 89.8), stub=(92.0, 100.5), lab_x=(99.0, 125.0)),
    "E1": dict(num_x=(293.0, 302.0), stub=(304.5, 313.0), lab_x=(312.0, 340.0)),
    "E2": dict(num_x=(376.0, 385.0), stub=(387.0, 396.0), lab_x=(390.0, 445.0)),
}


def main(pdf):
    words = pdf_words(pdf)
    segs = pdf_segments(pdf)
    H = 599.76  # A4 横向 848.16 x 599.76

    horiz, diag = [], []
    for x1, y1, x2, y2, c in segs:
        yt1, yt2 = round(H - y1, 2), round(H - y2, 2)
        if abs(yt1 - yt2) < 0.01:
            horiz.append(dict(y=yt1, x0=min(x1, x2), x1=max(x1, x2), c=c))
        elif abs(x1 - x2) >= 0.01:
            diag.append((yt1, yt2, x1, x2, c))

    for name, g in CONN.items():
        print("=" * 78)
        print(f"连接器 {name}")
        print("=" * 78)
        stubs = sorted({round(h["y"], 2) for h in horiz
                        if abs(h["c"][0] - 0.53) < 1e-6 and g["stub"][0] <= h["x0"] and h["x1"] <= g["stub"][1] + 0.1})
        greens = {}
        for h in horiz:
            if (abs(h["c"][1] - 0.53) < 1e-6 and abs(h["c"][0]) < 1e-6
                    and g["stub"][0] <= h["x0"] <= g["stub"][1] + 1.5 and h["x1"] > h["x0"] + 2):
                greens[round(h["y"], 2)] = (round(h["x0"], 2), round(h["x1"], 2))
        nums = [(w[1], w[4]) for w in words
                if g["num_x"][0] <= w[0] <= g["num_x"][1] and w[4].isdigit() and len(w[4]) <= 2]
        # X 标记：中心点落在引脚根部
        xmarks = []
        for yt1, yt2, x1, x2, c in diag:
            if c == (0.2, 0.8, 0.2) and abs(x1 - x2) > 1:
                cx, cy = (x1 + x2) / 2, (yt1 + yt2) / 2
                if g["stub"][0] <= cx <= g["stub"][1] + 4:
                    xmarks.append(round(cy, 2))
        used = set()
        print(f"{'引脚号':<6}{'引脚导线y':>10}   {'网名':<14}{'网名文字y':>10}  {'绿色导线x范围':<22}{'标记'}")
        for sy in stubs:  # 从上到下 = 引脚号从大到小
            # 引脚号文字：中心 y 最接近导线 y
            cand = [(abs((ny + 3.08) - sy), ny, t) for ny, t in nums if ny not in used]
            cand.sort()
            _, ny, pin = cand[0]
            used.add(ny)
            # 网名：文字底边(ny+6.16) 恰好贴在导线y上方(|差|<2)，且 x 落在本连接器标注区
            lab = None
            for w in words:
                if g["lab_x"][0] - 0.5 <= w[0] <= g["lab_x"][1] and abs(w[1] + 6.16 - sy) < 2.0:
                    lab = (w[4], round(w[1], 2))
                    break
            gx = greens.get(sy)
            mark = "X=禁止连接(NC)" if sy in xmarks else ("" if gx else "无水平导线")
            if mark == "无水平导线":
                # 可能靠垂直导线接到电源符号（如 S1.1/1.2 -> GND，E2.13 -> GND）
                for x1v, y1v, x2v, y2v, cv in segs:
                    if abs(x1v - x2v) < 0.01 and abs(cv[1] - 0.53) < 1e-6:
                        ytv1, ytv2 = sorted((round(H - y1v, 2), round(H - y2v, 2)))
                        if (g["stub"][0] - 2 <= x1v <= g["stub"][1] + 8
                                and ytv1 - 0.05 <= sy <= ytv2 + 0.05 and ytv2 - ytv1 > 2):
                            mark = f"垂直导线 x={x1v:.2f} (接电源/GND符号, 见放大图)"
                            break
            print(f"{pin:<6}{sy:>10.2f}   {(lab[0] if lab else '—'):<14}{(lab[1] if lab else ''):>10}  "
                  f"{str(gx) if gx else '—':<22}{mark}")
        print()


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else PDF_DEFAULT)
