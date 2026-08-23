#!/usr/bin/env python3
"""Generate a dense procedural laser engraving test pattern."""

from __future__ import annotations

import math
from pathlib import Path

from PIL import Image, ImageDraw


OUT_GCODE = Path("test_gcode/ultra_complex_cyber_mandala_15mm_onoff.gcode")
OUT_PREVIEW = Path("test_gcode/ultra_complex_cyber_mandala_15mm_preview.png")

CX = 164.0
CY = 142.0
SIZE_MM = 15.0
DESIGN_SIZE_MM = 210.0
SCALE = SIZE_MM / DESIGN_SIZE_MM
R_MAX = SIZE_MM / 2.0
POWER = 1000
FEED = 900
RAPID_FEED = 3000
PREVIEW_SCALE = 50
MARGIN_PX = 30


class Writer:
    def __init__(self) -> None:
        self.lines: list[str] = []
        self.draw_paths: list[list[tuple[float, float]]] = []

    def header(self) -> None:
        self.lines.extend(
            [
                "; Ultra complex cyber mandala laser test",
                "; Fits CNC_Laser_Fw travel X=328mm Y=284mm",
                "; Size 15x15mm, center X164 Y142",
                "; Binary laser: G1 engraving uses S1000 F900",
                "G21",
                "G90",
                "G17",
                "G0 Z5.0",
                "M3 S0",
            ]
        )

    def footer(self) -> None:
        self.lines.extend(["M5", "M2"])

    def point(self, x: float, y: float) -> tuple[float, float]:
        return CX + x * SCALE, CY + y * SCALE

    def emit_path(self, pts: list[tuple[float, float]], power: int = POWER, feed: int = FEED) -> None:
        if len(pts) < 2:
            return

        power = POWER
        machine = [self.point(x, y) for x, y in pts]
        self.draw_paths.append(machine)
        x0, y0 = machine[0]
        self.lines.append(f"G0 X{x0:.3f} Y{y0:.3f}")
        x1, y1 = machine[1]
        self.lines.append(f"G1 X{x1:.3f} Y{y1:.3f} F{feed} S{power}")
        for x, y in machine[2:]:
            self.lines.append(f"G1 X{x:.3f} Y{y:.3f}")


def polar(r: float, t: float) -> tuple[float, float]:
    return r * math.cos(t), r * math.sin(t)


def closed_polar_path(fn, steps: int, phase: float = 0.0) -> list[tuple[float, float]]:
    pts = []
    for i in range(steps + 1):
        t = phase + 2.0 * math.pi * i / steps
        pts.append(polar(fn(t), t))
    return pts


def add_gear_rings(w: Writer) -> None:
    for base, amp, teeth, power in [
        (100.0, 3.8, 40, 620),
        (94.0, 2.6, 80, 520),
        (84.0, 2.2, 32, 500),
        (72.0, 1.7, 24, 470),
        (58.0, 1.2, 18, 450),
    ]:
        w.emit_path(
            closed_polar_path(
                lambda t, b=base, a=amp, n=teeth: b + a * math.sin(n * t) + 0.7 * a * math.sin(3 * n * t),
                960,
            ),
            power=power,
        )


def add_spiro_knots(w: Writer) -> None:
    configs = [
        (72.0, 5.0, 0.0, 640),
        (60.0, 7.0, math.pi / 9.0, 560),
        (48.0, 9.0, math.pi / 5.0, 500),
    ]
    for radius, k, phase, power in configs:
        pts = []
        for i in range(1500):
            t = 2.0 * math.pi * i / 1499.0
            r = radius * (0.72 + 0.28 * math.sin(k * t + phase))
            twist = t + 0.34 * math.sin((k + 2.0) * t)
            pts.append(polar(r, twist))
        w.emit_path(pts, power=power)


def add_petals(w: Writer) -> None:
    petals = 28
    for ring in range(6):
        r_inner = 18.0 + ring * 11.0
        r_outer = r_inner + 26.0
        for p in range(petals):
            center = 2.0 * math.pi * (p + 0.5 * (ring % 2)) / petals
            width = math.pi / petals * (0.88 - ring * 0.055)
            pts = []
            for i in range(42):
                u = i / 41.0
                a = center - width + 2.0 * width * u
                bulge = math.sin(math.pi * u)
                r = r_inner + (r_outer - r_inner) * (bulge ** 0.55)
                r += 1.3 * math.sin(9.0 * math.pi * u + ring)
                pts.append(polar(r, a))
            w.emit_path(pts, power=430 + ring * 25)


def add_radial_lace(w: Writer) -> None:
    for i in range(144):
        t = 2.0 * math.pi * i / 144.0
        r0 = 14.0 + 8.0 * ((i % 3) / 2.0)
        r1 = 100.0 + 3.0 * math.sin(24.0 * t)
        if i % 4 == 0:
            r0 = 5.0
        x0, y0 = polar(r0, t + 0.018 * math.sin(11.0 * t))
        x1, y1 = polar(r1, t + 0.018 * math.sin(13.0 * t))
        w.emit_path([(x0, y0), (x1, y1)], power=360 if i % 4 else 480)


def add_star_polygons(w: Writer) -> None:
    for points, step, radius, power in [
        (19, 7, 39.0, 620),
        (23, 9, 52.0, 560),
        (31, 11, 66.0, 500),
        (37, 15, 82.0, 460),
    ]:
        pts = []
        idx = 0
        for _ in range(points + 1):
            a = 2.0 * math.pi * idx / points
            pts.append(polar(radius, a))
            idx = (idx + step) % points
        w.emit_path(pts, power=power)


def add_cross_hatch_maze(w: Writer) -> None:
    for layer, angle in enumerate([0.0, math.pi / 3.0, -math.pi / 3.0]):
        ca = math.cos(angle)
        sa = math.sin(angle)
        spacing = 5.2
        for row in range(-17, 18):
            v = row * spacing
            half = math.sqrt(max(0.0, 78.0 * 78.0 - v * v))
            segments = 4 + (row + layer) % 5
            for seg in range(segments):
                u0 = -half + (2.0 * half) * seg / segments
                u1 = -half + (2.0 * half) * (seg + 0.55) / segments
                wave0 = 1.7 * math.sin(0.17 * u0 + row)
                wave1 = 1.7 * math.sin(0.17 * u1 + row)
                p0 = (u0 * ca - (v + wave0) * sa, u0 * sa + (v + wave0) * ca)
                p1 = (u1 * ca - (v + wave1) * sa, u1 * sa + (v + wave1) * ca)
                w.emit_path([p0, p1], power=330 + layer * 35)


def add_center_engine(w: Writer) -> None:
    for radius in [4, 7, 10, 13, 17, 21, 25, 29]:
        w.emit_path(closed_polar_path(lambda _t, r=radius: r, 240), power=650)

    for teeth in [10, 14, 18]:
        pts = []
        for i in range(teeth * 2 + 1):
            r = 31.0 if i % 2 == 0 else 22.0
            a = math.pi * i / teeth
            pts.append(polar(r, a))
        w.emit_path(pts, power=700)


def write_preview(paths: list[list[tuple[float, float]]]) -> None:
    width_px = int(SIZE_MM * PREVIEW_SCALE) + 2 * MARGIN_PX
    image = Image.new("L", (width_px, width_px), 255)
    draw = ImageDraw.Draw(image)

    def tx(pt: tuple[float, float]) -> tuple[int, int]:
        x, y = pt
        local_x = x - (CX - R_MAX)
        local_y = (CY + R_MAX) - y
        return int(round(local_x * PREVIEW_SCALE)) + MARGIN_PX, int(round(local_y * PREVIEW_SCALE)) + MARGIN_PX

    for path in paths:
        if len(path) >= 2:
            draw.line([tx(p) for p in path], fill=0, width=1)

    OUT_PREVIEW.parent.mkdir(parents=True, exist_ok=True)
    image.save(OUT_PREVIEW)


def main() -> None:
    w = Writer()
    w.header()
    add_gear_rings(w)
    add_spiro_knots(w)
    add_petals(w)
    add_radial_lace(w)
    add_star_polygons(w)
    add_cross_hatch_maze(w)
    add_center_engine(w)
    w.footer()

    OUT_GCODE.parent.mkdir(parents=True, exist_ok=True)
    OUT_GCODE.write_text("\n".join(w.lines) + "\n", encoding="ascii")
    write_preview(w.draw_paths)

    xs = []
    ys = []
    for path in w.draw_paths:
        for x, y in path:
            xs.append(x)
            ys.append(y)
    print(f"wrote {OUT_GCODE}")
    print(f"preview {OUT_PREVIEW}")
    print(f"lines={len(w.lines)} bytes={OUT_GCODE.stat().st_size}")
    print(f"bounds X{min(xs):.3f}..{max(xs):.3f} Y{min(ys):.3f}..{max(ys):.3f}")


if __name__ == "__main__":
    main()
