#!/usr/bin/env python3
"""Generate a VIET NAM text + Vietnam flag on/off laser G-code test."""

from __future__ import annotations

import math
from pathlib import Path

from PIL import Image, ImageDraw


OUT_GCODE = Path("test_gcode/viet_nam_flag_150x60_onoff.gcode")
OUT_PREVIEW = Path("test_gcode/viet_nam_flag_150x60_preview.png")

POWER = 1000
FEED = 900
PREVIEW_SCALE = 5
MARGIN_PX = 40

TEXT_X = 90.0
TEXT_TOP_Y = 142.0
TEXT_BOTTOM_Y = 116.0
TEXT_SCALE = 3.0
LETTER_W = 5.0
LETTER_GAP = 1.2
SPACE_W = 2.6

FLAG_X = 176.0
FLAG_Y = 120.0
FLAG_W = 58.0
FLAG_H = 38.0
FLAG_CX = FLAG_X + FLAG_W / 2.0
FLAG_CY = FLAG_Y + FLAG_H / 2.0
STAR_R_OUT = 12.0
STAR_R_IN = STAR_R_OUT * 0.382


FONT: dict[str, list[list[tuple[float, float]]]] = {
    "V": [[(0.0, 7.0), (2.5, 0.0), (5.0, 7.0)]],
    "I": [[(0.0, 7.0), (5.0, 7.0)], [(2.5, 7.0), (2.5, 0.0)], [(0.0, 0.0), (5.0, 0.0)]],
    "E": [[(5.0, 7.0), (0.0, 7.0), (0.0, 0.0), (5.0, 0.0)], [(0.0, 3.5), (4.4, 3.5)]],
    "T": [[(0.0, 7.0), (5.0, 7.0)], [(2.5, 7.0), (2.5, 0.0)]],
    "N": [[(0.0, 0.0), (0.0, 7.0), (5.0, 0.0), (5.0, 7.0)]],
    "A": [[(0.0, 0.0), (2.5, 7.0), (5.0, 0.0)], [(1.15, 3.2), (3.85, 3.2)]],
    "M": [[(0.0, 0.0), (0.0, 7.0), (2.5, 3.6), (5.0, 7.0), (5.0, 0.0)]],
}


class Writer:
    def __init__(self) -> None:
        self.lines: list[str] = []
        self.paths: list[list[tuple[float, float]]] = []

    def header(self) -> None:
        self.lines.extend(
            [
                "; VIET NAM text + Vietnam flag on/off laser test",
                "; Size about 144x47mm, centered safely in 328x284mm travel",
                "; Binary laser: G1 engraving uses S1000 F900",
                "G21",
                "G90",
                "G17",
                "G0 Z5.0",
                "M3 S0",
            ]
        )

    def footer(self) -> None:
        self.lines.extend(
            [
                "M5",
                "; M2/M30 makes STM32 firmware home Z/X/Y after job",
                "M2",
            ]
        )

    def emit_path(self, pts: list[tuple[float, float]], feed: int = FEED) -> None:
        if len(pts) < 2:
            return

        self.paths.append(pts)
        x0, y0 = pts[0]
        self.lines.append(f"G0 X{x0:.3f} Y{y0:.3f}")
        x1, y1 = pts[1]
        self.lines.append(f"G1 X{x1:.3f} Y{y1:.3f} F{feed} S{POWER}")
        for x, y in pts[2:]:
            self.lines.append(f"G1 X{x:.3f} Y{y:.3f}")


def transform_text_point(origin_x: float, origin_y: float, point: tuple[float, float]) -> tuple[float, float]:
    x, y = point
    return origin_x + x * TEXT_SCALE, origin_y + y * TEXT_SCALE


def add_text(w: Writer, text: str, origin_x: float, origin_y: float) -> None:
    cursor_x = origin_x
    for char in text:
        if char == " ":
            cursor_x += SPACE_W * TEXT_SCALE
            continue

        for stroke in FONT[char]:
            w.emit_path([transform_text_point(cursor_x, origin_y, pt) for pt in stroke])
        cursor_x += (LETTER_W + LETTER_GAP) * TEXT_SCALE


def add_flag_border(w: Writer) -> None:
    x0 = FLAG_X
    y0 = FLAG_Y
    x1 = FLAG_X + FLAG_W
    y1 = FLAG_Y + FLAG_H
    w.emit_path([(x0, y0), (x1, y0), (x1, y1), (x0, y1), (x0, y0)])


def add_flag_hatch(w: Writer) -> None:
    blank_r = STAR_R_OUT * 1.05
    y = FLAG_Y + 2.2
    while y <= FLAG_Y + FLAG_H - 2.0:
        dy = y - FLAG_CY
        if abs(dy) < blank_r:
            gap = math.sqrt(blank_r * blank_r - dy * dy)
            left = [(FLAG_X + 2.0, y), (FLAG_CX - gap - 1.2, y)]
            right = [(FLAG_CX + gap + 1.2, y), (FLAG_X + FLAG_W - 2.0, y)]
            if left[1][0] - left[0][0] > 2.0:
                w.emit_path(left)
            if right[1][0] - right[0][0] > 2.0:
                w.emit_path(right)
        else:
            w.emit_path([(FLAG_X + 2.0, y), (FLAG_X + FLAG_W - 2.0, y)])
        y += 3.0


def star_points(cx: float, cy: float, outer: float, inner: float) -> list[tuple[float, float]]:
    pts = []
    for i in range(10):
        radius = outer if i % 2 == 0 else inner
        angle = -math.pi / 2.0 + i * math.pi / 5.0
        pts.append((cx + radius * math.cos(angle), cy + radius * math.sin(angle)))
    pts.append(pts[0])
    return pts


def add_star(w: Writer) -> None:
    outline = star_points(FLAG_CX, FLAG_CY, STAR_R_OUT, STAR_R_IN)
    w.emit_path(outline)
    for idx in [0, 2, 4, 6, 8]:
        w.emit_path([(FLAG_CX, FLAG_CY), outline[idx]])


def add_caption_rule(w: Writer) -> None:
    w.emit_path([(90.0, 110.0), (234.0, 110.0)])


def write_preview(paths: list[list[tuple[float, float]]]) -> None:
    min_x = min(x for path in paths for x, _ in path)
    max_x = max(x for path in paths for x, _ in path)
    min_y = min(y for path in paths for _, y in path)
    max_y = max(y for path in paths for _, y in path)
    width = int((max_x - min_x) * PREVIEW_SCALE) + 2 * MARGIN_PX
    height = int((max_y - min_y) * PREVIEW_SCALE) + 2 * MARGIN_PX
    image = Image.new("L", (width, height), 255)
    draw = ImageDraw.Draw(image)

    def tx(point: tuple[float, float]) -> tuple[int, int]:
        x, y = point
        return (
            int(round((x - min_x) * PREVIEW_SCALE)) + MARGIN_PX,
            int(round((max_y - y) * PREVIEW_SCALE)) + MARGIN_PX,
        )

    for path in paths:
        draw.line([tx(point) for point in path], fill=0, width=2)

    OUT_PREVIEW.parent.mkdir(parents=True, exist_ok=True)
    image.save(OUT_PREVIEW)


def main() -> None:
    w = Writer()
    w.header()
    add_text(w, "VIET", TEXT_X, TEXT_TOP_Y)
    add_text(w, "NAM", TEXT_X, TEXT_BOTTOM_Y)
    add_flag_border(w)
    add_flag_hatch(w)
    add_star(w)
    add_caption_rule(w)
    w.footer()

    OUT_GCODE.parent.mkdir(parents=True, exist_ok=True)
    OUT_GCODE.write_text("\n".join(w.lines) + "\n", encoding="ascii")
    write_preview(w.paths)
    print(f"Wrote {OUT_GCODE} with {len(w.lines)} lines")
    print(f"Wrote {OUT_PREVIEW}")


if __name__ == "__main__":
    main()
