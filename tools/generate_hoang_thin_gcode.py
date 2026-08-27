#!/usr/bin/env python3
"""Generate thin outline G-code for HOANG with a grave accent over A."""

from __future__ import annotations

import math
from pathlib import Path

from matplotlib.font_manager import FontProperties, findfont
from matplotlib.path import Path as MplPath
from matplotlib.textpath import TextPath
from PIL import Image, ImageDraw


TEXT = "HOÀNG"
OUT_GCODE = Path("test_gcode/hoang_dau_huyen_times_thin_80x60_onoff.gcode")
OUT_PREVIEW = Path("test_gcode/hoang_dau_huyen_times_thin_80x60_preview.png")

BOX_CX = 164.0
BOX_CY = 142.0
BOX_W = 80.0
BOX_H = 60.0
TEXT_W = 72.0
POWER = 1000
FEED = 900
MIN_POINT_MM = 0.08
PREVIEW_SCALE = 7
MARGIN_PX = 36

Point = tuple[float, float]


class Writer:
    def __init__(self) -> None:
        self.lines: list[str] = []
        self.paths: list[list[Point]] = []

    def header(self, font_name: str, bounds: tuple[float, float, float, float]) -> None:
        min_x, max_x, min_y, max_y = bounds
        self.lines.extend(
            [
                "; Thin Times-style HOANG on/off laser test",
                "; Text: HOANG with grave accent over A",
                "; Work box: 80x60mm, centered at X164 Y142",
                f"; Actual engraved bounds: X{min_x:.3f}..{max_x:.3f} Y{min_y:.3f}..{max_y:.3f}",
                f"; Font source: {font_name}",
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

    def emit_path(self, pts: list[Point]) -> None:
        pts = simplify_path(pts)
        if len(pts) < 2:
            return

        self.paths.append(pts)
        x0, y0 = pts[0]
        self.lines.append(f"G0 X{x0:.3f} Y{y0:.3f}")
        x1, y1 = pts[1]
        self.lines.append(f"G1 X{x1:.3f} Y{y1:.3f} F{FEED} S{POWER}")
        for x, y in pts[2:]:
            self.lines.append(f"G1 X{x:.3f} Y{y:.3f}")


def resolve_font() -> tuple[FontProperties, str]:
    prop = FontProperties(family=["Times New Roman", "Liberation Serif", "Nimbus Roman", "DejaVu Serif"])
    font_path = findfont(prop, fallback_to_default=True)
    return FontProperties(fname=font_path), Path(font_path).name


def distance(a: Point, b: Point) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def simplify_path(points: list[Point]) -> list[Point]:
    simplified: list[Point] = []
    for point in points:
        if not simplified or distance(point, simplified[-1]) >= MIN_POINT_MM:
            simplified.append(point)
    if len(simplified) > 2 and distance(simplified[0], simplified[-1]) < MIN_POINT_MM:
        simplified[-1] = simplified[0]
    return simplified


def text_paths() -> tuple[list[list[Point]], str]:
    prop, font_name = resolve_font()
    path = TextPath((0.0, 0.0), TEXT, size=100.0, prop=prop)
    bbox = path.get_extents()
    raw_w = bbox.x1 - bbox.x0
    raw_h = bbox.y1 - bbox.y0
    scale = TEXT_W / raw_w
    text_h = raw_h * scale
    offset_x = BOX_CX - TEXT_W / 2.0 - bbox.x0 * scale
    offset_y = BOX_CY - text_h / 2.0 - bbox.y0 * scale

    paths: list[list[Point]] = []
    current: list[Point] = []
    start: Point | None = None

    for vertices, code in path.iter_segments(curves=False, simplify=False):
        point = (offset_x + float(vertices[0]) * scale, offset_y + float(vertices[1]) * scale)
        if code == MplPath.MOVETO:
            if len(current) >= 2:
                paths.append(current)
            current = [point]
            start = point
        elif code == MplPath.LINETO:
            current.append(point)
        elif code == MplPath.CLOSEPOLY:
            if start is not None:
                current.append(start)
            if len(current) >= 2:
                paths.append(current)
            current = []
            start = None

    if len(current) >= 2:
        paths.append(current)

    return paths, font_name


def bounds(paths: list[list[Point]]) -> tuple[float, float, float, float]:
    min_x = min(x for path in paths for x, _ in path)
    max_x = max(x for path in paths for x, _ in path)
    min_y = min(y for path in paths for _, y in path)
    max_y = max(y for path in paths for _, y in path)
    return min_x, max_x, min_y, max_y


def write_preview(paths: list[list[Point]]) -> None:
    min_x = BOX_CX - BOX_W / 2.0
    max_x = BOX_CX + BOX_W / 2.0
    min_y = BOX_CY - BOX_H / 2.0
    max_y = BOX_CY + BOX_H / 2.0
    width = int(BOX_W * PREVIEW_SCALE) + 2 * MARGIN_PX
    height = int(BOX_H * PREVIEW_SCALE) + 2 * MARGIN_PX
    image = Image.new("L", (width, height), 255)
    draw = ImageDraw.Draw(image)

    def tx(point: Point) -> tuple[int, int]:
        x, y = point
        return (
            int(round((x - min_x) * PREVIEW_SCALE)) + MARGIN_PX,
            int(round((max_y - y) * PREVIEW_SCALE)) + MARGIN_PX,
        )

    draw.rectangle([tx((min_x, min_y)), tx((max_x, max_y))], outline=210, width=1)
    for path in paths:
        draw.line([tx(point) for point in path], fill=0, width=2)

    OUT_PREVIEW.parent.mkdir(parents=True, exist_ok=True)
    image.save(OUT_PREVIEW)


def main() -> None:
    paths, font_name = text_paths()
    path_bounds = bounds(paths)

    w = Writer()
    w.header(font_name, path_bounds)
    for path in paths:
        w.emit_path(path)
    w.footer()

    OUT_GCODE.parent.mkdir(parents=True, exist_ok=True)
    OUT_GCODE.write_text("\n".join(w.lines) + "\n", encoding="ascii")
    write_preview(w.paths)
    print(f"Wrote {OUT_GCODE} with {len(w.lines)} lines")
    print(f"Wrote {OUT_PREVIEW}")
    print(f"Bounds X{path_bounds[0]:.3f}..{path_bounds[1]:.3f} Y{path_bounds[2]:.3f}..{path_bounds[3]:.3f}")
    print(f"Font {font_name}")


if __name__ == "__main__":
    main()
