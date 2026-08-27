#!/usr/bin/env python3
"""Generate a Vietnam map sticker-style on/off laser G-code test."""

from __future__ import annotations

import json
import math
import urllib.request
from pathlib import Path

from matplotlib.font_manager import FontProperties, findfont
from matplotlib.path import Path as MplPath
from matplotlib.textpath import TextPath
from PIL import Image, ImageDraw


OUT_GCODE = Path("test_gcode/vietnam_map_sticker_80x60_onoff.gcode")
OUT_PREVIEW = Path("test_gcode/vietnam_map_sticker_80x60_preview.png")
NE_URL = "https://raw.githubusercontent.com/nvkelso/natural-earth-vector/master/geojson/ne_10m_admin_0_countries.geojson"

BOX_CX = 164.0
BOX_CY = 142.0
BOX_W = 80.0
BOX_H = 60.0
BOX_X = BOX_CX - BOX_W / 2.0
BOX_Y = BOX_CY - BOX_H / 2.0
MAP_X = 11.0
MAP_Y = 3.0
MAP_W = 40.0
MAP_H = 55.0
TEXT_W = 31.0
POWER = 1000
FEED = 900
PREVIEW_SCALE = 7
MARGIN_PX = 36

Point = tuple[float, float]


class Writer:
    def __init__(self) -> None:
        self.lines: list[str] = []
        self.paths: list[list[Point]] = []

    def header(self, font_name: str, path_bounds: tuple[float, float, float, float]) -> None:
        min_x, max_x, min_y, max_y = path_bounds
        self.lines.extend(
            [
                "; Vietnam map sticker on/off laser test",
                "; Source outline: Natural Earth 10m admin-0 countries, Vietnam",
                "; Work box: 80x60mm, centered at X164 Y142",
                f"; Actual bounds: X{min_x:.3f}..{max_x:.3f} Y{min_y:.3f}..{max_y:.3f}",
                f"; Text font: {font_name}",
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

    def emit_machine_path(self, pts: list[Point]) -> None:
        pts = simplify_short_segments(pts)
        if len(pts) < 2:
            return

        self.paths.append(pts)
        x0, y0 = pts[0]
        self.lines.append(f"G0 X{x0:.3f} Y{y0:.3f}")
        x1, y1 = pts[1]
        self.lines.append(f"G1 X{x1:.3f} Y{y1:.3f} F{FEED} S{POWER}")
        for x, y in pts[2:]:
            self.lines.append(f"G1 X{x:.3f} Y{y:.3f}")


def distance(a: Point, b: Point) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def perpendicular_distance(point: Point, start: Point, end: Point) -> float:
    if start == end:
        return distance(point, start)
    x, y = point
    x1, y1 = start
    x2, y2 = end
    num = abs((y2 - y1) * x - (x2 - x1) * y + x2 * y1 - y2 * x1)
    den = math.hypot(y2 - y1, x2 - x1)
    return num / den


def rdp(points: list[Point], epsilon: float) -> list[Point]:
    if len(points) < 3:
        return points

    max_dist = 0.0
    index = 0
    for i in range(1, len(points) - 1):
        dist = perpendicular_distance(points[i], points[0], points[-1])
        if dist > max_dist:
            max_dist = dist
            index = i

    if max_dist > epsilon:
        left = rdp(points[: index + 1], epsilon)
        right = rdp(points[index:], epsilon)
        return left[:-1] + right
    return [points[0], points[-1]]


def simplify_closed(points: list[Point], epsilon: float) -> list[Point]:
    if points[0] == points[-1]:
        points = points[:-1]
    simplified = rdp(points + [points[0]], epsilon)
    if simplified[-1] != simplified[0]:
        simplified.append(simplified[0])
    return simplified


def simplify_short_segments(points: list[Point]) -> list[Point]:
    simplified: list[Point] = []
    for point in points:
        if not simplified or distance(point, simplified[-1]) >= 0.08:
            simplified.append(point)
    if len(simplified) > 2 and distance(simplified[0], simplified[-1]) < 0.08:
        simplified[-1] = simplified[0]
    return simplified


def polygon_area(points: list[Point]) -> float:
    area = 0.0
    for i, (x0, y0) in enumerate(points):
        x1, y1 = points[(i + 1) % len(points)]
        area += x0 * y1 - x1 * y0
    return abs(area) * 0.5


def load_vietnam_rings() -> list[list[Point]]:
    with urllib.request.urlopen(NE_URL, timeout=30) as response:
        data = json.load(response)

    for feature in data["features"]:
        props = feature["properties"]
        if props.get("ADMIN") == "Vietnam" or props.get("ISO_A3") == "VNM":
            geom = feature["geometry"]
            if geom["type"] == "MultiPolygon":
                rings = [poly[0] for poly in geom["coordinates"]]
            else:
                rings = [geom["coordinates"][0]]
            return [[(float(lon), float(lat)) for lon, lat in ring] for ring in rings]

    raise RuntimeError("Vietnam polygon not found in Natural Earth data")


def make_transform(rings_lonlat: list[list[Point]]):
    main = max(rings_lonlat, key=polygon_area)
    min_lon = min(lon for lon, _ in main)
    max_lon = max(lon for lon, _ in main)
    min_lat = min(lat for _, lat in main)
    max_lat = max(lat for _, lat in main)
    lat0 = math.radians((min_lat + max_lat) * 0.5)

    def project(point: Point) -> Point:
        lon, lat = point
        return ((lon - min_lon) * math.cos(lat0), lat - min_lat)

    projected_main = [project(point) for point in main]
    min_x = min(x for x, _ in projected_main)
    max_x = max(x for x, _ in projected_main)
    min_y = min(y for _, y in projected_main)
    max_y = max(y for _, y in projected_main)
    sx = MAP_W / (max_x - min_x)
    sy = MAP_H / (max_y - min_y)

    def transform(point: Point) -> Point:
        x, y = project(point)
        local = (MAP_X + (x - min_x) * sx, MAP_Y + (y - min_y) * sy)
        return (BOX_X + local[0], BOX_Y + local[1])

    return transform


def star_points(cx: float, cy: float, outer: float, inner: float) -> list[Point]:
    points: list[Point] = []
    for i in range(10):
        radius = outer if i % 2 == 0 else inner
        angle = -math.pi / 2.0 + i * math.pi / 5.0
        points.append((cx + radius * math.cos(angle), cy + radius * math.sin(angle)))
    points.append(points[0])
    return points


def horizontal_intervals(poly: list[Point], y: float) -> list[tuple[float, float]]:
    xs: list[float] = []
    for i, p0 in enumerate(poly):
        p1 = poly[(i + 1) % len(poly)]
        x0, y0 = p0
        x1, y1 = p1
        if y0 == y1:
            continue
        if (y >= min(y0, y1)) and (y < max(y0, y1)):
            t = (y - y0) / (y1 - y0)
            xs.append(x0 + t * (x1 - x0))
    xs.sort()
    return [(xs[i], xs[i + 1]) for i in range(0, len(xs) - 1, 2) if xs[i + 1] - xs[i] > 0.65]


def split_around_star(x0: float, x1: float, y: float, star_center: Point, radius: float) -> list[tuple[float, float]]:
    sx, sy = star_center
    dy = y - sy
    if abs(dy) >= radius:
        return [(x0, x1)]
    half_gap = math.sqrt(radius * radius - dy * dy)
    left = (x0, min(x1, sx - half_gap))
    right = (max(x0, sx + half_gap), x1)
    out = []
    for a, b in [left, right]:
        if b - a > 0.85:
            out.append((a, b))
    return out


def add_map_hatch(w: Writer, main_outline: list[Point], star_center: Point) -> None:
    min_y = min(y for _, y in main_outline)
    max_y = max(y for _, y in main_outline)
    y = min_y + 2.0
    flip = False
    while y < max_y - 1.2:
        for x0, x1 in horizontal_intervals(main_outline, y):
            for a, b in split_around_star(x0 + 0.5, x1 - 0.5, y, star_center, 5.0):
                w.emit_machine_path([(b, y), (a, y)] if flip else [(a, y), (b, y)])
                flip = not flip
        y += 2.25


def resolve_text_font() -> tuple[FontProperties, str]:
    prop = FontProperties(
        family=["Times New Roman", "Liberation Serif", "Nimbus Roman", "DejaVu Serif"],
        weight="bold",
        style="italic",
    )
    font_path = findfont(prop, fallback_to_default=True)
    return FontProperties(fname=font_path), Path(font_path).name


def add_text_paths(w: Writer) -> str:
    prop, font_name = resolve_text_font()
    text_path = TextPath((0.0, 0.0), "VIETNAM", size=100.0, prop=prop)
    bbox = text_path.get_extents()
    scale = TEXT_W / (bbox.x1 - bbox.x0)
    angle = math.radians(11.0)
    ca = math.cos(angle)
    sa = math.sin(angle)
    target_cx = BOX_X + 48.0
    target_cy = BOX_Y + 6.7
    raw_cx = (bbox.x0 + bbox.x1) * 0.5
    raw_cy = (bbox.y0 + bbox.y1) * 0.5

    current: list[Point] = []
    start: Point | None = None

    def transform(vertex: Point) -> Point:
        x = (vertex[0] - raw_cx) * scale
        y = (vertex[1] - raw_cy) * scale
        return (target_cx + x * ca - y * sa, target_cy + x * sa + y * ca)

    for vertices, code in text_path.iter_segments(curves=False, simplify=False):
        point = transform((float(vertices[0]), float(vertices[1])))
        if code == MplPath.MOVETO:
            if len(current) >= 2:
                w.emit_machine_path(current)
            current = [point]
            start = point
        elif code == MplPath.LINETO:
            current.append(point)
        elif code == MplPath.CLOSEPOLY:
            if start is not None:
                current.append(start)
            if len(current) >= 2:
                w.emit_machine_path(current)
            current = []
            start = None

    if len(current) >= 2:
        w.emit_machine_path(current)

    return font_name


def scale_about(points: list[Point], center: Point, sx: float, sy: float) -> list[Point]:
    cx, cy = center
    return [(cx + (x - cx) * sx, cy + (y - cy) * sy) for x, y in points]


def bounds(paths: list[list[Point]]) -> tuple[float, float, float, float]:
    min_x = min(x for path in paths for x, _ in path)
    max_x = max(x for path in paths for x, _ in path)
    min_y = min(y for path in paths for _, y in path)
    max_y = max(y for path in paths for _, y in path)
    return min_x, max_x, min_y, max_y


def write_preview(paths: list[list[Point]]) -> None:
    width = int(BOX_W * PREVIEW_SCALE) + 2 * MARGIN_PX
    height = int(BOX_H * PREVIEW_SCALE) + 2 * MARGIN_PX
    image = Image.new("L", (width, height), 255)
    draw = ImageDraw.Draw(image)

    def tx(point: Point) -> tuple[int, int]:
        x, y = point
        return (
            int(round((x - BOX_X) * PREVIEW_SCALE)) + MARGIN_PX,
            int(round((BOX_Y + BOX_H - y) * PREVIEW_SCALE)) + MARGIN_PX,
        )

    draw.rectangle([tx((BOX_X, BOX_Y)), tx((BOX_X + BOX_W, BOX_Y + BOX_H))], outline=220, width=1)
    for path in paths:
        draw.line([tx(point) for point in path], fill=0, width=2)

    OUT_PREVIEW.parent.mkdir(parents=True, exist_ok=True)
    image.save(OUT_PREVIEW)


def main() -> None:
    rings = load_vietnam_rings()
    transform = make_transform(rings)
    transformed_rings = [[transform(point) for point in ring] for ring in rings]
    transformed_rings.sort(key=polygon_area, reverse=True)

    main_outline = simplify_closed(transformed_rings[0], 0.32)
    center = (
        (min(x for x, _ in main_outline) + max(x for x, _ in main_outline)) * 0.5,
        (min(y for _, y in main_outline) + max(y for _, y in main_outline)) * 0.5,
    )
    sticker_outline = scale_about(main_outline, center, 1.08, 1.035)
    star_center = transform((105.85, 21.03))

    w = Writer()
    w.emit_machine_path(sticker_outline)
    w.emit_machine_path(main_outline)
    add_map_hatch(w, main_outline, star_center)
    star = star_points(star_center[0], star_center[1], 4.7, 1.9)
    w.emit_machine_path(star)
    for point in star[0:10:2]:
        w.emit_machine_path([star_center, point])

    for ring in transformed_rings[1:]:
        if polygon_area(ring) >= 0.18:
            w.emit_machine_path(simplify_closed(ring, 0.10))

    font_name = add_text_paths(w)
    path_bounds = bounds(w.paths)
    body_lines = w.lines
    body_paths = w.paths

    out = Writer()
    out.header(font_name, path_bounds)
    out.paths = body_paths
    out.lines.extend(body_lines)
    out.footer()

    OUT_GCODE.parent.mkdir(parents=True, exist_ok=True)
    OUT_GCODE.write_text("\n".join(out.lines) + "\n", encoding="ascii")
    write_preview(body_paths)
    print(f"Wrote {OUT_GCODE} with {len(out.lines)} lines")
    print(f"Wrote {OUT_PREVIEW}")
    print(f"Bounds X{path_bounds[0]:.3f}..{path_bounds[1]:.3f} Y{path_bounds[2]:.3f}..{path_bounds[3]:.3f}")
    print(f"Font {font_name}")


if __name__ == "__main__":
    main()
