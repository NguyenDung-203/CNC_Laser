#!/usr/bin/env python3
"""Convert an alpha PNG line-art image into simple laser raster G-code."""

from __future__ import annotations

import argparse
from pathlib import Path

from PIL import Image


def clamp(value: int, low: int, high: int) -> int:
    return max(low, min(high, value))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--preview", type=Path)
    parser.add_argument("--width-mm", type=float, default=180.0)
    parser.add_argument("--machine-x-mm", type=float, default=328.0)
    parser.add_argument("--machine-y-mm", type=float, default=284.0)
    parser.add_argument("--step-mm", type=float, default=0.30)
    parser.add_argument("--threshold", type=int, default=80)
    parser.add_argument("--power", type=int, default=650)
    parser.add_argument("--feed", type=int, default=900)
    parser.add_argument("--min-run-mm", type=float, default=0.25)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    image = Image.open(args.input).convert("RGBA")
    bbox = image.getchannel("A").getbbox()
    if bbox is None:
        raise SystemExit("input image has no opaque pixels")

    image = image.crop(bbox)
    aspect = image.height / image.width
    width_mm = args.width_mm
    height_mm = width_mm * aspect
    if height_mm > args.machine_y_mm:
        height_mm = args.machine_y_mm
        width_mm = height_mm / aspect

    cols = max(1, int(round(width_mm / args.step_mm)))
    rows = max(1, int(round(height_mm / args.step_mm)))
    width_mm = cols * args.step_mm
    height_mm = rows * args.step_mm
    origin_x = (args.machine_x_mm - width_mm) / 2.0
    origin_y = (args.machine_y_mm - height_mm) / 2.0
    power = clamp(args.power, 0, 1000)
    threshold = clamp(args.threshold, 0, 255)
    min_run_cols = max(1, int(round(args.min_run_mm / args.step_mm)))

    resample = getattr(Image, "Resampling", Image).LANCZOS
    raster = image.resize((cols, rows), resample)
    preview = Image.new("L", (cols, rows), 255)
    runs_by_row: list[list[tuple[int, int]]] = []
    total_runs = 0

    for y in range(rows):
        row_runs: list[tuple[int, int]] = []
        run_start: int | None = None
        for x in range(cols):
            r, g, b, a = raster.getpixel((x, y))
            lum = (299 * r + 587 * g + 114 * b) // 1000
            darkness = ((255 - lum) * a) // 255
            on = darkness >= threshold
            preview.putpixel((x, y), 0 if on else 255)

            if on and run_start is None:
                run_start = x
            elif (not on) and run_start is not None:
                if (x - run_start) >= min_run_cols:
                    row_runs.append((run_start, x - 1))
                run_start = None

        if run_start is not None and (cols - run_start) >= min_run_cols:
            row_runs.append((run_start, cols - 1))

        runs_by_row.append(row_runs)
        total_runs += len(row_runs)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", encoding="ascii", newline="\n") as gcode:
        gcode.write("; Dragon line-art raster for CNC_Laser_Fw\n")
        source_name = args.input.name.encode("ascii", "ignore").decode("ascii") or "image"
        gcode.write(f"; source={source_name}\n")
        gcode.write(f"; size={width_mm:.1f}x{height_mm:.1f}mm step={args.step_mm:.3f}mm\n")
        gcode.write(f"; threshold={threshold} power=S{power} feed=F{args.feed}\n")
        gcode.write(f"; rows={rows} cols={cols} runs={total_runs}\n")
        gcode.write("G21\n")
        gcode.write("G90\n")
        gcode.write("G17\n")
        gcode.write("G0 Z5.0\n")
        gcode.write("M3 S0\n")

        for y, row_runs in enumerate(runs_by_row):
            y_mm = origin_y + (rows - 1 - y) * args.step_mm
            if y % 2 == 0:
                iterable = row_runs
                for x0, x1 in iterable:
                    start_x = origin_x + x0 * args.step_mm
                    end_x = origin_x + x1 * args.step_mm
                    gcode.write(f"G0 X{start_x:.3f} Y{y_mm:.3f}\n")
                    gcode.write(f"G1 X{end_x:.3f} Y{y_mm:.3f} F{args.feed} S{power}\n")
            else:
                iterable = reversed(row_runs)
                for x0, x1 in iterable:
                    start_x = origin_x + x1 * args.step_mm
                    end_x = origin_x + x0 * args.step_mm
                    gcode.write(f"G0 X{start_x:.3f} Y{y_mm:.3f}\n")
                    gcode.write(f"G1 X{end_x:.3f} Y{y_mm:.3f} F{args.feed} S{power}\n")

        gcode.write("M5\n")
        gcode.write("M2\n")

    if args.preview is not None:
        args.preview.parent.mkdir(parents=True, exist_ok=True)
        preview.resize((cols * 2, rows * 2), Image.NEAREST).save(args.preview)

    print(f"wrote {args.output}")
    print(f"size_mm={width_mm:.3f}x{height_mm:.3f}")
    print(f"grid={cols}x{rows} runs={total_runs}")
    if args.preview is not None:
        print(f"preview={args.preview}")


if __name__ == "__main__":
    main()
