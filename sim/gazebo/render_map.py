"""Render a warehouse map.ply as a picture: an oblique view, and a plan against the truth.

No 3D library needed: points are projected through a pinhole camera and
painted far to near, so the nearest one wins each pixel. The roof is left out
so the inside can be seen.

    python render_map.py output/warehouse/map.ply --out output/warehouse/map_render.png
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).parent


def load_ply(path: Path) -> tuple[np.ndarray, np.ndarray]:
    data = Path(path).read_bytes()
    header, body = data.split(b"end_header\n", 1)
    count = next(int(line.split()[2]) for line in header.decode("ascii").splitlines()
                 if line.startswith("element vertex"))
    record = np.frombuffer(body, dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                                        ("red", "u1"), ("green", "u1"), ("blue", "u1")], count=count)
    return np.column_stack([record["x"], record["y"], record["z"]]), record["red"].copy()


def colours(points: np.ndarray, grey: np.ndarray, height: float) -> np.ndarray:
    """Height as colour, with the surface's own texture shading it."""
    level = np.clip(points[:, 2] / height, 0, 1)
    tint = cv2.applyColorMap((level * 255).astype(np.uint8), cv2.COLORMAP_VIRIDIS)[:, 0, :].astype(np.float32)
    shade = 0.45 + 0.55 * (grey.astype(np.float32) / 255.0)
    return np.clip(tint * shade[:, None], 0, 255).astype(np.uint8)


def oblique(points: np.ndarray, rgb: np.ndarray, size=(1280, 720), eye=(-15.5, -13.0, 11.0),
            target=(0.5, 0.0, 0.5), fov_deg: float = 52.0) -> np.ndarray:
    width, height = size
    eye, target = np.asarray(eye, float), np.asarray(target, float)
    forward = target - eye
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, [0.0, 0.0, 1.0])
    right /= np.linalg.norm(right)
    down = np.cross(forward, right)
    relative = points - eye
    x, y, z = relative @ right, relative @ down, relative @ forward
    focal = width / (2 * math.tan(math.radians(fov_deg) / 2))
    front = z > 0.1
    u = (focal * x[front] / z[front] + width / 2).astype(int)
    v = (focal * y[front] / z[front] + height / 2).astype(int)
    depth, colour = z[front], rgb[front]
    inside = (u >= 0) & (u < width - 1) & (v >= 0) & (v < height - 1)
    u, v, depth, colour = u[inside], v[inside], depth[inside], colour[inside]
    order = np.argsort(-depth)  # far first, so near points overwrite them
    image = np.full((height, width, 3), 250, np.uint8)
    for du in (0, 1):
        for dv in (0, 1):
            image[v[order] + dv, u[order] + du] = colour[order]
    return image


def plan(points: np.ndarray, rgb: np.ndarray, truth: dict, size: int = 720) -> np.ndarray:
    """From above, with every true box outlined, so misplaced structure shows."""
    building = truth["building"]
    span_x, span_y = 2 * building["half_x"] + 1.0, 2 * building["half_y"] + 1.0
    scale = min(size * 16 / 9 / span_x, size / span_y) * 0.98
    width, height = int(span_x * scale), int(span_y * scale)

    def px(x, y):
        return int(round(width / 2 + x * scale)), int(round(height / 2 - y * scale))

    image = np.full((height, width, 3), 250, np.uint8)
    order = np.argsort(points[:, 2])  # low first, so the tops of things are what shows
    columns = np.clip((width / 2 + points[order, 0] * scale).astype(int), 0, width - 1)
    rows = np.clip((height / 2 - points[order, 1] * scale).astype(int), 0, height - 1)
    image[rows, columns] = rgb[order]
    for item in truth["boxes"]:
        if item["name"] in ("floor", "roof"):
            continue
        cx, cy, _ = item["centre"]
        hx, hy, _ = item["half_size"]
        corners = cv2.boxPoints(((cx, cy), (2 * hx, 2 * hy), math.degrees(item["yaw"])))
        cv2.polylines(image, [np.array([px(*c) for c in corners], np.int32)], True, (40, 40, 200), 1)
    return image


def caption(image: np.ndarray, text: str) -> np.ndarray:
    (w, h), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
    cv2.rectangle(image, (6, 6), (18 + w, 18 + h), (30, 30, 30), -1)
    cv2.putText(image, text, (12, 12 + h), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
    return image


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("map", type=Path)
    parser.add_argument("--truth", type=Path, default=HERE / "worlds" / "warehouse.json")
    parser.add_argument("--out", type=Path, default=None, help="PNG to write (default: beside the map)")
    args = parser.parse_args()

    truth = json.loads(args.truth.read_text())
    points, grey = load_ply(args.map)
    below_roof = points[:, 2] < truth["building"]["height"] - 0.2
    points, grey = points[below_roof], grey[below_roof]
    rgb = colours(points, grey, truth["building"]["height"])

    view = caption(oblique(points, rgb), f"Ster-Vis map, {len(points):,} points (roof hidden)")
    top = plan(points, rgb, truth, size=view.shape[0])
    top = caption(cv2.resize(top, (view.shape[1], int(top.shape[0] * view.shape[1] / top.shape[1]))),
                  "from above; red outlines are the true boxes")
    out = args.out or args.map.with_name("map_render.png")
    cv2.imwrite(str(out), cv2.vconcat([view, top]), [cv2.IMWRITE_PNG_COMPRESSION, 9])
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
