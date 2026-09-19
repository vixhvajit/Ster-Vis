"""Inspect a depth map or point cloud.

Depth maps (.png 16-bit mm, or .npy float mm) open in an OpenCV window:

  move the mouse   read the depth under the cursor
  click            print that pixel's depth; with --calibration, two clicks
                   also print the real-world distance between the points
  c                cycle colour maps
  s                save what is on screen as a PNG next to the input
  q or ESC         quit

Point clouds (.ply) open in Open3D if it is installed
(pip install -r requirements-viewer.txt). Without it, use the browser viewer
in viewer/index.html, MeshLab or CloudCompare; see the README.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from stereo_vision.calibration import load_calibration  # noqa: E402
from stereo_vision.depth import load_depth  # noqa: E402

COLORMAPS = [
    ("turbo", cv2.COLORMAP_TURBO),
    ("viridis", cv2.COLORMAP_VIRIDIS),
    ("inferno", cv2.COLORMAP_INFERNO),
    ("grey", None),
]
WINDOW = "depth (q quits, c colours, s saves)"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("path", type=Path, help="depth .png / .npy, or a .ply point cloud")
    parser.add_argument(
        "--calibration",
        type=Path,
        default=None,
        help="stereo .npz; enables click-to-measure distances in millimetres",
    )
    parser.add_argument("--near", type=float, default=None, help="colour scale near end, mm")
    parser.add_argument("--far", type=float, default=None, help="colour scale far end, mm")
    return parser.parse_args()


def depth_range(depth: np.ndarray) -> tuple[float, float]:
    values = depth[np.isfinite(depth)]
    if not values.size:
        return 0.0, 1.0
    return float(np.percentile(values, 1)), float(np.percentile(values, 99))


def render(depth: np.ndarray, near: float, far: float, colormap: int | None) -> np.ndarray:
    """Colour the map with near as the warm end; no-depth pixels are black."""
    valid = np.isfinite(depth)
    scaled = np.zeros(depth.shape, np.uint8)
    span = max(far - near, 1e-6)
    scaled[valid] = np.clip((far - depth[valid]) / span * 255.0, 0, 255).astype(np.uint8)
    if colormap is None:
        image = cv2.cvtColor(scaled, cv2.COLOR_GRAY2BGR)
    else:
        image = cv2.applyColorMap(scaled, colormap)
    image[~valid] = 0
    return image


def back_project(u: int, v: int, depth_mm: float, P1: np.ndarray) -> np.ndarray:
    """3D point in the rectified left camera frame for a pixel and its depth."""
    focal, cx, cy = P1[0, 0], P1[0, 2], P1[1, 2]
    return np.array([(u - cx) * depth_mm / focal, (v - cy) * depth_mm / focal, depth_mm])


def describe(depth: np.ndarray, u: int, v: int) -> str:
    value = depth[v, u]
    if not np.isfinite(value):
        return f"({u}, {v})  no depth"
    return f"({u}, {v})  {value:.0f} mm  = {value / 1000:.3f} m"


def status_bar(image: np.ndarray, text: str, scale_text: str) -> np.ndarray:
    bar = np.full((34, image.shape[1], 3), 24, np.uint8)
    cv2.putText(bar, text, (10, 23), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (240, 240, 240), 1, cv2.LINE_AA)
    size = cv2.getTextSize(scale_text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0]
    cv2.putText(
        bar, scale_text, (image.shape[1] - size[0] - 10, 23),
        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (170, 170, 170), 1, cv2.LINE_AA,
    )
    return cv2.vconcat([image, bar])


def view_point_cloud(path: Path) -> int:
    try:
        import open3d as o3d
    except ImportError:
        print(
            "Open3D is not installed, so .ply files cannot open here.\n"
            "  install it:        pip install -r requirements-viewer.txt\n"
            "  or, no install:    open viewer/index.html in a browser and drop the file in\n"
            "  or use MeshLab / CloudCompare (install commands in the README)"
        )
        return 1
    cloud = o3d.io.read_point_cloud(str(path))
    if cloud.is_empty():
        print(f"{path} has no points")
        return 1
    print(f"{len(cloud.points)} points; drag to orbit, scroll to zoom, Q to close")
    o3d.visualization.draw_geometries([cloud], window_name=path.name)
    return 0


def main() -> int:
    args = parse_args()
    if args.path.suffix.lower() == ".ply":
        return view_point_cloud(args.path)

    depth = load_depth(args.path)
    P1 = load_calibration(args.calibration).P1 if args.calibration else None
    near, far = depth_range(depth)
    near = args.near if args.near is not None else near
    far = args.far if args.far is not None else far
    height, width = depth.shape

    state = {"cursor": (width // 2, height // 2), "colormap": 0, "picked": []}

    def on_mouse(event, x, y, flags, param):
        if not (0 <= x < width and 0 <= y < height):
            return
        state["cursor"] = (x, y)
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        print(describe(depth, x, y))
        if P1 is None or not np.isfinite(depth[y, x]):
            return
        state["picked"] = (state["picked"] + [(x, y)])[-2:]
        if len(state["picked"]) == 2:
            (u1, v1), (u2, v2) = state["picked"]
            a = back_project(u1, v1, float(depth[v1, u1]), P1)
            b = back_project(u2, v2, float(depth[v2, u2]), P1)
            print(f"  distance between the last two clicks: {np.linalg.norm(a - b):.1f} mm")

    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW, min(width, 1400), int(min(width, 1400) * (height + 34) / width))
    cv2.setMouseCallback(WINDOW, on_mouse)
    valid_share = 100.0 * np.isfinite(depth).mean()
    print(f"{args.path}: {width}x{height}, {valid_share:.1f}% of pixels have depth, "
          f"colour scale {near:.0f}-{far:.0f} mm")

    while True:
        name, colormap = COLORMAPS[state["colormap"]]
        image = render(depth, near, far, colormap)
        u, v = state["cursor"]
        cv2.drawMarker(image, (u, v), (255, 255, 255), cv2.MARKER_CROSS, 16, 1)
        for x, y in state["picked"]:
            cv2.circle(image, (x, y), 5, (255, 255, 255), 2)
        if len(state["picked"]) == 2:
            cv2.line(image, state["picked"][0], state["picked"][1], (255, 255, 255), 1, cv2.LINE_AA)
        shown = status_bar(image, describe(depth, u, v), f"{name}  near {near:.0f} mm  far {far:.0f} mm")
        cv2.imshow(WINDOW, shown)

        key = cv2.waitKey(30) & 0xFF
        if key in (ord("q"), 27) or cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
            break
        if key == ord("c"):
            state["colormap"] = (state["colormap"] + 1) % len(COLORMAPS)
        if key == ord("s"):
            target = args.path.with_name(args.path.stem + "_view.png")
            cv2.imwrite(str(target), shown)
            print(f"saved {target}")

    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
