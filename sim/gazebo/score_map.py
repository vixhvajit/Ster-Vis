"""Score a point cloud map of the warehouse against the world it was built in.

Two questions, measured separately:

- **accuracy**: how far is each mapped point from the nearest real surface?
  The warehouse is made of boxes with known poses, so this is exact geometry,
  not a comparison against another estimate.
- **completeness**: how much of what an ideal depth camera would have seen on
  the same flight did the stereo map get? The flight also records a map built
  from Gazebo's perfect depth camera, at the same poses and settings, so the
  two differ only in where the depth came from. That handles occlusion and
  field of view without modelling either: a shelf the drone never looked
  behind is missing from both.

    python score_map.py output/warehouse/map.ply --truth-map output/warehouse/truth_map.ply
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).parent
TRUTH_FILE = HERE / "worlds" / "warehouse.json"


def load_ply(path: Path) -> np.ndarray:
    """(N, 3) float32 from a binary little-endian PLY written by Ster-Vis."""
    data = Path(path).read_bytes()
    header, body = data.split(b"end_header\n", 1)
    lines = header.decode("ascii").splitlines()
    count = next(int(line.split()[2]) for line in lines if line.startswith("element vertex"))
    coloured = any(line.startswith("property uchar red") for line in lines)
    dtype = [("x", "<f4"), ("y", "<f4"), ("z", "<f4")]
    if coloured:
        dtype += [("red", "u1"), ("green", "u1"), ("blue", "u1")]
    record = np.frombuffer(body, dtype=dtype, count=count)
    return np.column_stack([record["x"], record["y"], record["z"]]).astype(np.float32)


def surface_distance(points: np.ndarray, boxes: list[dict]) -> np.ndarray:
    """Distance from each point to the nearest surface of the nearest box.

    Boxes are hollow as far as a camera is concerned - it only ever sees their
    faces - so the distance is to the surface, inside or out.
    """
    best = np.full(len(points), np.inf, np.float64)
    for item in boxes:
        centre = np.asarray(item["centre"], np.float64)
        half = np.asarray(item["half_size"], np.float64)
        yaw = float(item["yaw"])
        local = points - centre
        if yaw:
            cos, sin = np.cos(-yaw), np.sin(-yaw)
            local = np.column_stack([local[:, 0] * cos - local[:, 1] * sin,
                                     local[:, 0] * sin + local[:, 1] * cos,
                                     local[:, 2]])
        q = np.abs(local) - half
        outside = np.linalg.norm(np.maximum(q, 0.0), axis=1)
        inside = np.minimum(q.max(axis=1), 0.0)
        np.minimum(best, np.abs(outside + inside), out=best)
    return best


def nearest_distance(points: np.ndarray, queries: np.ndarray) -> np.ndarray:
    """Distance from each query to the nearest point, through OpenCV's FLANN index.

    A k-d tree over a few hundred thousand points answers in a second or two;
    the brute-force version of the same question is hours.
    """
    points = np.ascontiguousarray(points, np.float32)
    queries = np.ascontiguousarray(queries, np.float32)
    if not len(points) or not len(queries):
        return np.full(len(queries), np.inf)
    index = cv2.flann_Index(points, {"algorithm": 1, "trees": 4})  # 1 = KDTREE
    _, squared = index.knnSearch(queries, 1, params={"checks": 128})
    return np.sqrt(squared[:, 0].astype(np.float64))


def score(map_points: np.ndarray, truth_points: np.ndarray | None, boxes: list[dict],
          tolerance_m: float = 0.10) -> dict:
    """Accuracy against the true geometry, and completeness against the ideal map."""
    error = surface_distance(map_points, boxes)
    result = {
        "points": int(len(map_points)),
        "accuracy_m": {
            "median": round(float(np.median(error)), 4),
            "mean": round(float(error.mean()), 4),
            "p90": round(float(np.percentile(error, 90)), 4),
            "p99": round(float(np.percentile(error, 99)), 4),
        },
        "within_5cm_pct": round(100.0 * float((error <= 0.05).mean()), 2),
        "within_10cm_pct": round(100.0 * float((error <= 0.10).mean()), 2),
        "stray_beyond_30cm_pct": round(100.0 * float((error > 0.30).mean()), 2),
    }
    if truth_points is not None and len(truth_points):
        to_map = nearest_distance(map_points, truth_points)
        to_ideal = nearest_distance(truth_points, map_points)
        truth_error = surface_distance(truth_points, boxes)
        result.update({
            "ideal_map_points": int(len(truth_points)),
            # What the ideal sensor saw that stereo also got, and the other way
            # round: points stereo put where the ideal map has nothing.
            "completeness_pct": round(100.0 * float((to_map <= tolerance_m).mean()), 2),
            "not_in_the_ideal_map_pct": round(100.0 * float((to_ideal > tolerance_m).mean()), 2),
            "ideal_map_accuracy_m": {"median": round(float(np.median(truth_error)), 4),
                                     "p90": round(float(np.percentile(truth_error, 90)), 4)},
            "tolerance_m": tolerance_m,
        })
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("map", type=Path, help="map.ply to score")
    parser.add_argument("--truth-map", type=Path, default=None,
                        help="map.ply built from the perfect depth camera on the same flight")
    parser.add_argument("--truth", type=Path, default=TRUTH_FILE, help="warehouse.json")
    parser.add_argument("--tolerance", type=float, default=0.10,
                        help="how close a point must be to count as the same surface (default 0.1)")
    parser.add_argument("--out", type=Path, default=None, help="write the numbers here as JSON")
    args = parser.parse_args()

    truth = json.loads(args.truth.read_text())
    points = load_ply(args.map)
    ideal = load_ply(args.truth_map) if args.truth_map and args.truth_map.exists() else None
    result = score(points, ideal, truth["boxes"], args.tolerance)
    print(json.dumps(result, indent=2))
    if args.out:
        args.out.write_text(json.dumps(result, indent=2), encoding="utf-8")


if __name__ == "__main__":
    main()
