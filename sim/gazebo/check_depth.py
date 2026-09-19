"""Score Ster-Vis depth pixel by pixel on one live frame from the sim.

Grabs a stereo pair and the true depth rendered at the same instant, runs the
same pipeline avoid.py uses (without the confidence filter) and prints the
error per distance band. Also writes output/check_depth.png: stereo depth
beside the true depth.

    python check_depth.py            # pi5 preset
    python check_depth.py quality    # any Ster-Vis preset
"""

import sys
import time
from pathlib import Path

import cv2
import numpy as np

from avoid import Mailbox, ideal_calibration  # also puts Ster-Vis on sys.path
from gz.msgs10.camera_info_pb2 import CameraInfo
from gz.msgs10.image_pb2 import Image
from gz.transport13 import Node
from stereo_vision.depth import colorize_depth
from stereo_vision.live import DepthPipeline
from stereo_vision.presets import get_preset
from stereo_vision.sources import StereoFrame


def main() -> None:
    preset = sys.argv[1] if len(sys.argv) > 1 else "pi5"
    out = Path(__file__).parent / "output"
    out.mkdir(exist_ok=True)
    box = Mailbox()
    node = Node()
    node.subscribe(Image, "/stereo/left/image", box.on_image("left"))
    node.subscribe(Image, "/stereo/right/image", box.on_image("right"))
    node.subscribe(Image, "/stereo/truth/depth", box.on_truth)
    node.subscribe(CameraInfo, "/stereo/left/camera_info", box.on_info)

    deadline = time.time() + 60
    pair = None
    while pair is None or box.truth_at(pair[0]) is None:
        if time.time() > deadline:
            sys.exit("no frames - is the server running?")
        pair = box.newest_pair(-1) or pair
        time.sleep(0.05)
    stamp, left, right = pair
    truth = box.truth_at(stamp)

    pipeline = DepthPipeline(ideal_calibration(box.info), get_preset(preset), min_distance_mm=450)
    result = pipeline.process(StereoFrame(cv2.cvtColor(left, cv2.COLOR_RGB2GRAY),
                                          cv2.cvtColor(right, cv2.COLOR_RGB2GRAY)))
    depth = result.depth_mm
    height, width = depth.shape
    true_mm = cv2.resize(truth, (width, height), interpolation=cv2.INTER_NEAREST) * 1000
    both = np.isfinite(depth) & np.isfinite(true_mm) & (true_mm < 8000)
    print(f"sim time {stamp:.1f} s, preset {preset}: {np.isfinite(depth).mean() * 100:.0f}% of pixels have depth")
    for low, high in ((0, 1000), (1000, 2000), (2000, 3000), (3000, 5000), (5000, 8000)):
        band = both & (true_mm >= low) & (true_mm < high)
        if band.any():
            error = depth[band] - true_mm[band]
            print(f"  {low / 1000:.0f}-{high / 1000:.0f} m: {band.sum():6d} px, median error {np.median(error):+5.0f} mm, "
                  f"median |error| {np.median(np.abs(error)):4.0f} mm")
    cv2.imwrite(str(out / "check_depth.png"),
                cv2.hconcat([colorize_depth(depth, 300, 5000), colorize_depth(true_mm, 300, 5000)]))


if __name__ == "__main__":
    main()
