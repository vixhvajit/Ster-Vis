"""Score depth-filter settings on recorded frames (avoid.py --record N).

Every setting runs the same Ster-Vis pipeline and scan on the same frames,
then compares with the true depth. Near = true range under 2 m.

  missed   near beam the stereo scan called clear (inf)  <- can cause a crash
  blind    near beam the stereo scan called unknown (nan)
  phantom  stereo beam under 2 m with nothing real within +0.5 m of it
"""

import sys
from pathlib import Path

import cv2
import numpy as np

import avoid
from avoid import scan_error
from stereo_vision.calibration import StereoCalibration
from stereo_vision.live import DepthPipeline
from stereo_vision.outputs import CameraModel, ScanConfig, build_robot_frame, laser_scan
from stereo_vision.presets import get_preset
from stereo_vision.sources import StereoFrame

frames = sorted(Path(sys.argv[1] if len(sys.argv) > 1 else "output/tune/frames").glob("*.npz"))
first = np.load(frames[0])
K = first["k"].reshape(3, 3)
h, w = first["truth"].shape


class Info:  # just what ideal_calibration reads
    class intrinsics:
        k = list(K.ravel())
    width, height = w, h


calib = avoid.ideal_calibration(Info)
tcam = CameraModel(w, h, K[0, 0], K[1, 1], K[0, 2], K[1, 2], 0.12)


def run(name, confidence, max_h, edge_blind_px=0):
    cfg = ScanConfig(beams=121, min_height_m=-(avoid.CAM_HEIGHT - 0.06), max_height_m=max_h,
                     range_min_m=0.2, range_max_m=5.0)
    pipe = DepthPipeline(calib, get_preset("pi5"), min_distance_mm=450, confidence=confidence > 0)
    cam = CameraModel.from_calibration(pipe.calibration)
    tot = dict(close=0, missed=0, blind=0, phantom=0, stereo_close=0)
    for f in frames:
        d = np.load(f)
        res = pipe.process(StereoFrame(cv2.cvtColor(d["left"], cv2.COLOR_RGB2GRAY),
                                       cv2.cvtColor(d["right"], cv2.COLOR_RGB2GRAY)))
        if confidence:
            low = res.confidence < confidence
            if edge_blind_px:
                # the right-view matcher has no data in the last columns; keep
                # the left matcher's depth there instead of dropping it
                low[:, -edge_blind_px:] = False
            res.depth_mm[low] = np.nan
        s = build_robot_frame(res, cam, 0, cfg).scan
        t = laser_scan(np.where(np.isfinite(d["truth"]), d["truth"], np.inf).astype(np.float32), tcam,
                       cfg.beams, cfg.min_height_m, cfg.max_height_m, cfg.range_min_m, cfg.range_max_m)
        e = scan_error(s, t)
        for k in tot:
            tot[k] += e[k]
    c, sc = max(tot["close"], 1), max(tot["stereo_close"], 1)
    print(f"{name:34s} missed {100*tot['missed']/c:5.2f}%  blind {100*tot['blind']/c:5.2f}%  "
          f"phantom {100*tot['phantom']/sc:5.2f}%")


print(f"{len(frames)} frames")
for max_h in (0.4, 0.1):
    run(f"no confidence, band top +{max_h}", 0, max_h)
    run(f"confidence 50, band top +{max_h}", 50, max_h)
    run(f"confidence 50 keep edge, top +{max_h}", 50, max_h, edge_blind_px=40)
