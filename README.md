# Ster-Vis

[![CI](https://github.com/vixhvajit/Ster-Vis/actions/workflows/ci.yml/badge.svg)](https://github.com/vixhvajit/Ster-Vis/actions/workflows/ci.yml)
[![License](https://img.shields.io/badge/license-Apache--2.0-blue.svg)](LICENSE)

Depth from a pair of cameras: calibrate the rig, rectify the views, match them,
and read distance out of the disparity.

## How it works

Two cameras looking at the same scene see each point at slightly different
horizontal positions. That offset is the **disparity**, and once the rig is
calibrated it converts straight to distance:

```
Z = f * B / d
```

where `f` is the rectified focal length in pixels, `B` the baseline between the
camera centres, and `d` the disparity in pixels. Near things shift a lot, far
things barely shift, and a point at infinity has zero disparity.

Matching is only cheap because of **rectification**: the two images are warped
so that corresponding points always land on the same image row, which turns the
search from two-dimensional into a scan along one line.

## Layout

```
src/stereo_vision/
  config.py        BoardSpec and SGBMParams
  capture.py       open two cameras, grab pairs, load them back
  calibration.py   corner detection, stereo solve, rectification maps
  disparity.py     StereoSGBM matcher, optional WLS filter, colorizing
  depth.py         disparity to depth, point cloud, PLY export
  pattern.py       chessboard target as a true-scale PDF or a raster
scripts/
  make_chessboard.py write a printable calibration target
  capture_pairs.py   capture board pairs from two cameras
  calibrate.py       solve the rig and write calib/stereo.npz
  run_disparity.py   rectify, match, save a depth map or point cloud
docs/              printable calibration targets
tests/             runs without hardware
```

## Hardware

Two cameras, ideally the same model, bolted to one rigid bar and pointing the
same way. The spacing between them, the **baseline**, is your choice. Nothing
in the code assumes a value, because calibration measures it. Recalibrate
whenever you change it.

A wider baseline makes far objects more precise but pushes the closest
measurable distance further out:

| Baseline | Closest usable distance | Depth error at 2 m |
|---|---|---|
| 3 cm | 16 cm | ±4.8 cm |
| 6 cm | 33 cm | ±2.4 cm |
| 12 cm | 66 cm | ±1.2 cm |
| 20 cm | 1.1 m | ±0.7 cm |

These numbers assume a 700 px focal length, 128 disparities and quarter-pixel
matching accuracy. They follow from:

```
closest distance = f * B / num_disparities
depth error      = Z^2 * 0.25 / (f * B)
```

The error grows with the square of distance, so doubling range costs four
times the precision. Choose the baseline for the distance you care about most.
Around 6 cm suits a desk or arm's length, 12-20 cm suits a room or a robot
looking several metres ahead. You can also get closer by raising
`--num-disparities`, though matching slows down.

## Setup

```powershell
python -m venv .venv
.venv\Scripts\activate
pip install -r requirements-dev.txt
```

`opencv-contrib-python` is used rather than the base package because the WLS
disparity filter lives in the contrib `ximgproc` module.

## Usage

### 0. Print the calibration target

A ready-made board matches the script defaults of 9x6 inner corners and 25 mm
squares:

- **[chessboard_9x6_25mm_a4.pdf](docs/chessboard_9x6_25mm_a4.pdf)** for A4
- **[chessboard_9x6_25mm_letter.pdf](docs/chessboard_9x6_25mm_letter.pdf)** for US Letter

When printing:

1. Set the scale to **100% / Actual size** and turn off "Fit to page". Fitting
   the page shrinks the squares, and every depth reading comes out scaled by
   the same factor.
2. Check the 100 mm scale bar with a ruler, then measure one square. If it
   isn't exactly 25.0 mm, pass the measured value to `--square-size`.
3. Glue it flat to foam board, acrylic or stiff cardboard. A board that bends
   ruins the calibration.
4. Matte paper is better than glossy, because glare hides corners.

For another size or paper, generate one:

```powershell
python scripts/make_chessboard.py --columns 9 --rows 6 --square-mm 20 --paper letter
```

Bigger squares are easier to detect from far away. If the board doesn't fit
the page, the script tells you so.

### 1. Capture calibration pairs

```powershell
python scripts/capture_pairs.py --left-index 0 --right-index 1 --columns 9 --rows 6
```

SPACE saves a pair, Q quits. Take 15-20 pairs, moving the board through the
corners of the frame and tilting it, not just holding it flat in the middle.
`--columns` and `--rows` count **inner corners**, so a board of 10x7 squares is
`--columns 9 --rows 6`.

### 2. Calibrate

```powershell
python scripts/calibrate.py --square-size 25 --preview output/rectified.png
```

`--square-size` sets the unit for everything downstream: pass millimetres and
depth comes out in millimetres. The preview draws horizontal rules over a
rectified pair — the same feature should sit on the same rule in both halves.
Reprojection RMS under about 0.5 px is good; over 1.0 px means recapturing.

### 3. Disparity and depth

```powershell
python scripts/run_disparity.py --left data/pairs/left/000.png --right data/pairs/right/000.png --wls --save output/disparity.png --ply output/cloud.ply
```

Or live, from the cameras:

```powershell
python scripts/run_disparity.py --live
```

The PLY opens in MeshLab or CloudCompare.

## Tuning

| Symptom | Knob |
|---|---|
| Near objects have no depth | raise `--num-disparities` (multiples of 16) |
| Noisy, speckled map | raise `block_size`, or `speckle_window_size` |
| Fine detail lost | lower `block_size` |
| Holes in textureless regions | `--wls`, or add projected texture to the scene |
| Same feature on different rows | recalibrate; the rig probably moved |

The cameras must be rigidly mounted. Any flex between them invalidates the
calibration, and disparity silently turns into nonsense rather than failing
loudly.

## Tests

```powershell
python -m pytest
```

No camera needed — the disparity test shifts a textured image by a known number
of pixels and checks the matcher recovers that shift.

## License

Apache License 2.0 — see [LICENSE](LICENSE) and [NOTICE](NOTICE).

Copyright 2026 Vishvajit S.
