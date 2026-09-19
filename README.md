# Stereo Vision

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
scripts/
  capture_pairs.py capture board pairs from two cameras
  calibrate.py     solve the rig and write calib/stereo.npz
  run_disparity.py rectify, match, save a depth map or point cloud
tests/             runs without hardware
```

## Setup

```powershell
python -m venv .venv
.venv\Scripts\activate
pip install -r requirements-dev.txt
```

`opencv-contrib-python` is used rather than the base package because the WLS
disparity filter lives in the contrib `ximgproc` module.

## Usage

### 1. Capture calibration pairs

Print a chessboard, glue it to something rigid, and measure one square.

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
