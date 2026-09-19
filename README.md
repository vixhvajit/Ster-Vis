# Ster-Vis

[![CI](https://github.com/vixhvajit/Ster-Vis/actions/workflows/ci.yml/badge.svg)](https://github.com/vixhvajit/Ster-Vis/actions/workflows/ci.yml)
[![License](https://img.shields.io/badge/license-Apache--2.0-blue.svg)](LICENSE)

Depth from a pair of cameras: calibrate the rig, rectify the views, match them,
and read distance out of the disparity.

![Depth measured by the pipeline on a synthetic scene, next to the ray-traced truth and the per-pixel error](docs/samples/scene_summary.png)

*The full pipeline on a ray-traced test scene: calibrated from rendered
chessboards, then matched with SGBM. 93% of the visible pixels get a depth,
with a median error of 1.6%. See [Try it without cameras](#try-it-without-cameras).*

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
  config.py          BoardSpec and SGBMParams
  capture.py         open two cameras, grab pairs, load them back
  calibration.py     corner detection, stereo solve, frame coverage, rectification
  disparity.py       StereoSGBM matcher, optional WLS filter, colorizing
  depth.py           disparity to depth, depth files, point cloud, PLY export
  pattern.py         chessboard target as a true-scale PDF or a raster
  synthetic.py       virtual stereo rig that photographs the chessboard
  scene.py           ray-traced 3D scene with exact per-pixel depth
scripts/
  make_chessboard.py write a printable calibration target
  capture_pairs.py   capture board pairs, with a live coverage grid
  calibrate.py       solve the rig and write calib/stereo.npz
  run_disparity.py   rectify, match, save a depth map or point cloud
  view_depth.py      inspect a depth map (hover for mm, click to measure)
  synthetic_check.py calibration check against a rig with known geometry
  scene_depth.py     depth map of a synthetic scene, scored pixel by pixel
viewer/index.html    browser viewer for point clouds and depth maps, no install
docs/                printable targets; docs/samples holds example outputs
tests/               runs without hardware
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

The table covers matching error only. In practice calibration error adds to it
and dominates at long range; see [what the synthetic scene shows](#what-the-synthetic-scene-shows).

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

SPACE saves a pair, Q quits. `--columns` and `--rows` count **inner corners**,
so a board of 10x7 squares is `--columns 9 --rows 6`.

A grid over each view turns green where saved pairs have put board corners.
**Keep capturing until nearly all of it is green in both views**, about 25
pairs, tilted and at several distances. The corners and edges of the frame
matter most.

This is the step that decides whether depth near the edges is any good. The
lens model is only trustworthy where the board has been. In the synthetic
tests, boards bunched in the centre covered 42% of the frame and left edge
pixels **15.8 px** off after rectification. Boards spread to the edges covered
78% and cut that to 1.2 px.

### 2. Calibrate

```powershell
python scripts/calibrate.py --square-size 25 --preview output/rectified.png
```

`--square-size` sets the unit for everything downstream: pass millimetres and
depth comes out in millimetres. The preview draws horizontal rules over a
rectified pair — the same feature should sit on the same rule in both halves.

Two numbers to check:

- **Reprojection RMS**: under about 0.5 px is good, over 1.0 px means recapturing.
- **Frame coverage**: aim for 75% or more. The script warns below 65%.

**A low RMS does not mean a good calibration.** RMS only measures the fit
where the boards were. The badly centred calibration above reported a *better*
RMS (0.19 px) than the well spread one (0.23 px), while being thirteen times
worse at the edges. Coverage is the number that catches this.

### 3. Disparity and depth

```powershell
python scripts/run_disparity.py --left data/pairs/left/000.png --right data/pairs/right/000.png --depth-out output/depth.png --ply output/cloud.ply
```

That writes:

| File | What it is |
|---|---|
| `output/depth.png` | depth map, 16-bit PNG in millimetres, 0 = no depth |
| `output/depth_color.png` | the same, coloured for a quick look |
| `output/cloud.ply` | 3D point cloud with the image's colours |

Use `--depth-out output/depth.npy` for float32 millimetres instead. The 16-bit
PNG is the format RealSense, Kinect and most RGB-D datasets use, so other
tools read it too.

Or live, from the cameras:

```powershell
python scripts/run_disparity.py --live
```

## Viewing results

Four ways to look at depth maps and point clouds, from no install at all to
full 3D editors. Sample files to try them on are in
[docs/samples](docs/samples): a depth map
([scene_depth_mm.png](docs/samples/scene_depth_mm.png)) and a point cloud
([scene_cloud.ply](docs/samples/scene_cloud.ply)).

### Browser viewer — nothing to install

Open **[viewer/index.html](viewer/index.html)** in Chrome, Edge, Firefox or
Safari and drop a file on it. It is one self-contained file: it works offline
and files never leave your machine.

- **Point clouds (.ply):** drag to orbit, right-drag or Shift-drag to pan,
  scroll to zoom. Colour by the image or by depth.
- **Depth maps (16-bit .png, .npy):** hover to read the exact depth in mm,
  click to pin points, adjust the colour range.

The *Sample* buttons and `?open=` links need the repo to be served, because
browsers block pages opened from disk from reading neighbouring files:

```powershell
python -m http.server 8000
```

Then open <http://localhost:8000/viewer/?open=../docs/samples/scene_cloud.ply>.
Add `&color=depth` to colour the cloud by distance.

### `view_depth.py` — uses what is already installed

```powershell
python scripts/view_depth.py output/depth.png --calibration calib/stereo.npz
```

Hover to read the depth under the cursor. Click two points and, with
`--calibration`, it prints the real-world distance between them in mm. `c`
cycles colour maps, `s` saves a screenshot, `q` quits.

Passing a `.ply` opens it in Open3D, if that is installed:

```powershell
pip install -r requirements-viewer.txt
python scripts/view_depth.py output/cloud.ply
```

Open3D is optional because it pulls in about 50 packages. It has wheels for
Windows, macOS, Linux and 64-bit ARM Linux, so it installs on a Raspberry Pi 5
too.

### MeshLab or CloudCompare — full 3D tools

Both are free and open source, and both open `.ply` directly. CloudCompare is
the better choice for measuring and comparing clouds; MeshLab is better for
cleaning them up and meshing.

| | MeshLab | CloudCompare |
|---|---|---|
| Windows | `winget install CNRISTI.MeshLab` | `winget install CloudCompare.CloudCompare` |
| macOS | `brew install --cask meshlab` | `brew install --cask cloudcompare` |
| Ubuntu, Debian, Raspberry Pi OS | `sudo apt install meshlab` | `sudo apt install cloudcompare` |
| Any Linux (Flatpak) | `flatpak install flathub net.meshlab.MeshLab` | `flatpak install flathub org.cloudcompare.CloudCompare` |
| Direct download | [meshlab.net](https://www.meshlab.net/#download) | [cloudcompare.org](https://www.cloudcompare.org/release/) |

Installers aren't bundled in this repo: they are 100+ MB each, GPL-licensed,
and would go stale. The package managers above always fetch the current
release.

## Try it without cameras

Two scripts run the whole pipeline on synthetic images with known ground truth,
so you can see it work, and see how accurate it is, before buying hardware.

### Calibration check

```powershell
python scripts/synthetic_check.py
```

Renders 25 chessboard pairs through a virtual 60 mm stereo rig with realistic
lens distortion, blur and sensor noise, runs the real `calibrate.py` on them,
and compares the result with the true rig:

| Check | Result |
|---|---|
| Baseline | 59.84 mm recovered, true 60.00 mm |
| Focal length error | 0.13% / 0.19% |
| Reprojection RMS | 0.24 px |
| Rectified row mismatch | 0.24 px |
| Depth error on boards the calibration never saw | 0.42% mean |

### Dense depth map

```powershell
python scripts/scene_depth.py
```

Ray traces a textured room (walls, floor, sphere, box, a tilted panel) through
the same rig, calibrates from rendered chessboards, runs rectification, SGBM and
triangulation, and scores every pixel against the exact depth. Outputs land in
`output/scene/`: the depth map, the truth, an error map, a point cloud and the
summary image at the top of this page.

| Surface | True distance | Pixels with depth | Median error |
|---|---|---|---|
| Tilted panel | 0.75 m | 99% | 0.27% |
| Sphere | 1.11 m | 99% | 0.37% |
| Floor | 1.10 m | 96% | 0.71% |
| Box | 1.34 m | 100% | 1.29% |
| Left wall (steep angle) | 1.41 m | 70% | 2.70% |
| Back wall | 2.16 m | 100% | 3.14% |

### What the synthetic scene shows

- **At range, calibration error dominates, not matching error.** Run the same
  images with the *true* calibration and every surface lands within 0.7%,
  including the back wall. A 0.1° calibration error shifts the whole right
  image sideways by about a pixel, and at 2 m, where disparity is only about
  20 px, that becomes a few percent. For long range: cover the frame well when
  calibrating, use a wider baseline, and use a bigger board so it can be held
  further away.
- **SGBM pixel-locks.** Sub-pixel disparities cluster near whole pixels, which
  shows as stripes and rings in the error map and as layered sheets in the
  point cloud. It is inherent to SGBM and worth about ±0.2 px.
- **The left edge has no depth.** SGBM cannot match the leftmost
  `num_disparities` columns, because the right image doesn't reach that far.
  Every SGBM depth map has this black band.
- **WLS fills holes by inventing depth.** It adds 0.2% coverage but reports a
  depth for 57% of occluded pixels, where no correct answer exists, against 6%
  without it. Use it for display, not for measurement.

## Tuning

| Symptom | Knob |
|---|---|
| Near objects have no depth | raise `--num-disparities` (multiples of 16) |
| Noisy, speckled map | raise `block_size`, or `speckle_window_size` |
| Fine detail lost | lower `block_size` |
| Holes in textureless regions | add projected texture to the scene; `--wls` hides them but invents depth |
| Depth wrong near the frame edges | recalibrate with better frame coverage |
| Same feature on different rows | recalibrate; the rig probably moved |

The cameras must be rigidly mounted. Any flex between them invalidates the
calibration, and disparity silently turns into nonsense rather than failing
loudly.

## Tests

```powershell
python -m pytest
```

No camera needed. The suite renders chessboards and a 3D scene with known
geometry, runs the real pipeline on them and checks the answers. That includes
the full depth map from an estimated calibration, and a regression test for a
corner-refinement bug that put corners up to 11 px off on small, tilted boards.

## License

Apache License 2.0 — see [LICENSE](LICENSE) and [NOTICE](NOTICE).

Copyright 2026 Vishvajit S.
