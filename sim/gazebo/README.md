# Gazebo simulation: stereo obstacle avoidance

A 4-wheel rover carrying a Ster-Vis head (two cameras on a 12 cm bar and a
Raspberry Pi 5) wanders a 12 m arena with 16 obstacles. It sees only through
the stereo pair.

![Gazebo run: chase camera, map, rectified left image, Ster-Vis depth, stereo scan against truth](../../docs/samples/gazebo_avoidance.gif)

```
left + right camera (Gazebo, 640x480, 10 Hz)
  -> Ster-Vis DepthPipeline, pi5 preset (rectify to 320x240, SGBM, depth)
  -> left-right confidence check
  -> Ster-Vis laser scan (121 beams) -> reactive avoider -> cmd_vel
```

Results and what they showed are in the main README, under
[Testing and validation](../../README.md#gazebo-simulation-obstacle-avoidance).

Ster-Vis runs from this checkout (`../../src`), unmodified. The calibration is
the ideal one built from the cameras' CameraInfo, since simulated cameras have
no lens error; a real rig still needs `ster-vis calibrate`.

The world also carries three things the real robot does not:

- a **perfect depth camera** at the left camera, to score every stereo scan
  against the truth. The controller never reads it;
- a **chase camera** behind and above the rover, so the run can be watched.
  Gazebo renders it like the other cameras;
- a **contact sensor** on the chassis, to count collisions (not the wheels).

## Run on Windows (tested)

Needs a conda env with Gazebo Harmonic from [RoboStack](https://robostack.github.io/).
The runs below used an env built from `robostack-jazzy` with
`ros-jazzy-ros-gz-sim`, which brings Gazebo Harmonic 8.6.0, its Python
bindings (`gz.transport13`, `gz.msgs10`) and conda-forge OpenCV with contrib.
That env was built up step by step; in one command it should be:

```powershell
conda create -n ros2gz -c conda-forge -c robostack-jazzy ros-jazzy-ros-gz-sim opencv
```

Then, from this folder:

```powershell
. .\activate-ros2gz.ps1        # conda root: set STER_VIS_CONDA if not C:\ProgramData\miniforge3
.\start-gz.ps1                 # server in the background; makes the world on first run
python avoid.py --show         # 180 s of sim time, live window
.\stop-gz.ps1
```

Gazebo on Windows runs the server only; the GUI is unsupported upstream
([gz-sim#168](https://github.com/gazebosim/gz-sim/issues/168)). The chase
camera in `avoid.py`'s window and video stands in for it.

## Run on Linux (not yet tested)

With Gazebo Harmonic and its Python bindings (`gz-harmonic`,
`python3-gz-transport13`, `python3-gz-msgs10` from packages.osrfoundation.org),
the Gazebo GUI works too:

```bash
python3 make_textures.py && python3 make_world.py
gz sim -r worlds/stereo_avoid.sdf &      # full Gazebo GUI
python3 avoid.py --show
```

## Files

| File | Does |
|---|---|
| `make_textures.py` | noise textures; stereo needs texture to match |
| `make_world.py` | world, obstacles (seed 7), rover with cameras and Pi 5; `worlds/obstacles.json` |
| `avoid.py` | the stereo-to-cmd_vel loop, scoring, live window, video |
| `check_depth.py` | per-pixel depth error on one live frame, by distance band |
| `tune.py` | scores filter settings on frames saved with `avoid.py --record 5` |
| `make_media.py` | a run's video as a GitHub-sized GIF, and H.264 |

Each run writes `run.mp4`, `trajectory.png`, `log.csv` and `summary.json` to
`output/` (or `--out`).

## What is scored

- **missed**: a beam whose true range is under 2 m, reported clear by stereo.
  This is the one that causes crashes.
- **blind**: the same, reported unknown. SGBM cannot match the leftmost
  `num_disparities` columns, about 20% of the left image, because the right
  image does not reach that far.
- **phantom**: stereo reported something under 2 m with nothing real within
  0.5 m behind it.
