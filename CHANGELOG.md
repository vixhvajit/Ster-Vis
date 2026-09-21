# Changelog

Every release of Ster-Vis, newest first. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and versions follow
[Semantic Versioning](https://semver.org/) as described in
[docs/RELEASING.md](docs/RELEASING.md): what counts as a breaking change, how
patches are made, and how to upgrade.

The release workflow publishes the section for each version as its GitHub
release notes, so write entries for the people who install it.

## [Unreleased]

## [2.1.0] - 2026-09-21

Ster-Vis stops forgetting: frames can now be fused into one point cloud map of
the place the camera moved through.

### Added

- **Mapping.** `stereo_vision.mapping` fuses per-frame point clouds into one
  voxel-averaged map, with each voxel keeping the running mean of the points
  in it and how many there were. Averaging removes stereo noise a single
  frame cannot, and the count separates real surfaces from single-frame
  flyers (`--min-hits`). Keyframe thresholds skip views the map already has.
- **`ster-vis map`**, which builds a map from a recording and a CSV of your
  poses, and writes `map.ply` (the reconstruction), `map.pgm` + `map.yaml` (a
  top-down floor plan in ROS map_server's format) and `map.json` (what was
  fused, the extent, the trajectory).
- **`ster-vis ros2 --map`**, which does the same live, taking the pose of each
  frame from TF and publishing the growing map on `map_points` as a latched
  `PointCloud2`. `--map-save DIR` writes it out when the node stops.
- `Pose`, `MapConfig`, `PointCloudMap` and `load_poses` are exported from
  `stereo_vision`.

  **Ster-Vis does not estimate where the camera is**, by design: every frame
  is placed at a pose you supply, from odometry, TF, a flight controller or
  motion capture. The map is exactly as good as those poses.

- **Two Gazebo simulations** (`sim/gazebo/`, not part of the package), both
  running Ster-Vis unmodified on a `pi5` preset:
  - a rover avoiding obstacles on the stereo laser scan: 180 s, no
    collisions, every scan scored against a perfect depth camera;
  - a drone flying an indoor warehouse and mapping it, with the map scored
    against the building's true geometry and against the map an ideal depth
    camera would have built on the same flight. Over 300 s and 127 m, with no
    collisions: 556,129 points at 5 cm, median error 2.7 cm, 90% of points
    within 10 cm of a real surface, and 92% of what the ideal camera mapped.

  Results, GIFs and videos are in the README under Testing and validation.

## [2.0.2] - 2026-09-19

### Fixed

- **The laser scan could report a blocked direction as clear.** A direction
  counted as seen if its image column had depth anywhere, including on the
  floor below the scan's height band. When the band itself had no depth, for
  example where the confidence check dropped it or SGBM could not match a plain
  obstacle, the direction read `inf` (clear) instead of `nan` (unknown).
  Directions now count as seen only with depth inside the band. Found by a
  Gazebo simulation, where 2% of nearby obstacles had read clear; on the same
  recorded frames, none do now.

  **This can change how a robot behaves**, by design. Directions that were
  wrongly clear now read unknown, so a robot that treats unknown as
  blocked is more cautious there. This changes the ROS 2 `scan` topic and
  `laser_scan` in Python. The obstacle outputs and the HTTP scan already treat
  clear and unknown alike (no reading, and `null`), so they are unchanged.

## [2.0.1] - 2026-09-19

### Fixed

- **`ster-vis upgrade` on Windows.** Started from `ster-vis.exe`, it had pip
  overwrite the program that was running, and Windows killed it partway
  through the install. It now stops before downloading and prints the
  equivalent `python -m stereo_vision upgrade ...` command, which works.
  Linux, macOS and Raspberry Pi were not affected.
- When pip failed, `ster-vis upgrade` claimed the previous version was still
  installed, which a partial install can make untrue. It now says to check
  with `ster-vis --version`, and how to retry.

## [2.0.0] - 2026-09-19

Ster-Vis becomes a depth camera for robots: the same kinds of output as a
commercial stereo camera, over ROS 2 or plain HTTP.

### Breaking

- **`constraints-pi-bookworm.txt` is now `constraints-numpy1.txt`.** ROS 2
  Jazzy's Python messages are built against numpy 1.x, just as Bookworm's
  picamera2 is, so one file now covers both. Replace the old name in your
  install commands: `pip install -c constraints-numpy1.txt .`

### Added

- **Robot outputs**, in ROS units and frames: depth in metres, camera model,
  organised point cloud, 2D laser scan, nearest obstacle overall and
  left/centre/right, optional per-pixel confidence, and the rectified image.
  Checked against ray-traced truth: point cloud 17 mm median, laser scan
  14 mm, nearest obstacle within about 1 mm.
- **`ster-vis ros2`**, a ROS 2 node publishing standard `sensor_msgs` topics
  (`depth/image`, `points`, `scan`, `obstacles/*`, `camera_info` and more)
  plus static TF from `--parent-frame` via `--mount`. Needs ROS 2 Jazzy or
  newer. Tested in CI against the official `ros:jazzy` container.
- **HTTP API** under `/api/v1/` whenever `--stream` is on: `info`, `frame`,
  `obstacles`, `scan`, `depth.npy`, `depth.png`, `points.ply`,
  `confidence.png`, `left.png`, and `events`, which pushes each frame's
  summary. Readable from any language; cross-origin reads are allowed.
- **Record and replay:** `--record DIR` saves raw frames losslessly with their
  timing; `--replay DIR` plays them back as a camera (`--loop` repeats).
- **`--confidence`**, a per-pixel confidence map from a left-right
  consistency check.
- **`ster-vis upgrade`**, which checks GitHub for a newer release, verifies
  its SHA-256 checksum, and installs it. `--to VERSION` installs a specific
  version, including an older one to roll back.
- Every frame carries its capture time, used to stamp ROS messages and API
  output. `/api/v1/info` reports the version serving it.
- `examples/http_client.py`: follows the event stream using only the Python
  standard library.
- Release tooling: an automated release workflow, a documented versioning and
  patch policy, issue and pull request templates, and Dependabot.

### Fixed

- A malformed API query value (`?step=abc`, `?hz=abc`) crashed that request's
  handler and dropped the connection. It now returns `400 Bad Request`.
- Scan settings that would have reported "all clear" regardless of what the
  camera saw (fewer than 2 beams, or an inverted height band or range) are now
  refused.

### Changed

- The live server speaks HTTP/1.1, so clients polling the API can reuse one
  connection.

## [0.1.0] - 2026-09-19

First release.

### Added

- The `ster-vis` command: `chessboard`, `capture`, `calibrate`, `depth`,
  `view`, `viewer`, `benchmark`, `doctor`, `check` and `scene`.
- Calibration that reports frame coverage and warns when it is too low.
  Tested: a low reprojection error can hide a calibration that is badly off at
  the frame edges.
- Speed presets that rectify straight to a smaller image, from `quality` to
  `pi5-fast`.
- Raspberry Pi 5 support: both camera connectors through Picamera2, software
  frame synchronisation, locked autofocus, threaded capture, a live browser
  stream, and calibration capture without a keyboard (`capture --auto`).
- A browser viewer for point clouds and depth maps, depth files in 16-bit PNG
  (mm) or `.npy`, and click-to-measure distances.
- A Pi installer and hardened systemd service (`deploy/pi/`).
- Synthetic ground truth: a virtual stereo rig and a ray-traced scene, so the
  whole pipeline is tested without cameras.

[Unreleased]: https://github.com/vixhvajit/Ster-Vis/compare/v2.1.0...HEAD
[2.1.0]: https://github.com/vixhvajit/Ster-Vis/compare/v2.0.2...v2.1.0
[2.0.2]: https://github.com/vixhvajit/Ster-Vis/compare/v2.0.1...v2.0.2
[2.0.1]: https://github.com/vixhvajit/Ster-Vis/compare/v2.0.0...v2.0.1
[2.0.0]: https://github.com/vixhvajit/Ster-Vis/compare/v0.1.0...v2.0.0
[0.1.0]: https://github.com/vixhvajit/Ster-Vis/releases/tag/v0.1.0
