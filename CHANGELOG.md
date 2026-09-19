# Changelog

Every release of Ster-Vis, newest first. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and versions follow
[Semantic Versioning](https://semver.org/) as described in
[docs/RELEASING.md](docs/RELEASING.md): what counts as a breaking change, how
patches are made, and how to upgrade.

The release workflow publishes the section for each version as its GitHub
release notes, so write entries for the people who install it.

## [Unreleased]

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

[Unreleased]: https://github.com/vixhvajit/Ster-Vis/compare/v2.0.0...HEAD
[2.0.0]: https://github.com/vixhvajit/Ster-Vis/compare/v0.1.0...v2.0.0
[0.1.0]: https://github.com/vixhvajit/Ster-Vis/releases/tag/v0.1.0
