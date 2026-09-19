"""Stereo frame sources: USB cameras through OpenCV, or Pi cameras through Picamera2.

Every source hands back a :class:`StereoFrame`. Greyscale is the default,
because that is all the matcher uses, and on a Raspberry Pi it comes free: the
YUV420 format the camera already produces starts with a full-resolution grey
(Y) plane, so taking it skips a colour conversion per frame.

:class:`ThreadedSource` wraps any source so capture runs on its own thread and
overlaps with matching, instead of the two taking turns.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass

import cv2
import numpy as np

PICAMERA2_INSTALL = (
    "Picamera2 is not available. On Raspberry Pi OS install it with\n"
    "  sudo apt install python3-picamera2\n"
    "and create the virtual environment with --system-site-packages so it is visible:\n"
    "  python3 -m venv --system-site-packages .venv"
)


@dataclass
class StereoFrame:
    left: np.ndarray
    right: np.ndarray
    # Sensor timestamp difference, left minus right, in milliseconds, when the
    # backend reports timestamps. None means unknown, as with USB webcams.
    skew_ms: float | None = None


class StereoSource:
    """Base class; use as a context manager so cameras are always released."""

    size: tuple[int, int]

    def read(self) -> StereoFrame:
        raise NotImplementedError

    def close(self) -> None:
        raise NotImplementedError

    def __enter__(self) -> "StereoSource":
        return self

    def __exit__(self, *exc) -> None:
        self.close()


class OpenCVSource(StereoSource):
    """Two cameras through cv2.VideoCapture: USB webcams on any platform.

    ``fourcc="MJPG"`` asks the cameras for compressed frames. Two uncompressed
    720p streams can exceed what one USB bus carries, which shows up as a
    camera refusing to open or dropping to a low frame rate; MJPG avoids that
    at the cost of decoding each frame.
    """

    def __init__(
        self,
        left_index: int = 0,
        right_index: int = 1,
        width: int | None = None,
        height: int | None = None,
        fps: float | None = None,
        grey: bool = True,
        fourcc: str | None = None,
        api: int = cv2.CAP_ANY,
    ) -> None:
        self.grey = grey
        self.captures = []
        try:
            for index in (left_index, right_index):
                capture = cv2.VideoCapture(index, api)
                self.captures.append(capture)
                if not capture.isOpened():
                    raise RuntimeError(f"could not open camera index {index}")
                if fourcc:
                    capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
                if width:
                    capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
                if height:
                    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
                if fps:
                    capture.set(cv2.CAP_PROP_FPS, fps)
        except Exception:
            self.close()
            raise
        first = self.captures[0]
        self.size = (int(first.get(cv2.CAP_PROP_FRAME_WIDTH)), int(first.get(cv2.CAP_PROP_FRAME_HEIGHT)))

    def read(self) -> StereoFrame:
        left, right = self.captures
        # Grab both before decoding either, so the exposures are as close in
        # time as the drivers allow.
        left.grab()
        right.grab()
        left_ok, left_frame = left.retrieve()
        right_ok, right_frame = right.retrieve()
        if not left_ok or not right_ok:
            raise RuntimeError("failed to read a frame from one of the cameras")
        if self.grey:
            left_frame = cv2.cvtColor(left_frame, cv2.COLOR_BGR2GRAY)
            right_frame = cv2.cvtColor(right_frame, cv2.COLOR_BGR2GRAY)
        return StereoFrame(left_frame, right_frame)

    def close(self) -> None:
        for capture in self.captures:
            capture.release()
        self.captures = []


class Picamera2Source(StereoSource):
    """Two CSI camera modules through Picamera2, on a Raspberry Pi 5.

    The Pi 5 has two camera connectors, so two modules run at full rate at
    once. Left and right must be exposed at the same moment, or anything that
    moves is seen in two places and gets the wrong depth.

    When the installed libcamera supports it, the cameras use Raspberry Pi's
    software camera synchronisation: the left camera acts as server, the right
    as client, and the client nudges its frame timing to match. Raspberry Pi
    documents the result as frame starts within several tens of microseconds.
    Frames are held back until both cameras report SyncReady.

    Autofocus is locked. A lens that refocuses changes its focal length, which
    silently invalidates the calibration, so cameras with autofocus (such as
    Camera Module 3) are set to manual focus at ``focus_dioptres``: the
    reciprocal of the focus distance in metres, so 1.0 focuses at 1 m and 0 at
    infinity. Use the same value when calibrating and when measuring.

    Without that support, pairs are matched by sensor timestamp instead: when
    the two are more than half a frame apart the older frame is replaced, up
    to three times, which normally leaves them within half a frame (about
    17 ms at 30 fps). Either way, the remaining skew is reported per frame.
    """

    def __init__(
        self,
        left_camera: int = 0,
        right_camera: int = 1,
        width: int = 1280,
        height: int = 720,
        fps: float = 30.0,
        grey: bool = True,
        max_skew_ms: float | None = None,
        sync: bool | None = None,
        sync_frames: int = 30,
        sync_timeout_s: float = 10.0,
        focus_dioptres: float | None = 1.0,
    ) -> None:
        """``sync``: True requires software sync, False disables it, None uses it if available."""
        try:
            from picamera2 import Picamera2
        except ImportError as error:
            raise RuntimeError(PICAMERA2_INSTALL) from error

        found = Picamera2.global_camera_info()
        if len(found) < 2:
            names = ", ".join(f"{c.get('Num')}: {c.get('Model')}" for c in found) or "none"
            raise RuntimeError(
                f"need two cameras, found {len(found)} ({names}). "
                "Check both ribbon cables, then run: rpicam-hello --list-cameras"
            )

        self.grey = grey
        self.max_skew_ms = max_skew_ms if max_skew_ms is not None else 500.0 / fps
        self.sync_timeout_s = sync_timeout_s
        self.cameras = []
        try:
            for number in (left_camera, right_camera):
                self.cameras.append(Picamera2(number))
            self.size = (width, height)

            supported = all("SyncMode" in camera.camera_controls for camera in self.cameras)
            if sync and not supported:
                raise RuntimeError(
                    "software camera sync needs a newer libcamera; update with "
                    "sudo apt update && sudo apt full-upgrade, or pass sync=False"
                )
            self.synced = supported if sync is None else bool(sync)

            # Server first in the list, client second, but start the client
            # first: it waits for the server, which then finds it ready.
            roles = [{"SyncMode": 1, "SyncFrames": sync_frames}, {"SyncMode": 2}] if self.synced else [{}, {}]
            self.focus_locked = False
            for camera, role in zip(self.cameras, roles):
                focus = {}
                if focus_dioptres is not None and "AfMode" in camera.camera_controls:
                    focus = {"AfMode": 0, "LensPosition": float(focus_dioptres)}  # 0 = manual
                    self.focus_locked = True
                config = camera.create_video_configuration(
                    main={"size": (width, height), "format": "YUV420" if grey else "RGB888"},
                    controls={"FrameRate": fps, **role, **focus},
                    buffer_count=4,
                )
                camera.configure(config)
            self.size = tuple(self.cameras[0].camera_config["main"]["size"])
            for camera in reversed(self.cameras):
                camera.start()
            if self.synced:
                self._wait_for_sync()
        except Exception:
            self.close()
            raise

        if self.size != (width, height):
            print(
                f"note: cameras run at {self.size[0]}x{self.size[1]}, not {width}x{height}; "
                "calibrate at the size you run at"
            )

    def _grab(self, camera, want_array: bool = True):
        request = camera.capture_request()
        try:
            array = request.make_array("main") if want_array else None
            metadata = request.get_metadata()
        finally:
            request.release()
        if array is not None and self.grey:
            # YUV420 arrives as (height * 3 / 2, stride): the Y plane first,
            # possibly padded on the right. It is exactly the grey image.
            width, height = self.size
            array = array[:height, :width]
        self._last_metadata = metadata
        return array, metadata.get("SensorTimestamp")

    def _wait_for_sync(self) -> None:
        """Discard frames until both cameras report that they are synchronised."""
        deadline = time.monotonic() + self.sync_timeout_s
        ready = [False, False]
        while not all(ready):
            if time.monotonic() > deadline:
                raise RuntimeError(
                    f"cameras did not synchronise within {self.sync_timeout_s:.0f} s; "
                    "check both run at the same frame rate, or pass sync=False"
                )
            for index, camera in enumerate(self.cameras):
                if not ready[index]:
                    self._grab(camera, want_array=False)
                    ready[index] = bool(self._last_metadata.get("SyncReady", False))

    def read(self) -> StereoFrame:
        left_cam, right_cam = self.cameras
        left, left_ts = self._grab(left_cam)
        right, right_ts = self._grab(right_cam)
        skew = None
        for _ in range(3):
            if left_ts is None or right_ts is None:
                break
            skew = (left_ts - right_ts) / 1e6
            if abs(skew) <= self.max_skew_ms:
                break
            if left_ts < right_ts:
                left, left_ts = self._grab(left_cam)
            else:
                right, right_ts = self._grab(right_cam)
        if left_ts is not None and right_ts is not None:
            skew = (left_ts - right_ts) / 1e6
        return StereoFrame(np.ascontiguousarray(left), np.ascontiguousarray(right), skew)

    def close(self) -> None:
        for camera in self.cameras:
            try:
                camera.stop()
            finally:
                camera.close()
        self.cameras = []


class ThreadedSource(StereoSource):
    """Capture on a background thread and always hand out the newest pair.

    Without this, the loop captures, then matches, then captures again, and the
    cameras sit idle during matching. With it, the next pair is already being
    captured while the current one is matched. Frames that arrive faster than
    they are processed are dropped, so latency stays at about one frame
    instead of growing.
    """

    def __init__(self, source: StereoSource, timeout_s: float = 5.0) -> None:
        self.source = source
        self.size = source.size
        self.timeout_s = timeout_s
        self._frame: StereoFrame | None = None
        self._serial = 0      # frames captured
        self._taken = 0       # serial of the last frame handed out
        self._delivered = 0   # frames handed out
        self._error: BaseException | None = None
        self._stop = threading.Event()
        self._ready = threading.Condition()
        self._thread = threading.Thread(target=self._run, name="stereo-capture", daemon=True)
        self._thread.start()

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                frame = self.source.read()
            except BaseException as error:  # handed to the reader, not lost
                with self._ready:
                    self._error = error
                    self._ready.notify_all()
                return
            with self._ready:
                self._frame = frame
                self._serial += 1
                self._ready.notify_all()

    def read(self) -> StereoFrame:
        deadline = time.monotonic() + self.timeout_s
        with self._ready:
            while self._serial == self._taken and self._error is None:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError(f"no frame from the cameras within {self.timeout_s:.0f} s")
                self._ready.wait(remaining)
            if self._error is not None:
                raise RuntimeError(f"capture failed: {self._error}") from self._error
            self._taken = self._serial
            self._delivered += 1
            return self._frame

    @property
    def dropped(self) -> int:
        """Frames captured but replaced before anyone read them."""
        with self._ready:
            pending = 1 if self._serial > self._taken else 0
            return max(0, self._serial - self._delivered - pending)

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=self.timeout_s)
        self.source.close()


def picamera2_available() -> bool:
    try:
        from picamera2 import Picamera2
    except ImportError:
        return False
    try:
        return len(Picamera2.global_camera_info()) >= 2
    except Exception:
        return False


def open_source(
    backend: str = "auto",
    left: int = 0,
    right: int = 1,
    width: int | None = None,
    height: int | None = None,
    fps: float | None = None,
    grey: bool = True,
    threaded: bool = True,
    fourcc: str | None = None,
    focus_dioptres: float | None = 1.0,
) -> StereoSource:
    """Open a stereo pair. ``auto`` uses Pi cameras when two are attached, else USB."""
    if backend == "auto":
        backend = "picamera2" if picamera2_available() else "opencv"
    if backend == "picamera2":
        source: StereoSource = Picamera2Source(
            left, right, width or 1280, height or 720, fps or 30.0, grey,
            focus_dioptres=focus_dioptres,
        )
    elif backend == "opencv":
        source = OpenCVSource(left, right, width, height, fps, grey, fourcc)
    else:
        raise ValueError("backend must be auto, opencv or picamera2")
    return ThreadedSource(source) if threaded else source
