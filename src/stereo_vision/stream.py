"""Serve the live view to people and the depth data to robots, over plain HTTP.

People: open http://<pi-address>:<port>/ for the live picture (MJPEG).

Robots and other programs, in any language, read the same server:

  GET /api/v1/info             camera model, units, frames (JSON)
  GET /api/v1/frame            latest summary: obstacles, laser scan, timing (JSON)
  GET /api/v1/obstacles        nearest obstacle and per-sector distances (JSON)
  GET /api/v1/scan             2D laser scan, sensor_msgs/LaserScan fields (JSON)
  GET /api/v1/depth.png        depth, 16-bit PNG in millimetres, 0 = unknown
  GET /api/v1/depth.npy        depth, float32 metres, NaN = unknown
  GET /api/v1/confidence.png   0-100 per pixel, when the pipeline computes it
  GET /api/v1/left.png         rectified left image the depth lines up with
  GET /api/v1/points.ply       point cloud, binary PLY in metres (?step=2 thins it)
  GET /api/v1/events           the /frame summary pushed every frame (Server-Sent
                               Events; ?hz=5 caps the rate)

Everything is encoded on request from the latest frame, so an endpoint nobody
asks for costs nothing. There is no password: anyone on the network can read
it, as with any network camera without access control.
"""

from __future__ import annotations

import io
import json
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlsplit

import cv2
import numpy as np

PAGE = """<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Ster-Vis Live</title>
<style>
  html, body {{ margin: 0; background: #111; color: #ddd; font: 14px system-ui, sans-serif; }}
  header {{ padding: 8px 14px; display: flex; gap: 16px; flex-wrap: wrap; }}
  a {{ color: #8ab4ff; }}
  img {{ display: block; width: 100%; height: auto; }}
</style></head>
<body><header><span>Ster-Vis live — {title}</span><a href="/api/v1/frame">robot API</a></header>
<img src="/stream" alt="live stereo view"></body></html>
"""

API = "/api/v1/"


class MjpegServer:
    def __init__(self, port: int = 8080, max_fps: float = 10.0, quality: int = 80, title: str = "") -> None:
        self.max_fps = max_fps
        self.quality = quality
        self.title = title
        self._jpeg: bytes | None = None
        self._serial = 0
        self._last_encode = 0.0
        self._viewers = 0
        self._robot = None          # latest outputs.RobotFrame
        self._info: dict | None = None
        self._robot_serial = 0
        self._lock = threading.Condition()
        server = self

        class Handler(BaseHTTPRequestHandler):
            protocol_version = "HTTP/1.1"

            def log_message(self, *args) -> None:  # keep the console for status lines
                pass

            def _send(self, body: bytes, content_type: str, api: bool = False) -> None:
                self.send_response(200)
                self.send_header("Content-Type", content_type)
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                if api:
                    # Lets a browser dashboard on another origin read the data.
                    self.send_header("Access-Control-Allow-Origin", "*")
                self.end_headers()
                self.wfile.write(body)

            def _json(self, value) -> None:
                self._send(json.dumps(value, allow_nan=False).encode(), "application/json", api=True)

            def do_GET(self) -> None:
                url = urlsplit(self.path)
                path, query = url.path, parse_qs(url.query)
                if path == "/":
                    self._send(PAGE.format(title=server.title).encode(), "text/html; charset=utf-8")
                elif path == "/stream":
                    self._stream()
                elif path == "/snapshot.jpg":
                    jpeg = server.latest()
                    if jpeg is None:
                        self.send_error(503, "no frame yet")
                    else:
                        self._send(jpeg, "image/jpeg")
                elif path.startswith(API):
                    self._api(path[len(API):], query)
                else:
                    self.send_error(404)

            def _api(self, name: str, query: dict) -> None:
                if name == "info":
                    if server._info is None:
                        self.send_error(503, "no data yet")
                    else:
                        self._json(server._info)
                    return
                frame = server._robot
                if frame is None:
                    self.send_error(503, "no depth frame yet")
                    return
                if name == "frame":
                    self._json(frame.summary())
                elif name == "obstacles":
                    self._json({"sequence": frame.sequence, "timestamp": frame.timestamp,
                                **frame.obstacles.as_dict()})
                elif name == "scan":
                    self._json({"sequence": frame.sequence, "timestamp": frame.timestamp,
                                **frame.scan.as_dict()})
                elif name == "depth.png":
                    mm = np.where(np.isfinite(frame.depth_m), np.rint(frame.depth_m * 1000), 0)
                    ok, png = cv2.imencode(".png", np.clip(mm, 0, 65535).astype(np.uint16))
                    self._send(png.tobytes(), "image/png", api=True)
                elif name == "depth.npy":
                    buffer = io.BytesIO()
                    np.save(buffer, frame.depth_m)
                    self._send(buffer.getvalue(), "application/octet-stream", api=True)
                elif name == "confidence.png":
                    if frame.confidence is None:
                        self.send_error(404, "confidence is off; start with --confidence")
                        return
                    ok, png = cv2.imencode(".png", np.rint(frame.confidence).astype(np.uint8))
                    self._send(png.tobytes(), "image/png", api=True)
                elif name == "left.png":
                    ok, png = cv2.imencode(".png", frame.left)
                    self._send(png.tobytes(), "image/png", api=True)
                elif name == "points.ply":
                    from .outputs import ply_bytes

                    step = max(1, int(query.get("step", ["1"])[0]))
                    self._send(ply_bytes(frame.points, frame.left, step), "application/octet-stream", api=True)
                elif name == "events":
                    self._events(float(query.get("hz", ["0"])[0]))
                else:
                    self.send_error(404, "unknown endpoint; see /api/v1/info")

            def _events(self, hz: float) -> None:
                self.send_response(200)
                self.send_header("Content-Type", "text/event-stream")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Access-Control-Allow-Origin", "*")
                # Endless body with no length: the stream ends when the connection does.
                self.send_header("Connection", "close")
                self.close_connection = True
                self.end_headers()
                seen, last = -1, 0.0
                try:
                    while True:
                        with server._lock:
                            server._lock.wait_for(lambda: server._robot_serial != seen, timeout=5.0)
                            frame, seen = server._robot, server._robot_serial
                        if frame is None:
                            self.wfile.write(b": waiting\n\n")  # keep-alive comment
                            self.wfile.flush()
                            continue
                        now = time.monotonic()
                        if hz > 0 and now - last < 1.0 / hz:
                            continue
                        last = now
                        data = json.dumps(frame.summary(), allow_nan=False)
                        self.wfile.write(f"id: {frame.sequence}\ndata: {data}\n\n".encode())
                        self.wfile.flush()
                except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                    pass

            def _stream(self) -> None:
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Cache-Control", "no-cache")
                self.send_header("Connection", "close")
                self.close_connection = True
                self.end_headers()
                with server._lock:
                    server._viewers += 1
                seen = -1
                try:
                    while True:
                        with server._lock:
                            server._lock.wait_for(lambda: server._serial != seen, timeout=5.0)
                            jpeg, seen = server._jpeg, server._serial
                        if jpeg is None:
                            continue
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n")
                        self.wfile.write(f"Content-Length: {len(jpeg)}\r\n\r\n".encode())
                        self.wfile.write(jpeg + b"\r\n")
                except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                    pass
                finally:
                    with server._lock:
                        server._viewers -= 1

        self.httpd = ThreadingHTTPServer(("0.0.0.0", port), Handler)
        self.httpd.daemon_threads = True
        self.port = self.httpd.server_address[1]
        self._thread = threading.Thread(target=self.httpd.serve_forever, name="mjpeg", daemon=True)
        self._thread.start()

    @property
    def viewers(self) -> int:
        with self._lock:
            return self._viewers

    def wants_frame(self) -> bool:
        """True when a viewer is waiting and a new frame is due.

        Check this before building the image to publish: composing the view
        costs several milliseconds a frame, which is pure waste when nobody is
        watching or the stream's frame rate cap has not come round yet.
        """
        with self._lock:
            watching = self._viewers > 0 or self._jpeg is None
        return watching and time.monotonic() - self._last_encode >= 1.0 / self.max_fps

    def publish(self, image: np.ndarray) -> bool:
        """Encode and send a frame if one is wanted; returns True if it was sent."""
        if not self.wants_frame():
            return False
        now = time.monotonic()
        ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
        if not ok:
            return False
        self._last_encode = now
        with self._lock:
            self._jpeg = encoded.tobytes()
            self._serial += 1
            self._lock.notify_all()
        return True

    def publish_robot(self, frame, info: dict) -> None:
        """Make a RobotFrame the one the API serves. Costs only a reference swap."""
        with self._lock:
            self._robot = frame
            self._info = info
            self._robot_serial += 1
            self._lock.notify_all()

    def latest(self) -> bytes | None:
        with self._lock:
            return self._jpeg

    def urls(self) -> list[str]:
        """Addresses to open from another machine."""
        hosts = {"localhost"}
        try:
            hosts.add(socket.gethostname() + ".local")
            probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            probe.connect(("192.0.2.1", 9))  # no packet is sent; this picks the LAN address
            hosts.add(probe.getsockname()[0])
            probe.close()
        except OSError:
            pass
        return [f"http://{host}:{self.port}/" for host in sorted(hosts)]

    def close(self) -> None:
        self.httpd.shutdown()
        self.httpd.server_close()
