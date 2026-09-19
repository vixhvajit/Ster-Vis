"""Serve live frames over HTTP as MJPEG, for a Pi running without a display.

Open http://<pi-address>:<port>/ in any browser on the same network. Frames
are JPEG-encoded only while someone is watching, and no faster than
``max_fps``, so an unwatched stream costs nothing and a watched one does not
starve the matcher of CPU.
"""

from __future__ import annotations

import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np

PAGE = """<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Ster-Vis Live</title>
<style>
  html, body {{ margin: 0; background: #111; color: #ddd; font: 14px system-ui, sans-serif; }}
  header {{ padding: 8px 14px; }}
  img {{ display: block; width: 100%; height: auto; }}
</style></head>
<body><header>Ster-Vis live — {title}</header><img src="/stream" alt="live stereo view"></body></html>
"""


class MjpegServer:
    def __init__(self, port: int = 8080, max_fps: float = 10.0, quality: int = 80, title: str = "") -> None:
        self.max_fps = max_fps
        self.quality = quality
        self.title = title
        self._jpeg: bytes | None = None
        self._serial = 0
        self._last_encode = 0.0
        self._viewers = 0
        self._lock = threading.Condition()
        server = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args) -> None:  # keep the console for FPS lines
                pass

            def do_GET(self) -> None:
                if self.path == "/":
                    body = PAGE.format(title=server.title).encode()
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html; charset=utf-8")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                elif self.path == "/stream":
                    self._stream()
                elif self.path == "/snapshot.jpg":
                    jpeg = server.latest()
                    if jpeg is None:
                        self.send_error(503, "no frame yet")
                        return
                    self.send_response(200)
                    self.send_header("Content-Type", "image/jpeg")
                    self.send_header("Content-Length", str(len(jpeg)))
                    self.end_headers()
                    self.wfile.write(jpeg)
                else:
                    self.send_error(404)

            def _stream(self) -> None:
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Cache-Control", "no-cache")
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
