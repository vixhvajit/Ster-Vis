"""Open the browser viewer, optionally with a file already loaded.

  ster-vis viewer                              empty viewer; drop files onto it
  ster-vis viewer output/cloud.ply             opens that point cloud
  ster-vis viewer output/depth.png --color depth

The viewer is one self-contained HTML page shipped inside the package. This
command serves it, together with the file you name, from a small local web
server, because browsers do not let a page opened from disk read other files.
It listens on 127.0.0.1 only unless you pass --host.
"""

from __future__ import annotations

import argparse
import threading
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from importlib import resources
from pathlib import Path
from urllib.parse import quote, unquote

CONTENT_TYPES = {".ply": "application/octet-stream", ".png": "image/png", ".npy": "application/octet-stream"}


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog="ster-vis viewer", description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("file", type=Path, nargs="?", help=".ply point cloud, or .png / .npy depth map")
    parser.add_argument("--color", choices=["rgb", "depth"], default=None,
                        help="point cloud colouring to start with")
    parser.add_argument("--port", type=int, default=0, help="default: any free port")
    parser.add_argument("--host", default="127.0.0.1",
                        help="0.0.0.0 to reach it from other machines (no password)")
    parser.add_argument("--no-browser", action="store_true", help="print the address instead of opening it")
    return parser.parse_args(argv)


def viewer_html() -> bytes:
    return resources.files("stereo_vision").joinpath("viewer.html").read_bytes()


def make_server(file: Path | None, host: str = "127.0.0.1", port: int = 0) -> ThreadingHTTPServer:
    """Serve the viewer at /, the named file under /file/, and ./docs/samples under /samples/."""
    html = viewer_html()
    samples = Path("docs/samples").resolve()

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args) -> None:
            pass

        def _send(self, body: bytes, content_type: str) -> None:
            self.send_response(200)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self) -> None:
            path = unquote(self.path.split("?", 1)[0])
            if path in ("/", "/index.html"):
                self._send(html, "text/html; charset=utf-8")
            elif file is not None and path == f"/file/{file.name}":
                self._send(file.read_bytes(), CONTENT_TYPES.get(file.suffix.lower(), "application/octet-stream"))
            elif path.startswith("/samples/"):
                target = (samples / path.removeprefix("/samples/")).resolve()
                # Only files directly inside docs/samples, never anything above it.
                if target.parent == samples and target.is_file():
                    self._send(target.read_bytes(), CONTENT_TYPES.get(target.suffix.lower(), "application/octet-stream"))
                else:
                    self.send_error(404, "samples come with the source repository: run from its folder")
            else:
                self.send_error(404)

    return ThreadingHTTPServer((host, port), Handler)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if args.file is not None:
        if not args.file.is_file():
            print(f"no such file: {args.file}")
            return 1
        if args.file.suffix.lower() not in CONTENT_TYPES:
            print("the viewer opens .ply, .png and .npy files")
            return 1
    server = make_server(args.file.resolve() if args.file else None, args.host, args.port)
    host = "127.0.0.1" if args.host == "0.0.0.0" else args.host
    url = f"http://{host}:{server.server_address[1]}/"
    query = []
    if args.file is not None:
        query.append("open=" + quote(f"/file/{args.file.name}"))
    if args.color:
        query.append(f"color={args.color}")
    if query:
        url += "?" + "&".join(query)

    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    print(f"viewer at {url}")
    if args.host != "127.0.0.1":
        print("note: reachable from other machines on this network, without a password")
    print("Ctrl+C stops it")
    if not args.no_browser:
        webbrowser.open(url)
    try:
        thread.join()
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
