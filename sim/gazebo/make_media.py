"""Turn a run's video into files small enough to share.

    python make_media.py output/run.mp4 --start 6 --seconds 26
        -> output/run.gif    (640 px, 5 fps; plays inline on GitHub)
        -> output/run_h264.mp4 when imageio-ffmpeg is installed
           (pip install imageio-ffmpeg); about 6x smaller than OpenCV's mp4v

avoid.py writes run.mp4 at one frame per processed pair, 10 fps by default.
"""

import argparse
import os
import shutil
import subprocess
from pathlib import Path

import cv2
import numpy as np
from PIL import Image


def gif(video: Path, out: Path, start: float, seconds: float, width: int, step: int, colours: int) -> None:
    capture = cv2.VideoCapture(str(video))
    fps = capture.get(cv2.CAP_PROP_FPS) or 10
    capture.set(cv2.CAP_PROP_POS_FRAMES, int(start * fps))
    frames = []
    for i in range(int(seconds * fps)):
        ok, frame = capture.read()
        if not ok:
            break
        if i % step == 0:
            frame = cv2.resize(frame, (width, width * frame.shape[0] // frame.shape[1]), interpolation=cv2.INTER_AREA)
            frames.append(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
    if not frames:
        raise SystemExit(f"no frames read from {video}")
    # One palette from frames across the whole clip: a single frame's palette
    # posterizes colours that only appear later, and dithering the noise
    # textures bloats the file.
    sample = np.vstack(frames[:: max(1, len(frames) // 10)])
    palette = Image.fromarray(sample).quantize(colors=colours, method=Image.Quantize.MEDIANCUT)
    images = [Image.fromarray(f).quantize(palette=palette, dither=Image.Dither.NONE) for f in frames]
    images[0].save(out, save_all=True, append_images=images[1:], duration=int(1000 * step / fps),
                   loop=0, optimize=True)
    print(f"wrote {out} ({len(images)} frames, {os.path.getsize(out) / 1e6:.1f} MB)")


def ffmpeg_exe() -> str | None:
    """imageio-ffmpeg's copy, or an ffmpeg on PATH (the RoboStack env ships one)."""
    try:
        import imageio_ffmpeg

        return imageio_ffmpeg.get_ffmpeg_exe()
    except ImportError:
        return shutil.which("ffmpeg")


def h264(video: Path, out: Path, crf: int) -> None:
    exe = ffmpeg_exe()
    if exe is None:
        print("skipped H.264: no ffmpeg (pip install imageio-ffmpeg)")
        return
    subprocess.run([exe, "-y", "-loglevel", "error", "-i", str(video),
                    "-c:v", "libx264", "-preset", "slow", "-crf", str(crf), "-pix_fmt", "yuv420p",
                    "-movflags", "+faststart", str(out)], check=True)
    print(f"wrote {out} ({os.path.getsize(out) / 1e6:.1f} MB)")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("video", type=Path)
    parser.add_argument("--start", type=float, default=0.0, help="GIF start, seconds into the video")
    parser.add_argument("--seconds", type=float, default=26.0, help="GIF length")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--step", type=int, default=2, help="keep every Nth frame in the GIF")
    parser.add_argument("--colours", type=int, default=160)
    parser.add_argument("--crf", type=int, default=31, help="H.264 quality, lower = better and bigger")
    args = parser.parse_args()
    gif(args.video, args.video.with_suffix(".gif"), args.start, args.seconds, args.width, args.step, args.colours)
    h264(args.video, args.video.with_name(args.video.stem + "_h264.mp4"), args.crf)


if __name__ == "__main__":
    main()
