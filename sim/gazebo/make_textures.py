"""Generate the textures the stereo world uses.

Stereo matching needs texture: a flat-coloured box gives SGBM nothing to lock
onto, so every surface a camera should range gets multi-scale noise.
"""

from pathlib import Path

import cv2
import numpy as np

OUT = Path(__file__).parent / "worlds" / "textures"


def noise(size: int, tint: tuple[int, int, int], seed: int) -> np.ndarray:
    rng = np.random.default_rng(seed)
    image = np.zeros((size, size), np.float32)
    # Sum octaves of upscaled random noise: blobs for coarse structure, grain
    # for fine structure, so matching works at any distance.
    for cells, weight in ((8, 0.35), (32, 0.3), (128, 0.2), (size, 0.15)):
        layer = rng.random((cells, cells), dtype=np.float32)
        image += weight * cv2.resize(layer, (size, size), interpolation=cv2.INTER_CUBIC)
    image = cv2.normalize(image, None, 0.15, 1.0, cv2.NORM_MINMAX)
    bgr = np.dstack([image * c for c in tint[::-1]])
    return np.clip(bgr, 0, 255).astype(np.uint8)


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    textures = {
        "obstacle_red.png": ((230, 90, 70), 1),
        "obstacle_yellow.png": ((230, 200, 80), 2),
        "obstacle_green.png": ((90, 200, 110), 3),
        "wall.png": ((200, 200, 210), 4),
        "ground.png": ((150, 130, 110), 5),
    }
    for name, (tint, seed) in textures.items():
        cv2.imwrite(str(OUT / name), noise(512, tint, seed))
        print("wrote", OUT / name)


if __name__ == "__main__":
    main()
