"""End-to-end calibration against a virtual rig with known geometry.

These render real chessboard images, run the full calibration on them, and
compare the result with the rig that produced them, so they exercise corner
detection, the stereo solve, rectification and triangulation together.
"""

from __future__ import annotations

import numpy as np
import pytest

from stereo_vision.calibration import calibrate_stereo, find_corners
from stereo_vision.config import BoardSpec
from stereo_vision.synthetic import TOLERANCES, default_rig, evaluate, render_pairs


@pytest.fixture(scope="module")
def rig():
    return default_rig()


# Seed 1 includes a board tilted 38 degrees and far enough away that its
# squares are about 19 px, which is the case that once broke corner refinement.
@pytest.mark.parametrize("seed", [0, 1])
def test_calibration_recovers_the_true_rig(rig, seed):
    board = BoardSpec()
    lefts, rights, _ = render_pairs(rig, board, count=15, seed=seed)

    calibration, used = calibrate_stereo(lefts, rights, board)
    metrics = evaluate(calibration, rig, board)

    assert len(used) == 15
    failures = {k: round(metrics[k], 4) for k, limit in TOLERANCES.items() if metrics[k] > limit}
    assert not failures, f"out of tolerance: {failures}"


def test_corner_refinement_stays_on_small_tilted_boards(rig):
    """No corner may jump to a neighbour, even on the hardest views."""
    board = BoardSpec()
    lefts, rights, poses = render_pairs(rig, board, count=15, seed=1)

    worst = 0.0
    for left, right, (rvec, tvec) in zip(lefts, rights, poses):
        for side, image in (("left", left), ("right", right)):
            found = find_corners(image, board).reshape(-1, 2)
            truth = rig.project_corners(side, board, rvec, tvec)
            worst = max(worst, float(np.abs(found - truth).max()))

    # A jump to the neighbouring corner costs a whole square, about 20 px here.
    assert worst < 1.0
