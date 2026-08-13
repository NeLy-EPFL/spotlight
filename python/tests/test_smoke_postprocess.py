"""End-to-end smoke tests for the `postprocess-recording` CLI: run the real
pipeline (subprocess, exactly as a user would invoke it) over a tiny
30-frame fixture (see `conftest.mini_recording_dir`) and check the outputs
it's supposed to produce actually exist and look sane. Not a substitute for
inspecting a real recording's output by eye, just a fast guard against the
pipeline crashing or silently producing empty/malformed files.
"""

import subprocess
import sys
from pathlib import Path

import h5py
import numpy as np
import pytest

N_FRAMES = 30  # frames 0-27 (10 raw files x 3), see conftest.mini_recording_dir


def run_postprocess(
    recording_dir: Path, *extra_args: str
) -> subprocess.CompletedProcess:
    return subprocess.run(
        [
            sys.executable,
            "-m",
            "spotlight.postprocessing.cli.postprocess_recording",
            str(recording_dir),
            "--overwrite",
            "--muscle.mode",
            "off",
            *extra_args,
        ],
        capture_output=True,
        text=True,
    )


def assert_valid_alignment_transforms(h5_path: Path) -> None:
    with h5py.File(h5_path) as f:
        assert f["transform_matrices"].shape == (N_FRAMES, 2, 3)
        assert f["keypoints_xy_post_alignment"].shape == (N_FRAMES, 3, 2)
        assert not np.isnan(f["transform_matrices"][:]).any()


def assert_nonempty_video(video_path: Path) -> None:
    assert video_path.is_file()
    assert video_path.stat().st_size > 0


def test_smoke_no_muscle(mini_recording_dir: Path) -> None:
    """Base pipeline (aligned video only, muscle explicitly off) completes
    and writes the expected outputs."""
    result = run_postprocess(mini_recording_dir)
    assert result.returncode == 0, result.stderr

    postprocessed_dir = mini_recording_dir / "postprocessed"
    assert_nonempty_video(postprocessed_dir / "aligned_behavior_video.mp4")
    assert not (postprocessed_dir / "fullsize_behavior_video.mp4").exists()
    assert not (postprocessed_dir / "aligned_muscle_images.h5").exists()
    assert_valid_alignment_transforms(
        postprocessed_dir / "behavior_alignment_transforms.h5"
    )
    assert_nonempty_video(postprocessed_dir / "summary_video.mp4")


def test_smoke_alignment_both(mini_recording_dir: Path) -> None:
    """`--alignment.mode both` produces both the aligned and full-size
    behavior videos, still with muscle off."""
    result = run_postprocess(mini_recording_dir, "--alignment.mode", "both")
    assert result.returncode == 0, result.stderr

    postprocessed_dir = mini_recording_dir / "postprocessed"
    assert_nonempty_video(postprocessed_dir / "aligned_behavior_video.mp4")
    assert_nonempty_video(postprocessed_dir / "fullsize_behavior_video.mp4")
    assert_valid_alignment_transforms(
        postprocessed_dir / "behavior_alignment_transforms.h5"
    )
    assert_nonempty_video(postprocessed_dir / "summary_video.mp4")


@pytest.mark.parametrize("bad_mode", ["fullsize"])
def test_pose2d_requires_aligned_output(
    mini_recording_dir: Path, bad_mode: str
) -> None:
    """`--pose2d.enabled` with `--alignment.mode fullsize` is rejected by
    upfront validation (see `_validate_inputs`), not a late crash."""
    result = run_postprocess(
        mini_recording_dir, "--alignment.mode", bad_mode, "--pose2d.enabled"
    )
    assert result.returncode != 0
    assert "--pose2d.enabled requires" in result.stderr
