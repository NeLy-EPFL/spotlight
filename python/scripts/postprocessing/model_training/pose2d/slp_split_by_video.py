#!/usr/bin/env python
"""Splits a merged, multi-video `.slp` (e.g. from `slp_merge.py`, or
`sample_predictions_after_training.py`'s own merge step) back into one
single-video `.slp` per trial, named by this project's
`<genotype>__<fly_trial>` stem convention (parsed from each video's own
path via
`spotlight_postprocessing.pose2d.io_utils.parse_trial_identity`).

`slp_convert_h5.py --slp-to-h5` is single-video only, so a merged `.slp`
needs this step first before it can be converted to the dense pose `.h5`
format tools like `train.py` read.

Usage:
    python tools/spotlight_pose2d/slp_split_by_video.py \\
        --input-path merged.slp \\
        --output-dir split_slp/
"""

from pathlib import Path

import sleap_io as sio
import tyro
from loguru import logger

from spotlight_postprocessing.pose2d.io_utils import (
    check_output_path,
    parse_trial_identity,
)


def main(input_path: Path, output_dir: Path, override: bool = False) -> None:
    """Split `input_path` into one single-video `.slp` per video.

    Args:
        input_path: Merged, multi-video `.slp` file.
        output_dir: Directory to save each trial's own
            `<genotype>__<fly_trial>.slp` into.
        override: If True, overwrite an existing output file for a trial.
    """
    labels = sio.load_file(str(input_path))
    output_dir.mkdir(parents=True, exist_ok=True)

    for video in labels.videos:
        genotype, fly_trial = parse_trial_identity(video.filename)
        stem = f"{genotype}__{fly_trial}"
        output_path = output_dir / f"{stem}.slp"
        check_output_path(output_path, override)

        frames = [lf for lf in labels.labeled_frames if lf.video is video]
        split_labels = sio.Labels(
            labeled_frames=frames, videos=[video], skeletons=[labels.skeleton]
        )
        sio.save_file(split_labels, str(output_path))
        logger.info(f"{stem}: {len(frames)} frames -> {output_path}")


if __name__ == "__main__":
    tyro.cli(main)
