#!/usr/bin/env python
"""Splits a directory of per-trial dense pose `.h5` files into train/val
sets for `pose2d`'s `train.py`: a random split (seed 0 by default), except
any trial with more than `min_hand_labeled_frames` genuine hand labels (see
`extract_metadata_from_initial_slp.py`) is always forced into train. Those
are this project's only real ground truth, and `train.py`'s `PoseDataset`
only trains on a trial's own `.h5` when that whole file is in the train
set (see `dataset.py`) -- letting one land in validation-only would mean
none of its labels ever reach a gradient update.

Trials with zero labeled (`is_label=True`) frames -- e.g. one a still-in-
progress hand-correction pass hasn't reached yet -- are excluded from the
split entirely, neither train nor validation: `PoseDataset` requires every
`.h5` path it's given to contribute at least one labeled frame, and a
trial with none would otherwise crash it (`np.stack` on an empty list) or,
in validation, be unscoreable anyway. Excluded trials reappear
automatically next time this is run, once they have real labels.

The random draw itself doesn't change to accommodate this: it's the same
seeded permutation over all trial stems as an unconstrained split would use,
just walked while skipping (deferring to train) any forced-train stem. A
forced trial that would have landed outside the first `val_fraction` share
of the permutation anyway (true of every trial in this project's current
29-trial dataset) reproduces the exact same split an unconstrained draw
would have produced.

Usage:
    python tools/spotlight_pose2d/split_train_val.py \\
        --data-root bulk_data/.../labels/ported_lm_predictions \\
        --metadata-json-path bulk_data/.../labels/metadata.json \\
        --output-path bulk_data/.../train_val_split.json
"""

import json
from pathlib import Path

import numpy as np
import tyro
from loguru import logger

from spotlight_postprocessing.spotlight_pose2d.io_utils import (
    check_output_path,
    load_pose_h5,
)


def main(
    data_root: Path,
    metadata_json_path: Path,
    output_path: Path,
    min_hand_labeled_frames: int = 20,
    val_fraction: float = 0.2,
    seed: int = 0,
    override: bool = False,
) -> None:
    """Split `data_root`'s trials into train/val, keeping well-hand-labeled
    trials in train.

    Args:
        data_root: Directory containing each trial's `*_pose.h5`; every
            match becomes one entry in the split.
        metadata_json_path: JSON from `extract_metadata_from_initial_slp.py`
            (only its `hand_labeled_frames` field is used here).
        output_path: Where to save `{"train": [...], "validation": [...]}`.
            Aborts if this already exists, unless `override` is set.
        min_hand_labeled_frames: A trial with more than this many genuine
            hand-labeled frames is always placed in train, never validation.
        val_fraction: Target fraction of trials in validation, before the
            hand-labeled-trial exclusion above; the actual validation count
            can only shrink from this, if a trial that would have landed
            there is forced into train instead (see module docstring).
        seed: Random seed for the trial permutation.
        override: If True, overwrite `output_path` if it already exists.
    """
    check_output_path(output_path, override)
    h5_paths = {p.name.removesuffix("_pose.h5"): p for p in data_root.glob("*_pose.h5")}
    if not h5_paths:
        raise SystemExit(f"No *_pose.h5 files found under {data_root}")

    unlabeled = {
        stem for stem, p in h5_paths.items() if not load_pose_h5(p)["is_label"].any()
    }
    if unlabeled:
        logger.warning(
            f"{len(unlabeled)}/{len(h5_paths)} trials have zero labeled frames, "
            f"excluding from the split: {sorted(unlabeled)}"
        )
    stems = sorted(set(h5_paths) - unlabeled)
    if not stems:
        raise SystemExit(f"Every trial under {data_root} has zero labeled frames")

    hand_labeled_frames = json.loads(metadata_json_path.read_text())[
        "hand_labeled_frames"
    ]
    forced_train = {
        stem
        for stem in stems
        if len(hand_labeled_frames.get(stem, [])) > min_hand_labeled_frames
    }
    logger.info(
        f"{len(forced_train)}/{len(stems)} trials forced into train "
        f"(>{min_hand_labeled_frames} hand-labeled frames): {sorted(forced_train)}"
    )

    n_val = round(len(stems) * val_fraction)
    permuted = np.random.default_rng(seed).permutation(stems).tolist()

    train, validation = [], []
    for stem in permuted:
        if stem not in forced_train and len(validation) < n_val:
            validation.append(stem)
        else:
            train.append(stem)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps({"train": sorted(train), "validation": sorted(validation)}, indent=2)
    )
    logger.info(
        f"{len(train)} train, {len(validation)} validation trials -> {output_path}"
    )


if __name__ == "__main__":
    tyro.cli(main)
