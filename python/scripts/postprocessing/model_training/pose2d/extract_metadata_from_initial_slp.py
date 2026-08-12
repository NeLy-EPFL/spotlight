#!/usr/bin/env python
"""Extracts everything downstream tools need from the original hand-labeled
`.slp` (`labels/first_round/labels.v005.slp`) into one small, versionable
JSON:

- The real skeleton (node names, edges, symmetries) -- see
  `spotlight_postprocessing.pose2d.io_utils.skeleton_json_dict`/
  `load_skeleton_json`. Building a skeleton from bare node names elsewhere
  drops edges, leaving the SLEAP GUI unable to draw it.
- `hand_labeled_frames`: `{"<genotype>__<fly_trial>": [frame_idx, ...]}` for
  every trial with a genuine hand label here (matched via
  `io_utils.parse_genotype_trial`/`user_labeled_frames`), so tools that need
  to protect real labels from re-prediction, or decide which trials have
  enough of them to matter (see `split_train_val.py`), don't each re-parse
  the `.slp` themselves.

Usage:
    python tools/spotlight_pose2d/extract_metadata_from_initial_slp.py \\
        --input-path bulk_data/.../labels/first_round/labels.v005.slp \\
        --output-path bulk_data/.../labels/metadata.json
"""

import json
from pathlib import Path

import sleap_io as sio
import tyro
from loguru import logger

from spotlight_postprocessing.pose2d.io_utils import (
    check_output_path,
    parse_genotype_trial,
    skeleton_json_dict,
    user_labeled_frames,
)


def hand_labeled_frames_by_trial(labels: sio.Labels) -> dict[str, list[int]]:
    """`{"<genotype>__<fly_trial>": [frame_idx, ...]}` for every trial with
    a genuine hand label in `labels`, per `io_utils.parse_genotype_trial`'s
    own matching logic (skips any video that doesn't match that convention,
    e.g. `labels.v005.slp`'s one unrelated video).
    """
    by_trial: dict[str, list[int]] = {}
    for video in labels.videos:
        parsed = parse_genotype_trial(video.filename)
        if parsed is None:
            continue
        genotype, fly_trial = parsed
        frames = user_labeled_frames(labels, video)
        by_trial[f"{genotype}__{fly_trial}"] = sorted(idx for idx, _ in frames)
    return by_trial


def main(input_path: Path, output_path: Path, override: bool = False) -> None:
    """Extract `input_path`'s skeleton and hand-labeled frame indices to
    `output_path` as JSON.

    Args:
        input_path: The original hand-labeled `.slp`
            (`labels/first_round/labels.v005.slp`).
        output_path: Where to save the JSON file. Aborts if this already
            exists, unless `override` is set.
        override: If True, overwrite `output_path` if it already exists.
    """
    check_output_path(output_path, override)
    labels = sio.load_file(str(input_path))

    data = skeleton_json_dict(labels.skeleton)
    data["hand_labeled_frames"] = hand_labeled_frames_by_trial(labels)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(data, indent=2))

    n_trials = len(data["hand_labeled_frames"])
    n_frames = sum(len(v) for v in data["hand_labeled_frames"].values())
    logger.info(
        f"Saved {len(labels.skeleton.nodes)} nodes, {len(labels.skeleton.edges)} "
        f"edges, {len(labels.skeleton.symmetries)} symmetries, "
        f"{n_frames} hand-labeled frames across {n_trials} trials -> {output_path}"
    )


if __name__ == "__main__":
    tyro.cli(main)
