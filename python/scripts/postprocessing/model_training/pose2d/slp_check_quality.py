#!/usr/bin/env python
"""Checks a merged `.slp` file (current format) for label-quality issues
across every labeled frame -- any `LabeledFrame` with at least one
non-`PredictedInstance`, so both genuinely hand-labeled and
confidence-promoted frames are checked, not just raw predictions.
Read-only: never writes to `input_path` or anywhere else.

Three checks, run over every labeled instance:

1. Visibility: every keypoint should be visible (and present -- a missing
   keypoint's coordinates are `nan`), unless `--allow-invisible` is set.
2. Segment-length outliers: for each skeleton edge, a frame's segment
   length (Euclidean distance between its two node positions) is flagged
   if it exceeds `--threshold-percentile-multiplier` times that edge's own
   `--threshold-percentile`th percentile, computed across every labeled
   frame in this file. Percentiles are computed per edge, not pooled
   across edges, since different segments have very different typical
   lengths.
3. Left/right separation: per frame, an independent two-sample t-test
   (`scipy.stats.ttest_ind`, one-sided: left's mean x < right's mean x)
   over all `L*`-node vs. `R*`-node x-coordinates (excluding any `nan`);
   flagged if p >= `--max-leftright-separation-p`, i.e. this frame doesn't
   confidently have every left-side node to the left of every right-side
   one, which should always hold given this project's coordinate
   convention (verified in `augment.py`'s own flip-correctness checks).
   Skipped (not flagged) if either side has fewer than 2 visible points
   to test.

Exits 1 if there are fewer than `--min-labeled-frames` labeled frames (the
other three checks aren't run in that case), or if any violation was
flagged; exits 0 only if every check passed.

Usage:
    python tools/spotlight_pose2d/slp_check_quality.py \\
        --input-path merged.slp
"""

import sys
import textwrap
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import sleap_io as sio
import tyro
from loguru import logger
from scipy import stats
from tqdm import tqdm

from spotlight_postprocessing.pose2d.io_utils import parse_trial_identity

REPORT_WIDTH = 80
REPORT_INDENT = "    "


@dataclass
class LabeledEntry:
    video: sio.Video
    frame_idx: int
    points: np.ndarray  # (n_nodes, 2), nan for missing/invisible


@dataclass
class Violation:
    video: sio.Video
    frame_idx: int
    reason: str


def real_instances(lf: sio.LabeledFrame) -> list[sio.Instance]:
    """This frame's non-predicted instances (empty if it has none)."""
    return [
        inst for inst in lf.instances if not isinstance(inst, sio.PredictedInstance)
    ]


def trial_stem(video: sio.Video) -> str:
    """`<genotype>__<fly_trial>`, this project's usual trial identifier."""
    genotype, fly_trial = parse_trial_identity(video.filename)
    return f"{genotype}__{fly_trial}"


def video_frame_count(video: sio.Video) -> int | None:
    """`video`'s total frame count, or None if the video can't be opened
    (`sio.Video.shape` is None rather than raising, in that case).
    """
    if video.shape is None:
        logger.warning(f"Could not open {video.filename} for its frame count")
        return None
    return int(video.shape[0])


def check_visibility(node_names: list[str], points: np.ndarray) -> str | None:
    """A violation reason if any keypoint is missing/invisible, else None.

    Args:
        node_names: Node name per row of `points`.
        points: `(n_nodes, 2)`, `nan` for missing/invisible keypoints (see
            `sio.Instance.numpy`'s `invisible_as_nan` default).
    """
    unseen = np.isnan(points).any(axis=-1)
    if not unseen.any():
        return None
    names = [name for name, is_unseen in zip(node_names, unseen) if is_unseen]
    return f"{len(names)} invisible/missing keypoint(s): {', '.join(names)}"


def check_leftright_separation(
    node_names: list[str], points: np.ndarray, max_p: float
) -> str | None:
    """A violation reason if left isn't confidently left of right, else None."""
    left_x = [
        points[i, 0]
        for i, name in enumerate(node_names)
        if name.startswith("L") and not np.isnan(points[i, 0])
    ]
    right_x = [
        points[i, 0]
        for i, name in enumerate(node_names)
        if name.startswith("R") and not np.isnan(points[i, 0])
    ]
    if len(left_x) < 2 or len(right_x) < 2:
        return None
    _, p_value = stats.ttest_ind(left_x, right_x, alternative="less")
    if p_value >= max_p:
        return f"L/R x-separation not significant (p={p_value:.4f} >= {max_p:g})"
    return None


def print_violation_report(
    violations: list[Violation], videos: list[sio.Video]
) -> None:
    """Prints one wrapped, blank-line-separated block per violation.

    Args:
        violations: In the order to report them.
        videos: Every video in the file, for each violation's video's
            position (1-indexed) among all of them.
    """
    video_index = {video: i + 1 for i, video in enumerate(videos)}
    frame_counts: dict[sio.Video, int | None] = {}

    for i, violation in enumerate(violations):
        video = violation.video
        if video not in frame_counts:
            frame_counts[video] = video_frame_count(video)
        total_frames = frame_counts[video]
        frame_total_str = str(total_frames) if total_frames is not None else "?"

        print(f"Warning {i + 1}/{len(violations)}")
        print(
            f"{REPORT_INDENT}{'Video':<8}{video_index[video]}/{len(videos)} "
            f"({trial_stem(video)})"
        )
        print(f"{REPORT_INDENT}{'Frame':<8}{violation.frame_idx + 1}/{frame_total_str}")
        print(
            textwrap.fill(
                violation.reason,
                width=REPORT_WIDTH,
                initial_indent=REPORT_INDENT,
                subsequent_indent=REPORT_INDENT,
                break_long_words=False,
                break_on_hyphens=False,
            )
        )
        print()


def main(
    input_path: Path,
    allow_invisible: bool = False,
    threshold_percentile: float = 95.0,
    threshold_percentile_multiplier: float = 1.5,
    max_leftright_separation_p: float = 0.01,
    min_labeled_frames: int = 20,
) -> None:
    """Check `input_path`'s labeled frames for quality issues.

    Args:
        input_path: Merged `.slp` file (current format) to check.
        allow_invisible: If False (the default), any invisible or missing
            keypoint is a violation.
        threshold_percentile: Percentile of each skeleton edge's own
            segment-length distribution (across every labeled frame here)
            used as its outlier threshold's base.
        threshold_percentile_multiplier: A frame's segment length is a
            violation if it exceeds this many times that edge's
            `threshold_percentile`.
        max_leftright_separation_p: A frame is a violation if a one-sided
            t-test (left's mean x < right's mean x) gives p at or above
            this.
        min_labeled_frames: Exit 1 immediately if fewer than this many
            frames are labeled; the checks above assume enough data to be
            statistically meaningful.
    """
    labels = sio.load_file(str(input_path))
    node_names = [n.name for n in labels.skeleton.nodes]
    edges = [(e.source.name, e.destination.name) for e in labels.skeleton.edges]
    node_index = {name: i for i, name in enumerate(node_names)}

    entries: list[LabeledEntry] = []
    is_tty = sys.stdout.isatty()
    iterator = (
        tqdm(labels.labeled_frames, desc="Scanning frames", mininterval=1.0)
        if is_tty
        else labels.labeled_frames
    )
    log_every = max(1, int(0.01 * len(labels.labeled_frames)))
    for i, lf in enumerate(iterator):
        instances = real_instances(lf)
        for inst in instances:
            entries.append(LabeledEntry(lf.video, lf.frame_idx, inst.numpy()))
        if not is_tty and i % log_every == 0:
            logger.info(f"{i}/{len(labels.labeled_frames)} frames scanned")

    n_labeled_frames = len({(e.video, e.frame_idx) for e in entries})
    logger.info(f"{n_labeled_frames} labeled frames found in {input_path}")
    if n_labeled_frames < min_labeled_frames:
        logger.error(
            f"Only {n_labeled_frames} labeled frames, fewer than "
            f"--min-labeled-frames={min_labeled_frames}"
        )
        raise SystemExit(1)

    violations: list[Violation] = []

    if not allow_invisible:
        for entry in entries:
            reason = check_visibility(node_names, entry.points)
            if reason is not None:
                violations.append(Violation(entry.video, entry.frame_idx, reason))

    edge_lengths = defaultdict(list)  # (src, dst) -> [(entry, length), ...]
    for entry in entries:
        for src, dst in edges:
            p_src = entry.points[node_index[src]]
            p_dst = entry.points[node_index[dst]]
            if np.isnan(p_src).any() or np.isnan(p_dst).any():
                continue
            edge_lengths[(src, dst)].append(
                (entry, float(np.linalg.norm(p_dst - p_src)))
            )
    for (src, dst), records in edge_lengths.items():
        lengths = np.array([length for _, length in records])
        threshold = threshold_percentile_multiplier * np.percentile(
            lengths, threshold_percentile
        )
        for entry, length in records:
            if length > threshold:
                reason = (
                    f"{src}-{dst} segment length {length:.1f} exceeds threshold "
                    f"{threshold:.1f} ({threshold_percentile_multiplier:g}x the "
                    f"p{threshold_percentile:g} across all labeled frames)"
                )
                violations.append(Violation(entry.video, entry.frame_idx, reason))

    for entry in entries:
        reason = check_leftright_separation(
            node_names, entry.points, max_leftright_separation_p
        )
        if reason is not None:
            violations.append(Violation(entry.video, entry.frame_idx, reason))

    violations.sort(key=lambda v: (trial_stem(v.video), v.frame_idx, v.reason))
    print_violation_report(violations, labels.videos)
    logger.info(
        f"{len(violations)} violation(s) across {n_labeled_frames} labeled frames"
    )

    if violations:
        raise SystemExit(1)


if __name__ == "__main__":
    tyro.cli(main)
