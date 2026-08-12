#!/usr/bin/env python
"""Convert one trial's poses between a `.slp` file and the shared `.h5` array
format (see
`spotlight_postprocessing.spotlight_pose2d.io_utils.save_pose_h5`/`load_pose_h5`).

Single-video only: a `.slp` file with more than one video can't round-trip
through this dense, one-row-per-frame `.h5` schema. Combining many trials
into one multi-video `.slp` is instead done incrementally, one trial's `.h5`
at a time, via `--h5-to-slp --append-to`.

Two modes, selected by `--slp-to-h5` or `--h5-to-slp`:

- `--slp-to-h5`: for each frame, `poses`/`keypoint_scores`/`instance_score`
  come from the best-scoring `PredictedInstance` (if any), and `is_label` is
  True wherever a user-labeled `Instance` also exists (in which case `poses`
  is that label's own points instead of the prediction's, matching the old
  pipeline's `--include-acceptance` convention). `--labels-only` instead
  leaves `poses`/`keypoint_scores`/`instance_score` NaN for any frame that
  isn't labeled, discarding prediction-only frames' data (`is_label` is
  unaffected either way).
- `--h5-to-slp`: builds one instance per non-NaN frame, either a
  `PredictedInstance` (default) or, with `--as-labels`, a plain `Instance`
  (no `from_predicted` link back to a prediction, since the `.h5` doesn't
  retain a label's original, pre-promotion prediction separately from its
  points). With `--append-to <existing.slp>`, merges into that file's own
  data instead of starting fresh: frames with no existing same-kind instance
  (a `PredictedInstance` if not `--as-labels`, else a plain `Instance`) are
  just added; frames where one already exists are resolved via `--priority`
  (`mine` replaces it, `theirs` keeps the existing one). `--append-to`
  requires `--priority` and an already-existing file.

Usage:
    python slp_convert_h5.py --slp-to-h5 \\
        --input-path predictions.slp --output-path predictions.h5
    python slp_convert_h5.py --h5-to-slp --as-labels \\
        --append-to combined.slp --priority mine \\
        --input-path trial_000.h5 --output-path combined.slp
"""

from pathlib import Path
from typing import Literal

import numpy as np
import sleap_io as sio
import tyro
from loguru import logger

from spotlight_postprocessing.spotlight_pose2d.io_utils import (
    check_output_path,
    load_pose_h5,
    load_skeleton_json,
    resolve_video_path,
    save_pose_h5,
)


def extract_video_arrays(
    labels: sio.Labels, video: sio.Video, node_names: list[str], labels_only: bool
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Build the shared `.h5` schema's dense per-frame arrays for one video.

    Args:
        labels: The full `Labels` object (frames from other videos are ignored).
        video: The video to extract frames for.
        node_names: Node name per column, in skeleton order.
        labels_only: See module docstring.

    Returns:
        poses, keypoint_scores, instance_score, is_label: See
        `spotlight_postprocessing.spotlight_pose2d.io_utils.save_pose_h5`.
    """
    n_nodes = len(node_names)
    frames = [lf for lf in labels.labeled_frames if lf.video is video]
    n_frames = max((lf.frame_idx for lf in frames), default=-1) + 1

    poses = np.full((n_frames, n_nodes, 2), np.nan, dtype=np.float32)
    keypoint_scores = np.full((n_frames, n_nodes), np.nan, dtype=np.float32)
    instance_score = np.full(n_frames, np.nan, dtype=np.float32)
    is_label = np.zeros(n_frames, dtype=bool)

    for lf in frames:
        predicted = [
            inst for inst in lf.instances if isinstance(inst, sio.PredictedInstance)
        ]
        # `PredictedInstance` is a subclass of `Instance`, so exclude it
        # explicitly to isolate actual user-labeled instances.
        user_instances = [
            inst for inst in lf.instances if not isinstance(inst, sio.PredictedInstance)
        ]

        if predicted and not (labels_only and not user_instances):
            best = max(predicted, key=lambda inst: inst.score)
            pts_scores = best.numpy(scores=True)  # (n_nodes, 3): x, y, score
            poses[lf.frame_idx] = pts_scores[:, :2]
            keypoint_scores[lf.frame_idx] = pts_scores[:, 2]
            instance_score[lf.frame_idx] = best.score

        if user_instances:
            is_label[lf.frame_idx] = True
            poses[lf.frame_idx] = user_instances[0].numpy()

    return poses, keypoint_scores, instance_score, is_label


def run_slp_to_h5(input_path: Path, output_path: Path, labels_only: bool) -> None:
    labels = sio.load_file(str(input_path))
    if len(labels.videos) != 1:
        raise SystemExit(
            f"{input_path} has {len(labels.videos)} videos; slp_convert_h5.py is "
            "single-video only (combine trials via --h5-to-slp --append-to instead)."
        )
    video = labels.videos[0]
    node_names = [node.name for node in labels.skeleton.nodes]

    poses, keypoint_scores, instance_score, is_label = extract_video_arrays(
        labels, video, node_names, labels_only
    )
    video_path = resolve_video_path(video.filename, str(input_path))
    save_pose_h5(
        output_path, poses, keypoint_scores, instance_score, is_label,
        node_names, video_path,
    )  # fmt: skip
    logger.info(
        f"Saved {len(poses)} frames ({int(is_label.sum())} labeled) to {output_path}"
    )


def build_instance(
    poses: np.ndarray,
    keypoint_scores: np.ndarray,
    instance_score: float,
    skeleton: sio.Skeleton,
    as_labels: bool,
) -> sio.Instance:
    """Build one `Instance`/`PredictedInstance` from one frame's h5 arrays."""
    if as_labels:
        return sio.Instance.from_numpy(points_data=poses, skeleton=skeleton)
    return sio.PredictedInstance.from_numpy(
        points_data=poses,
        skeleton=skeleton,
        point_scores=keypoint_scores,
        score=float(instance_score),
    )


def is_same_kind(instance: sio.Instance, as_labels: bool) -> bool:
    """Whether `instance` is the kind `--as-labels` is adding (label vs. prediction)."""
    return isinstance(instance, sio.PredictedInstance) != as_labels


def run_h5_to_slp(
    input_path: Path,
    output_path: Path,
    as_labels: bool,
    append_to: Path | None,
    priority: Literal["mine", "theirs"] | None,
    skeleton_json_path: Path | None,
) -> None:
    data = load_pose_h5(input_path)
    poses = data["poses"]
    frame_idxs = np.flatnonzero(~np.isnan(poses).all(axis=(1, 2)))

    if append_to is None:
        if skeleton_json_path is None:
            raise SystemExit(
                "--skeleton-json-path is required when --append-to is not "
                "set (a bare sio.Skeleton(nodes=...) has no edges, which leaves "
                "the output .slp skeleton-less in the GUI)."
            )
        skeleton = load_skeleton_json(skeleton_json_path)
        reference_node_names = [n.name for n in skeleton.nodes]
        if reference_node_names != data["node_names"]:
            raise SystemExit(
                f"{skeleton_json_path}'s skeleton nodes {reference_node_names} "
                f"don't match {input_path}'s {data['node_names']}."
            )
        video = sio.Video(filename=str(data["video_path"]))
        labeled_frames = [
            sio.LabeledFrame(
                video=video,
                frame_idx=int(i),
                instances=[
                    build_instance(
                        poses[i],
                        data["keypoint_scores"][i],
                        data["instance_score"][i],
                        skeleton,
                        as_labels,
                    )
                ],
            )
            for i in frame_idxs
        ]
        labels = sio.Labels(
            labeled_frames=labeled_frames, videos=[video], skeletons=[skeleton]
        )
        output_path.parent.mkdir(parents=True, exist_ok=True)
        sio.save_file(labels, str(output_path))
        logger.info(f"Saved {len(labeled_frames)} frames to {output_path}")
        return

    if priority is None:
        raise SystemExit("--priority is required when --append-to is set.")
    if not append_to.is_file():
        raise SystemExit(f"--append-to file does not exist: {append_to}")

    existing = sio.load_file(str(append_to))
    existing_node_names = [node.name for node in existing.skeleton.nodes]
    if existing_node_names != data["node_names"]:
        raise SystemExit(
            f"Node name mismatch between {append_to} ({existing_node_names}) "
            f"and {input_path} ({data['node_names']})."
        )
    skeleton = existing.skeleton

    resolved_video_path = str(data["video_path"])
    video = next(
        (v for v in existing.videos if str(Path(v.filename).resolve()) == resolved_video_path),
        None,
    )  # fmt: skip
    is_new_video = video is None
    if is_new_video:
        video = sio.Video(filename=resolved_video_path)
        existing.videos.append(video)

    frames_by_idx = {
        lf.frame_idx: lf for lf in existing.labeled_frames if lf.video is video
    }

    n_added, n_replaced, n_kept_theirs = 0, 0, 0
    for i in frame_idxs:
        frame_idx = int(i)
        new_instance = build_instance(
            poses[i], data["keypoint_scores"][i], data["instance_score"][i],
            skeleton, as_labels,
        )  # fmt: skip

        lf = frames_by_idx.get(frame_idx)
        if lf is None:
            lf = sio.LabeledFrame(
                video=video, frame_idx=frame_idx, instances=[new_instance]
            )
            existing.labeled_frames.append(lf)
            frames_by_idx[frame_idx] = lf
            n_added += 1
            continue

        conflicting = [inst for inst in lf.instances if is_same_kind(inst, as_labels)]
        if not conflicting:
            lf.instances.append(new_instance)
            n_added += 1
        elif priority == "mine":
            for inst in conflicting:
                lf.instances.remove(inst)
            lf.instances.append(new_instance)
            n_replaced += 1
        else:
            n_kept_theirs += 1

    output_path.parent.mkdir(parents=True, exist_ok=True)
    sio.save_file(existing, str(output_path))
    logger.info(
        f"Appended {resolved_video_path} into {append_to}: {n_added} added, "
        f"{n_replaced} replaced (mine), {n_kept_theirs} kept (theirs). "
        f"Saved to {output_path}"
    )


def main(
    input_path: Path,
    output_path: Path,
    slp_to_h5: bool = False,
    h5_to_slp: bool = False,
    labels_only: bool = False,
    as_labels: bool = False,
    append_to: Path | None = None,
    priority: Literal["mine", "theirs"] | None = None,
    skeleton_json_path: Path | None = None,
    override: bool = False,
) -> None:
    """Convert one trial's poses between `.slp` and the shared `.h5` format.

    Args:
        input_path: `.slp` file for --slp-to-h5, `.h5` file for --h5-to-slp.
        output_path: `.h5` file for --slp-to-h5, `.slp` file for --h5-to-slp.
            Aborts if this already exists, unless `override` is set.
        slp_to_h5: Convert `input_path` (a single-video `.slp`) to the shared
            `.h5` format. Exactly one of `slp_to_h5`/`h5_to_slp` must be set.
        h5_to_slp: Convert `input_path` (a `.h5` from `--slp-to-h5`) to a
            `.slp` file. Exactly one of `slp_to_h5`/`h5_to_slp` must be set.
        labels_only: --slp-to-h5 only. See module docstring.
        as_labels: --h5-to-slp only. Build plain `Instance`s (user labels)
            instead of `PredictedInstance`s.
        append_to: --h5-to-slp only. Merge into this existing `.slp` file's
            data instead of building a fresh single-video file. Requires
            `priority`. Its own skeleton (edges included) is reused, so
            `skeleton_json_path` is not needed alongside it.
        priority: --h5-to-slp with --append-to only. How to resolve a frame
            that already has a same-kind (label vs. prediction) instance in
            `append_to`: `"mine"` replaces it with this run's, `"theirs"`
            keeps the existing one.
        skeleton_json_path: --h5-to-slp without --append-to only. Required:
            a JSON file with the real skeleton (nodes, edges, symmetries),
            from `extract_metadata_from_initial_slp.py`, used as-is for the
            output. Its node
            names must match `input_path`'s own `node_names` exactly, order
            included. Without this, a nodes-only `sio.Skeleton` would have
            to be built from scratch, silently dropping edges.
        override: If True, overwrite `output_path` if it already exists.
    """
    if slp_to_h5 == h5_to_slp:
        raise SystemExit("Specify exactly one of --slp-to-h5 or --h5-to-slp.")
    if not input_path.is_file():
        raise SystemExit(f"Input file does not exist: {input_path}")
    check_output_path(output_path, override)

    if slp_to_h5:
        run_slp_to_h5(input_path, output_path, labels_only)
    else:
        run_h5_to_slp(
            input_path, output_path, as_labels, append_to, priority,
            skeleton_json_path,
        )  # fmt: skip


if __name__ == "__main__":
    tyro.cli(main)
