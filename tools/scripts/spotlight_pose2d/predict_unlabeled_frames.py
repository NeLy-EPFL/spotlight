#!/usr/bin/env python
"""Runs a trained pose2d checkpoint over one trial's not-yet-labeled frames,
producing a new dense pose `.h5` (see
`spotlight_tools.spotlight_pose2d.io_utils`) and a matching
current-format `.slp` for GUI correction, where frames explicitly listed in
`protected_frame_indices` are kept as real (unpromoted-safe) labels and
every other frame gets a fresh, unpromoted prediction from this model --
for seeding the next labeling iteration from an already-ported dataset
without re-running the older LM model.

Deliberately does NOT trust the input `.h5`'s own `is_label` column to
decide what to protect: that flag also covers this project's earlier
confidence-based auto-promotion (see `slp_promote_by_confidence.py`), which
are not real hand labels and should be re-predicted like anything else, not
carried forward as fixed. Callers must pass the exact frame indices that
are genuine hand labels (see `sample_predictions_after_training.py`, which
derives them from `labels/first_round/labels.v005.slp`).

Reads frames from `cache_video_frames.py`'s JPEG cache (see
`cache_all_trial_frames.py`), not the source video, keyed by the input
`.h5`'s own `video_path` attr (via `parse_trial_identity`), not its
filename.

Usage:
    python tools/spotlight_pose2d/predict_unlabeled_frames.py \\
        --checkpoint-path bulk_data/.../best.pt \\
        --input-path bulk_data/.../<trial>_pose.h5 \\
        --frame-cache-root bulk_data/motion_prior/2dpose_model/frame_cache \\
        --skeleton-json-path bulk_data/.../labels/metadata.json \\
        --output-h5-path <trial>_pose2d_predictions.h5 \\
        --output-slp-path <trial>_pose2d_predictions.slp \\
        --protected-frame-indices 103 139 618
"""

import sys
from pathlib import Path

import cv2
import numpy as np
import sleap_io as sio
import torch
import tyro
from infer import heatmaps_to_points
from loguru import logger
from tqdm import tqdm

from spotlight_tools.spotlight_pose2d.dataset import (
    ALIGNED_FRAME_SIZE,
    INPUT_SIZE,
)
from spotlight_tools.spotlight_pose2d.io_utils import (
    check_output_path,
    load_pose_h5,
    load_skeleton_json,
    parse_trial_identity,
    save_pose_h5,
)
from spotlight_tools.spotlight_pose2d.model import RepVGGPoseModel


def build_slp(
    poses: np.ndarray,
    keypoint_scores: np.ndarray,
    instance_score: np.ndarray,
    is_label: np.ndarray,
    included_frame_idxs: np.ndarray,
    skeleton: sio.Skeleton,
    video_path: Path,
) -> sio.Labels:
    """One `LabeledFrame` per index in `included_frame_idxs` (skipping
    everything else, e.g. frames `frames_per_trial` sampling left
    unpredicted): a plain `Instance` where `is_label`, else a
    `PredictedInstance` built from this model's own output.
    """
    video = sio.Video(filename=str(video_path))
    labeled_frames = []
    for frame_idx in included_frame_idxs:
        frame_idx = int(frame_idx)
        if is_label[frame_idx]:
            instance = sio.Instance.from_numpy(
                points_data=poses[frame_idx], skeleton=skeleton
            )
        else:
            instance = sio.PredictedInstance.from_numpy(
                points_data=poses[frame_idx],
                skeleton=skeleton,
                point_scores=keypoint_scores[frame_idx],
                score=float(instance_score[frame_idx]),
            )
        labeled_frames.append(
            sio.LabeledFrame(video=video, frame_idx=frame_idx, instances=[instance])
        )
    return sio.Labels(
        labeled_frames=labeled_frames, videos=[video], skeletons=[skeleton]
    )


def main(
    checkpoint_path: Path,
    input_path: Path,
    frame_cache_root: Path,
    skeleton_json_path: Path,
    output_h5_path: Path,
    output_slp_path: Path,
    protected_frame_indices: list[int] | None = None,
    frames_per_trial: int | None = None,
    n_keypoints: int = 37,
    batch_size: int = 128,
    override: bool = False,
) -> None:
    """Predict non-protected frames in one trial's dense pose `.h5`.

    Args:
        checkpoint_path: Trained `RepVGGPoseModel` state dict.
        input_path: This trial's dense pose `.h5` (see `save_pose_h5`).
        frame_cache_root: Root directory `cache_video_frames.py` wrote into;
            must already have this trial cached at `INPUT_SIZE`.
        skeleton_json_path: JSON file with the real skeleton (nodes, edges,
            symmetries), from `extract_metadata_from_initial_slp.py`. Its
            node names must match `input_path`'s own `node_names` exactly,
            order included. Used as-is for the output `.slp`'s skeleton,
            rather than building a nodes-only `sio.Skeleton` from scratch,
            which would silently drop the edges the SLEAP GUI needs to
            draw it.
        output_h5_path: Where to save the new dense pose `.h5`. Aborts if
            this already exists, unless `override` is set.
        output_slp_path: Where to save the new current-format `.slp`. Aborts
            if this already exists, unless `override` is set.
        protected_frame_indices: Frame indices to keep exactly as-is from
            `input_path` (a real label, not predicted); every other
            candidate frame is freshly predicted regardless of what
            `input_path` had there (a prior prediction or an auto-promoted
            "label"). Empty/unset means every frame is a candidate.
        frames_per_trial: If set, only predict this many randomly-sampled
            (fixed seed 0, independently per trial) non-protected frames,
            instead of every one -- for a fast per-round quality check
            rather than exhaustive predictions. Frames neither protected
            nor sampled are left as NaN in the `.h5` (not carried over from
            `input_path`, for the same reason `protected_frame_indices`
            aren't trusted from there) and excluded entirely from the
            `.slp` (no empty placeholder frames). Unset predicts every
            non-protected frame.
        n_keypoints: Must match the checkpoint's `n_keypoints`.
        batch_size: Frames per inference batch.
        override: If True, overwrite the output paths if they already exist.
    """
    check_output_path(output_h5_path, override)
    check_output_path(output_slp_path, override)
    logger.info(f"Checkpoint: {checkpoint_path}")
    logger.info(f"Input: {input_path}")
    logger.info(f"Frame cache root: {frame_cache_root}")
    logger.info(f"Skeleton JSON: {skeleton_json_path}")
    logger.info(f"Output .h5: {output_h5_path}")
    logger.info(f"Output .slp: {output_slp_path}")

    skeleton = load_skeleton_json(skeleton_json_path)
    data = load_pose_h5(input_path)
    reference_node_names = [n.name for n in skeleton.nodes]
    if reference_node_names != data["node_names"]:
        raise SystemExit(
            f"{skeleton_json_path}'s skeleton nodes {reference_node_names} "
            f"don't match {input_path}'s {data['node_names']}."
        )
    n_frames = len(data["poses"])
    poses = np.full_like(data["poses"], np.nan)
    keypoint_scores = np.full_like(data["keypoint_scores"], np.nan)
    instance_score = np.full_like(data["instance_score"], np.nan)
    is_label = np.zeros(n_frames, dtype=bool)

    protected = np.array(sorted(protected_frame_indices or []), dtype=int)
    is_label[protected] = True
    poses[protected] = data["poses"][protected]
    keypoint_scores[protected] = data["keypoint_scores"][protected]
    instance_score[protected] = data["instance_score"][protected]

    candidate_idxs = np.array(
        [i for i in range(n_frames) if i not in set(protected.tolist())]
    )
    if frames_per_trial is not None and frames_per_trial < len(candidate_idxs):
        rng = np.random.default_rng(0)
        predict_idxs = np.sort(
            rng.choice(candidate_idxs, size=frames_per_trial, replace=False)
        )
    else:
        predict_idxs = candidate_idxs
    included_frame_idxs = np.sort(np.concatenate([protected, predict_idxs]))
    logger.info(
        f"{len(protected)} protected (real label), {len(predict_idxs)} to predict "
        f"(of {len(candidate_idxs)} candidates)"
    )

    genotype, fly_trial = parse_trial_identity(data["video_path"])
    frame_dir = (
        frame_cache_root / f"{genotype}__{fly_trial}" / f"{INPUT_SIZE}x{INPUT_SIZE}"
    )

    device = "cuda" if torch.cuda.is_available() else "cpu"
    model = RepVGGPoseModel(n_keypoints, pretrained_backbone=False).to(device)
    model.load_state_dict(torch.load(checkpoint_path, map_location=device))
    model.eval()

    starts = range(0, len(predict_idxs), batch_size)
    n_batches = len(starts)
    log_every = max(1, int(0.01 * n_batches))
    is_tty = sys.stdout.isatty()
    iterator = tqdm(starts, mininterval=1.0) if is_tty else starts

    with torch.no_grad():
        for batch_idx, start in enumerate(iterator):
            batch_idxs = predict_idxs[start : start + batch_size]
            frames = []
            for frame_idx in batch_idxs:
                frame_path = frame_dir / f"frame_{frame_idx:09d}.jpg"
                frame = cv2.imread(str(frame_path), cv2.IMREAD_UNCHANGED)
                if frame is None:
                    raise SystemExit(
                        f"Missing cached frame {frame_path}; run "
                        "cache_all_trial_frames.py first."
                    )
                frames.append(frame)
            batch = np.repeat(np.stack(frames)[:, :, :, None], 3, axis=-1)
            tensor = (
                torch.from_numpy(batch).permute(0, 3, 1, 2).float().to(device) / 255.0
            )

            heatmaps = model(tensor).cpu().numpy()
            heatmap_scale = ALIGNED_FRAME_SIZE / heatmaps.shape[-1]
            for offset, heatmap in enumerate(heatmaps):
                frame_idx = batch_idxs[offset]
                points, scores = heatmaps_to_points(heatmap)
                poses[frame_idx] = points * heatmap_scale
                keypoint_scores[frame_idx] = scores
                instance_score[frame_idx] = scores.mean()

            if not is_tty and batch_idx % log_every == 0:
                logger.info(f"{start}/{len(predict_idxs)} frames")

    save_pose_h5(
        output_h5_path,
        poses,
        keypoint_scores,
        instance_score,
        is_label,
        data["node_names"],
        data["video_path"],
    )
    labels = build_slp(
        poses, keypoint_scores, instance_score, is_label, included_frame_idxs,
        skeleton, data["video_path"],
    )  # fmt: skip
    sio.save_file(labels, str(output_slp_path))
    logger.info(
        f"Saved {len(included_frame_idxs)}/{n_frames} frames -> "
        f"{output_h5_path}, {output_slp_path}"
    )


if __name__ == "__main__":
    tyro.cli(main)
