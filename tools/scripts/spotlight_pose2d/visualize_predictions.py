#!/usr/bin/env python
"""Renders a quick annotated video of one trial's dense pose predictions:
the model's own raw per-frame heatmap output (max-pooled across all
keypoints into one combined map, linearly upsampled from its native low
resolution to the display size -- no extra smoothing, this is exactly
what the model predicted) overlaid live, with the predicted skeleton
drawn on top. Skeleton edges/nodes are read from a metadata JSON (see
extract_metadata_from_initial_slp.py) rather than hardcoded fly-leg
conventions. cv2-only drawing (no matplotlib, much faster for per-frame
overlays); video I/O via `pvio`, GPU (NVENC) encoded.

Usage:
    python tools/spotlight_pose2d/visualize_predictions.py \\
        --input-path bulk_data/.../<trial>_pose2d_final_predictions.h5 \\
        --checkpoint-path bulk_data/.../checkpoints/iter2a/best.pt \\
        --frame-cache-root bulk_data/motion_prior/2dpose_model/frame_cache \\
        --skeleton-json-path bulk_data/.../labels/metadata.json \\
        --output-path <trial>_pose2d_final_predictions.mp4 \\
        --crf 23
"""

import sys
from pathlib import Path

import cmasher as cmr
import cv2
import numpy as np
import pvio
import torch
import tyro
from loguru import logger
from tqdm import tqdm

from spotlight_tools.spotlight_pose2d.dataset import INPUT_SIZE
from spotlight_tools.spotlight_pose2d.io_utils import (
    check_output_path,
    load_pose_h5,
    load_skeleton_json,
    parse_trial_identity,
)
from spotlight_tools.spotlight_pose2d.model import RepVGGPoseModel
from spotlight_tools.spotlight_pose2d.viz import (
    build_edge_colors,
    build_node_colors,
    draw_pose,
)

HEATMAP_ALPHA = 0.6  # max blend strength, where the model is most confident
HEATMAP_COLORMAP = cmr.ghostlight


def load_model_input_frames(
    frame_cache_root: Path, genotype: str, fly_trial: str, frame_indices: list[int]
) -> np.ndarray:
    """This trial's cached `INPUT_SIZE`x`INPUT_SIZE` grayscale frames (see
    `cache_video_frames.py`), matching `predict_unlabeled_frames.py`'s own
    preprocessing exactly, so the heatmap reflects the model's real input.
    """
    frame_dir = (
        frame_cache_root / f"{genotype}__{fly_trial}" / f"{INPUT_SIZE}x{INPUT_SIZE}"
    )
    frames = []
    for idx in frame_indices:
        frame_path = frame_dir / f"frame_{idx:09d}.jpg"
        frame = cv2.imread(str(frame_path), cv2.IMREAD_UNCHANGED)
        if frame is None:
            raise SystemExit(
                f"Missing cached frame {frame_path}; run cache_all_trial_frames.py first."
            )
        frames.append(frame)
    return np.stack(frames)


def predict_combined_heatmaps(
    model: RepVGGPoseModel, device: str, frames: np.ndarray, batch_size: int
) -> np.ndarray:
    """The model's raw output heatmap for every frame, max-pooled across
    keypoints into one map per frame and clipped to `[0, 1]` (the model's
    own training target range; MSE regression can slightly over/undershoot
    it) -- no additional smoothing.

    Args:
        model: Trained `RepVGGPoseModel`, in eval mode.
        device: `"cuda"` or `"cpu"`.
        frames: `(n_frames, INPUT_SIZE, INPUT_SIZE)` uint8 grayscale.
        batch_size: Frames per inference batch.

    Returns:
        `(n_frames, heatmap_size, heatmap_size)` float32 in `[0, 1]`.
    """
    combined = []
    with torch.no_grad():
        for start in range(0, len(frames), batch_size):
            batch = frames[start : start + batch_size]
            tensor = (
                torch.from_numpy(np.repeat(batch[:, :, :, None], 3, axis=-1))
                .permute(0, 3, 1, 2)
                .float()
                .to(device)
                / 255.0
            )
            heatmaps = model(tensor).cpu().numpy()
            combined.append(np.clip(heatmaps.max(axis=1), 0, 1))
    return np.concatenate(combined)


def main(
    input_path: Path,
    checkpoint_path: Path,
    frame_cache_root: Path,
    skeleton_json_path: Path,
    output_path: Path,
    scale: float = 0.5,
    max_frames: int | None = None,
    n_keypoints: int = 37,
    batch_size: int = 128,
    crf: int = 23,
    override: bool = False,
) -> None:
    """Render an annotated pose video for one trial.

    Args:
        input_path: Dense pose `.h5` (see `save_pose_h5`), e.g. one of
            `run_inference_final.sh`'s `*_pose2d_final_predictions.h5`; its
            `poses` are drawn as the live skeleton.
        checkpoint_path: Trained `RepVGGPoseModel` state dict, re-run here
            to get its raw heatmap output for the overlay (not saved by the
            inference pipeline, which only keeps the extracted points).
        frame_cache_root: Root directory `cache_video_frames.py` wrote into;
            must already have this trial cached at `INPUT_SIZE`.
        skeleton_json_path: JSON with the real skeleton (nodes, edges), from
            `extract_metadata_from_initial_slp.py`. Its node names must
            match `input_path`'s own `node_names` exactly, order included.
        output_path: Where to save the rendered `.mp4`. Aborts if this
            already exists, unless `override` is set.
        scale: Output video size relative to the source (aligned, 900x900)
            video, e.g. 0.5 -> 450x450.
        max_frames: If set, only renders the first this-many frames (a quick
            spot check instead of the whole trial). Unset renders every frame.
        n_keypoints: Must match the checkpoint's `n_keypoints`.
        batch_size: Frames per inference batch.
        crf: H.264 quality, 0-51 (lower is higher quality, larger files);
            passed to `pvio.write_frames_to_video` as `quality`.
        override: If True, overwrite `output_path` if it already exists.
    """
    check_output_path(output_path, override)
    data = load_pose_h5(input_path)
    skeleton = load_skeleton_json(skeleton_json_path)
    node_names = [node.name for node in skeleton.nodes]
    if node_names != data["node_names"]:
        raise SystemExit(
            f"{skeleton_json_path}'s skeleton nodes {node_names} don't match "
            f"{input_path}'s {data['node_names']}."
        )
    name_to_idx = {name: i for i, name in enumerate(node_names)}
    edges = [
        (name_to_idx[edge.source.name], name_to_idx[edge.destination.name])
        for edge in skeleton.edges
    ]
    point_colors = build_node_colors(node_names)
    edge_colors = build_edge_colors(edges, node_names)

    n_frames = len(data["poses"])
    if max_frames is not None:
        n_frames = min(max_frames, n_frames)
    frame_indices = list(range(n_frames))
    points = data["poses"][:n_frames] * scale

    logger.info(f"Reading {n_frames} display frames from {data['video_path']}")
    frames, fps = pvio.read_frames_from_video(data["video_path"], frame_indices)
    fps = fps or 30.0

    frame_size = round(frames[0].shape[0] * scale)
    logger.info(f"Resizing display frames to {frame_size}x{frame_size} (scale={scale})")
    frames = [
        cv2.resize(frame, (frame_size, frame_size), interpolation=cv2.INTER_AREA)
        for frame in frames
    ]

    genotype, fly_trial = parse_trial_identity(data["video_path"])
    logger.info(
        f"Loading {n_frames} cached {INPUT_SIZE}x{INPUT_SIZE} model-input frames"
    )
    model_frames = load_model_input_frames(
        frame_cache_root, genotype, fly_trial, frame_indices
    )

    device = "cuda" if torch.cuda.is_available() else "cpu"
    model = RepVGGPoseModel(n_keypoints, pretrained_backbone=False).to(device)
    model.load_state_dict(torch.load(checkpoint_path, map_location=device))
    model.eval()

    logger.info(f"Running model over {n_frames} frames (batch_size={batch_size})")
    combined_heatmaps = predict_combined_heatmaps(
        model, device, model_frames, batch_size
    )

    is_tty = sys.stdout.isatty()
    log_every = max(1, int(0.1 * n_frames))
    iterator = (
        tqdm(range(n_frames), desc="Drawing poses", mininterval=1.0)
        if is_tty
        else range(n_frames)
    )
    for i in iterator:
        heatmap = cv2.resize(
            combined_heatmaps[i],
            (frame_size, frame_size),
            interpolation=cv2.INTER_LINEAR,
        )
        colored = (HEATMAP_COLORMAP(heatmap)[..., :3] * 255).astype(np.uint8)
        alpha = (heatmap * HEATMAP_ALPHA).astype(np.float32)[..., None]

        frame = frames[i].astype(np.float32)
        frame = frame * (1 - alpha) + colored * alpha
        frames[i] = frame.astype(np.uint8)
        draw_pose(frames[i], points[i], edges, edge_colors, point_colors)
        if not is_tty and i % log_every == 0:
            logger.info(f"Drew {i}/{n_frames} frames")

    logger.info(f"Encoding {n_frames} frames -> {output_path} (crf={crf}, mode=gpu)")
    pvio.write_frames_to_video(
        output_path, frames, fps, mode="gpu", quality=crf, log_interval=log_every
    )
    logger.info(f"Saved {output_path}")


if __name__ == "__main__":
    tyro.cli(main)
