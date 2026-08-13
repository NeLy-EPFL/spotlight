#!/usr/bin/env python
"""Renders a quick annotated video of one trial's `TinyLocalizationModel`
predictions (see `infer.py`): the model's own raw per-keypoint heatmaps
(re-run live from cached model-input frames, since `infer.py` only saves
the extracted points, not the heatmaps themselves), turned into proper
spatial probabilities and max-pooled into one combined map per frame, same
convention as `spotlight_pose2d.visualize_predictions`'s overlay:
overlaid live, then the predicted head/thorax/abdomen as small colored
dots, plus the RepVGG-A0 pipeline's own 900x900 aligned-domain box
reconstructed on the raw frame (see `box.py`), colored yellow when upright
and gray when flipped. cv2-only drawing; video I/O via `pvio`, GPU (NVENC)
encoded.

Reads the source video in chunks (`READ_CHUNK_SIZE` frames at a time,
resizing each chunk immediately and discarding the native-resolution copy)
rather than all at once: at this model's native fullsize resolution
(dataset.NATIVE_FRAME_SIZE, much bigger than spotlight_pose2d's 900x900
aligned domain), reading an entire ~20k-frame trial natively before
resizing would need on the order of 150-200GB of RAM. The final resized
frame list still has to fit in memory before `pvio.write_frames_to_video`
(which takes a plain list, not a stream): at `scale=0.5` that's still
roughly 45GB for a full ~20k-frame trial, so `max_frames` is worth using
for a full-trial run on a memory-constrained machine.

Usage:
    python scripts/postprocessing/model_training/localization/visualize_predictions.py \\
        --input-path bulk_data/.../localization_model/predictions/<trial>_localization_predictions.h5 \\
        --checkpoint-path bulk_data/.../localization_model/checkpoints/v1/best.pt \\
        --canonical-h5-paths bulk_data/.../final_predictions/*.h5 \\
        --frame-cache-root bulk_data/motion_prior/localization_model/frame_cache \\
        --output-path <trial>_localization_predictions.mp4 \\
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
from infer import cached_frame_paths, load_batch
from loguru import logger
from tqdm import tqdm

from spotlight.postprocessing.localization.box import (
    measure_canonical_aligned_points,
    raw_domain_box_corners,
)
from spotlight.postprocessing.localization.constants import (
    COARSE_KEYPOINTS,
    OUTPUT_SIZE,
    RAW_TO_ALIGNED_SCALE,
)
from spotlight.postprocessing.localization.io_utils import (
    load_localization_predictions_h5,
)
from spotlight.postprocessing.localization.model import (
    TinyLocalizationModel,
    heatmap_probs,
)
from spotlight.postprocessing.pose2d.io_utils import (
    check_output_path,
    parse_trial_identity,
)

# RGB, since `pvio` reads/writes frames in RGB (unlike OpenCV's usual BGR).
POINT_COLORS = {
    "head": (52, 152, 219),
    "neck": (155, 89, 182),
    "thorax": (46, 204, 113),
    "abdomen": (230, 126, 34),
}
POINT_RADIUS = 4
BOX_THICKNESS = 2
BOX_COLOR_UPRIGHT = (255, 255, 0)  # yellow
BOX_COLOR_FLIPPED = (160, 160, 160)  # gray
# The model's own sigmoid decision boundary: unrelated to
# flip_label.FLIPPED_THRESHOLD, which derives *training* labels from a
# different model's (RepVGG-A0's) confidence, not this model's own output.
FLIP_DECISION_THRESHOLD = 0.5
READ_CHUNK_SIZE = 1000  # bounds native-resolution frames held in memory at once
HEATMAP_ALPHA = 0.6  # max blend strength, where the model is most confident
HEATMAP_COLORMAP = cmr.ghostlight  # matches spotlight_pose2d's own overlay
HEATMAP_BATCH_SIZE = 256


def predict_combined_heatmaps(
    model: TinyLocalizationModel,
    device: str,
    frame_dir: Path,
    frame_indices: list[int],
    batch_size: int,
) -> dict[int, np.ndarray]:
    """Each of `frame_indices`'s combined per-frame heatmap: `TinyLocalizationModel`'s
    raw per-keypoint heatmaps turned into proper spatial probabilities
    (`model.heatmap_probs`), each keypoint's own map rescaled by its own
    peak, then max-pooled across keypoints into one map: the same
    combined-heatmap convention `spotlight_pose2d.visualize_predictions`
    uses for its own overlay.

    The per-keypoint rescaling matters here in a way it doesn't for
    `spotlight_pose2d`'s heatmap head: this model's heatmap is a spatial
    softmax, so once a keypoint is well localized, its probability mass
    concentrates onto one or two pixels out of the whole (coarse,
    ~124x92) heatmap, leaving a raw peak value too small to render after
    upsampling to display size. Rescaling by each keypoint's own peak
    keeps the overlay visible regardless of how confident the model is,
    while still showing *peakiness* honestly: a diffuse heatmap rescales
    into a wide blob (many pixels were already close to its own max), a
    peaked one into a small dot (only its true peak was).

    Re-runs the model rather than reading anything from `infer.py`'s own
    output, since that only saves the extracted `(x, y)` points, not the
    heatmaps they came from.

    Args:
        model: Trained `TinyLocalizationModel`, in eval mode.
        device: `"cuda"` or `"cpu"`.
        frame_dir: This trial's cached `OUTPUT_SIZE` frame directory (see
            `cache_fullsize_frames.py`).
        frame_indices: Raw video frame indices to predict.
        batch_size: Frames per inference batch.

    Returns:
        `{frame_idx: (heatmap_height, heatmap_width) float32 array}`, only
        for frame indices with a cached input frame: exactly the frames
        `infer.py` itself could predict (others are already reported via
        `main`'s own NaN-keypoint check).
    """
    idx_to_path = dict(cached_frame_paths(frame_dir))
    available = [idx for idx in frame_indices if idx in idx_to_path]
    combined = {}
    with torch.no_grad():
        for start in range(0, len(available), batch_size):
            batch_indices = available[start : start + batch_size]
            batch_images = load_batch([idx_to_path[idx] for idx in batch_indices])
            batch_tensor = (
                torch.from_numpy(batch_images).permute(0, 3, 1, 2).float().to(device)
                / 255.0
            )
            with torch.autocast(
                device_type=device, dtype=torch.float16, enabled=device == "cuda"
            ):
                _, _, heatmaps = model(batch_tensor, return_heatmaps=True)
            probs = heatmap_probs(heatmaps).float().cpu().numpy()
            peaks = probs.max(axis=(2, 3), keepdims=True)
            batch_combined = (probs / peaks).max(axis=1)
            for idx, single in zip(batch_indices, batch_combined, strict=True):
                combined[idx] = single
    return combined


def overlay_heatmap(frame: np.ndarray, heatmap: np.ndarray) -> None:
    """Alpha-blends `heatmap` (this frame's combined keypoint-attention map,
    already rescaled to `[0, 1]` by `predict_combined_heatmaps`) onto
    `frame`, in place: upsampled to `frame`'s own size and colored via
    `HEATMAP_COLORMAP`, alpha scaling with the map's own value so confident
    regions are visibly tinted and everything else stays close to the raw
    frame.
    """
    resized = cv2.resize(
        heatmap, (frame.shape[1], frame.shape[0]), interpolation=cv2.INTER_LINEAR
    )
    colored = (HEATMAP_COLORMAP(resized)[..., :3] * 255).astype(np.float32)
    alpha = (resized * HEATMAP_ALPHA).astype(np.float32)[..., None]
    frame[:] = (frame.astype(np.float32) * (1 - alpha) + colored * alpha).astype(
        np.uint8
    )


def draw_predictions(
    frame: np.ndarray,
    points: np.ndarray,
    keypoint_names: list[str],
    flipped_prob: float,
    canonical_points: np.ndarray,
    box_scale: float,
    display_scale: float,
    heatmap: np.ndarray | None,
) -> None:
    """Draws one frame's heatmap overlay, reconstructed aligned-domain box,
    and keypoint dots onto `frame`, in place.

    Args:
        keypoint_names: `points`' own names, in order (see
            `dataset.KeypointSpec`): each must have a `POINT_COLORS` entry.
        box_scale: Raw-to-aligned scale (see `box.RAW_TO_ALIGNED_SCALE`):
            a fixed physical constant, keeps the reconstructed box's size
            constant across frames.
        display_scale: This video's own display resize factor (`main`'s
            `scale`): unrelated to `box_scale`, just converts raw-domain
            pixel coordinates to this particular output video's resolution.
        heatmap: This frame's combined heatmap (see
            `predict_combined_heatmaps`), or None to skip the overlay
            (e.g. no cached model-input frame for this one).
    """
    if heatmap is not None:
        overlay_heatmap(frame, heatmap)
    corners = raw_domain_box_corners(points, canonical_points, box_scale)
    if corners is not None:
        color = (
            BOX_COLOR_FLIPPED
            if flipped_prob >= FLIP_DECISION_THRESHOLD
            else BOX_COLOR_UPRIGHT
        )
        cv2.polylines(
            frame,
            [np.round(corners * display_scale).astype(np.int32)],
            isClosed=True,
            color=color,
            thickness=BOX_THICKNESS,
            lineType=cv2.LINE_AA,
        )
    for name, point in zip(keypoint_names, points):
        cv2.circle(
            frame,
            tuple(np.round(point * display_scale).astype(int)),
            POINT_RADIUS,
            POINT_COLORS[name],
            -1,
            cv2.LINE_AA,
        )


def main(
    input_path: Path,
    checkpoint_path: Path,
    canonical_h5_paths: list[Path],
    frame_cache_root: Path,
    output_path: Path,
    use_global_context: bool = True,
    scale: float = 0.5,
    max_frames: int | None = None,
    heatmap_batch_size: int = HEATMAP_BATCH_SIZE,
    crf: int = 23,
    override: bool = False,
) -> None:
    """Render an annotated localization video for one trial.

    Args:
        input_path: `TinyLocalizationModel` predictions (see
            `infer.py`/`io_utils.save_localization_predictions_h5`).
        checkpoint_path: Trained `TinyLocalizationModel` state dict, re-run here
            to get its raw heatmap output for the overlay (not saved by
            `infer.py`, which only keeps the extracted points).
        canonical_h5_paths: pose2d `final_predictions.h5` files to measure
            the canonical aligned-domain coarse-keypoint position from
            (see `box.measure_canonical_aligned_points`): typically every
            trial's, not just this one's.
        frame_cache_root: Root directory
            `scripts/postprocessing/model_training/localization/cache_fullsize_frames.py` wrote
            into; must already have this trial cached at `OUTPUT_SIZE`.
        output_path: Where to save the rendered `.mp4`. Aborts if this
            already exists, unless `override` is set.
        use_global_context: Must match what `checkpoint_path` was actually
            trained with: False for v1-v5 checkpoints, True from v6 on
            (see `model.GlobalContextBlock`/`train.py`'s own flag).
        scale: Output video size relative to the raw fullsize source video
            (`dataset.NATIVE_FRAME_SIZE`), e.g. 0.5 -> half resolution.
        max_frames: If set, only renders the first this-many frames (a
            quick spot check instead of the whole trial). Unset renders
            every frame.
        heatmap_batch_size: Frames per heatmap inference batch.
        crf: H.264 quality, 0-51 (lower is higher quality, larger files);
            passed to `pvio.write_frames_to_video` as `quality`.
        override: If True, overwrite `output_path` if it already exists.
    """
    check_output_path(output_path, override)
    keypoint_names = COARSE_KEYPOINTS
    data = load_localization_predictions_h5(input_path)
    if data["node_names"] != keypoint_names:
        raise SystemExit(
            f"{input_path}'s node_names {data['node_names']} != "
            f"expected {keypoint_names}"
        )

    logger.info(
        f"Measuring canonical aligned-domain position from "
        f"{len(canonical_h5_paths)} trial(s)"
    )
    canonical_points = measure_canonical_aligned_points(canonical_h5_paths)
    logger.info(f"Canonical {keypoint_names}: {canonical_points.tolist()}")
    box_scale = RAW_TO_ALIGNED_SCALE
    logger.info(
        f"Raw-to-aligned box scale: {box_scale} (fixed physical constant, not fit; "
        "see box.py's docstring)"
    )

    n_frames = len(data["keypoints"])
    if max_frames is not None:
        n_frames = min(max_frames, n_frames)
    frame_indices = list(range(n_frames))
    keypoints = data["keypoints"][:n_frames]
    flipped_prob = data["flipped_prob"][:n_frames]

    device = "cuda" if torch.cuda.is_available() else "cpu"
    model = TinyLocalizationModel(
        n_keypoints=len(keypoint_names), use_global_context=use_global_context
    ).to(device)
    model.load_state_dict(torch.load(checkpoint_path, map_location=device))
    model.eval()
    genotype, fly_trial = parse_trial_identity(data["video_path"])
    width, height = OUTPUT_SIZE
    frame_dir = frame_cache_root / f"{genotype}__{fly_trial}" / f"{width}x{height}"
    logger.info(f"Running model over {n_frames} frames for the heatmap overlay")
    combined_heatmaps = predict_combined_heatmaps(
        model, device, frame_dir, frame_indices, heatmap_batch_size
    )

    logger.info(
        f"Reading {n_frames} display frames from {data['video_path']} "
        f"({READ_CHUNK_SIZE} at a time, resized immediately)"
    )
    frames: list[np.ndarray] = []
    frame_height = frame_width = None
    fps = None
    for chunk_start in range(0, n_frames, READ_CHUNK_SIZE):
        chunk_indices = frame_indices[chunk_start : chunk_start + READ_CHUNK_SIZE]
        chunk_frames, chunk_fps = pvio.read_frames_from_video(
            data["video_path"], chunk_indices
        )
        if frame_height is None:
            fps = chunk_fps or 30.0
            frame_height = round(chunk_frames[0].shape[0] * scale)
            frame_width = round(chunk_frames[0].shape[1] * scale)
            logger.info(
                f"Resizing display frames to {frame_width}x{frame_height} (scale={scale})"
            )
        frames.extend(
            cv2.resize(frame, (frame_width, frame_height), interpolation=cv2.INTER_AREA)
            for frame in chunk_frames
        )
        logger.info(f"Read {len(frames)}/{n_frames} frames")

    is_tty = sys.stdout.isatty()
    log_every = max(1, int(0.1 * n_frames))
    iterator = (
        tqdm(range(n_frames), desc="Drawing", mininterval=1.0)
        if is_tty
        else range(n_frames)
    )
    n_missing = 0
    for i in iterator:
        if np.isnan(keypoints[i]).any():
            n_missing += 1
        else:
            draw_predictions(
                frames[i],
                keypoints[i],
                keypoint_names,
                flipped_prob[i],
                canonical_points,
                box_scale,
                scale,
                combined_heatmaps.get(i),
            )
        if not is_tty and i % log_every == 0:
            logger.info(f"Drew {i}/{n_frames} frames")
    if n_missing:
        logger.warning(
            f"{n_missing}/{n_frames} frame(s) had no prediction, left undrawn"
        )

    logger.info(f"Encoding {n_frames} frames -> {output_path} (crf={crf}, mode=gpu)")
    pvio.write_frames_to_video(
        output_path, frames, fps, mode="gpu", quality=crf, log_interval=log_every
    )
    logger.info(f"Saved {output_path}")


if __name__ == "__main__":
    tyro.cli(main)
