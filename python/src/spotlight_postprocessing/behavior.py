"""Behavior frame processing: decode pseudo-BGR JPEGs, run the `TinyOrientModel`
(replacing the old SLEAP-based 3-keypoint aligner), align/crop each frame, and
-- in the same streaming pass, no video round-trip -- run the pose2d model on
the aligned crop and warp whichever muscle frames land in the current chunk.

See `spotlight_postprocessing.muscle`/`visualize` for the muscle and
QA-video stages that consume this module's outputs.
"""

import json
import logging
import time
from collections import defaultdict
from pathlib import Path

import cv2
import h5py
import numpy as np
import torch
from joblib import Parallel, delayed

from spotlight_postprocessing.common.video import StreamingVideoWriter
from spotlight_postprocessing.io import (
    MuscleH5Writer,
    check_output_path_against_alignment_flag,
)
from spotlight_postprocessing.muscle import (
    MuscleBehaviorMapping,
    warp_muscle_chunk,
)
from spotlight_postprocessing.spotlight_orient.dataset import (
    OUTPUT_SIZE as ORIENT_OUTPUT_SIZE,
)
from spotlight_postprocessing.spotlight_orient.dataset import (
    SCALE_FACTOR as ORIENT_SCALE_FACTOR,
)
from spotlight_postprocessing.spotlight_orient.model import TinyOrientModel
from spotlight_postprocessing.spotlight_pose2d.dataset import (
    INPUT_SIZE as POSE2D_INPUT_SIZE,
)
from spotlight_postprocessing.spotlight_pose2d.model import RepVGGPoseModel

# See `tools/scripts/spotlight_orient/visualize_predictions.py`'s own
# FLIP_DECISION_THRESHOLD -- the model's own sigmoid decision boundary,
# distinct from `flip_label.FLIPPED_THRESHOLD` (a *training*-label proxy).
FLIP_DECISION_THRESHOLD = 0.5

# "auto" batch size = this fraction of the active GPU's total VRAM (MiB).
# Hardcoded so a batch size tuned (via measured fp16 peak usage, see
# `PostprocessingParams`) on a 12 GB GPU scales automatically to bigger
# GPUs instead of leaving headroom unused there.
ORIENT_BATCH_SIZE_VRAM_FRACTION = 0.04
POSE2D_BATCH_SIZE_VRAM_FRACTION = 0.02
# No GPU to size a batch against; CPU inference isn't this pipeline's
# tuned/expected path, so this is just a modest, safe constant.
CPU_FALLBACK_BATCH_SIZE = 32


def _resolve_batch_size(
    batch_size: int | str, vram_fraction: float, device: str
) -> int:
    """`batch_size` as given, or (if `"auto"`) `vram_fraction` of the
    active GPU's total VRAM in MiB -- see `ORIENT_BATCH_SIZE_VRAM_
    FRACTION`/`POSE2D_BATCH_SIZE_VRAM_FRACTION`."""
    if batch_size != "auto":
        return int(batch_size)
    if device != "cuda":
        return CPU_FALLBACK_BATCH_SIZE
    vram_mib = torch.cuda.get_device_properties(0).total_memory / (1024**2)
    return int(vram_fraction * vram_mib)


def expand_single_pseudo_bgr_image(
    pseudo3ch_frame_path: Path,
) -> list[np.ndarray]:
    """Spotlight saves 3 adjacent behavior images as a single pseudo-BGR
    JPEG (an IO optimization at recording time). Expand it into 3 separate
    monochrome frames, in memory (no intermediate file)."""
    pseudo3ch_image = cv2.imread(str(pseudo3ch_frame_path))
    blue, green, red = cv2.split(pseudo3ch_image)
    return [blue, green, red]


def _expand_chunk(paths: list[Path], num_cpu_workers: int) -> list[np.ndarray]:
    parallel_mapper = Parallel(n_jobs=num_cpu_workers, backend="loky")
    grouped = parallel_mapper(delayed(expand_single_pseudo_bgr_image)(p) for p in paths)
    frames = []
    for group in grouped:
        frames.extend(group)
    return frames


def _load_orient_model(checkpoint_path: Path, device: str) -> TinyOrientModel:
    model = TinyOrientModel(n_keypoints=3, use_global_context=True).to(device)
    model.load_state_dict(torch.load(checkpoint_path, map_location=device))
    model.eval()
    return model


def _run_orient_batch(
    model: TinyOrientModel, frames: list[np.ndarray], device: str
) -> tuple[np.ndarray, np.ndarray]:
    """`frames`: list of `(H, W)` uint8 monochrome. Returns
    `(keypoints, flipped_prob)`: keypoints `(n, 3, 2)` in raw fullsize
    pixel space (matching `dataset.NATIVE_FRAME_SIZE`), flipped_prob `(n,)`."""
    width, height = ORIENT_OUTPUT_SIZE
    batch = np.stack([cv2.resize(f, (width, height)) for f in frames])
    batch = np.repeat(batch[:, :, :, None], 3, axis=-1)
    tensor = torch.from_numpy(batch).permute(0, 3, 1, 2).float().to(device) / 255.0

    with (
        torch.no_grad(),
        torch.autocast(
            device_type=device, dtype=torch.float16, enabled=device == "cuda"
        ),
    ):
        pred_keypoints, pred_flip_logit = model(tensor)

    raw_points = (
        pred_keypoints.float().cpu().numpy()
        * np.array(ORIENT_OUTPUT_SIZE, dtype=np.float32)
        * ORIENT_SCALE_FACTOR
    )
    flipped_prob = torch.sigmoid(pred_flip_logit).float().cpu().numpy()[:, 0]
    return raw_points, flipped_prob


def _load_pose2d_model(checkpoint_path: Path, n_keypoints: int, device: str):
    model = RepVGGPoseModel(n_keypoints, pretrained_backbone=False).to(device)
    model.load_state_dict(torch.load(checkpoint_path, map_location=device))
    model.eval()
    return model


def _heatmaps_to_points(heatmaps: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Per-keypoint argmax location + peak value, in heatmap pixel coords.
    `heatmaps`: `(n_nodes, H, W)`."""
    n_nodes, h, w = heatmaps.shape
    flat = heatmaps.reshape(n_nodes, -1)
    idx = flat.argmax(axis=1)
    scores = flat[np.arange(n_nodes), idx]
    ys, xs = np.unravel_index(idx, (h, w))
    return np.stack([xs, ys], axis=1).astype(np.float32), scores.astype(np.float32)


def _run_pose2d_batch(
    model, frames_bgr_or_gray: np.ndarray, device: str, batch_size: int
) -> tuple[np.ndarray, np.ndarray]:
    """`frames_bgr_or_gray`: `(n, H, W)` uint8 aligned crops (single channel).
    Returns `(poses, keypoint_scores)`: `(n, n_keypoints, 2)` in the aligned
    crop's own pixel space, `(n, n_keypoints)`."""
    n = len(frames_bgr_or_gray)
    all_poses, all_scores = [], []
    for start in range(0, n, batch_size):
        sub = frames_bgr_or_gray[start : start + batch_size]
        sub_rgb = np.repeat(sub[:, :, :, None], 3, axis=-1)
        resized = np.stack(
            [cv2.resize(f, (POSE2D_INPUT_SIZE, POSE2D_INPUT_SIZE)) for f in sub_rgb]
        )
        tensor = (
            torch.from_numpy(resized).permute(0, 3, 1, 2).float().to(device) / 255.0
        )
        with (
            torch.no_grad(),
            torch.autocast(
                device_type=device, dtype=torch.float16, enabled=device == "cuda"
            ),
        ):
            heatmaps = model(tensor).float().cpu().numpy()
        input_scale = POSE2D_INPUT_SIZE / sub.shape[2]  # sub width (crop_dim)
        heatmap_scale = POSE2D_INPUT_SIZE / heatmaps.shape[-1] / input_scale
        for h in heatmaps:
            points, scores = _heatmaps_to_points(h)
            all_poses.append(points * heatmap_scale)
            all_scores.append(scores)
    return np.stack(all_poses), np.stack(all_scores)


def transform_single_frame_to_align(
    input_frame: np.ndarray,
    keypoints: np.ndarray,
    crop_dim: int,
    thorax_idx: int,
    neck_idx: int,
    abdomen_idx: int,
    thorax_y_normalized: float = 0.5,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Rotate so the fly faces upward (head toward negative y) and crop
    around the thorax. `thorax_y_normalized` places the thorax at that
    fraction of the crop's height (0=top, 1=bottom, 0.5=old centered
    behavior); horizontal placement is always centered."""
    rotation_pivot = keypoints[thorax_idx, :]
    heading = keypoints[neck_idx, :] - keypoints[abdomen_idx, :]
    current_angle = np.rad2deg(np.arctan2(heading[1], heading[0]))
    target_angle = -90
    rotation_angle = target_angle - current_angle

    transform_matrix = cv2.getRotationMatrix2D(
        rotation_pivot, -rotation_angle, scale=1.0
    )
    translation_x = -rotation_pivot[0] + crop_dim / 2
    translation_y = -rotation_pivot[1] + crop_dim * thorax_y_normalized
    transform_matrix[0, 2] += translation_x
    transform_matrix[1, 2] += translation_y

    output_frame = cv2.warpAffine(
        input_frame, transform_matrix, (crop_dim, crop_dim),
        flags=cv2.INTER_NEAREST, borderMode=cv2.BORDER_CONSTANT, borderValue=0,
    )  # fmt: skip

    keypoints_homogeneous = np.hstack([keypoints, np.ones((keypoints.shape[0], 1))])
    transformed_keypoints = (transform_matrix @ keypoints_homogeneous.T).T
    return output_frame, transformed_keypoints, transform_matrix


def process_behavior_pipeline(
    *,
    raw_behavior_frame_paths: list[Path],
    orient_checkpoint_path: Path,
    alignment: str,  # "aligned" | "fullsize" | "both"
    thorax_y_normalized: float,
    crop_dim: int,
    output_aligned_video_path: Path | None,
    output_fullsize_video_path: Path | None,
    output_alignment_metadata_path: Path,
    behavior_video_fps: float,
    behavior_video_crf: int,
    behavior_video_preset: str,
    orient_batch_size: int | str,
    run_pose2d: bool,
    pose2d_checkpoint_path: Path | None,
    pose2d_skeleton_json_path: Path | None,
    pose2d_batch_size: int | str,
    output_pose2d_h5_path: Path | None,
    run_muscle: bool,
    muscle_mapping: MuscleBehaviorMapping | None,
    output_muscle_h5_path: Path | None,
    muscle_dataset_name: str | None,
    num_cpu_workers: int = -1,
) -> dict:
    """The single streaming pass: decode -> orient -> align -> (pose2d,
    muscle) -> write video(s), chunk by chunk. Model inference always runs
    on in-memory arrays, strictly before any frame is bound into a video
    (the aligned/fullsize videos and the muscle H5 are the only things
    encoded/written to disk here; pose2d never has to decode a video).

    Returns a dict with `flipped_prob` (`(n_frames,)`, for visualization),
    `n_frames`, and whichever output paths were actually produced.
    """
    logger = logging.getLogger(__name__)
    device = "cuda" if torch.cuda.is_available() else "cpu"
    orient_batch_size = _resolve_batch_size(
        orient_batch_size, ORIENT_BATCH_SIZE_VRAM_FRACTION, device
    )
    pose2d_batch_size = _resolve_batch_size(
        pose2d_batch_size, POSE2D_BATCH_SIZE_VRAM_FRACTION, device
    )
    # `num_cpu_workers` is constant for the rest of this call, so joblib's
    # own resolution of it (relevant mainly for -1 = "all cores") only
    # needs checking once here, not on every _expand_chunk/warp_muscle_
    # chunk call below.
    effective_cpu_workers = Parallel(
        n_jobs=num_cpu_workers, backend="loky"
    )._effective_n_jobs()
    logger.info(
        f"Orient/pose2d device: {device}, orient_batch_size={orient_batch_size}, "
        f"pose2d_batch_size={pose2d_batch_size}, "
        f"num_cpu_workers={num_cpu_workers} (effective: {effective_cpu_workers})"
    )  # fmt: skip

    if output_aligned_video_path is not None:
        check_output_path_against_alignment_flag(output_aligned_video_path, alignment)
    if output_fullsize_video_path is not None:
        check_output_path_against_alignment_flag(output_fullsize_video_path, alignment)

    orient_model = _load_orient_model(orient_checkpoint_path, device)
    write_aligned = alignment in ("aligned", "both")
    write_fullsize = alignment in ("fullsize", "both")

    aligned_writer = (
        StreamingVideoWriter(
            output_aligned_video_path, behavior_video_fps, behavior_video_crf,
            behavior_video_preset,
        )
        if write_aligned
        else None
    )  # fmt: skip
    fullsize_writer = (
        StreamingVideoWriter(
            output_fullsize_video_path, behavior_video_fps, behavior_video_crf,
            behavior_video_preset,
        )
        if write_fullsize
        else None
    )  # fmt: skip

    pose2d_model = None
    pose2d_node_names = None
    if run_pose2d:
        pose2d_node_names = json.loads(pose2d_skeleton_json_path.read_text())[
            "node_names"
        ]
        pose2d_model = _load_pose2d_model(
            pose2d_checkpoint_path, len(pose2d_node_names), device
        )

    muscle_writer = None
    if run_muscle:
        muscle_writer = MuscleH5Writer(
            output_muscle_h5_path, muscle_dataset_name, crop_dim, crop_dim
        )

    # Every pseudo-BGR file on disk packs 3 consecutive monochrome behavior
    # frames (a recording-time IO optimization); n_frames counts the real,
    # expanded frames, not raw files.
    n_frames = len(raw_behavior_frame_paths) * 3
    keypoints_xy_pre_alignment = np.full((n_frames, 3, 2), np.nan, dtype=np.float32)
    keypoints_xy_post_alignment = np.full((n_frames, 3, 2), np.nan, dtype=np.float32)
    transform_matrices = np.zeros((n_frames, 2, 3), dtype=np.float64)
    flipped_prob = np.full(n_frames, np.nan, dtype=np.float32)
    all_pose2d_poses = [] if run_pose2d else None
    all_pose2d_scores = [] if run_pose2d else None

    thorax_idx, neck_idx, abdomen_idx = (
        1,
        0,
        2,
    )  # COARSE_KEYPOINTS = [neck, thorax, abdomen]
    last_valid_keypoints = None

    # Granular per-phase timing (Task: "video binding is probably the
    # slowest part") -- accumulated across every chunk, logged as totals
    # once the streaming pass finishes, plus separately for the final
    # video-encode calls in `.close()` below.
    timers = defaultdict(float)

    def _tick():
        return time.perf_counter()

    # Chunk over raw (pseudo-BGR) files, sized so the expanded frame count
    # per chunk is close to orient_batch_size (each file expands to 3 frames).
    path_chunk_size = max(orient_batch_size // 3, 1)
    for path_start in range(0, len(raw_behavior_frame_paths), path_chunk_size):
        path_end = min(path_start + path_chunk_size, len(raw_behavior_frame_paths))
        chunk_start = path_start * 3
        chunk_end = min(path_end * 3, n_frames)
        chunk_paths = raw_behavior_frame_paths[path_start:path_end]

        t0 = _tick()
        chunk_frames = _expand_chunk(chunk_paths, num_cpu_workers)[
            : chunk_end - chunk_start
        ]
        timers["decode"] += _tick() - t0

        t0 = _tick()
        raw_points, flip_probs = _run_orient_batch(orient_model, chunk_frames, device)
        timers["orient_infer"] += _tick() - t0
        keypoints_xy_pre_alignment[chunk_start:chunk_end] = raw_points
        flipped_prob[chunk_start:chunk_end] = flip_probs

        aligned_batch = np.empty(
            (len(chunk_frames), crop_dim, crop_dim), dtype=np.uint8
        )
        fullsize_batch = (
            np.stack(chunk_frames).astype(np.uint8) if write_fullsize else None
        )
        t0 = _tick()
        for i, frame in enumerate(chunk_frames):
            keypoints = raw_points[i]
            if np.isnan(keypoints).any():
                # Forward-fill (matches the old whole-trial fill_gaps_in_2dpose_sequence
                # for interior gaps); a leading all-NaN run at the very start of the
                # trial is the one case that can't be forward-filled -- fall back to
                # this chunk's own first valid frame once one appears.
                keypoints = last_valid_keypoints
                if keypoints is None:
                    valid_in_chunk = [
                        p for p in raw_points[i:] if not np.isnan(p).any()
                    ]
                    if not valid_in_chunk:
                        raise RuntimeError(
                            "Orient model produced NaN keypoints for the entire "
                            "leading chunk; cannot align."
                        )
                    keypoints = valid_in_chunk[0]
            else:
                last_valid_keypoints = keypoints

            aligned_frame, aligned_kp, transform_matrix = transform_single_frame_to_align(
                frame, keypoints, crop_dim, thorax_idx, neck_idx, abdomen_idx,
                thorax_y_normalized,
            )  # fmt: skip
            aligned_batch[i] = aligned_frame
            keypoints_xy_post_alignment[chunk_start + i] = aligned_kp
            transform_matrices[chunk_start + i] = transform_matrix
        timers["warp_loop"] += _tick() - t0

        t0 = _tick()
        if aligned_writer is not None:
            aligned_writer.write_chunk(aligned_batch)
        if fullsize_writer is not None:
            fullsize_writer.write_chunk(fullsize_batch)
        timers["video_write_chunk"] += _tick() - t0

        if run_pose2d:
            t0 = _tick()
            poses, scores = _run_pose2d_batch(
                pose2d_model, aligned_batch, device, pose2d_batch_size
            )
            timers["pose2d_infer"] += _tick() - t0
            all_pose2d_poses.append(poses)
            all_pose2d_scores.append(scores)

        if run_muscle:
            t0 = _tick()
            transforms_by_frame = {
                chunk_start + i: transform_matrices[chunk_start + i]
                for i in range(len(chunk_frames))
            }
            muscle_frames = warp_muscle_chunk(
                muscle_mapping,
                chunk_start,
                chunk_end,
                transforms_by_frame,
                num_cpu_workers,
            )
            timers["muscle_warp"] += _tick() - t0
            # Separate from the warp above -- profiling found the HDF5 gzip
            # write, not the cv2 warp, was the actual dominant cost here
            # (see MuscleH5Writer's docstring).
            t0 = _tick()
            muscle_writer.append(muscle_frames)
            timers["muscle_h5_write"] += _tick() - t0

        logger.info(f"Processed {chunk_end}/{n_frames} behavior frames")

    logger.info(
        "Streaming pass phase totals: "
        + ", ".join(f"{k}={v:.1f}s" for k, v in timers.items())
    )

    t0 = _tick()
    if aligned_writer is not None:
        aligned_writer.close()
    timers["aligned_video_encode"] = _tick() - t0
    logger.info(
        f"Aligned video encode (writer.close()): {timers['aligned_video_encode']:.1f}s"
    )
    if fullsize_writer is not None:
        t0 = _tick()
        fullsize_writer.close()
        timers["fullsize_video_encode"] = _tick() - t0
        logger.info(
            f"Fullsize video encode (writer.close()): {timers['fullsize_video_encode']:.1f}s"
        )
    if muscle_writer is not None:
        t0 = _tick()
        muscle_writer.close()
        timers["muscle_h5_close"] = _tick() - t0
        logger.info(f"Muscle H5 close/flush: {timers['muscle_h5_close']:.1f}s")

    output_alignment_metadata_path.parent.mkdir(parents=True, exist_ok=True)
    with h5py.File(output_alignment_metadata_path, "w") as f:
        f.create_dataset(
            "keypoints_xy_pre_alignment", data=keypoints_xy_pre_alignment,
            compression="gzip", dtype="float32",
        )  # fmt: skip
        f.create_dataset(
            "keypoints_xy_post_alignment", data=keypoints_xy_post_alignment,
            compression="gzip",
        )  # fmt: skip
        f.create_dataset(
            "transform_matrices", data=transform_matrices, compression="gzip"
        )
        f.create_dataset("flipped_prob", data=flipped_prob, compression="gzip")
        f.attrs["output_dim"] = [crop_dim, crop_dim]
        f.attrs["keypoint_names"] = ["neck", "thorax", "abdomen"]

    result = {
        "n_frames": n_frames,
        "flipped_prob": flipped_prob,
        "aligned_video_path": output_aligned_video_path if write_aligned else None,
        "fullsize_video_path": output_fullsize_video_path if write_fullsize else None,
        "alignment_metadata_path": output_alignment_metadata_path,
        "timers": dict(timers),
    }

    if run_pose2d:
        poses = np.concatenate(all_pose2d_poses)
        keypoint_scores = np.concatenate(all_pose2d_scores)
        instance_score = np.nanmean(keypoint_scores, axis=-1)
        from spotlight_postprocessing.spotlight_pose2d.io_utils import save_pose_h5

        save_pose_h5(
            output_pose2d_h5_path, poses, keypoint_scores, instance_score,
            np.zeros(n_frames, dtype=bool), pose2d_node_names,
            video_path=output_aligned_video_path,
        )  # fmt: skip
        result["pose2d_h5_path"] = output_pose2d_h5_path
        result["pose2d_node_names"] = pose2d_node_names

    return result
