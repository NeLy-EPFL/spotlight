"""Behavior frame processing: decode pseudo-BGR JPEGs, run the localization
model (replacing the old SLEAP-based 3-keypoint aligner), align/crop each
frame, and (in the same streaming pass, no video round-trip) run the
pose2d model on the aligned crop and warp whichever muscle frames land in
the current chunk. Both models load as self-contained fp16 TorchScript
exports (`torch.jit.load`, see `_load_torchscript_model`), not their own
Python model classes plus a state-dict checkpoint.

See `spotlight.postprocessing.muscle`/`visualize` for the muscle and
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
from scipy.ndimage import gaussian_filter1d

from spotlight.postprocessing.common.video import StreamingVideoWriter
from spotlight.postprocessing.io import (
    MuscleH5Writer,
    check_output_path_against_alignment_flag,
)
from spotlight.postprocessing.muscle import (
    MuscleBehaviorMapping,
    warp_muscle_chunk,
)
from spotlight.postprocessing.localization.constants import (
    OUTPUT_SIZE as LOCALIZATION_OUTPUT_SIZE,
)
from spotlight.postprocessing.localization.constants import (
    SCALE_FACTOR as LOCALIZATION_SCALE_FACTOR,
)
from spotlight.postprocessing.pose2d.constants import (
    INPUT_SIZE as POSE2D_INPUT_SIZE,
)

# See `scripts/postprocessing/model_training/localization/visualize_predictions.py`'s own
# FLIP_DECISION_THRESHOLD: the model's own sigmoid decision boundary,
# distinct from `flip_label.FLIPPED_THRESHOLD` (a *training*-label proxy).
FLIP_DECISION_THRESHOLD = 0.5

# "auto" batch size = this fraction of the active GPU's total VRAM (MiB).
# Hardcoded so a batch size tuned (via measured fp16 peak usage, see
# `PostprocessingParams`) on a 12 GB GPU scales automatically to bigger
# GPUs instead of leaving headroom unused there.
LOCALIZATION_BATCH_SIZE_VRAM_FRACTION = 0.04
POSE2D_BATCH_SIZE_VRAM_FRACTION = 0.02
# No GPU to size a batch against; CPU inference isn't this pipeline's
# tuned/expected path, so this is just a modest, safe constant.
CPU_FALLBACK_BATCH_SIZE = 32


def _resolve_batch_size(
    batch_size: int | str, vram_fraction: float, device: str
) -> int:
    """`batch_size` as given, or (if `"auto"`) `vram_fraction` of the
    active GPU's total VRAM in MiB (see `LOCALIZATION_BATCH_SIZE_VRAM_
    FRACTION`/`POSE2D_BATCH_SIZE_VRAM_FRACTION`)."""
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


def _load_torchscript_model(
    checkpoint_path: Path, device: str
) -> torch.jit.ScriptModule:
    """Loads an fp16 TorchScript export directly (see `common.export.
    export_onnx_and_torchscript`). It's self-contained, no model class needed
    here at all, unlike a plain state-dict checkpoint. Downgraded to fp32
    on CPU, since fp16 CPU inference is unsupported/slow for many ops."""
    # `map_location` alone doesn't reliably relocate every tensor a traced
    # module carries (e.g. a coordinate grid computed once and traced in as
    # a constant, rather than a registered buffer); an explicit `.to()`
    # does, since it recurses over the whole module.
    model = torch.jit.load(checkpoint_path, map_location=device).to(device)
    if device == "cpu":
        model = model.float()
    model.eval()
    return model


def _run_localization_batch(
    model: torch.jit.ScriptModule, frames: list[np.ndarray], device: str
) -> tuple[np.ndarray, np.ndarray]:
    """`frames`: list of `(H, W)` uint8 monochrome. Returns
    `(keypoints, flipped_prob)`: keypoints `(n, 3, 2)` in raw fullsize
    pixel space (matching `dataset.NATIVE_FRAME_SIZE`), flipped_prob `(n,)`."""
    width, height = LOCALIZATION_OUTPUT_SIZE
    batch = np.stack([cv2.resize(f, (width, height)) for f in frames])
    batch = np.repeat(batch[:, :, :, None], 3, axis=-1)
    # Matches the model's own weight dtype (see `_load_torchscript_model`):
    # fp16 on GPU (what it was exported as), fp32 on CPU.
    dtype = torch.float16 if device == "cuda" else torch.float32
    tensor = torch.from_numpy(batch).permute(0, 3, 1, 2).to(device=device, dtype=dtype)
    tensor = tensor / 255.0

    with torch.no_grad():
        pred_keypoints, pred_flip_logit = model(tensor)

    raw_points = (
        pred_keypoints.float().cpu().numpy()
        * np.array(LOCALIZATION_OUTPUT_SIZE, dtype=np.float32)
        * LOCALIZATION_SCALE_FACTOR
    )
    flipped_prob = torch.sigmoid(pred_flip_logit).float().cpu().numpy()[:, 0]
    return raw_points, flipped_prob


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
    # Matches the model's own weight dtype (see `_load_torchscript_model`):
    # fp16 on GPU (what it was exported as), fp32 on CPU.
    dtype = torch.float16 if device == "cuda" else torch.float32
    all_poses, all_scores = [], []
    for start in range(0, n, batch_size):
        sub = frames_bgr_or_gray[start : start + batch_size]
        sub_rgb = np.repeat(sub[:, :, :, None], 3, axis=-1)
        resized = np.stack(
            [cv2.resize(f, (POSE2D_INPUT_SIZE, POSE2D_INPUT_SIZE)) for f in sub_rgb]
        )
        tensor = (
            torch.from_numpy(resized).permute(0, 3, 1, 2).to(device=device, dtype=dtype)
        )
        tensor = tensor / 255.0
        with torch.no_grad():
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
    position: np.ndarray | None = None,
    heading: np.ndarray | None = None,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Rotate so the fly faces upward (head toward negative y) and crop
    around the thorax. `thorax_y_normalized` places the thorax at that
    fraction of the crop's height (0=top, 1=bottom, 0.5=old centered
    behavior); horizontal placement is always centered.

    `position`/`heading` (both `(2,)`) override this frame's own raw thorax
    position / neck-abdomen heading vector for the rotation itself: the
    streaming pass passes in time-smoothed values here (see
    `_smooth_position_and_heading`), while `keypoints` (used below to build
    `transformed_keypoints`) always stays this exact frame's own raw fit,
    smoothed or not. `None` (the default) uses this frame's own raw values
    for both, unchanged from the pre-smoothing behavior.
    """
    rotation_pivot = keypoints[thorax_idx, :] if position is None else position
    if heading is None:
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


def _fill_nan_keypoints(
    window: np.ndarray, seed: np.ndarray | None
) -> tuple[np.ndarray, np.ndarray | None]:
    """Forward-fills any all-NaN frame in `window` (`(n, 3, 2)`) from the
    previous valid frame, seeded by `seed` (the last valid frame carried
    over from the previous chunk, or `None` only at the very start of the
    trial). A leading run with no valid predecessor at all (only possible
    when `seed` is `None`) is filled from `window`'s own first valid frame
    instead: the same fallback the old per-frame-only version of this fill
    used, just applied over this whole (possibly padded) window at once.

    Returns:
        `(filled, new_seed)`: `new_seed` is the last valid (pre-fill) frame
        in `window`, to seed the next chunk's call.
    """
    filled = window.copy()
    valid = ~np.isnan(window).any(axis=(1, 2))
    if not valid.any():
        if seed is None:
            raise RuntimeError(
                "Localization model produced NaN keypoints for the entire "
                "leading window; cannot align."
            )
        filled[:] = seed
        return filled, seed

    last = seed
    first_valid = window[np.flatnonzero(valid)[0]]
    for i in range(len(filled)):
        if valid[i]:
            last = filled[i]
        elif last is not None:
            filled[i] = last
        else:
            filled[i] = first_valid
    return filled, last


def _smooth_position_and_heading(
    filled_keypoints: np.ndarray,
    thorax_idx: int,
    neck_idx: int,
    abdomen_idx: int,
    position_denoise_sigma: float,
    heading_denoise_sigma: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Time-smoothed thorax position and neck-abdomen heading vector, over
    `filled_keypoints`'s (NaN-free, see `_fill_nan_keypoints`) frame axis.

    Smooths the heading VECTOR's x/y components (not the angle
    `transform_single_frame_to_align` derives from it), which sidesteps the
    wraparound a naive angle average would hit near +/-180 degrees:
    `transform_single_frame_to_align` only ever reads this vector's angle,
    so no renormalization is needed afterward. `*_denoise_sigma == -1`
    disables smoothing on that one axis (position or heading independently).

    Args:
        filled_keypoints: `(n, 3, 2)`, NaN-free.
        position_denoise_sigma / heading_denoise_sigma: Gaussian sigma, in
            frames, or -1 to disable.

    Returns:
        `(position, heading)`, both `(n, 2)` float64.
    """
    position = filled_keypoints[:, thorax_idx, :].astype(np.float64)
    heading = (
        filled_keypoints[:, neck_idx, :] - filled_keypoints[:, abdomen_idx, :]
    ).astype(np.float64)
    if position_denoise_sigma != -1:
        position = np.stack(
            [gaussian_filter1d(position[:, a], sigma=position_denoise_sigma) for a in range(2)],
            axis=1,
        )  # fmt: skip
    if heading_denoise_sigma != -1:
        heading = np.stack(
            [gaussian_filter1d(heading[:, a], sigma=heading_denoise_sigma) for a in range(2)],
            axis=1,
        )  # fmt: skip
    return position, heading


def process_behavior_pipeline(
    *,
    raw_behavior_frame_paths: list[Path],
    localization_checkpoint_path: Path,
    alignment: str,  # "aligned" | "fullsize" | "both"
    thorax_y_normalized: float,
    crop_dim: int,
    output_aligned_video_path: Path | None,
    output_fullsize_video_path: Path | None,
    output_alignment_metadata_path: Path,
    behavior_video_fps: float,
    behavior_video_crf: int,
    behavior_video_preset: str,
    localization_batch_size: int | str,
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
    position_denoise_sigma: float = -1,
    heading_denoise_sigma: float = -1,
    encode_mode: str = "auto",
) -> dict:
    """The single streaming pass: decode -> localize -> align -> (pose2d,
    muscle) -> write video(s), chunk by chunk. Model inference always runs
    on in-memory arrays, strictly before any frame is bound into a video
    (the aligned/fullsize videos and the muscle H5 are the only things
    encoded/written to disk here; pose2d never has to decode a video).

    `position_denoise_sigma`/`heading_denoise_sigma` (frames, `-1` disables)
    Gaussian-smooth the thorax position / neck-abdomen heading actually used
    for each frame's crop (see `_smooth_position_and_heading`). This is a
    real change to the alignment itself, not a display-only effect: a small
    lookahead (see `pad_frames` below) is decoded and localized past each
    chunk's own end purely to give that smoothing real context at the chunk
    boundary, then discarded.

    Returns a dict with `flipped_prob` (`(n_frames,)`, for visualization),
    `n_frames`, and whichever output paths were actually produced.
    """
    logger = logging.getLogger(__name__)
    device = "cuda" if torch.cuda.is_available() else "cpu"
    localization_batch_size = _resolve_batch_size(
        localization_batch_size, LOCALIZATION_BATCH_SIZE_VRAM_FRACTION, device
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
        f"Localization/pose2d device: {device}, "
        f"localization_batch_size={localization_batch_size}, "
        f"pose2d_batch_size={pose2d_batch_size}, "
        f"num_cpu_workers={num_cpu_workers} (effective: {effective_cpu_workers})"
    )  # fmt: skip

    if output_aligned_video_path is not None:
        check_output_path_against_alignment_flag(output_aligned_video_path, alignment)
    if output_fullsize_video_path is not None:
        check_output_path_against_alignment_flag(output_fullsize_video_path, alignment)

    localization_model = _load_torchscript_model(localization_checkpoint_path, device)
    write_aligned = alignment in ("aligned", "both")
    write_fullsize = alignment in ("fullsize", "both")

    aligned_writer = (
        StreamingVideoWriter(
            output_aligned_video_path, behavior_video_fps, behavior_video_crf,
            behavior_video_preset, mode=encode_mode,
        )
        if write_aligned
        else None
    )  # fmt: skip
    fullsize_writer = (
        StreamingVideoWriter(
            output_fullsize_video_path, behavior_video_fps, behavior_video_crf,
            behavior_video_preset, mode=encode_mode,
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
        pose2d_model = _load_torchscript_model(pose2d_checkpoint_path, device)

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
    # slowest part"): accumulated across every chunk, logged as totals
    # once the streaming pass finishes, plus separately for the final
    # video-encode calls in `.close()` below.
    timers = defaultdict(float)

    def _tick():
        return time.perf_counter()

    # Padding (raw files) purely to give position/heading smoothing real
    # context at each chunk's boundary (see the docstring above). `+1`
    # keeps a whole-frame margin against `2 * sigma`'s own rounding.
    sigmas = [s for s in (position_denoise_sigma, heading_denoise_sigma) if s != -1]
    pad_frames = int(np.ceil(2 * max(sigmas))) + 1 if sigmas else 0
    pad_files = -(-pad_frames // 3)  # ceil division

    # Chunk over raw (pseudo-BGR) files, sized so the expanded frame count
    # per chunk is close to localization_batch_size (each file expands to 3 frames).
    path_chunk_size = max(localization_batch_size // 3, 1)
    for path_start in range(0, len(raw_behavior_frame_paths), path_chunk_size):
        path_end = min(path_start + path_chunk_size, len(raw_behavior_frame_paths))
        chunk_start = path_start * 3
        chunk_end = min(path_end * 3, n_frames)
        chunk_paths = raw_behavior_frame_paths[path_start:path_end]
        # Lookahead-only: extra raw files past this chunk's own end, decoded
        # and localized purely for the smoothing window's right edge below,
        # never written to any output. The left edge instead reuses this
        # trial's own already-computed history (free, no extra decode),
        # see `pad_pre`.
        lookahead_paths = raw_behavior_frame_paths[path_end : path_end + pad_files]

        t0 = _tick()
        chunk_frames = _expand_chunk(chunk_paths, num_cpu_workers)[
            : chunk_end - chunk_start
        ]
        lookahead_frames = (
            _expand_chunk(lookahead_paths, num_cpu_workers) if lookahead_paths else []
        )
        timers["decode"] += _tick() - t0

        t0 = _tick()
        raw_points, flip_probs = _run_localization_batch(
            localization_model, chunk_frames, device
        )
        lookahead_points = (
            _run_localization_batch(localization_model, lookahead_frames, device)[0]
            if lookahead_frames
            else np.empty((0, 3, 2), dtype=np.float32)
        )
        timers["localization_infer"] += _tick() - t0
        keypoints_xy_pre_alignment[chunk_start:chunk_end] = raw_points
        flipped_prob[chunk_start:chunk_end] = flip_probs

        pad_pre = min(pad_frames, chunk_start)
        window = np.concatenate(
            [
                keypoints_xy_pre_alignment[chunk_start - pad_pre : chunk_start],
                raw_points,
                lookahead_points,
            ],
            axis=0,
        )
        window, last_valid_keypoints = _fill_nan_keypoints(window, last_valid_keypoints)
        smoothed_position, smoothed_heading = _smooth_position_and_heading(
            window, thorax_idx, neck_idx, abdomen_idx,
            position_denoise_sigma, heading_denoise_sigma,
        )  # fmt: skip
        # Trim the padding back off: everything above ran over
        # [chunk_start - pad_pre, chunk_end + len(lookahead_frames)) so the
        # Gaussian filter had real neighbors at this chunk's own edges; only
        # this chunk's own frames are actually used below.
        filled_keypoints = window[pad_pre : pad_pre + len(raw_points)]
        smoothed_position = smoothed_position[pad_pre : pad_pre + len(raw_points)]
        smoothed_heading = smoothed_heading[pad_pre : pad_pre + len(raw_points)]

        aligned_batch = np.empty(
            (len(chunk_frames), crop_dim, crop_dim), dtype=np.uint8
        )
        fullsize_batch = (
            np.stack(chunk_frames).astype(np.uint8) if write_fullsize else None
        )
        t0 = _tick()
        for i, frame in enumerate(chunk_frames):
            aligned_frame, aligned_kp, transform_matrix = transform_single_frame_to_align(
                frame, filled_keypoints[i], crop_dim, thorax_idx, neck_idx, abdomen_idx,
                thorax_y_normalized, position=smoothed_position[i],
                heading=smoothed_heading[i],
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
            # Separate from the warp above: profiling found the HDF5 gzip
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
        from spotlight.postprocessing.pose2d.io_utils import save_pose_h5

        save_pose_h5(
            output_pose2d_h5_path, poses, keypoint_scores, instance_score,
            np.zeros(n_frames, dtype=bool), pose2d_node_names,
            video_path=output_aligned_video_path,
        )  # fmt: skip
        result["pose2d_h5_path"] = output_pose2d_h5_path
        result["pose2d_node_names"] = pose2d_node_names

    return result
