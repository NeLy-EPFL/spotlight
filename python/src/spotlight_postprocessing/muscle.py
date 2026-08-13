"""Muscle frame processing: warp raw PCO-camera TIFF frames into the behavior
camera's coordinate system and apply the same fly-alignment transforms used
for the behavior channel.

Spatial mapping uses a pre-computed homography (homography_parameters.yaml,
produced by the ChArUco homography scan/fit) rather than the legacy
spotlight_tools.calibration stage-position mappers.

Restructured for the streaming pipeline (see `behavior.py`): timestamp-based
planning (orphan/drop detection, muscle<->behavior frame mapping) is metadata-
only and cheap, so it all happens upfront in `plan_muscle_behavior_mapping`.
Actual pixel warping only happens for whichever muscle frames fall in the
behavior chunk currently being processed, via `warp_muscle_chunk`, writing
straight into a `postprocessing.io.MuscleH5Writer` instead of per-frame TIFFs.
"""

import logging
from dataclasses import dataclass

import cv2
import numpy as np
import pandas as pd
import yaml
from joblib import Parallel, delayed
from pathlib import Path

from spotlight_tools.calibration import HomographyMapper


# Muscle<->behavior frame correspondence. Once the orphan muscle frames captured
# before the excitation LED turned on are skipped (see
# _count_leading_orphan_muscle_frames) and the survivors are re-indexed 0-based,
# (re-indexed) muscle frame N corresponds to behavior frame
# (N + MUSCLE_BEHAVIOR_OFFSET) * sync_ratio. The excitation LED fires on the 0th
# behavior frame of each group, i.e. the behavior frame exposed simultaneously
# with the muscle frame, so the first illuminated muscle frame lines up with
# behavior frame 0 and the correct offset for spatial overlay is 0.
MUSCLE_BEHAVIOR_OFFSET = 0


@dataclass
class MuscleBehaviorMapping:
    """Cheap, metadata-only plan for warping muscle frames, built once
    upfront by `plan_muscle_behavior_mapping`. `warp_muscle_chunk` uses this
    to know which raw muscle frames belong to a given behavior-frame chunk,
    without touching any image pixels."""

    muscle_image_paths: list[Path]
    """Raw muscle TIFF paths, one per kept (post-orphan-skip) muscle frame,
    in muscle-frame order."""
    corresponding_behavior_frame_id: np.ndarray
    """`(n_muscle_frames,)`, drop-aware behavior frame id for each kept
    muscle frame -- the true source of truth for the mapping."""
    x_pos_mm_interp: np.ndarray
    y_pos_mm_interp: np.ndarray
    acquired_time_us: np.ndarray
    received_time_us: np.ndarray
    homography_mapper: HomographyMapper
    output_dim: tuple[int, int]
    """(width, height) of the warped output, matching the behavior stream's
    own output (aligned crop_dim x crop_dim, or fullsize)."""


def get_behavior_muscle_sync_ratio(
    *,
    experiment_parameters_path: Path | str | None,
    recording_dir: Path | str | None = None,
) -> int:
    """Extract the behavior-to-muscle frame synchronization ratio.

    Reads ``muscle_sync_ratio`` from ``experiment_parameters.yaml``. For backward
    compatibility with recordings made before the timing metadata was merged into
    ``experiment_parameters.yaml``, it also accepts the legacy ``sync_ratio`` key
    (written by the now-removed ``dual_recording_timing.yaml``), emitting a
    deprecation warning. ``experiment_parameters_path`` may point at either file.
    """
    if experiment_parameters_path is None and recording_dir is None:
        raise ValueError(
            "Either experiment_parameters_path or recording_dir must be provided."
        )
    if experiment_parameters_path is not None and recording_dir is not None:
        raise ValueError(
            "Only one of experiment_parameters_path or recording_dir should be provided."
        )

    logger = logging.getLogger(__name__)

    def _read_sync_ratio(path: Path) -> int | None:
        if not path.exists():
            return None
        with open(path, "r") as f:
            metadata = yaml.safe_load(f) or {}
        if "muscle_sync_ratio" in metadata:
            return metadata["muscle_sync_ratio"]
        if "sync_ratio" in metadata:  # legacy dual_recording_timing.yaml
            logger.warning(
                "DEPRECATION: reading the behavior-muscle sync ratio from the legacy "
                "'sync_ratio' key in '%s'.",
                path,
            )
            return metadata["sync_ratio"]
        return None

    if experiment_parameters_path is not None:
        explicit_path = Path(experiment_parameters_path)
        candidate_paths = [
            explicit_path,
            explicit_path.parent / "dual_recording_timing.yaml",
        ]
    else:
        metadata_dir = Path(recording_dir) / "metadata"
        candidate_paths = [
            metadata_dir / "experiment_parameters.yaml",
            metadata_dir / "dual_recording_timing.yaml",
        ]
    candidate_paths = list(dict.fromkeys(candidate_paths))

    for path in candidate_paths:
        sync_ratio = _read_sync_ratio(path)
        if sync_ratio is not None:
            return sync_ratio

    raise FileNotFoundError(
        "Could not determine the behavior-muscle sync ratio from any of: "
        + ", ".join(f"'{p}'" for p in candidate_paths)
    )


def _get_stage_pos_df_at_muscle_frames(
    interpolated_stage_pos_path: Path,
    behavior_muscle_sync_ratio: int,
    offset: int = MUSCLE_BEHAVIOR_OFFSET,
) -> pd.DataFrame:
    first_behavior_frame_id = offset * behavior_muscle_sync_ratio
    stage_pos_df_at_behavior_frames = pd.read_csv(interpolated_stage_pos_path)
    behavior_frame_id = stage_pos_df_at_behavior_frames["behavior_frame_id"]
    return stage_pos_df_at_behavior_frames[
        (behavior_frame_id % behavior_muscle_sync_ratio == 0)
        & (behavior_frame_id >= first_behavior_frame_id)
    ].reset_index(drop=True)


# Onset threshold for structure-based leading-orphan detection -- see
# `plan_muscle_behavior_mapping`'s module docstring in the original
# (poseforge2-derived) implementation for the full derivation.
ORPHAN_ONSET_AUTOCORR_THRESHOLD = 0.12


def _diff_spatial_autocorrelation(prev_frame: np.ndarray, frame: np.ndarray) -> float:
    diff = frame - prev_frame
    diff = diff - diff.mean()
    denom = float((diff * diff).mean())
    if denom <= 0:
        return 0.0
    horizontal = float((diff[:, :-1] * diff[:, 1:]).mean())
    vertical = float((diff[:-1, :] * diff[1:, :]).mean())
    return 0.5 * (horizontal + vertical) / denom


def _count_leading_orphan_muscle_frames(
    raw_muscle_images_dir: Path,
    *,
    autocorr_threshold: float = ORPHAN_ONSET_AUTOCORR_THRESHOLD,
    num_frames_to_scan: int = 50,
) -> int:
    logger = logging.getLogger(__name__)
    paths_by_id = {}
    for path in raw_muscle_images_dir.glob("*.tif"):
        try:
            paths_by_id[int(path.stem.split("_")[-1])] = path
        except ValueError:
            continue
    sorted_ids = sorted(paths_by_id)[:num_frames_to_scan]
    if len(sorted_ids) < 2:
        return 0

    prev = None
    for i, frame_id in enumerate(sorted_ids):
        im = cv2.imread(str(paths_by_id[frame_id]), cv2.IMREAD_UNCHANGED).astype(
            np.float32
        )
        if (
            prev is not None
            and _diff_spatial_autocorrelation(prev, im) > autocorr_threshold
            and im.mean() > prev.mean()
        ):
            return i
        prev = im

    logger.warning(
        "Orphan detection: no dark->illuminated transition found in the first %d muscle "
        "frame(s). Reporting 0 orphan frames.",
        len(sorted_ids),
    )
    return 0


def _detect_dropped_muscle_frames(
    acquired_time_us, *, gap_threshold: float = 1.5, persistence_window: int = 12
):
    t = np.asarray(acquired_time_us, dtype=float)
    n = len(t)
    if n < 3:
        return np.arange(n), 0
    t = t - t[0]
    period = float(np.median(np.diff(t)))
    if period <= 0:
        return np.arange(n), 0
    excess = t / period - np.arange(n)
    w = persistence_window
    num_dropped_before = np.zeros(n, dtype=int)
    running = 0
    for i in range(1, n):
        gap_periods = (t[i] - t[i - 1]) / period
        if gap_periods > gap_threshold:
            before = np.median(excess[max(0, i - w) : i])
            after = np.median(excess[i : min(n, i + w)])
            shift = after - before
            missed = int(round(shift))
            if missed >= 1 and abs(shift - missed) <= 0.35:
                running += missed
        num_dropped_before[i] = running
    slots = np.arange(n) + num_dropped_before
    return slots, int(running)


def _read_muscle_frame_times(muscle_image_paths):
    acquired, received = [], []
    for muscle_path in muscle_image_paths:
        metadata_path = str(muscle_path).replace(".tif", ".csv")
        frame_ds = pd.read_csv(metadata_path).iloc[0]
        acquired.append(int(frame_ds["acquired_time_us"]))
        received.append(int(frame_ds["received_time_us"]))
    return np.array(acquired, dtype=np.int64), np.array(received, dtype=np.int64)


def _filter_muscle_frames_by_availability(
    raw_muscle_images_dir: Path,
    stage_pos_df_at_muscle_frames: pd.DataFrame,
    missing_muscle_frames_tolerance: int = 3,
    first_frameid: int = 0,
) -> list[Path]:
    _muscle_paths_by_frameid = {}
    for path in raw_muscle_images_dir.glob("*.tif"):
        try:
            frameid = int(path.stem.split("_")[-1])
        except ValueError:
            logging.warning(f"Could not parse frame index from {path.name}. Skipping.")
            continue
        _muscle_paths_by_frameid[frameid] = path

    last_available_frameid = (
        max(_muscle_paths_by_frameid) if _muscle_paths_by_frameid else first_frameid - 1
    )

    muscle_image_paths = []
    num_expected_frames = stage_pos_df_at_muscle_frames.shape[0]
    for frameid in range(first_frameid, num_expected_frames + first_frameid):
        if frameid in _muscle_paths_by_frameid:
            muscle_image_paths.append(_muscle_paths_by_frameid[frameid])
            continue
        if frameid > last_available_frameid:
            break
        logging.error(f"Frame {frameid} not found in {raw_muscle_images_dir}.")
        raise RuntimeError("Dataset is incomplete.")

    num_muscle_frames = len(muscle_image_paths)
    num_missing = num_expected_frames - num_muscle_frames
    unexplained_missing = num_missing - first_frameid
    if unexplained_missing > missing_muscle_frames_tolerance:
        raise RuntimeError(
            f"Muscle recording is short by {num_missing} frame(s) relative to the "
            f"{num_expected_frames} expected from the behavior stream (found "
            f"{num_muscle_frames}). {first_frameid} explained by leading orphans; "
            f"the remaining {unexplained_missing} exceed "
            f"--missing-muscle-frames-tolerance ({missing_muscle_frames_tolerance})."
        )
    if num_missing > 0:
        logging.info(
            f"Found {num_muscle_frames} muscle images, expected {num_expected_frames} "
            f"({first_frameid} explained by leading-orphan skip, "
            f"{max(0, unexplained_missing)} within tolerance)."
        )

    return muscle_image_paths


def plan_muscle_behavior_mapping(
    *,
    raw_muscle_images_dir: Path,
    experiment_parameters_path: Path,
    behavior_frames_metadata_path: Path,
    homography_path: Path,
    output_dim: tuple[int, int],
    missing_muscle_frames_tolerance: int = 3,
    num_orphan_muscle_frames: int | None = None,
) -> MuscleBehaviorMapping:
    """Cheap, metadata-only pass (timestamps/paths, no image pixels):
    determine which raw muscle frames to use and which (drop-aware) behavior
    frame each corresponds to. See module docstring."""
    logger = logging.getLogger(__name__)

    sync_ratio = get_behavior_muscle_sync_ratio(
        experiment_parameters_path=experiment_parameters_path
    )
    stage_pos_df_at_muscle_frames = _get_stage_pos_df_at_muscle_frames(
        behavior_frames_metadata_path, sync_ratio
    )
    homography_mapper = HomographyMapper(homography_path)

    if num_orphan_muscle_frames is not None:
        first_muscle_frameid = num_orphan_muscle_frames
    else:
        first_muscle_frameid = _count_leading_orphan_muscle_frames(
            raw_muscle_images_dir
        )
        if first_muscle_frameid > 0:
            logger.info(
                f"Detected {first_muscle_frameid} leading orphan muscle frame(s)"
            )

    muscle_image_paths = _filter_muscle_frames_by_availability(
        raw_muscle_images_dir,
        stage_pos_df_at_muscle_frames,
        missing_muscle_frames_tolerance,
        first_frameid=first_muscle_frameid,
    )
    muscle_acquired_time_us, muscle_received_time_us = _read_muscle_frame_times(
        muscle_image_paths
    )
    slots, num_dropped = _detect_dropped_muscle_frames(muscle_acquired_time_us)
    if num_dropped:
        logger.warning(
            f"Detected {num_dropped} dropped muscle exposure(s) mid-recording."
        )

    keep = slots < len(stage_pos_df_at_muscle_frames)
    if not keep.all():
        muscle_image_paths = [p for p, k in zip(muscle_image_paths, keep) if k]
        muscle_acquired_time_us = muscle_acquired_time_us[keep]
        muscle_received_time_us = muscle_received_time_us[keep]
        slots = slots[keep]

    behavior_group_df = stage_pos_df_at_muscle_frames.iloc[slots].reset_index(drop=True)

    return MuscleBehaviorMapping(
        muscle_image_paths=muscle_image_paths,
        corresponding_behavior_frame_id=behavior_group_df[
            "behavior_frame_id"
        ].to_numpy(),
        x_pos_mm_interp=behavior_group_df["x_pos_mm_interp"].to_numpy(dtype=np.float32),
        y_pos_mm_interp=behavior_group_df["y_pos_mm_interp"].to_numpy(dtype=np.float32),
        acquired_time_us=muscle_acquired_time_us,
        received_time_us=muscle_received_time_us,
        homography_mapper=homography_mapper,
        output_dim=output_dim,
    )


def restrict_mapping_to_frame_range(
    mapping: MuscleBehaviorMapping, frame_start: int, frame_end: int
) -> MuscleBehaviorMapping:
    """Restrict a mapping (built against the *full* trial) to muscle frames
    whose `corresponding_behavior_frame_id` falls in `[frame_start,
    frame_end)`, re-expressed relative to `frame_start` (see
    `--frame-range`'s CLI docs): once the behavior pipeline itself only
    processes that sub-range, its own frame 0 *is* `frame_start`, so this
    mapping's behavior-frame ids must shift to match, not just filter.
    """
    ids = mapping.corresponding_behavior_frame_id
    keep = (ids >= frame_start) & (ids < frame_end)
    return MuscleBehaviorMapping(
        muscle_image_paths=[p for p, k in zip(mapping.muscle_image_paths, keep) if k],
        corresponding_behavior_frame_id=ids[keep] - frame_start,
        x_pos_mm_interp=mapping.x_pos_mm_interp[keep],
        y_pos_mm_interp=mapping.y_pos_mm_interp[keep],
        acquired_time_us=mapping.acquired_time_us[keep],
        received_time_us=mapping.received_time_us[keep],
        homography_mapper=mapping.homography_mapper,
        output_dim=mapping.output_dim,
    )


def warp_single_muscle_frame_to_behavior(
    muscle2behavior_trans_mat: np.ndarray,
    behavior_alignment_trans_mat: np.ndarray,
    input_path: Path,
    output_dim: tuple[int, int],
) -> np.ndarray:
    """Apply composed muscle->behavior homography + behavior alignment
    transform to one raw muscle frame. Returns the warped uint16 image
    (never written to disk directly -- callers append it to a `MuscleH5Writer`)."""
    in_image = cv2.imread(str(input_path), cv2.IMREAD_UNCHANGED)

    if muscle2behavior_trans_mat.shape == (2, 3):
        muscle2behavior_trans_mat = np.vstack([muscle2behavior_trans_mat, [0, 0, 1]])
    behavior_alignment_trans_mat_3x3 = np.vstack(
        [behavior_alignment_trans_mat, [0, 0, 1]]
    )
    composed_trans_mat = behavior_alignment_trans_mat_3x3 @ muscle2behavior_trans_mat
    return cv2.warpPerspective(in_image, composed_trans_mat, output_dim)


def warp_muscle_chunk(
    mapping: MuscleBehaviorMapping,
    chunk_behavior_frame_start: int,
    chunk_behavior_frame_end: int,
    alignment_transforms_by_behavior_frame: dict[int, np.ndarray],
    num_cpu_workers: int = -1,
) -> np.ndarray:
    """Warp every muscle frame whose `corresponding_behavior_frame_id` falls
    in `[chunk_behavior_frame_start, chunk_behavior_frame_end)`, using this
    chunk's already-computed per-behavior-frame alignment transforms (no
    forward reference -- muscle frames are always sparser than behavior
    frames, so their corresponding behavior frame is always within the
    current or an earlier chunk).

    Returns `(n, H, W)` uint16 (n may be 0 for chunks with no muscle frame).
    """
    idxs = np.nonzero(
        (mapping.corresponding_behavior_frame_id >= chunk_behavior_frame_start)
        & (mapping.corresponding_behavior_frame_id < chunk_behavior_frame_end)
    )[0]
    if len(idxs) == 0:
        return np.empty(
            (0, mapping.output_dim[1], mapping.output_dim[0]), dtype=np.uint16
        )

    parallel_mapper = Parallel(n_jobs=num_cpu_workers, backend="loky")
    results = parallel_mapper(
        delayed(warp_single_muscle_frame_to_behavior)(
            mapping.homography_mapper.H_muscle2beh,
            alignment_transforms_by_behavior_frame[
                int(mapping.corresponding_behavior_frame_id[i])
            ],
            mapping.muscle_image_paths[i],
            mapping.output_dim,
        )
        for i in idxs
    )
    return np.stack(results).astype(np.uint16)


def save_muscle_metadata_csv(mapping: MuscleBehaviorMapping, output_path: Path) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    n = len(mapping.muscle_image_paths)
    pd.DataFrame(
        data={
            "muscle_frame_id": np.arange(n, dtype=np.uint32),
            "corresponding_behavior_frame_id": mapping.corresponding_behavior_frame_id.astype(
                np.uint32
            ),
            "x_pos_mm_interp": mapping.x_pos_mm_interp,
            "y_pos_mm_interp": mapping.y_pos_mm_interp,
            "acquired_time_us": mapping.acquired_time_us.astype(np.uint64),
            "received_time_us": mapping.received_time_us.astype(np.uint64),
        }
    ).to_csv(output_path, index=False)


def match_muscle_frameid_to_behavior_frameid(
    muscle_frameid: int | list[int],
    *,
    sync_ratio: int,
    offset: int = MUSCLE_BEHAVIOR_OFFSET,
):
    is_singleton_int = isinstance(muscle_frameid, (int, np.integer))
    if is_singleton_int:
        muscle_frameid = [muscle_frameid]
    behavior_frameid = [(mfid + offset) * sync_ratio for mfid in muscle_frameid]
    return behavior_frameid[0] if is_singleton_int else behavior_frameid
