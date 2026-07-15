"""
Muscle frame processing: warp raw PCO-camera TIFF frames into the behavior
camera's coordinate system and apply the same fly-alignment transforms used
for the behavior channel.

Spatial mapping uses a pre-computed homography (homography_parameters.yaml,
produced by the ChArUco homography scan/fit) rather than the legacy
spotlight_tools.calibration stage-position mappers.
"""

import logging
import cv2
import numpy as np
import pandas as pd
import yaml
import h5py
from joblib import Parallel, delayed
from pathlib import Path

from spotlight_tools.calibration import HomographyMapper
from spotlight_tools.common.video import get_video_info
from spotlight_tools.postprocessing.io import check_output_path_against_alignment_flag


# Muscle<->behavior frame correspondence. Once the orphan muscle frames captured
# before the excitation LED turned on are skipped (see
# _count_leading_orphan_muscle_frames) and the survivors are re-indexed 0-based,
# (re-indexed) muscle frame N corresponds to behavior frame
# (N + MUSCLE_BEHAVIOR_OFFSET) * sync_ratio. The excitation LED fires on the 0th
# behavior frame of each group, i.e. the behavior frame exposed simultaneously
# with the muscle frame, so the first illuminated muscle frame lines up with
# behavior frame 0 and the correct offset for spatial overlay is 0. (Note this is
# exposure-time correspondence; the muscle frame is *read out* about one period
# later, but readout/arrival time is irrelevant for spatial alignment.)
#
# This is the *nominal* correspondence, valid when the muscle stream is gap-free. A
# muscle exposure dropped mid-recording (see _detect_dropped_muscle_frames) breaks it:
# every later muscle frame then belongs to a behavior group one further along than
# N * sync_ratio. warp_all_muscle_frames_to_behavior handles this and writes the true,
# drop-aware behavior frame id into the muscle metadata's corresponding_behavior_frame_id
# column, which is therefore the single source of truth; consumers should read that
# column rather than recompute N * sync_ratio via match_muscle_frameid_to_behavior_frameid.
MUSCLE_BEHAVIOR_OFFSET = 0


_imwrite_compression_params = [cv2.IMWRITE_TIFF_COMPRESSION, 5]
# TIFF compression methods:
#   cv::IMWRITE_TIFF_COMPRESSION_NONE = 1 ,
#   cv::IMWRITE_TIFF_COMPRESSION_LZW = 5 ,
#   cv::IMWRITE_TIFF_COMPRESSION_JPEG = 7 ,
#   cv::IMWRITE_TIFF_COMPRESSION_PACKBITS = 32773 ,
#   ... see https://docs.opencv.org/4.x/d8/d6a/group__imgcodecs__flags.html
# cv::IMWRITE_TIFF_COMPRESSION_LZW is used by the recording GUI


def warp_all_muscle_frames_to_behavior(
    *,
    raw_muscle_images_dir: Path,
    transformed_muscle_images_output_dir: Path,
    experiment_parameters_path: Path,
    processed_behavior_frame_metadata_path: Path,
    muscle_metadata_output_path: Path,
    homography_path: Path,
    align_fly: bool = True,
    behavior_alignment_metadata_path: Path | None = None,
    processed_behavior_video_path: Path | None = None,
    missing_muscle_frames_tolerance: int = 3,
    num_orphan_muscle_frames: int | None = None,
    num_workers: int = -1,
):
    """Warp raw PCO muscle frames into the behavior camera's coordinate system.

    1. Determines timing synchronisation between muscle and behavior recordings,
       including leading orphan frames (skipped) and muscle exposures dropped
       mid-recording (accounted for so they do not offset later frames -- see
       _detect_dropped_muscle_frames).
    2. Maps muscle pixels to behavior pixels via a pre-computed homography.
    3. Applies the same per-frame alignment transforms used for the behavior channel.
    4. Saves transformed TIFF frames and a metadata CSV whose
       ``corresponding_behavior_frame_id`` is the drop-aware source of truth for the
       muscle<->behavior correspondence.

    Args:
        raw_muscle_images_dir: Directory containing raw muscle TIFF files.
        transformed_muscle_images_output_dir: Directory for output frames.
        experiment_parameters_path: Path to ``experiment_parameters.yaml``.
        processed_behavior_frame_metadata_path: Path to behavior frames metadata CSV.
        muscle_metadata_output_path: Output path for muscle frames metadata CSV.
        homography_path: Path to homography calibration YAML
            (``homography_parameters.yaml`` produced by the ChArUco scan).
        align_fly (bool): Whether behavior frames were transformed to align the fly.
            If True, ``behavior_alignment_metadata_path`` must be provided.
            If False, ``processed_behavior_video_path`` must be provided.
        behavior_alignment_metadata_path: Path to behavior alignment transforms (HDF5).
            Required when ``align_fly`` is True.
        processed_behavior_video_path: Path to processed behavior video (used only to
            read output dimensions when ``align_fly`` is False).
        missing_muscle_frames_tolerance: Maximum allowed consecutive missing frames at
            the end of the recording.
        num_orphan_muscle_frames: Number of leading orphan (pre-excitation, dark)
            muscle frames to skip. If None (default), the count is detected
            automatically from muscle-frame image structure via
            ``_count_leading_orphan_muscle_frames``. Provide an explicit integer to
            override the automatic detection (e.g. when it misfires).
        num_workers: Parallel workers for warping (-1 = all cores).
    """
    logger = logging.getLogger(__name__)

    # Check if output path suggests alignment status consistent with `align_fly`
    check_output_path_against_alignment_flag(
        transformed_muscle_images_output_dir, align_fly
    )

    # Get behavior-muscle sync ratio
    behavior_muscle_sync_ratio = get_behavior_muscle_sync_ratio(
        experiment_parameters_path=experiment_parameters_path
    )

    # Load the stage positions at muscle recording frames
    stage_pos_df_at_muscle_frames = _get_stage_pos_df_at_muscle_frames(
        processed_behavior_frame_metadata_path, behavior_muscle_sync_ratio
    )

    # Build homography mapper
    logger.info(f"Using homography transformation from {homography_path}")
    homography_mapper = HomographyMapper(homography_path)

    # Determine how many orphan muscle frames precede the first behavior-synced
    # frame. In continuous (free-run) PCO mode the muscle camera starts grabbing
    # frames before the behavior trigger -- and therefore the blue excitation LED
    # -- turns on, so those leading frames receive no excitation and are much
    # darker than the illuminated frames that follow. They have no corresponding
    # behavior group and must be skipped. The onset is detected from the muscle
    # frames' own image structure (a within-camera signal needing no common clock),
    # not from a frame-count difference: the latter silently conflates leading
    # orphans with a trailing stop-time mismatch between the two cameras and can
    # therefore land on the wrong frame.
    if num_orphan_muscle_frames is not None:
        if num_orphan_muscle_frames < 0:
            raise ValueError(
                "num_orphan_muscle_frames must be non-negative, got "
                f"{num_orphan_muscle_frames}."
            )
        first_muscle_frameid = num_orphan_muscle_frames
        logger.info(
            f"Using user-specified count of {first_muscle_frameid} leading orphan "
            "muscle frame(s) to skip (automatic brightness-based detection disabled)."
        )
    else:
        first_muscle_frameid = _count_leading_orphan_muscle_frames(
            raw_muscle_images_dir
        )
        if first_muscle_frameid > 0:
            logger.info(
                f"Detected {first_muscle_frameid} orphan muscle frame(s) at the start "
                "of the recording (captured before the behavior trigger / excitation "
                "LED turned on); skipping them."
            )

    # One row per behavior group (behavior frames at multiples of the sync ratio); each
    # muscle frame is paired with one of these groups.
    behavior_group_df = stage_pos_df_at_muscle_frames

    # Check if we have all the muscle images
    muscle_image_paths = _filter_muscle_frames_by_availability(
        raw_muscle_images_dir,
        behavior_group_df,
        missing_muscle_frames_tolerance,
        first_frameid=first_muscle_frameid,
    )

    # Read each kept muscle frame's timestamps once (reused for drop detection just below
    # and for the metadata CSV at the end).
    muscle_acquired_time_us, muscle_received_time_us = _read_muscle_frame_times(
        muscle_image_paths
    )

    # Detect muscle exposures dropped mid-recording and map each muscle frame to its
    # true behavior group. Without this, a single silent drop (contiguous ids on disk)
    # offsets every later muscle<->behavior pairing by one group -- see
    # _detect_dropped_muscle_frames.
    slots, num_dropped = _detect_dropped_muscle_frames(muscle_acquired_time_us)
    if num_dropped:
        skipped_groups = sorted(
            set(range(int(slots[0]), int(slots[-1]) + 1)) - set(slots.tolist())
        )
        logger.warning(
            "Detected %d dropped muscle exposure(s) mid-recording (missed frame(s) not "
            "saved to disk). Re-aligning later muscle frames to their true behavior "
            "groups so the one-frame offset does not propagate. Behavior frame(s) left "
            "without a muscle counterpart: %s.",
            num_dropped,
            [g * behavior_muscle_sync_ratio for g in skipped_groups],
        )

    # A drop pushes later frames into later behavior groups; discard any muscle frame
    # whose group runs past the behavior recording (only the final frame(s), if any).
    keep = slots < len(behavior_group_df)
    if not keep.all():
        logger.info(
            "Discarding %d trailing muscle frame(s) whose behavior group is beyond the "
            "behavior recording.",
            int((~keep).sum()),
        )
        muscle_image_paths = [p for p, k in zip(muscle_image_paths, keep) if k]
        muscle_acquired_time_us = muscle_acquired_time_us[keep]
        muscle_received_time_us = muscle_received_time_us[keep]
        slots = slots[keep]

    # One behavior-group row per kept muscle frame, selected by its true (drop-aware)
    # group rather than by contiguous position.
    stage_pos_df_at_muscle_frames = behavior_group_df.iloc[slots].reset_index(drop=True)

    # Load transformation matrices applied to behavior frames (for alignment)
    if align_fly:
        alignment_transforms, output_dim = _load_alignment_transform_metadata(
            behavior_alignment_metadata_path, behavior_muscle_sync_ratio
        )
        # Use each muscle frame's true behavior group's alignment transform.
        alignment_transforms = alignment_transforms[slots]
    else:
        ident_transform = np.eye(2, 3)
        alignment_transforms = np.repeat(
            ident_transform[None, :, :], len(muscle_image_paths), axis=0
        )
        width, height, _ = get_video_info(processed_behavior_video_path)
        output_dim = (width, height)

    # Prepare input kwargs for parallel processing. Output frames are named by
    # their re-indexed (0-based, orphan-free) muscle frame ID so that file index,
    # the muscle_frame_id column in the metadata CSV, and the IDs returned by
    # match_*_frameid all agree -- the downstream visualizers rely on this.
    input_kwargs = []
    for i, input_path in enumerate(muscle_image_paths):
        output_path = transformed_muscle_images_output_dir / f"muscle_frame_{i:09d}.tif"
        input_kwargs.append({
            "muscle2behavior_trans_mat": homography_mapper.H_muscle2beh,
            "behavior_alignment_trans_mat": alignment_transforms[i],
            "input_path": input_path,
            "output_dim": output_dim,
            "output_path": output_path,
            "return_output": False,  # reduce IO stress
            "use_perspective": True,
        })

    # Process muscle images in parallel
    transformed_muscle_images_output_dir.mkdir(parents=True, exist_ok=True)
    parallel_runner = Parallel(n_jobs=num_workers, backend="loky")
    logger.info(
        f"Warping {len(input_kwargs)} muscle images using {num_workers} workers"
        f" (effectively {parallel_runner._effective_n_jobs()} workers)"
    )
    parallel_runner(
        delayed(warp_single_muscle_frame_to_behavior)(**kwargs)
        for kwargs in input_kwargs
    )
    logger.info(
        "Finished warping muscle images; "
        f"outputs saved to {transformed_muscle_images_output_dir}"
    )

    # Save muscle metadata as a dataframe
    muscle_frame_metadata = _make_muscle_metadata_dataframe(
        stage_pos_df_at_muscle_frames,
        muscle_acquired_time_us,
        muscle_received_time_us,
    )
    muscle_metadata_output_path.parent.mkdir(parents=True, exist_ok=True)
    muscle_frame_metadata.to_csv(muscle_metadata_output_path, index=False)
    logger.info(f"Muscle frame metadata saved to {muscle_metadata_output_path}")


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
        """Return the sync ratio stored in `path`, or None if it is not there.

        Accepts both the current ``experiment_parameters.yaml`` (``muscle_sync_ratio``)
        and the legacy ``dual_recording_timing.yaml`` (``sync_ratio``), so the caller
        may pass either file.
        """
        if not path.exists():
            return None
        with open(path, "r") as f:
            metadata = yaml.safe_load(f) or {}
        if "muscle_sync_ratio" in metadata:
            return metadata["muscle_sync_ratio"]
        if "sync_ratio" in metadata:  # legacy dual_recording_timing.yaml
            logger.warning(
                "DEPRECATION: reading the behavior-muscle sync ratio from the legacy "
                "'sync_ratio' key in '%s'. The recorder now writes 'muscle_sync_ratio' "
                "to 'experiment_parameters.yaml'; legacy support will be removed in a "
                "future release.",
                path,
            )
            return metadata["sync_ratio"]
        return None

    # Resolve the metadata file(s) to try, in priority order. A caller may point
    # experiment_parameters_path at either the current experiment_parameters.yaml or
    # the legacy dual_recording_timing.yaml -- _read_sync_ratio handles both keys. When
    # only recording_dir is given, try the current file, then the legacy sibling.
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
    candidate_paths = list(dict.fromkeys(candidate_paths))  # dedupe, preserve order

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
    # Keep one behavior-frame row per muscle frame: those at multiples of the sync
    # ratio, starting at offset * sync_ratio (the behavior frame that muscle frame 0
    # is synced to). For offset == 1 this drops behavior frame 0, which has no muscle
    # correspondence due to the readout lag.
    first_behavior_frame_id = offset * behavior_muscle_sync_ratio
    stage_pos_df_at_behavior_frames = pd.read_csv(interpolated_stage_pos_path)
    behavior_frame_id = stage_pos_df_at_behavior_frames["behavior_frame_id"]
    stage_pos_df_at_muscle_frames = stage_pos_df_at_behavior_frames[
        (behavior_frame_id % behavior_muscle_sync_ratio == 0)
        & (behavior_frame_id >= first_behavior_frame_id)
    ].reset_index(drop=True)

    if len(stage_pos_df_at_muscle_frames) > 0:
        assert (
            match_muscle_frameid_to_behavior_frameid(
                0, sync_ratio=behavior_muscle_sync_ratio, offset=offset
            )
            == stage_pos_df_at_muscle_frames.iloc[0]["behavior_frame_id"]
        ), "Muscle-to-behavior frame ID mapping mismatch."
    return stage_pos_df_at_muscle_frames


# Onset threshold for structure-based leading-orphan detection (see
# _count_leading_orphan_muscle_frames). The detection metric is the lag-1 spatial
# autocorrelation of the frame-to-frame difference: it is ~0.08 for a dark->dark
# difference (spatially white sensor noise, essentially the same across driver lines and
# rigs) and jumps to >=~0.15 at the first illuminated frame, when the excitation LED
# turns on and a spatially coherent muscle blob appears in the difference. 0.12 sits in
# the (0.09, 0.15) gap between those two regimes.
#
# How this value was obtained: it recovered the hand-labelled leading-orphan count on all
# 12 recordings of a ground-truth set spanning pan-muscle and sparse (~100 px) driver
# lines, several rigs, and partly-out-of-frame flies -- and every threshold in
# (0.09, 0.15) does so, so it is not knife-edge. Because the metric keys on the *shape* of
# the change rather than its brightness, one fixed value generalises across lines whose
# signal magnitude differs ~100-fold. The exploration and validation harness is archived
# under scripts/archive/ (detect_orphan_structure.py, orphan_gt_dataset.txt,
# orphan_gt_plots/).
ORPHAN_ONSET_AUTOCORR_THRESHOLD = 0.12


def _diff_spatial_autocorrelation(prev_frame: np.ndarray, frame: np.ndarray) -> float:
    """Lag-1 spatial autocorrelation of the frame-to-frame difference.

    ~0 for a white-noise (dark->dark) difference; jumps toward 1 when a spatially coherent
    muscle blob appears in the difference. Scale-invariant -- it depends on the *shape* of
    the change, not its brightness -- so it behaves the same for pan and sparse driver
    lines and for a partly-visible fly.
    """
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
    """Count muscle frames captured before the behavior trigger / excitation LED turned on.

    In continuous (free-run) PCO mode the muscle camera begins grabbing frames before the
    behavior trigger -- and therefore the blue excitation LED -- turns on. These leading
    "orphan" frames receive no excitation; they have no corresponding behavior group and
    must be skipped.

    Detection is structure-based: it needs no common clock between the two cameras (there
    is none) and no per-recording tuning. For each consecutive pair of leading frames we
    take the lag-1 spatial autocorrelation of their difference
    (``_diff_spatial_autocorrelation``). A dark->dark difference is spatially white sensor
    noise (autocorrelation ~0.08, essentially the same across driver lines and rigs),
    whereas the dark->illuminated transition makes a spatially coherent muscle blob appear
    in the difference (autocorrelation jumps to >=~0.15).

    The onset is the first frame whose difference from its predecessor both (a) exceeds
    ``autocorr_threshold`` and (b) is a *brightening* transition (the frame is brighter than
    its predecessor). Requirement (b) is what handles the PCO first-frame artifact: in
    free-run mode the very first readout integrates charge accumulated while the sensor was
    arming, so frame 0 can be a genuinely bright, structured frame even though the
    excitation LED is still off. Its difference to the following dark orphans is a
    structured but *darkening* crossing -- the artifact switching off -- which would
    otherwise be mistaken for the onset. A real onset (dark->lit) is always a brightening
    crossing, so keeping only brightening crossings skips the artifact and lands on the
    true first illuminated frame. Its index is the leading-orphan count.

    Keying on the *shape* of the change rather than its magnitude is what makes a single
    fixed threshold work whether the driver line lights the whole fly or only ~100 pixels,
    and whether or not the fly is partly out of frame. The brightening test adds no tuning
    parameter (it is just the sign of the mean change). See
    ``ORPHAN_ONSET_AUTOCORR_THRESHOLD`` for how the threshold was chosen and validated.

    Args:
        raw_muscle_images_dir: Directory containing raw muscle TIFF files.
        autocorr_threshold: Onset threshold on the diff autocorrelation.
        num_frames_to_scan: How many leading frames to load and scan. Orphans number only
            a handful in practice, so scanning the first several dozen frames finds the
            onset while avoiding a read of the whole (multi-thousand-frame) recording.

    Returns:
        Number of leading orphan frames to skip (0 if no onset is found within the scanned
        window -- e.g. the recording starts already illuminated or the excitation LED
        never turned on).
    """
    logger = logging.getLogger(__name__)

    # Sort by parsed frame id so "first" means earliest-captured, not glob order, and scan
    # only the leading window (the onset is within the first handful of frames).
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
        im = cv2.imread(str(paths_by_id[frame_id]), cv2.IMREAD_UNCHANGED).astype(np.float32)
        if (
            prev is not None
            and _diff_spatial_autocorrelation(prev, im) > autocorr_threshold
            and im.mean() > prev.mean()  # brightening -> real onset, not the artifact off
        ):
            return i  # first illuminated frame -> i leading orphan frame(s) precede it
        prev = im

    logger.warning(
        "Orphan detection: no dark->illuminated transition found in the first %d muscle "
        "frame(s) (diff autocorrelation never exceeded %.3f). Reporting 0 orphan frames "
        "-- the recording may start already illuminated, or the excitation LED may not "
        "have turned on. Inspect with scripts/archive/detect_orphan_structure.py, or set "
        "the count explicitly via --num-orphan-muscle-frames.",
        len(sorted_ids),
        autocorr_threshold,
    )
    return 0


def _detect_dropped_muscle_frames(
    acquired_time_us,
    *,
    gap_threshold: float = 1.5,
    persistence_window: int = 12,
):
    """Locate muscle exposures dropped mid-recording from the frames' own timestamps.

    In continuous (free-run) PCO mode the muscle ``frame_id`` on disk is a sequential
    save counter, so if an exposure is missed the surviving frames' ids stay contiguous
    -- the gap is invisible to a frame-id/contiguity check. But a dropped exposure adds a
    real ~2x inter-frame gap and shifts every later frame one muscle period later in
    time, permanently. The index-based muscle<->behavior correspondence (muscle frame N
    -> behavior group N) then puts every subsequent muscle frame in the wrong behavior
    group, so a single silent drop misaligns the whole remainder of the recording.

    This counts, for each kept muscle frame, how many muscle periods have actually
    elapsed (from the muscle frames' own ``acquired_time_us`` -- no cross-camera clock is
    needed). A genuine drop shows up as a *permanent* +1 step in that surplus. Timestamp
    jitter can also momentarily stretch one interval past the threshold, but that deficit
    is repaid within a few frames; we therefore accept a large gap as a real drop only
    when the elapsed-time surplus straddling it persists *and* is close to an integer
    number of periods (a dropped exposure removes ~1 whole period, so a partial timing
    hiccup that permanently shifts the baseline by e.g. ~0.6 period is not a drop). The
    surplus magnitude is read from the persistent level shift, not from the raw gap size,
    which jitter can inflate.

    Args:
        acquired_time_us: Per-frame acquisition timestamps (microseconds) of the kept
            muscle frames, in capture order.
        gap_threshold: An inter-frame interval above this many nominal periods is a
            candidate drop location.
        persistence_window: Number of frames on each side used to measure whether a
            candidate gap's elapsed-time surplus persists.

    Returns:
        (slots, num_dropped): ``slots[i]`` is the behavior-group index that kept muscle
        frame ``i`` maps to (behavior frame ``slots[i] * sync_ratio``); it equals ``i``
        plus the number of dropped frames before it. ``num_dropped`` is the total.
    """
    t = np.asarray(acquired_time_us, dtype=float)
    n = len(t)
    if n < 3:
        return np.arange(n), 0
    t = t - t[0]
    # Robust single-frame period: the median interval is unaffected by the rare ~2x drop
    # gaps and by jitter tails.
    period = float(np.median(np.diff(t)))
    if period <= 0:
        return np.arange(n), 0
    # Elapsed muscle periods relative to a no-drop grid: rises by ~1 permanently at a
    # real drop, wanders (bounded, self-correcting) under timestamp jitter.
    excess = t / period - np.arange(n)
    w = persistence_window
    num_dropped_before = np.zeros(n, dtype=int)
    running = 0
    for i in range(1, n):
        gap_periods = (t[i] - t[i - 1]) / period
        if gap_periods > gap_threshold:
            # Missed frames = persistent level shift straddling the gap (robust to the
            # exact gap size), positive only when the surplus is not repaid by jitter.
            before = np.median(excess[max(0, i - w):i])
            after = np.median(excess[i:min(n, i + w)])
            shift = after - before
            missed = int(round(shift))
            # A dropped exposure removes ~1 whole period, so accept a candidate only when
            # the persistent surplus is close to an integer number of periods. This
            # rejects a partial timing hiccup (e.g. a 1.6x gap that permanently shifts the
            # baseline by only ~0.6 period) that round() would otherwise inflate to a full
            # dropped frame.
            if missed >= 1 and abs(shift - missed) <= 0.35:
                running += missed
        num_dropped_before[i] = running
    slots = np.arange(n) + num_dropped_before
    return slots, int(running)


def _read_muscle_frame_times(muscle_image_paths):
    """Read ``(acquired_time_us, received_time_us)`` for each muscle frame from its
    sidecar CSV, returned as two int64 arrays aligned with ``muscle_image_paths``."""
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
    # Index all available frames
    _muscle_paths_by_frameid = {}
    for path in raw_muscle_images_dir.glob("*.tif"):
        try:
            frameid = int(path.stem.split("_")[-1])
        except ValueError:
            logging.warning(
                f"Problem scanning muscle images: Could not parse frame index from "
                f"file name {path.name}. Skipping this file."
            )
            continue
        _muscle_paths_by_frameid[frameid] = path

    # Highest muscle frame index actually present on disk. Once we ask for a frame
    # beyond this, the muscle recording has simply ended: it is shorter than the
    # behavior recording because it started later (leading orphan frames are skipped
    # via first_frameid) and/or stopped earlier. Those trailing behavior frames just
    # have no muscle counterpart -- a clean end, not corruption. Note the number of
    # available muscle frames (total on disk minus the skipped orphans) is what bounds
    # the output; num_expected_frames comes from the behavior stream and is unaffected
    # by the orphan count.
    last_available_frameid = (
        max(_muscle_paths_by_frameid) if _muscle_paths_by_frameid else first_frameid - 1
    )

    # Check if each expected frame is among the frames found, starting from
    # first_frameid to skip orphan frames at the start of the recording.
    muscle_image_paths = []
    num_expected_frames = stage_pos_df_at_muscle_frames.shape[0]
    for frameid in range(first_frameid, num_expected_frames + first_frameid):
        if frameid in _muscle_paths_by_frameid:
            muscle_image_paths.append(_muscle_paths_by_frameid[frameid])
            continue
        # Frame missing. If we have run past the last muscle frame on disk, the
        # recording has simply ended -> stop cleanly. A gap *before* the last
        # available frame is genuine corruption: skipping it would misalign every
        # subsequent muscle<->behavior correspondence, so raise.
        if frameid > last_available_frameid:
            break
        logging.error(
            f"Problem scanning muscle images: Frame {frameid} not found in "
            f"{raw_muscle_images_dir} (a total of {num_expected_frames} is "
            f"expected). Dataset is incomplete."
        )
        raise RuntimeError("Dataset is incomplete.")

    num_muscle_frames = len(muscle_image_paths)
    num_missing = num_expected_frames - num_muscle_frames
    # Split the shortfall into the part the orphan skip accounts for and the rest. The two
    # cameras record about the same number of frames and are stopped together, so the
    # `first_frameid` orphan exposures the muscle camera spent before the trigger leave it
    # exactly that many frames short at the end -- an expected shortfall of `first_frameid`.
    # Anything beyond that (plus a few frames of stop-signal jitter, the tolerance) means
    # the muscle recording is genuinely truncated: those behavior frames are lost with no
    # muscle counterpart and every downstream alignment past the muscle end is missing. That
    # is not recoverable here, so fail loudly rather than silently emit a short recording.
    unexplained_missing = num_missing - first_frameid
    if unexplained_missing > missing_muscle_frames_tolerance:
        raise RuntimeError(
            f"Muscle recording is short by {num_missing} frame(s) relative to the "
            f"{num_expected_frames} expected from the behavior stream (found "
            f"{num_muscle_frames}). The {first_frameid} leading orphan frame(s) account "
            f"for {first_frameid} of these; the remaining {unexplained_missing} exceed the "
            f"tolerance of {missing_muscle_frames_tolerance} and are not explained by "
            f"orphans or stop-signal jitter -- the muscle recording is truncated (camera "
            f"stopped early or frames were not saved). Fix the recording, or if this "
            f"truncation is acceptable raise --missing-muscle-frames-tolerance to at least "
            f"{unexplained_missing} to process it anyway."
        )
    if num_missing > 0:
        logging.info(
            f"Found {num_muscle_frames} muscle images, expected {num_expected_frames} "
            f"({num_missing} fewer; {first_frameid} explained by the leading orphan "
            f"skip, {max(0, unexplained_missing)} within the tolerance of "
            f"{missing_muscle_frames_tolerance}). This is normal: the two cameras receive "
            f"the stop signal at slightly different times."
        )

    return muscle_image_paths


def _load_alignment_transform_metadata(
    behavior_alignment_metadata_path,
    behavior_muscle_sync_ratio,
    offset: int = MUSCLE_BEHAVIOR_OFFSET,
):
    with h5py.File(behavior_alignment_metadata_path, "r") as f:
        transforms_ds = f["transform_matrices"]
        # Pick the alignment transform of the behavior frame each muscle frame is
        # synced to: muscle frame N maps to behavior frame
        # (N + offset) * sync_ratio, so start the stride at offset * sync_ratio.
        first_behavior_frame_id = offset * behavior_muscle_sync_ratio
        alignment_transforms = transforms_ds[
            first_behavior_frame_id::behavior_muscle_sync_ratio, :, :
        ]
        output_dim = transforms_ds.attrs["output_dim"]
    return alignment_transforms, output_dim


def warp_single_muscle_frame_to_behavior(
    muscle2behavior_trans_mat: np.ndarray,
    behavior_alignment_trans_mat: np.ndarray,
    input_path: Path,
    output_dim: tuple[int, int],
    output_path: Path,
    return_output: bool = True,
    use_perspective: bool = False,
):
    """Apply composed transformation to align a single muscle frame with the
    corresponding behavior frame.

    Args:
        muscle2behavior_trans_mat (np.ndarray): Transformation matrix mapping muscle
            to behavior coordinates. Can be either:
            - 2x3 affine matrix (from Spotlight calibration)
            - 3x3 homography matrix (from ChArUco calibration)
        behavior_alignment_trans_mat (np.ndarray): 2x3 transformation matrix for
            behavior frame alignment (whatever that's been applied to the behavior
            frame; this can be read out from behavior alignment transform metadata).
        input_path (Path): Path to the input muscle image file.
        output_dim (tuple[int, int]): Output image dimensions (width, height).
        output_path (Path): Path where the transformed image will be saved.
        return_output (bool): Whether to return the transformed image array. Not
            returning anything may help reduce IO load if called in parallel (automatic
            garbage collection might not happen until after the map operation).
        use_perspective (bool): If True, treat muscle2behavior_trans_mat as a 3x3
            homography matrix and compose with affine alignment. If False, treat it
            as a 2x3 affine matrix. Default is False.

    Returns:
        np.ndarray or None (depending on return_output): Transformed muscle frame.
    """
    # Load muscle image
    in_image = cv2.imread(str(input_path), cv2.IMREAD_UNCHANGED)

    if use_perspective:
        # Homography case: muscle2behavior_trans_mat is 3x3
        # First apply homography, then apply affine alignment
        # We need to compose: alignment @ homography

        # Ensure homography is 3x3
        if muscle2behavior_trans_mat.shape == (2, 3):
            muscle2behavior_trans_mat = np.vstack(
                [muscle2behavior_trans_mat, [0, 0, 1]]
            )

        # Convert affine alignment to 3x3 homogeneous
        behavior_alignment_trans_mat_3x3 = np.vstack(
            [behavior_alignment_trans_mat, [0, 0, 1]]
        )

        # Compose: first homography, then alignment
        composed_trans_mat = (
            behavior_alignment_trans_mat_3x3 @ muscle2behavior_trans_mat
        )

        # Apply perspective warp
        out_image = cv2.warpPerspective(in_image, composed_trans_mat, output_dim)
    else:
        # Affine case: both are 2x3, compose them
        # Convert 2x3 transform matrices to 3x3 homogeneous matrices for easier composition
        muscle2behavior_trans_mat_3x3 = np.vstack(
            [muscle2behavior_trans_mat, [0, 0, 1]]
        )
        behavior_alignment_trans_mat_3x3 = np.vstack(
            [behavior_alignment_trans_mat, [0, 0, 1]]
        )

        # Compose transforms: first muscle->behavior, then apply the same alignment
        # transform that was applied to the behavior frame.
        composed_trans_mat = (
            behavior_alignment_trans_mat_3x3 @ muscle2behavior_trans_mat_3x3
        )
        assert np.allclose(composed_trans_mat[2, :], [0, 0, 1])

        # Convert back to 2x3 for cv2.warpAffine
        composed_trans_mat = composed_trans_mat[:2, :]

        # Apply affine warp
        out_image = cv2.warpAffine(in_image, composed_trans_mat, output_dim)

    # Save output image
    cv2.imwrite(str(output_path), out_image, _imwrite_compression_params)

    if return_output:
        return out_image
    else:
        # don't return anything - might help reduce IO traffic if called in parallel
        return None


def _make_muscle_metadata_dataframe(
    stage_pos_df_at_muscle_frames, acquired_time_us, received_time_us
):
    # ``corresponding_behavior_frame_id`` is drop-aware: it comes from the behavior group
    # each muscle frame was mapped to (see warp_all_muscle_frames_to_behavior), so it is
    # authoritative even when a mid-recording drop breaks the nominal muscle_id * sync
    # relationship. Downstream consumers should read this column rather than recompute it.
    num_frames = len(stage_pos_df_at_muscle_frames)
    muscle_frameids = np.arange(num_frames)
    behavior_frameids = stage_pos_df_at_muscle_frames["behavior_frame_id"].values
    x_pos_mm_interp = stage_pos_df_at_muscle_frames["x_pos_mm_interp"].values
    y_pos_mm_interp = stage_pos_df_at_muscle_frames["y_pos_mm_interp"].values

    return pd.DataFrame(
        data={
            "muscle_frame_id": muscle_frameids.astype(np.uint32),
            "corresponding_behavior_frame_id": behavior_frameids.astype(np.uint32),
            "x_pos_mm_interp": x_pos_mm_interp.astype(np.float32),
            "y_pos_mm_interp": y_pos_mm_interp.astype(np.float32),
            "acquired_time_us": np.asarray(acquired_time_us, dtype=np.uint64),
            "received_time_us": np.asarray(received_time_us, dtype=np.uint64),
        }
    )


def match_muscle_frameid_to_behavior_frameid(
    muscle_frameid: int | list[int],
    *,
    sync_ratio: int | None = None,
    offset: int = MUSCLE_BEHAVIOR_OFFSET,
    experiment_parameters_path: Path | None = None,
    recording_dir: Path | None = None,
):
    """Map muscle frame ID or IDs to corresponding behavior frame ID(s) using the
    provided synchronization ratio.

    (Re-indexed) muscle frame N corresponds to behavior frame
    (N + ``offset``) * sync_ratio. With the default offset of 0, muscle frame 0 is
    exposed simultaneously with behavior frame 0 (the excitation-LED frame).

    Args:
        muscle_frameid: Re-indexed muscle frame ID(s) (0-based, as stored in
            muscle_frames_metadata.csv after orphan frames at the start have been
            excluded).
        offset: Muscle-to-behavior readout lag in muscle-frame periods. Defaults to
            ``MUSCLE_BEHAVIOR_OFFSET``; see that constant for the full rationale.
    """
    if sync_ratio is None:
        sync_ratio = get_behavior_muscle_sync_ratio(
            experiment_parameters_path=experiment_parameters_path,
            recording_dir=recording_dir,
        )

    is_singleton_int = isinstance(muscle_frameid, (int, np.integer))
    if is_singleton_int:
        muscle_frameid = [muscle_frameid]

    behavior_frameid = [(mfid + offset) * sync_ratio for mfid in muscle_frameid]

    if is_singleton_int:
        behavior_frameid = behavior_frameid[0]
    return behavior_frameid


def match_behavior_frameid_to_muscle_frameid(
    behavior_frameid: int | list[int],
    method: str,
    *,
    sync_ratio: int | None = None,
    offset: int = MUSCLE_BEHAVIOR_OFFSET,
    experiment_parameters_path: Path | None = None,
    recording_dir: Path | None = None,
):
    """Map behavior frame ID or IDs to corresponding muscle frame ID(s) using the
    provided synchronization ratio.

    Because there are more behavior frames than muscle frames, multiple behavior frames
    will correspond to the same muscle frame. The selection of which muscle frame to
    return is controlled by the `method` argument:
    - "floor": use the last available muscle frame.
    - "nearest": use the temporally closest muscle frame (might be in the future).

    The inverse of ``match_muscle_frameid_to_behavior_frameid``: with the default
    offset of 0, behavior frame ``b`` maps to muscle frame ``b / sync_ratio``
    (floored or rounded per ``method``).

    Args:
        offset: Must match the value used in
            ``match_muscle_frameid_to_behavior_frameid`` (defaults to
            ``MUSCLE_BEHAVIOR_OFFSET``).
    """
    if sync_ratio is None:
        sync_ratio = get_behavior_muscle_sync_ratio(
            experiment_parameters_path=experiment_parameters_path,
            recording_dir=recording_dir,
        )
    if method.lower() not in ["floor", "nearest"]:
        raise ValueError(
            f"Invalid method '{method}'. Supported methods are 'floor' and 'nearest'."
        )

    is_singleton_int = isinstance(behavior_frameid, (int, np.integer))
    if is_singleton_int:
        behavior_frameid = [behavior_frameid]

    muscle_frameid = []
    for bfid in behavior_frameid:
        if method == "floor":
            mfid = int((bfid - offset * sync_ratio) / sync_ratio)
        elif method == "nearest":
            mfid = round((bfid - offset * sync_ratio) / sync_ratio)
        muscle_frameid.append(mfid)

    if is_singleton_int:
        muscle_frameid = muscle_frameid[0]
    return muscle_frameid


# if __name__ == "__main__":
#     logging.basicConfig(
#         level=logging.DEBUG, format="%(asctime)s - %(levelname)s - %(message)s"
#     )

#     # fmt: off
#     recording_dir = Path("~/data/spotlight/20250613-fly1b-002/").expanduser()
#     map_muscle_frames_to_behavior(
#         muscle_calibration_path=recording_dir / "metadata/calibration_parameters_muscle.yaml",
#         behavior_calibration_path=recording_dir / "metadata/calibration_parameters_behavior.yaml",
#         experiment_parameters_path=recording_dir / "metadata/experiment_parameters.yaml",
#         processed_behavior_frame_metadata_path=recording_dir / "processed/behavior_frames_metadata.csv",
#         behavior_alignment_metadata_path=recording_dir / "processed/behavior_alignment_transforms.h5",
#         raw_muscle_images_dir=recording_dir / "muscle_images/",
#         transformed_muscle_images_output_dir=recording_dir / "processed/muscle_images/",
#         muscle_metadata_output_path=recording_dir / "processed/muscle_frames_metadata.csv",
#     )
#     # fmt: on
