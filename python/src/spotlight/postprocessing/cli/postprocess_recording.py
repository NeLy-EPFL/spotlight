"""CLI for post-processing one Spotlight recording. See help message for
details.

Pipeline (each stage's own params group below): alignment (+ pose2d, muscle,
fused into one streaming pass, see `behavior.process_behavior_pipeline`)
-> inverse-kinematics -> physics-replay -> summary-video. `--start-from`
resumes partway through this chain, reusing whichever earlier stages'
outputs are already on disk instead of recomputing them.
"""

import logging
import os
import shutil
import sys
import time
from dataclasses import dataclass, field
from fractions import Fraction
from pathlib import Path
from typing import Annotated, Literal

import tyro
import yaml

from spotlight.postprocessing.behavior import process_behavior_pipeline
from spotlight.postprocessing.common.frame_range import (
    resolve_frame_range_to_file_slice,
)
from spotlight.postprocessing.common.parallel import resolve_num_workers
from spotlight.postprocessing.common.smoothing import (
    seconds_to_frames,
    seconds_to_odd_frames,
)
from spotlight.postprocessing.replay.constants import (
    DEFAULT_ACTUATOR_GAIN,
    DEFAULT_ADHESION_GAIN,
    DEFAULT_FALL_TILT_THRESHOLD_DEG,
    DEFAULT_TARSAL_STIFFNESS,
)
from spotlight.postprocessing.muscle import (
    plan_muscle_behavior_mapping,
    restrict_mapping_to_frame_range,
    save_muscle_metadata_csv,
)
from spotlight.postprocessing.stage import interp_stage_pos_at_behavior_frames
from spotlight.postprocessing.visualize import generate_summary_video
from spotlight.common import get_assets_dir

sys.stdout = os.fdopen(sys.stdout.fileno(), "w", buffering=1)

POSE2D_SKELETON_JSON_PATH = (
    get_assets_dir() / "models" / "pose2d_skeleton_metadata.json"
)
DEFAULT_LOCALIZATION_MODEL_PATH = get_assets_dir() / "models" / "localization_model.pt"
DEFAULT_POSE2D_MODEL_PATH = get_assets_dir() / "models" / "pose2d_model.pt"

StartFrom = Literal["start", "inverse-kinematics", "physics-replay"]


@dataclass
class BehaviorVideoParams:
    """The aligned/full-size behavior video(s) themselves (encoding only,
    see `AlignmentParams` for what gets cropped/aligned)."""

    playback_speed: float = 0.1
    """Output videos play at this fraction of the recording's real-time
    rate. The achieved fps is rounded to a nearby value ffmpeg can encode
    cleanly, so actual speed may differ slightly; the QA video's on-screen
    speed label always shows the real value."""

    crf: int = 12
    """Encode quality (CRF). Lower is higher quality and a larger file."""

    preset: str = "medium"
    """Encoder preset. "medium" measured faster and smaller than "slow"
    here with no visible quality loss."""


@dataclass
class AlignmentParams:
    """Thorax-crop alignment. Also owns the raw-frame decode this pipeline's
    pose2d/muscle stages piggyback on (see `behavior.process_behavior_pipeline`),
    since decoding/localizing/aligning/pose2d/muscle-warping all happen in
    one streaming pass over the raw frames, not as independent stages."""

    mode: Literal["aligned", "fullsize", "both"] = "aligned"
    """Which behavior video(s) to produce: the thorax-aligned crop, the raw
    full-size frame, or both."""

    crop_dim: int = 900
    """Aligned-frame output size, in pixels (`crop_dim` x `crop_dim`).
    Ignored if `mode` is "fullsize"."""

    thorax_y_normalized: float = 0.5
    """Where the thorax sits on the crop's y-axis: 0 is the top, 1 is the
    bottom."""

    localization_model: Path = DEFAULT_LOCALIZATION_MODEL_PATH
    """Localization model checkpoint."""

    localization_batch_size: int | Literal["auto"] = "auto"
    """Localization model batch size. "auto" (default) scales with the
    GPU's VRAM, about 512 on a 12 GB GPU."""

    flip_confidence_threshold: float = 0.5
    """Localization model's flip-probability cutoff. At or above this, a
    frame is treated as flipped."""

    flip_denoise_window_sec: float = 0.05
    """QA video only: smooths the flip decision over this many seconds
    (converted to an odd frame count via `behavior_fps`), so a single noisy
    frame doesn't flicker the display. -1 disables smoothing."""

    heading_denoise_sigma_sec: float = 0.015
    """Gaussian smoothing (seconds, converted to frames via `behavior_fps`)
    applied to the fly's fitted heading before cropping: a real change to
    the aligned crop itself, not just a display effect. -1 disables."""

    position_denoise_sigma_sec: float = 0.005
    """Gaussian smoothing (seconds) applied to the thorax position used to
    center each frame's crop (same real-not-display caveat as
    `heading_denoise_sigma_sec`). -1 disables."""

    transform_workers: int | Literal["auto"] = "auto"
    """CPU worker count feeding the localization/pose2d models: raw-frame
    decode plus muscle-image warping (both plain cv2, no GPU). "auto"
    (default) uses `$SLURM_CPUS_PER_TASK` if set, else all cores."""


@dataclass
class MuscleParams:
    """Muscle-imaging channel."""

    mode: Literal["on", "off", "auto"] = "auto"
    """"auto" (default) processes the muscle channel if `muscle_images/`
    exists in the recording, else skips it silently. "on" processes it
    unconditionally, erroring out if the folder is missing. "off" always
    skips it."""

    max_missing_frames: int = 3
    """Max unexplained missing muscle frames tolerated before raising an
    error."""

    orphan_frames: int | Literal["auto"] = "auto"
    """Number of leading muscle frames to skip as recorded-before-behavior-
    started orphans. "auto" (default) detects this from the frames
    themselves."""

    homography_path: Path | Literal["native"] = "native"
    """Homography calibration file for muscle-to-behavior frame mapping.
    "native" (default) uses `metadata/homography_parameters.yaml` in the
    recording directory."""


@dataclass
class Pose2DParams:
    """2D pose estimation, on the aligned crop."""

    enabled: bool = False
    """Run 2D pose estimation. Requires `--alignment.mode` to be "aligned"
    or "both"."""

    output_path: Path | None = None
    """Where to save the dense per-frame 2D pose predictions. Defaults to
    `pose2d.h5` under the recording's `postprocessed/` directory."""

    model: Path = DEFAULT_POSE2D_MODEL_PATH
    """2D pose model checkpoint."""

    batch_size: int | Literal["auto"] = "auto"
    """2D pose model batch size. "auto" (default) scales with the GPU's
    VRAM, about 256 on a 12 GB GPU."""

    min_confidence: float = 0.5
    """QA video only: below this weighted keypoint confidence, a frame's
    2D pose/IK overlay is hidden (the frame itself still shows)."""

    confidence_denoise_window_sec: float = 0.05
    """QA video only: smooths the confidence-acceptance decision (a binary
    mask, via morphological opening/closing; the confidence values
    themselves are never filtered) over this many seconds, converted to an
    odd frame count via `behavior_fps`. -1 disables smoothing."""


@dataclass
class InverseKinematicsParams:
    """Fit inverse kinematics (QuickIK) from the 2D pose. Requires
    `--pose2d.enabled`."""

    enabled: bool = False
    """Fit inverse kinematics."""

    output_path: Path | None = None
    """Where to save the IK fit. Defaults to `inverse_kinematics.h5` under
    the recording's `postprocessed/` directory."""

    dof_prior_weight: float = 0.2
    """IK solver's pull toward the body plan's neutral pose. Higher trusts
    the neutral pose over the observed keypoints more."""

    upright_prior_weight: float = 0.1
    """Reserved for a future upright-body prior; currently unused."""

    max_mismatch: float = 0.3
    """QA video only: max mismatch (mm) allowed between the 2D prediction
    and the IK fit before its display-acceptance mask (`mismatch_mask`,
    saved alongside the fit itself) rejects that frame."""

    mismatch_denoise_window_sec: float = 0.05
    """`mismatch_mask` is denoised over this many seconds (converted to an
    odd frame count via `behavior_fps`); both this and `max_mismatch` are
    saved as attrs alongside the fit."""

    viz_heading_denoise_sigma_sec: float = 0.015
    """QA video only: Gaussian smoothing (seconds, converted to frames via
    `behavior_fps`) applied to the synthetic 3D IK panel's own camera, so
    keypoint noise doesn't jitter its yaw-tracking. Display-only: unlike
    `--alignment.heading-denoise-sigma-sec`, this never touches the real
    IK fit. -1 disables smoothing."""


@dataclass
class PhysicsReplayParams:
    """Replay the solved IK joint angles in FlyGym (CPU physics), for the
    QA video's row-2 replay panel. Requires `--inverse-kinematics.enabled`."""

    enabled: bool = False
    """Run the physics replay."""

    output_path: Path | None = None
    """Where to save the replay. Defaults to `physics_replay.h5` under the
    recording's `postprocessed/` directory."""

    actuator_gain: float = DEFAULT_ACTUATOR_GAIN
    """Position-actuator gain (uN*mm/rad) on the leg DOFs."""

    tarsus_stiffness: float = DEFAULT_TARSAL_STIFFNESS
    """Passive spring stiffness of the (unactuated) tarsal joints."""

    adhesion_force: float = DEFAULT_ADHESION_GAIN
    """Leg adhesion force (uN)."""

    max_tilt: float = DEFAULT_FALL_TILT_THRESHOLD_DEG
    """Restart the physics state (the mechanism for recovering a fallen
    fly) once the thorax tips this many degrees from vertical."""

    viz_heading_denoise_sigma_sec: float = 0.015
    """Display-only: Gaussian smoothing (seconds, converted to frames via
    `behavior_fps`) applied to the replay camera's own yaw-tracking, so
    keypoint/physics noise doesn't jitter it; it never touches the physics
    itself. Costs a second, renderer-less physics pass per period. -1
    disables smoothing (and that second pass)."""

    workers: int | Literal["auto"] = "auto"
    """One IK period per task, across this many workers. "auto" (default)
    uses `$SLURM_CPUS_PER_TASK` if set, else all cores."""


@dataclass
class SummaryVideoParams:
    """The two-row QA panel-grid video."""

    enabled: bool = True
    """Generate the summary video."""

    force_redo: bool = False
    """Regenerate the summary video even if `output_path` already exists."""

    output_path: Path | None = None
    """Where to save the summary video. Defaults to `summary_video.mp4`
    under the recording's `postprocessed/` directory."""

    crf: int = 21
    """Encode quality (CRF). Lower is higher quality and a larger file."""

    preset: str = "medium"
    """Encoder preset."""

    muscle_vrange: tuple[int, int] | Literal["auto"] = "auto"
    """Muscle-image intensity range (min, max) mapped onto `muscle_colormap`.
    "auto" (default) computes this from a percentile sample of the data.
    Ignored if muscle processing is skipped."""

    muscle_colormap: str = "lilac"
    """Muscle-panel colormap: a cmasher name (e.g. "lilac") or a
    matplotlib-registered name (e.g. "viridis")."""

    profile: bool = False
    """Profile the video's first render chunk with cProfile and save it to
    `viz_chunk_profile.prof` (inspect with e.g. `snakeviz`)."""

    compose_workers: int | Literal["auto"] = "auto"
    """CPU threads for compositing (not encoding) frames. "auto" (default)
    uses `$SLURM_CPUS_PER_TASK` if set, else all cores."""


@dataclass
class VideoCodecParams:
    """Encoding backend, shared by the behavior video and the summary
    video (decoding has no GPU path to select, see `pvio`'s own docs)."""

    prefer_gpu: bool = True
    """Try GPU (NVENC) encoding, falling back to CPU (libx264) if
    unavailable."""

    nvenc_sessions: int = 8
    """Concurrent NVENC sessions the GPU driver allows; the summary
    video's encode-worker count is capped a couple below this for margin.
    Ignored if `prefer_gpu` is False."""


@dataclass
class PostprocessingParams:
    behavior_video: BehaviorVideoParams = field(default_factory=BehaviorVideoParams)
    alignment: AlignmentParams = field(default_factory=AlignmentParams)
    muscle: MuscleParams = field(default_factory=MuscleParams)
    pose2d: Pose2DParams = field(default_factory=Pose2DParams)
    inverse_kinematics: InverseKinematicsParams = field(
        default_factory=InverseKinematicsParams
    )
    physics_replay: PhysicsReplayParams = field(default_factory=PhysicsReplayParams)
    summary_video: SummaryVideoParams = field(default_factory=SummaryVideoParams)
    video_codec: VideoCodecParams = field(default_factory=VideoCodecParams)

    log_level: str = "info"
    """Logging verbosity, e.g. "debug", "info", "warning"."""

    overwrite: bool = False
    """Required to proceed at all if `postprocessed/` already exists (this
    CLI never silently reuses a directory)."""

    frame_range: tuple[int, int] | None = None
    """Restrict processing to real frames `[start, end)` (rounded outward
    to whole raw files, so up to 2 frames wider than requested on either
    side), for quick iteration on a slice of a trial, not full runs."""

    start_from: StartFrom = "start"
    """Resume partway through the pipeline, reusing earlier stages'
    already-saved outputs instead of recomputing them: "inverse-kinematics"
    skips alignment/pose2d/muscle (requires their outputs to already
    exist); "physics-replay" additionally skips inverse kinematics
    (requires `inverse_kinematics.h5` to already exist)."""


def _validate_inputs(recording_dir: Path, params: PostprocessingParams) -> None:
    """Fast, upfront logical/file-existence checks, run before any heavy
    (GPU/decode) work starts, so a misconfigured invocation fails
    immediately instead of after minutes of wasted processing."""
    errors = []

    if params.pose2d.enabled and params.alignment.mode == "fullsize":
        errors.append("--pose2d.enabled requires --alignment.mode aligned or both.")
    if params.inverse_kinematics.enabled and not params.pose2d.enabled:
        errors.append("--inverse-kinematics.enabled requires --pose2d.enabled.")
    if params.physics_replay.enabled and not params.inverse_kinematics.enabled:
        errors.append("--physics-replay.enabled requires --inverse-kinematics.enabled.")

    if params.muscle.mode == "on" and not (recording_dir / "muscle_images").is_dir():
        errors.append(
            f"--muscle.mode on requires {recording_dir / 'muscle_images'} to exist."
        )
    if not (recording_dir / "behavior_images").is_dir():
        errors.append(f"{recording_dir / 'behavior_images'} does not exist.")

    metadata_dir = recording_dir / "metadata"
    required_metadata = ["experiment_parameters.yaml"]
    if (
        params.muscle.mode in ("on", "auto")
        and (recording_dir / "muscle_images").is_dir()
    ):
        if params.muscle.homography_path == "native":
            required_metadata.append("homography_parameters.yaml")
    for name in required_metadata:
        if not (metadata_dir / name).is_file():
            errors.append(f"{metadata_dir / name} does not exist.")

    if params.start_from != "start":
        postprocessed_dir = recording_dir / "postprocessed"
        required_outputs = [postprocessed_dir / _pose2d_output_name(params)]
        if params.alignment.mode in ("aligned", "both"):
            required_outputs.append(postprocessed_dir / "aligned_behavior_video.mp4")
        if params.alignment.mode == "fullsize":
            required_outputs.append(postprocessed_dir / "fullsize_behavior_video.mp4")
        if params.start_from == "physics-replay":
            required_outputs.append(postprocessed_dir / _ik_output_name(params))
        for path in required_outputs:
            if not path.exists():
                errors.append(f"--start-from {params.start_from} requires {path}.")

    if errors:
        raise SystemExit(
            "Invalid configuration:\n" + "\n".join(f"  - {e}" for e in errors)
        )


def _pose2d_output_name(params: PostprocessingParams) -> str:
    return params.pose2d.output_path.name if params.pose2d.output_path else "pose2d.h5"


def _ik_output_name(params: PostprocessingParams) -> str:
    return (
        params.inverse_kinematics.output_path.name
        if params.inverse_kinematics.output_path
        else "inverse_kinematics.h5"
    )


def postprocess_recording_data(
    recording_dir: tyro.conf.Positional[Path],
    behavior_video: BehaviorVideoParams = BehaviorVideoParams(),  # noqa: B008
    alignment: AlignmentParams = AlignmentParams(),  # noqa: B008
    muscle: MuscleParams = MuscleParams(),  # noqa: B008
    pose2d: Pose2DParams = Pose2DParams(),  # noqa: B008
    inverse_kinematics: InverseKinematicsParams = InverseKinematicsParams(),  # noqa: B008
    physics_replay: PhysicsReplayParams = PhysicsReplayParams(),  # noqa: B008
    summary_video: SummaryVideoParams = SummaryVideoParams(),  # noqa: B008
    video_codec: VideoCodecParams = VideoCodecParams(),  # noqa: B008
    log_level: Annotated[
        str, tyro.conf.arg(help='Logging verbosity, e.g. "debug", "info", "warning".')
    ] = "info",
    overwrite: Annotated[
        bool,
        tyro.conf.arg(
            help="Required to proceed at all if `postprocessed/` already exists "
            "(this CLI never silently reuses a directory)."
        ),
    ] = False,
    frame_range: Annotated[
        tuple[int, int] | None,
        tyro.conf.arg(
            help="Restrict processing to real frames [start, end) (rounded "
            "outward to whole raw files, so up to 2 frames wider than "
            "requested on either side), for quick iteration on a slice of "
            "a trial, not full runs."
        ),
    ] = None,
    start_from: Annotated[
        StartFrom,
        tyro.conf.arg(
            help="Resume partway through the pipeline, reusing earlier stages' "
            "already-saved outputs instead of recomputing them: "
            "'inverse-kinematics' skips alignment/pose2d/muscle (requires "
            "their outputs to already exist); 'physics-replay' additionally "
            "skips inverse kinematics (requires inverse_kinematics.h5 to "
            "already exist)."
        ),
    ] = "start",
) -> None:
    """High-level post-processing pipeline for a single Spotlight recording.

    Args:
        recording_dir: Path to the recording directory created by the
            Spotlight recorder program.
        See each other group's own fields (`--help`) for details.
    """
    params = PostprocessingParams(
        behavior_video=behavior_video, alignment=alignment, muscle=muscle,
        pose2d=pose2d, inverse_kinematics=inverse_kinematics,
        physics_replay=physics_replay, summary_video=summary_video,
        video_codec=video_codec, log_level=log_level, overwrite=overwrite,
        frame_range=frame_range, start_from=start_from,
    )  # fmt: skip
    t_start = time.perf_counter()
    numeric_level = getattr(logging, params.log_level.upper(), None)
    if not isinstance(numeric_level, int):
        raise ValueError(f"Invalid log level: {params.log_level}")
    logging.basicConfig(
        level=numeric_level, format="%(asctime)s - %(levelname)s - %(message)s",
        force=True,
    )  # fmt: skip
    logger = logging.getLogger(__name__)

    recording_dir = Path(recording_dir)
    _validate_inputs(recording_dir, params)

    postprocessed_dir = recording_dir / "postprocessed"
    if postprocessed_dir.exists() and not params.overwrite:
        raise FileExistsError(
            f"{postprocessed_dir} already exists. Use --overwrite to proceed."
        )
    postprocessed_dir.mkdir(exist_ok=True, parents=True)

    metadata_dir = recording_dir / "metadata"
    postprocessed_metadata_dir = postprocessed_dir / "metadata"
    if not postprocessed_metadata_dir.exists():
        shutil.copytree(metadata_dir, postprocessed_metadata_dir)

    experiment_parameters = yaml.safe_load(
        (metadata_dir / "experiment_parameters.yaml").read_text()
    )
    behavior_fps = float(experiment_parameters["behavior_fps"])
    # Playback speed for every output video (behavior and summary alike)
    # tracks the recording's own rate, not real time. The exact fps this
    # implies is usually not an integer; approximating it as a small
    # fraction (rather than just rounding to the nearest integer fps) gets
    # much closer to the requested speed while still landing on a fps
    # ffmpeg encodes cleanly.
    play_fps = float(
        Fraction(params.behavior_video.playback_speed * behavior_fps).limit_denominator(20)
    )  # fmt: skip
    actual_playback_speed = play_fps / behavior_fps
    logger.info(
        f"behavior_fps={behavior_fps}, "
        f"playback_speed={params.behavior_video.playback_speed} -> "
        f"play_fps={play_fps} (actual_playback_speed={actual_playback_speed:.4f})"
    )

    run_muscle = params.muscle.mode == "on" or (
        params.muscle.mode == "auto" and (recording_dir / "muscle_images").is_dir()
    )
    logger.info(f"Muscle processing: {'on' if run_muscle else 'off'} "
                f"(--muscle.mode {params.muscle.mode})")  # fmt: skip

    encode_mode = "auto" if params.video_codec.prefer_gpu else "cpu"
    transform_workers = resolve_num_workers(params.alignment.transform_workers)

    raw_behavior_images_dir = recording_dir / "behavior_images"
    all_raw_paths = sorted(raw_behavior_images_dir.glob("behavior_frame_*.jpg"))
    file_start, file_end = resolve_frame_range_to_file_slice(
        params.frame_range, len(all_raw_paths)
    )
    raw_behavior_paths = all_raw_paths[file_start:file_end]

    stage_positions_path = recording_dir / "stage_position/stage_position.csv"
    behavior_frames_metadata_path = postprocessed_dir / "behavior_frames_metadata.csv"
    alignment_metadata_path = postprocessed_dir / "behavior_alignment_transforms.h5"

    write_aligned = params.alignment.mode in ("aligned", "both")
    write_fullsize = params.alignment.mode in ("fullsize", "both")
    aligned_video_path = (
        postprocessed_dir / "aligned_behavior_video.mp4" if write_aligned else None
    )
    fullsize_video_path = (
        postprocessed_dir / "fullsize_behavior_video.mp4" if write_fullsize else None
    )
    pose2d_h5_path = (
        params.pose2d.output_path or postprocessed_dir / "pose2d.h5"
        if params.pose2d.enabled
        else None
    )
    inverse_kinematics_h5_path = (
        params.inverse_kinematics.output_path
        or postprocessed_dir / "inverse_kinematics.h5"
        if params.inverse_kinematics.enabled
        else None
    )
    physics_replay_h5_path = (
        params.physics_replay.output_path or postprocessed_dir / "physics_replay.h5"
        if params.physics_replay.enabled
        else None
    )
    summary_video_output_path = (
        params.summary_video.output_path or postprocessed_dir / "summary_video.mp4"
    )

    muscle_dataset_name = (
        "aligned_muscle_images" if write_aligned else "fullsize_muscle_images"
    )
    muscle_h5_path = postprocessed_dir / f"{muscle_dataset_name}.h5"

    flipped_prob = None
    if params.start_from == "start":
        t_step = time.perf_counter()
        logger.info("Interpolating stage positions for behavior frames...")
        interp_stage_pos_at_behavior_frames(
            frames_dir=raw_behavior_images_dir,
            stage_positions_path=stage_positions_path,
            output_path=behavior_frames_metadata_path,
        )
        logger.info(f"STEP TIME stage_interp: {time.perf_counter() - t_step:.1f}s")

        muscle_mapping = None
        if run_muscle:
            t_step = time.perf_counter()
            logger.info("Planning muscle<->behavior frame mapping...")
            output_dim = (
                (params.alignment.crop_dim, params.alignment.crop_dim)
                if write_aligned
                else None
            )
            if output_dim is None:
                # fullsize-only: output_dim is the raw frame's own size.
                import cv2

                sample = cv2.imread(str(raw_behavior_paths[0]))
                output_dim = (sample.shape[1], sample.shape[0])
            homography_path = (
                metadata_dir / "homography_parameters.yaml"
                if params.muscle.homography_path == "native"
                else Path(params.muscle.homography_path)
            )
            muscle_mapping = plan_muscle_behavior_mapping(
                raw_muscle_images_dir=recording_dir / "muscle_images",
                experiment_parameters_path=metadata_dir / "experiment_parameters.yaml",
                behavior_frames_metadata_path=behavior_frames_metadata_path,
                homography_path=homography_path,
                output_dim=output_dim,
                missing_muscle_frames_tolerance=params.muscle.max_missing_frames,
                num_orphan_muscle_frames=(
                    None
                    if params.muscle.orphan_frames == "auto"
                    else params.muscle.orphan_frames
                ),
            )
            if params.frame_range is not None:
                muscle_mapping = restrict_mapping_to_frame_range(
                    muscle_mapping, file_start * 3, file_end * 3
                )
            logger.info(
                f"STEP TIME muscle_planning: {time.perf_counter() - t_step:.1f}s"
            )

        t_step = time.perf_counter()
        logger.info(f"Processing {len(raw_behavior_paths)} behavior frames "
                    "(decode + localize align + pose2d + muscle, streaming)...")  # fmt: skip
        behavior_result = process_behavior_pipeline(
            raw_behavior_frame_paths=raw_behavior_paths,
            localization_checkpoint_path=params.alignment.localization_model,
            alignment=params.alignment.mode,
            thorax_y_normalized=params.alignment.thorax_y_normalized,
            crop_dim=params.alignment.crop_dim,
            output_aligned_video_path=aligned_video_path,
            output_fullsize_video_path=fullsize_video_path,
            output_alignment_metadata_path=alignment_metadata_path,
            behavior_video_fps=play_fps,
            behavior_video_crf=params.behavior_video.crf,
            behavior_video_preset=params.behavior_video.preset,
            localization_batch_size=params.alignment.localization_batch_size,
            run_pose2d=params.pose2d.enabled,
            pose2d_checkpoint_path=params.pose2d.model,
            pose2d_skeleton_json_path=POSE2D_SKELETON_JSON_PATH,
            pose2d_batch_size=params.pose2d.batch_size,
            output_pose2d_h5_path=pose2d_h5_path,
            run_muscle=run_muscle,
            muscle_mapping=muscle_mapping,
            output_muscle_h5_path=muscle_h5_path,
            muscle_dataset_name=muscle_dataset_name,
            num_cpu_workers=transform_workers,
            encode_mode=encode_mode,
            position_denoise_sigma=seconds_to_frames(
                params.alignment.position_denoise_sigma_sec, behavior_fps
            ),
            heading_denoise_sigma=seconds_to_frames(
                params.alignment.heading_denoise_sigma_sec, behavior_fps
            ),
        )
        logger.info(
            f"STEP TIME behavior_pipeline_total: {time.perf_counter() - t_step:.1f}s"
        )
        if run_muscle:
            save_muscle_metadata_csv(
                muscle_mapping, postprocessed_dir / "muscle_frames_metadata.csv"
            )
        flipped_prob = behavior_result["flipped_prob"]
    else:
        logger.info(
            f"--start-from {params.start_from}: skipping alignment/pose2d/muscle, "
            "reusing existing outputs."
        )

    if params.inverse_kinematics.enabled and params.start_from != "physics-replay":
        t_step = time.perf_counter()
        logger.info("Solving inverse kinematics...")
        try:
            from spotlight.postprocessing.invkin.solve_ik import solve_ik
        except ImportError as e:
            raise SystemExit(
                "--inverse-kinematics.enabled requires the postprocessing "
                "extra's IK dependencies (quickik). Install with "
                "`uv sync --extra postprocessing` (from `python/`)."
            ) from e
        solve_ik(
            input_path=pose2d_h5_path,
            output_path=inverse_kinematics_h5_path,
            neutral_weight=params.inverse_kinematics.dof_prior_weight,
            upright_prior_weight=params.inverse_kinematics.upright_prior_weight,
            flip_confidence_threshold=params.alignment.flip_confidence_threshold,
            min_confidence=params.pose2d.min_confidence,
            max_mismatch=params.inverse_kinematics.max_mismatch,
            mismatch_denoise_window_sec=params.inverse_kinematics.mismatch_denoise_window_sec,
            frame_start=file_start * 3,
            override=True,
        )
        logger.info(f"STEP TIME solve_ik: {time.perf_counter() - t_step:.1f}s")

    if params.physics_replay.enabled:
        t_step = time.perf_counter()
        logger.info("Replaying IK in FlyGym...")
        try:
            from spotlight.postprocessing.replay.physics import replay_physics
        except ImportError as e:
            raise SystemExit(
                "--physics-replay.enabled requires the postprocessing "
                "extra's replay dependencies (flygym). Install with "
                "`uv sync --extra postprocessing` (from `python/`)."
            ) from e
        replay_physics(
            input_path=inverse_kinematics_h5_path,
            output_path=physics_replay_h5_path,
            behavior_fps=behavior_fps,
            actuator_gain=params.physics_replay.actuator_gain,
            tarsal_stiffness=params.physics_replay.tarsus_stiffness,
            adhesion_gain=params.physics_replay.adhesion_force,
            fall_tilt_threshold_deg=params.physics_replay.max_tilt,
            viz_heading_denoise_sigma_sec=params.physics_replay.viz_heading_denoise_sigma_sec,
            num_workers=params.physics_replay.workers,
            override=True,
        )
        logger.info(f"STEP TIME replay_physics: {time.perf_counter() - t_step:.1f}s")

    summary_video_total_s = None
    if params.summary_video.enabled:
        if summary_video_output_path.exists() and not params.summary_video.force_redo:
            logger.info(
                f"{summary_video_output_path} already exists; skipping "
                "(--summary-video.force-redo to regenerate)."
            )
        else:
            t_step = time.perf_counter()
            logger.info("Generating summary video...")
            generate_summary_video(
                recording_dir=recording_dir,
                postprocessed_dir=postprocessed_dir,
                alignment=params.alignment.mode,
                crop_dim=params.alignment.crop_dim,
                with_muscle=run_muscle,
                with_pose2d=params.pose2d.enabled,
                with_ik=params.inverse_kinematics.enabled,
                with_replay=params.physics_replay.enabled,
                behavior_fps=behavior_fps,
                flipped_prob=flipped_prob,
                muscle_h5_path=muscle_h5_path if run_muscle else None,
                muscle_dataset_name=muscle_dataset_name if run_muscle else None,
                pose2d_h5_path=pose2d_h5_path,
                inverse_kinematics_h5_path=inverse_kinematics_h5_path,
                physics_replay_h5_path=physics_replay_h5_path,
                pose2d_skeleton_json_path=POSE2D_SKELETON_JSON_PATH,
                output_path=summary_video_output_path,
                muscle_vrange=(
                    None
                    if params.summary_video.muscle_vrange == "auto"
                    else params.summary_video.muscle_vrange
                ),
                play_fps=play_fps,
                playback_speed=actual_playback_speed,
                crf=params.summary_video.crf,
                preset=params.summary_video.preset,
                flip_confidence_threshold=params.alignment.flip_confidence_threshold,
                flip_denoise_window=seconds_to_odd_frames(
                    params.alignment.flip_denoise_window_sec, behavior_fps
                ),
                confidence_denoise_window=seconds_to_odd_frames(
                    params.pose2d.confidence_denoise_window_sec, behavior_fps
                ),
                viz_heading_denoise_sigma=seconds_to_frames(
                    params.inverse_kinematics.viz_heading_denoise_sigma_sec,
                    behavior_fps,
                ),
                num_encode_workers=(
                    max(1, params.video_codec.nvenc_sessions - 2)
                    if params.video_codec.prefer_gpu
                    else transform_workers
                ),
                num_compose_workers=resolve_num_workers(
                    params.summary_video.compose_workers
                ),
                frame_range=params.frame_range,
                profile=params.summary_video.profile,
                colormap=params.summary_video.muscle_colormap,
                prefer_gpu=params.video_codec.prefer_gpu,
            )
            summary_video_total_s = time.perf_counter() - t_step
            logger.info(f"STEP TIME summary_video_total: {summary_video_total_s:.1f}s")
    else:
        logger.info("--no-summary-video.enabled: skipping.")

    logger.info(
        "Walltime report: "
        + ", ".join(
            f"{name}={secs:.1f}s"
            for name, secs in [
                ("summary_video", summary_video_total_s),
                ("total", time.perf_counter() - t_start),
            ]
            if secs is not None
        )
    )
    logger.info(f"Done. Outputs under {postprocessed_dir}")


def main():
    tyro.cli(postprocess_recording_data)


if __name__ == "__main__":
    main()
