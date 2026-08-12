import logging
import os
import shutil
import subprocess
import sys
import time
from dataclasses import dataclass
from fractions import Fraction
from pathlib import Path
from typing import Literal

import tyro
import yaml

from spotlight_postprocessing.behavior import process_behavior_pipeline
from spotlight_postprocessing.muscle import (
    plan_muscle_behavior_mapping,
    save_muscle_metadata_csv,
)
from spotlight_postprocessing.stage import interp_stage_pos_at_behavior_frames
from spotlight_postprocessing.visualize import generate_summary_video

sys.stdout = os.fdopen(sys.stdout.fileno(), "w", buffering=1)

PYTHON_ROOT = Path(__file__).resolve().parents[3]  # python/
ORIENT_CHECKPOINT_PATH = PYTHON_ROOT / "bulk_data/orient_model/checkpoints/v13/best.pt"
POSE2D_CHECKPOINT_PATH = (
    PYTHON_ROOT / "bulk_data/pose2d_model/checkpoints/iter2b/best.pt"
)
POSE2D_SKELETON_JSON_PATH = (
    PYTHON_ROOT / "bulk_data/pose2d_model/skeleton_metadata.json"
)
SOLVE_IK_SCRIPT_PATH = PYTHON_ROOT / "scripts/spotlight_ik/solve_ik.py"


@dataclass
class PostprocessingParams:
    """Configuration for `postprocess_recording_data`. Pipeline: stage-position
    interpolation, behavior frame decode + TinyOrientModel alignment (+
    optional pose2d/muscle, run in the same streaming pass, no video
    round-trip), optional IK/FK fit, optional 5-panel QA visualization."""

    crop_dim: int = 900
    """Output aligned-frame dimensions (crop_dim x crop_dim)."""

    thorax_y_normalized: float = 0.5
    """Where the thorax sits in the crop's y-axis (0=top, 1=bottom)."""

    alignment: Literal["aligned", "fullsize", "both"] = "aligned"
    """Which behavior video(s) to produce."""

    playback_speed: float = 0.1
    """Output videos (behavior and visualization alike) play at this
    fraction of the recording's own real-time rate. The exact fps this
    implies (`playback_speed * behavior_fps`) is usually not an integer,
    so it's approximated as a fraction with denominator <= 20 (Python's
    `Fraction.limit_denominator`); the QA video's on-screen speed label
    shows the speed this approximation actually achieves, which can differ
    slightly from the requested value."""

    visualize: bool = True
    muscle: bool = True
    pose2d: bool = True
    ik: bool = True

    orient_batch_size: int | Literal["auto"] = "auto"
    """fp16 peak ~5.1GB on a 12GB GPU, measured (see Task #54 benchmark).
    "auto" (default) sizes it as `ORIENT_BATCH_SIZE_VRAM_FRACTION` (0.04)
    times the active GPU's total VRAM in MiB, so it scales up on bigger
    GPUs instead of leaving headroom unused -- ~512 on a 12 GB GPU,
    matching this pipeline's previously-hardcoded default."""

    pose2d_batch_size: int | Literal["auto"] = "auto"
    """fp16 peak ~4.1GB on a 12GB GPU, measured (see Task #54 benchmark).
    "auto" (default) sizes it as `POSE2D_BATCH_SIZE_VRAM_FRACTION` (0.02)
    times the active GPU's total VRAM in MiB -- ~256 on a 12 GB GPU,
    matching this pipeline's previously-hardcoded default."""

    ik_prior_weight: float = 0.2
    """Passed through as `solve_ik.py`'s `neutral_weight`."""

    ik_max_mismatch: float = 0.3
    """Max tolerated xy mismatch (mm) between the 2D prediction and the
    IK/FK fit before the QA video stops drawing the IK overlay for that
    frame. Purely a `visualize.py` display decision (see its module
    docstring); `kinematics.h5` itself always keeps whatever IK result
    `solve_ik.py` computed, mismatch included, for downstream curation in
    poseforge2."""

    flip_confidence_threshold: float = 0.5
    """Orient model's flip-probability cutoff for the QA video's flip
    decision (box color, and darkening panels 2-5): >= this counts as
    flipped. Raising it calls more frames not-flipped."""

    visualize_flip_mask_window: int = 5
    """Binary opening+closing window (frames) denoising the flip decision
    above before it's used, so a single noisy frame doesn't flicker the QA
    video's darkened panels. -1 disables this (use the raw per-frame
    threshold decision as-is)."""

    orientation_filter_sigma: float = 5.0
    """Gaussian smoothing sigma (frames) applied to the fitted neck-thorax-
    abdomen orientation line before computing the QA video's raw-frame box
    overlay, so per-frame keypoint noise doesn't make the box jitter/rotate
    independently of the fly's own motion. -1 uses each frame's own fit
    unsmoothed."""

    num_orphan_muscle_frames: int | None = None
    muscle_vrange: tuple[int, int] | None = None

    behavior_video_crf: int = 12
    behavior_video_preset: str = "medium"
    """NVENC "medium" measured ~2.5x faster AND smaller output than "slow"
    at every CRF tested (constant-QP mode, not libx264 -- preset here
    mainly affects encoder search effort/file size, not achieved quality;
    visually indistinguishable from "slow" at CRF 18 on a real trial)."""

    visualization_crf: int = 20
    visualization_preset: str = "medium"

    colormap: str = "lilac"
    """The QA video's muscle panel colormap: any cmasher name (e.g.
    "lilac") or matplotlib-registered name (e.g. "viridis", or cmasher's
    own "cmr.lilac" form)."""

    missing_muscle_frames_tolerance: int = 3
    num_muscle_samples: int = 100
    num_cpu_workers: int = -1
    """CPU-only parallelism: raw-frame decode and muscle-image warping
    (both plain cv2 work, never GPU compute) -- also doubles as this
    pipeline's only inference-time "data loading" worker count, for the
    decode step that feeds the orient model. Passed straight through to
    joblib's own `n_jobs`, so -1 (default) means joblib auto-detects all
    cores; never resolved to a concrete number ourselves."""
    visualization_num_workers: int = 8
    """Separate from num_cpu_workers: the QA video's per-chunk render+
    encode step's own worker count. Encoding shares the GPU's own
    concurrent-NVENC-session limit across workers (unlike decode/muscle
    warping, which don't touch the GPU), so this defaults much lower than
    num_cpu_workers' usual -1 (all cores) to avoid exceeding it."""

    visualization_composite_workers: int = 4
    """Threads per visualization_num_workers *process* used to composite
    (not encode) frames. Compositing is CPU-bound and dominated by cv2/
    numpy calls that release the GIL, so it can use more parallelism than
    visualization_num_workers without touching the GPU-session limit that
    caps that number."""

    profile_visualization: bool = False
    """Profile the QA video's first render chunk with cProfile (run
    synchronously, not in a parallel worker, so the profile reflects only
    that chunk's own work). Logs the top 20 functions by cumulative time
    and saves the full profile to postprocessed_dir/viz_chunk_profile.prof
    (inspect with e.g. `snakeviz`)."""

    log_level: str = "info"
    overwrite: bool = False
    skip_behavior: bool = False
    """Renamed from the old `reuse_behavior_alignment`; skip stage
    interpolation + orient decode/align and reuse existing outputs."""
    homography_path: Path | str | None = None


def postprocess_recording_data(
    recording_dir: tyro.conf.Positional[Path],
    params: tyro.conf.OmitArgPrefixes[PostprocessingParams] = PostprocessingParams(),
) -> None:
    """High-level post-processing pipeline for a single Spotlight recording.

    Args:
        recording_dir: Path to the recording directory created by the
            Spotlight recorder program.
        params: See `PostprocessingParams`.
    """
    t_start = time.perf_counter()
    numeric_level = getattr(logging, params.log_level.upper(), None)
    if not isinstance(numeric_level, int):
        raise ValueError(f"Invalid log level: {params.log_level}")
    logging.basicConfig(
        level=numeric_level, format="%(asctime)s - %(levelname)s - %(message)s",
        force=True,
    )  # fmt: skip
    logger = logging.getLogger(__name__)

    if params.pose2d and params.alignment == "fullsize":
        raise SystemExit("--pose2d requires --alignment aligned or both.")
    if params.ik and not params.pose2d:
        raise SystemExit("--ik requires --pose2d.")

    recording_dir = Path(recording_dir)
    postprocessed_dir = recording_dir / "postprocessed"
    if postprocessed_dir.exists() and not params.overwrite and not params.skip_behavior:
        raise FileExistsError(
            f"{postprocessed_dir} already exists. Use --overwrite to overwrite."
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
    # Playback speed for every output video (behavior and visualization
    # alike) tracks the recording's own rate, not real time. The exact fps
    # this implies is usually not an integer; approximating it as a small
    # fraction (rather than just rounding to the nearest integer fps) gets
    # much closer to the requested speed while still landing on a fps
    # ffmpeg encodes cleanly.
    play_fps = float(
        Fraction(params.playback_speed * behavior_fps).limit_denominator(20)
    )
    actual_playback_speed = play_fps / behavior_fps
    logger.info(
        f"behavior_fps={behavior_fps}, playback_speed={params.playback_speed} -> "
        f"play_fps={play_fps} (actual_playback_speed={actual_playback_speed:.4f})"
    )

    raw_behavior_images_dir = recording_dir / "behavior_images"
    raw_behavior_paths = sorted(raw_behavior_images_dir.glob("behavior_frame_*.jpg"))
    stage_positions_path = recording_dir / "stage_position/stage_position.csv"
    behavior_frames_metadata_path = postprocessed_dir / "behavior_frames_metadata.csv"
    alignment_metadata_path = postprocessed_dir / "behavior_alignment_transforms.h5"

    write_aligned = params.alignment in ("aligned", "both")
    write_fullsize = params.alignment in ("fullsize", "both")
    aligned_video_path = (
        postprocessed_dir / "aligned_behavior_video.mp4" if write_aligned else None
    )
    fullsize_video_path = (
        postprocessed_dir / "fullsize_behavior_video.mp4" if write_fullsize else None
    )
    pose2d_h5_path = (
        postprocessed_dir / "pose2d_predictions.h5" if params.pose2d else None
    )
    kinematics_h5_path = postprocessed_dir / "kinematics.h5" if params.pose2d else None

    muscle_mapping = None
    muscle_dataset_name = None
    muscle_h5_path = None
    if params.muscle:
        homography_path = (
            Path(params.homography_path) if params.homography_path is not None
            else metadata_dir / "homography_parameters.yaml"
        )  # fmt: skip
        # Muscle frames are produced in whichever domain the behavior video
        # itself uses: aligned/cropped whenever an aligned output exists (i.e.
        # alignment in ("aligned", "both")), fullsize only when alignment is
        # exclusively "fullsize" -- matching output_dim's own selection below.
        # Name by domain, not by the raw `alignment` string, so `--alignment
        # both` doesn't produce a misleading "both_muscle_images.h5" (the data
        # inside is actually aligned-domain only, never both at once).
        muscle_dataset_name = (
            "aligned_muscle_images" if write_aligned else "fullsize_muscle_images"
        )
        muscle_h5_path = postprocessed_dir / f"{muscle_dataset_name}.h5"

    if params.skip_behavior:
        missing = [
            p
            for p in [behavior_frames_metadata_path, alignment_metadata_path]
            if not p.exists()
        ]
        if missing:
            raise FileNotFoundError(
                f"--skip-behavior set but missing required existing outputs: {missing}"
            )
        logger.info(
            "Skipping behavior processing (--skip-behavior); reusing existing outputs."
        )
        behavior_result = None
    else:
        t_step = time.perf_counter()
        logger.info("Interpolating stage positions for behavior frames...")
        interp_stage_pos_at_behavior_frames(
            frames_dir=raw_behavior_images_dir,
            stage_positions_path=stage_positions_path,
            output_path=behavior_frames_metadata_path,
        )
        logger.info(f"STEP TIME stage_interp: {time.perf_counter() - t_step:.1f}s")

        if params.muscle:
            t_step = time.perf_counter()
            logger.info("Planning muscle<->behavior frame mapping...")
            output_dim = (params.crop_dim, params.crop_dim) if write_aligned else None
            if output_dim is None:
                # fullsize-only: output_dim is the raw frame's own size.
                import cv2

                sample = cv2.imread(str(raw_behavior_paths[0]))
                output_dim = (sample.shape[1], sample.shape[0])
            muscle_mapping = plan_muscle_behavior_mapping(
                raw_muscle_images_dir=recording_dir / "muscle_images",
                experiment_parameters_path=metadata_dir / "experiment_parameters.yaml",
                behavior_frames_metadata_path=behavior_frames_metadata_path,
                homography_path=homography_path,
                output_dim=output_dim,
                missing_muscle_frames_tolerance=params.missing_muscle_frames_tolerance,
                num_orphan_muscle_frames=params.num_orphan_muscle_frames,
            )
            logger.info(
                f"STEP TIME muscle_planning: {time.perf_counter() - t_step:.1f}s"
            )

        t_step = time.perf_counter()
        logger.info(f"Processing {len(raw_behavior_paths)} behavior frames "
                    "(decode + orient align + pose2d + muscle, streaming)...")  # fmt: skip
        behavior_result = process_behavior_pipeline(
            raw_behavior_frame_paths=raw_behavior_paths,
            orient_checkpoint_path=ORIENT_CHECKPOINT_PATH,
            alignment=params.alignment,
            thorax_y_normalized=params.thorax_y_normalized,
            crop_dim=params.crop_dim,
            output_aligned_video_path=aligned_video_path,
            output_fullsize_video_path=fullsize_video_path,
            output_alignment_metadata_path=alignment_metadata_path,
            behavior_video_fps=play_fps,
            behavior_video_crf=params.behavior_video_crf,
            behavior_video_preset=params.behavior_video_preset,
            orient_batch_size=params.orient_batch_size,
            run_pose2d=params.pose2d,
            pose2d_checkpoint_path=POSE2D_CHECKPOINT_PATH,
            pose2d_skeleton_json_path=POSE2D_SKELETON_JSON_PATH,
            pose2d_batch_size=params.pose2d_batch_size,
            output_pose2d_h5_path=pose2d_h5_path,
            run_muscle=params.muscle,
            muscle_mapping=muscle_mapping,
            output_muscle_h5_path=muscle_h5_path,
            muscle_dataset_name=muscle_dataset_name,
            num_cpu_workers=params.num_cpu_workers,
        )
        logger.info(
            f"STEP TIME behavior_pipeline_total: {time.perf_counter() - t_step:.1f}s"
        )
        if params.muscle:
            save_muscle_metadata_csv(
                muscle_mapping, postprocessed_dir / "muscle_frames_metadata.csv"
            )

    if params.pose2d:
        t_step = time.perf_counter()
        logger.info("Building kinematics.h5 (pose2d + optional IK/FK)...")
        subprocess.run(
            [
                sys.executable, str(SOLVE_IK_SCRIPT_PATH),
                "--input-path", str(pose2d_h5_path),
                "--output-path", str(kinematics_h5_path),
                "--neutral-weight", str(params.ik_prior_weight),
                "--with-ik" if params.ik else "--no-with-ik",
                "--override",
            ],
            check=True,
        )  # fmt: skip
        logger.info(f"STEP TIME kinematics_solve: {time.perf_counter() - t_step:.1f}s")

    visualization_total_s = None
    if params.visualize:
        t_step = time.perf_counter()
        logger.info("Generating QA visualization video...")
        generate_summary_video(
            recording_dir=recording_dir,
            postprocessed_dir=postprocessed_dir,
            alignment=params.alignment,
            crop_dim=params.crop_dim,
            with_muscle=params.muscle,
            with_pose2d=params.pose2d,
            with_ik=params.ik,
            flipped_prob=(behavior_result["flipped_prob"] if behavior_result else None),
            muscle_h5_path=muscle_h5_path,
            muscle_dataset_name=muscle_dataset_name,
            kinematics_h5_path=kinematics_h5_path,
            pose2d_skeleton_json_path=POSE2D_SKELETON_JSON_PATH,
            output_path=postprocessed_dir / "summary_video.mp4",
            muscle_vrange=params.muscle_vrange,
            play_fps=play_fps,
            playback_speed=actual_playback_speed,
            crf=params.visualization_crf,
            preset=params.visualization_preset,
            flip_confidence_threshold=params.flip_confidence_threshold,
            ik_mismatch_threshold=params.ik_max_mismatch,
            flip_mask_window=params.visualize_flip_mask_window,
            orientation_filter_sigma=params.orientation_filter_sigma,
            num_workers=params.visualization_num_workers,
            composite_workers=params.visualization_composite_workers,
            profile=params.profile_visualization,
            colormap=params.colormap,
        )
        visualization_total_s = time.perf_counter() - t_step
        logger.info(f"STEP TIME visualization_total: {visualization_total_s:.1f}s")

    timers = behavior_result["timers"] if behavior_result else {}
    report = [
        ("orient model", timers.get("orient_infer")),
        ("2d pose model", timers.get("pose2d_infer")),
        ("muscle warping", timers.get("muscle_warp")),
        (
            "saving behavior video",
            (
                timers["aligned_video_encode"] + timers.get("fullsize_video_encode", 0)
                if "aligned_video_encode" in timers
                else timers.get("fullsize_video_encode")
            ),
        ),
        (
            "saving muscle h5",
            (
                timers["muscle_h5_write"] + timers.get("muscle_h5_close", 0)
                if "muscle_h5_write" in timers
                else None
            ),
        ),
        ("making summary video", visualization_total_s),
        ("total", time.perf_counter() - t_start),
    ]
    logger.info(
        "Walltime report: "
        + ", ".join(f"{name}={secs:.1f}s" for name, secs in report if secs is not None)
    )

    logger.info(f"Done. Outputs under {postprocessed_dir}")


def main():
    tyro.cli(postprocess_recording_data)


if __name__ == "__main__":
    main()
