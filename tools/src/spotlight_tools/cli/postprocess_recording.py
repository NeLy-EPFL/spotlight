import logging
import os
import shutil
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Literal

import tyro
import yaml

from spotlight_tools.postprocessing.behavior import process_behavior_pipeline
from spotlight_tools.postprocessing.muscle import (
    plan_muscle_behavior_mapping,
    save_muscle_metadata_csv,
)
from spotlight_tools.postprocessing.stage import interp_stage_pos_at_behavior_frames
from spotlight_tools.postprocessing.visualize import generate_summary_video

sys.stdout = os.fdopen(sys.stdout.fileno(), "w", buffering=1)

TOOLS_ROOT = Path(__file__).resolve().parents[3]  # tools/
ORIENT_CHECKPOINT_PATH = TOOLS_ROOT / "bulk_data/orient_model/checkpoints/v13/best.pt"
POSE2D_CHECKPOINT_PATH = (
    TOOLS_ROOT / "bulk_data/pose2d_model/checkpoints/iter2b/best.pt"
)
POSE2D_SKELETON_JSON_PATH = TOOLS_ROOT / "bulk_data/pose2d_model/skeleton_metadata.json"
SOLVE_IK_SCRIPT_PATH = TOOLS_ROOT / "scripts/spotlight_ik/solve_ik.py"


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

    visualize: bool = True
    muscle: bool = True
    pose2d: bool = True
    ik: bool = True

    orient_batch_size: int = 512
    """fp16 peak ~5.1GB on a 12GB GPU, measured (see Task #54 benchmark)."""

    pose2d_batch_size: int = 256
    """fp16 peak ~4.1GB on a 12GB GPU, measured (see Task #54 benchmark)."""

    ik_prior_weight: float = 0.2
    """Passed through as `solve_ik.py`'s `neutral_weight`."""

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

    missing_muscle_frames_tolerance: int = 3
    num_muscle_samples: int = 100
    num_workers: int = -1
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
    # alike) tracks the recording's own rate, not real time.
    play_fps = round(behavior_fps / 10)
    logger.info(f"behavior_fps={behavior_fps}, play_fps={play_fps}")

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
    ikfk_h5_path = postprocessed_dir / "inverse_kinematics.h5" if params.ik else None

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
            num_workers=params.num_workers,
        )
        logger.info(
            f"STEP TIME behavior_pipeline_total: {time.perf_counter() - t_step:.1f}s"
        )
        if params.muscle:
            save_muscle_metadata_csv(
                muscle_mapping, postprocessed_dir / "muscle_frames_metadata.csv"
            )

    if params.ik:
        t_step = time.perf_counter()
        logger.info("Solving IK/FK...")
        subprocess.run(
            [
                sys.executable, str(SOLVE_IK_SCRIPT_PATH),
                "--input-path", str(pose2d_h5_path),
                "--output-path", str(ikfk_h5_path),
                "--neutral-weight", str(params.ik_prior_weight),
                # Disable the mismatch-based re-segmentation step (rejecting
                # frames by xy distance between pred_2d_mm and fk_3d_mm) --
                # inf accepts every frame the confidence-based period-finder
                # already kept, so periods are never further split on this.
                "--max-mismatch", "inf",
                "--override",
            ],
            check=True,
        )  # fmt: skip
        logger.info(f"STEP TIME ik_solve: {time.perf_counter() - t_step:.1f}s")

    if params.visualize:
        t_step = time.perf_counter()
        logger.info("Generating QA visualization video...")
        generate_summary_video(
            recording_dir=recording_dir,
            postprocessed_dir=postprocessed_dir,
            alignment=params.alignment,
            with_muscle=params.muscle,
            with_pose2d=params.pose2d,
            with_ik=params.ik,
            aligned_video_path=aligned_video_path,
            fullsize_video_path=fullsize_video_path,
            flipped_prob=(behavior_result["flipped_prob"] if behavior_result else None),
            muscle_h5_path=muscle_h5_path,
            muscle_dataset_name=muscle_dataset_name,
            pose2d_h5_path=pose2d_h5_path,
            pose2d_skeleton_json_path=POSE2D_SKELETON_JSON_PATH,
            ikfk_h5_path=ikfk_h5_path,
            output_path=postprocessed_dir / "summary_video.mp4",
            muscle_vrange=params.muscle_vrange,
            play_fps=play_fps,
            crf=params.visualization_crf,
            preset=params.visualization_preset,
            num_workers=params.num_workers,
        )
        logger.info(
            f"STEP TIME visualization_total: {time.perf_counter() - t_step:.1f}s"
        )

    logger.info(f"Done. Outputs under {postprocessed_dir}")


def main():
    tyro.cli(postprocess_recording_data)


if __name__ == "__main__":
    main()
