import sys
import os
import tyro
import logging
from pathlib import Path
from dataclasses import dataclass

from spotlight_tools.common import load_spotlight_tools_config
from spotlight_tools.postprocessing.stage import interp_stage_pos_at_behavior_frames
from spotlight_tools.postprocessing.behavior import decode_and_align_all_behavior_frames
from spotlight_tools.postprocessing.muscle import warp_all_muscle_frames_to_behavior
from spotlight_tools.postprocessing.visualize import (
    generate_summary_video,
    generate_overlay_samples,
    visualize_stage_trajectory,
)


# Reopen STDOUT in unbuffered mode to ensure that print statements print immediately
# This is useful when we run this script from a bash script and redirect the output to
# a non-TTY file.
sys.stdout = os.fdopen(sys.stdout.fileno(), "w", buffering=1)


@dataclass
class PostprocessingParams:
    """Configuration for `postprocess_recording_data`, the high-level post-processing
    pipeline for a single Spotlight recording.

    This pipeline executes:
    1. Stage position interpolation for behavior frames: due to hardware constraints,
       stage positions are typically logged at a lower frequency than behavior
       recording. Here we interpolate stage positions at the timestamp of each behavior
       frame.
    2. Behavior frame processing:
       a. Splitting pseudo-BGR JPEGs (during recording, every three consecutive frames
          are bundled into a single JPEG file; this is performance hack)
       b. (If `align_fly` is True) Running a simple 3-keypoint 2D pose estimation using
          SLEAP (in order to detect fly position and orientation)
       c. (If `align_fly` is True) Rotating the image around the detected thorax so that
          the fly faces upward, and cropping image to a square centered on the fly.
    3. Muscle frame transformation and alignment (if requested): Warp muscle images to
       be consistent with behavior images (using a pre-computed homography), and apply
       the same alignment transforms used for behavior frames to maintain pixel-wise
       correspondence. Note that depending on whether `align_fly` is True, the output
       muscle frames are either full-sized or aligned/cropped.
    4. Generate summary video (showing behavior frames, 2D pose, and optionally muscle
       frames). If `with_muscle` is True, also generate muscle-upon-behavior overlays
       for a subset of frames for visual inspection.
    """

    crop_dim: int = 900
    """Output frame dimensions for aligned frames (so that the output is crop_dim x
    crop_dim pixels)."""
    
    overwrite: bool = False
    """Whether to overwrite existing processed outputs."""
    
    align_fly: bool = True
    """Whether to align and crop behavior frames based on fly pose."""
    
    with_muscle: bool = False
    """Whether to process muscle images and align with behavior."""
    
    reuse_behavior_alignment: bool = False
    """If True, skip the behavior pipeline (stage interpolation and SLEAP decode/align)
    and reuse the previously computed behavior outputs (behavior_frames_metadata.csv,
    the aligned behavior video, and, when align_fly is True,
    behavior_alignment_transforms.h5). Muscle warping and visualizations are still run.
    Useful for iterating on muscle alignment (e.g. tuning `num_orphan_muscle_frames`)
    without re-running the expensive pose-estimation step. Requires those behavior
    outputs to already exist in the processed directory."""
    
    homography_path: Path | str | None = None
    """Path to the muscle-to-behavior homography calibration YAML to use for muscle
    warping. If None (default), the homography snapshotted inside the recording
    (metadata/homography_parameters.yaml) is used. Provide a path to override it with a
    more recent calibration. Only relevant when with_muscle is True."""
    
    num_orphan_muscle_frames: int | None = None
    """Number of leading orphan (pre-excitation, dark) muscle frames to skip. If None
    (default), the count is detected automatically from muscle-frame brightness.
    Provide an integer to override the automatic detection."""
    
    make_visualizations: bool = True
    """Whether to generate summary videos and overlays."""
    
    play_fps: int = 33
    """Frame rate for generated videos. This is for visualization only. It has no
    impact on the actual data saved. It merely sets the metadata that tells video
    players how fast "1x speed" is. For example, if behavior frames are recorded at 330
    FPS, and play_fps is 33, then the default playback speed ("1x" as far as your video
    player is concerned) will be 0.1x speed."""
    
    behavior_video_crf: int = 12
    """Constant Rate Factor for video encoding quality. Lower is better. 12-17 is
    visually lossless for most purposes. <10 is overkill."""
    
    behavior_video_preset: str = "slow"
    """ffmpeg preset for encoding speed vs compression. Slower setting = better
    compression. "slow" or "slower" is recommended."""
    
    visualization_crf: int = 20
    """Same as `behavior_video_crf` but for summary video."""
    
    visualization_preset: str = "slow"
    """Same as `behavior_video_preset` but for summary video."""
    
    sleap_batch_size: int = 128
    """Batch size for SLEAP pose estimation."""
    
    muscle_vrange: tuple[int, int] | None = None
    """Value range for muscle visualization. If None, the range will be determined
    automatically based on muscle image statistics."""
    
    num_muscle_samples: int = 100
    """Number of sample muscle-behavior pairs to generate overlays for."""
    
    use_shm: bool = False
    """Whether to use shared memory (/dev/shm) for temporary files. Doing so will avoid
    duplicated disk read and write, but it is extremely sketchy - if the process runs
    out of shared memory, the entire OS will likely crash."""
    
    missing_muscle_frames_tolerance: int = 3
    """Maximum muscle-frame shortfall, beyond the part explained by the leading orphan
    skip, tolerated before the pipeline raises. The muscle recording is expected to end
    a few frames short (the two cameras receive the stop signal at slightly different
    times); a shortfall larger than this once the orphan count is subtracted means the
    muscle recording is truncated and is treated as a hard error rather than silently
    producing a short recording."""
    
    num_workers: int = -1
    """Number of parallel workers (-1 for all available cores)."""
    
    log_level: str = "INFO"
    """Logging level for the processing pipeline. Options are: "DEBUG", "INFO",
    "WARNING", "ERROR", "CRITICAL"."""


def postprocess_recording_data(
    recording_dir: Path | str,
    params: tyro.conf.OmitArgPrefixes[PostprocessingParams] = PostprocessingParams(),
) -> None:
    """High-level post-processing pipeline for a single Spotlight recording.

    This pipeline executes:
    1. Stage position interpolation for behavior frames: due to hardware constraints,
       stage positions are typically logged at a lower frequency than behavior
       recording. Here we interpolate stage positions at the timestamp of each behavior
       frame.
    2. Behavior frame processing:
       a. Splitting pseudo-BGR JPEGs (during recording, every three consecutive frames
          are bundled into a single JPEG file; this is performance hack)
       b. (If `align_fly` is True) Running a simple 3-keypoint 2D pose estimation using
          SLEAP (in order to detect fly position and orientation)
       c. (If `align_fly` is True) Rotating the image around the detected thorax so that
          the fly faces upward, and cropping image to a square centered on the fly.
    3. Muscle frame transformation and alignment (if requested): Warp muscle images to
       be consistent with behavior images (using a pre-computed homography), and apply
       the same alignment transforms used for behavior frames to maintain pixel-wise
       correspondence. Note that depending on whether `align_fly` is True, the output
       muscle frames are either full-sized or aligned/cropped.
    4. Generate summary video (showing behavior frames, 2D pose, and optionally muscle
       frames). If `with_muscle` is True, also generate muscle-upon-behavior overlays
       for a subset of frames for visual inspection.
    
    Args:
        recording_dir: Path to the recording directory created by the Spotlight
            recorder program.
        args: See `PostprocessingParams`.
    """
    # Set up logging with the specified level
    numeric_level = getattr(logging, params.log_level.upper(), None)
    if not isinstance(numeric_level, int):
        raise ValueError(f"Invalid log level: {params.log_level}")

    logging.basicConfig(
        level=numeric_level,
        format="%(asctime)s - %(levelname)s - %(message)s",
        force=True,  # Override any existing logging configuration
    )
    logger = logging.getLogger(__name__)

    # Validate recording directory
    recording_dir = Path(recording_dir)
    processed_dir = recording_dir / "processed"
    # When reusing existing behavior outputs the processed directory is expected to
    # already exist, so the "already exists" guard does not apply.
    if (
        processed_dir.exists()
        and not params.overwrite
        and not params.reuse_behavior_alignment
    ):
        logger.error(
            f"Processed directory {processed_dir} already exists. "
            "Use --overwrite to overwrite existing outputs."
        )
        raise FileExistsError(f"Processed directory {processed_dir} already exists.")
    processed_dir.mkdir(exist_ok=True, parents=True)

    # Define paths
    # Input
    raw_behavior_images_dir = recording_dir / "behavior_images/"
    raw_behavior_images_paths = sorted(
        raw_behavior_images_dir.glob("behavior_frame_*.jpg")
    )
    stage_positions_path = recording_dir / "stage_position/stage_position.csv"
    metadata_dir = recording_dir / "metadata/"
    # Use the caller-supplied homography if given (e.g. a more recent calibration),
    # otherwise fall back to the one snapshotted inside the recording.
    if params.homography_path is None:
        homography_path = metadata_dir / "homography_parameters.yaml"
    else:
        homography_path = Path(params.homography_path)
        if not homography_path.exists():
            raise FileNotFoundError(
                f"Supplied homography file does not exist: {homography_path}"
            )
    experiment_parameters_path = metadata_dir / "experiment_parameters.yaml"
    if params.with_muscle:
        raw_muscle_images_dir = recording_dir / "muscle_images/"
    # Output
    behavior_frames_metadata_path = processed_dir / "behavior_frames_metadata.csv"
    stage_trajectory_viz_path = processed_dir / "stage_trajectory.png"
    processed_muscle_frames_dir = None
    muscle_frames_metadata_path = None
    overlay_samples_dir = None
    if params.align_fly:
        processed_behavior_video_path = processed_dir / "aligned_behavior_video.mkv"
        alignment_metadata_path = processed_dir / "behavior_alignment_transforms.h5"
        summary_video_path = processed_dir / "aligned_summary_video.mp4"
        if params.with_muscle:
            processed_muscle_frames_dir = processed_dir / "aligned_muscle_images/"
            overlay_samples_dir = processed_dir / "aligned_2camera_overlay_samples/"
    else:
        processed_behavior_video_path = processed_dir / "fullsize_behavior_video.mkv"
        alignment_metadata_path = None
        summary_video_path = processed_dir / "fullsize_summary_video.mp4"
        if params.with_muscle:
            processed_muscle_frames_dir = processed_dir / "fullsize_muscle_images/"
            overlay_samples_dir = processed_dir / "fullsize_2camera_overlay_samples/"
    if params.with_muscle:
        muscle_frames_metadata_path = processed_dir / "muscle_frames_metadata.csv"

    if params.reuse_behavior_alignment:
        # Reuse previously computed behavior outputs; skip stage interpolation and the
        # expensive SLEAP decode/align. Verify the outputs we depend on downstream
        # actually exist before continuing.
        required_files = [behavior_frames_metadata_path, processed_behavior_video_path]
        if params.align_fly:
            required_files.append(alignment_metadata_path)
        missing_outputs = [p for p in required_files if not p.exists()]
        if missing_outputs:
            raise FileNotFoundError(
                "reuse_behavior_alignment=True but required existing outputs are "
                "missing: "
                + ", ".join(str(p) for p in missing_outputs)
                + ". Run the full pipeline (without --reuse-behavior-alignment) first."
            )
        logger.info(
            "Reusing previously computed behavior outputs; skipping stage "
            "interpolation and SLEAP decode/align."
        )
    else:
        # Interpolate stage positions for behavior frames
        logger.info("Interpolating stage positions for behavior frames...")
        interp_stage_pos_at_behavior_frames(
            frames_dir=raw_behavior_images_dir,
            stage_positions_path=stage_positions_path,
            output_path=behavior_frames_metadata_path,
        )

        # Process behavior frames:
        # 1. Decode pseudo-BGR JPEGs into single frames
        # 2. Run SLEAP to detect fly position and orientation for each frame
        # 3. Rotate and crop each frame to align the fly (centered, facing up)
        config = load_spotlight_tools_config()
        logger.info("Decoding and transforming behavior frames...")
        decode_and_align_all_behavior_frames(
            raw_behavior_frame_paths=raw_behavior_images_paths,
            sleap_model_dir=Path(config["pose2d"]["sleap_model_dir"]).expanduser(),
            output_video_path=processed_behavior_video_path,
            output_metadata_path=alignment_metadata_path,
            keypoints_code2name=config["pose2d"]["keypoint_names"],
            align_fly=params.align_fly,
            use_shm=params.use_shm,
            sleap_batch_size=params.sleap_batch_size,
            crop_dim=params.crop_dim,
            play_fps=params.play_fps,
            behavior_video_crf=params.behavior_video_crf,
            behavior_video_preset=params.behavior_video_preset,
            num_workers=params.num_workers,
        )

    # Process muscle frames (if requested)
    # 1. Warp muscle images to align with behavior frames
    # 2. Apply the same alignment transforms used for behavior frames
    if params.with_muscle:
        logger.info("Mapping muscle frames to behavior frames...")
        warp_all_muscle_frames_to_behavior(
            experiment_parameters_path=experiment_parameters_path,
            processed_behavior_frame_metadata_path=behavior_frames_metadata_path,
            raw_muscle_images_dir=raw_muscle_images_dir,
            transformed_muscle_images_output_dir=processed_muscle_frames_dir,
            muscle_metadata_output_path=muscle_frames_metadata_path,
            homography_path=homography_path,
            align_fly=params.align_fly,
            behavior_alignment_metadata_path=alignment_metadata_path,
            processed_behavior_video_path=processed_behavior_video_path,
            missing_muscle_frames_tolerance=params.missing_muscle_frames_tolerance,
            num_orphan_muscle_frames=params.num_orphan_muscle_frames,
            num_workers=params.num_workers,
        )

    # Generate visualizations (if requested)
    if params.make_visualizations:
        logger.info("Generating summary video...")
        generate_summary_video(
            behavior_video_path=processed_behavior_video_path,
            output_path=summary_video_path,
            with_muscle=params.with_muscle,
            draw_2dpose=params.align_fly,
            muscle_images_dir=processed_muscle_frames_dir,
            muscle_metadata_path=muscle_frames_metadata_path,
            experiment_parameters_path=experiment_parameters_path,
            pose_2d_path=alignment_metadata_path,
            muscle_vrange=params.muscle_vrange,
            play_fps=params.play_fps,
            crf=params.visualization_crf,
            preset=params.visualization_preset,
        )

        visualize_stage_trajectory(
            behavior_frame_metadata_path=behavior_frames_metadata_path,
            output_path=stage_trajectory_viz_path,
        )

        if params.with_muscle:
            logger.info("Generating muscle-behavior overlay samples...")
            generate_overlay_samples(
                behavior_video_path=processed_behavior_video_path,
                muscle_images_dir=processed_muscle_frames_dir,
                muscle_metadata_path=muscle_frames_metadata_path,
                experiment_parameters_path=experiment_parameters_path,
                output_dir=overlay_samples_dir,
                muscle_vrange=params.muscle_vrange,
                num_samples=params.num_muscle_samples,
            )


def main():
    tyro.cli(postprocess_recording_data)


if __name__ == "__main__":
    main()

    # * Example from Python natively
    # logging.basicConfig(
    #     level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
    # )
    # # With muscle, aligned
    # postprocess_recording_data(
    #     recording_dir=Path("~/data/spotlight/20250613-fly1b-002/").expanduser(),
    #     params=PostprocessingParams(with_muscle=True, overwrite=True, align_fly=True),
    # )
    # # With muscle, full-size
    # postprocess_recording_data(
    #     recording_dir=Path("~/data/spotlight/20250613-fly1b-002/").expanduser(),
    #     params=PostprocessingParams(with_muscle=True, overwrite=True, align_fly=False),
    # )
    # # Without muscle, aligned
    # postprocess_recording_data(
    #     recording_dir=Path("~/data/spotlight/20250613-fly1b-002/").expanduser(),
    #     params=PostprocessingParams(with_muscle=False, overwrite=True, align_fly=True),
    # )
    # # Without muscle, full-size
    # postprocess_recording_data(
    #     recording_dir=Path("~/data/spotlight/20250613-fly1b-002/").expanduser(),
    #     params=PostprocessingParams(with_muscle=False, overwrite=True, align_fly=False),
    # )
