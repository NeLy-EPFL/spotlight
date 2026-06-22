import sys
import os
import tyro
import logging
from pathlib import Path

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


def postprocess_recording_data(
    recording_dir: Path | str,
    crop_dim: int = 900,
    overwrite: bool = False,
    align_fly: bool = True,
    with_muscle: bool = False,
    make_visualizations: bool = True,
    play_fps: int = 33,
    behavior_video_crf: int = 12,
    behavior_video_preset: str = "slow",
    visualization_crf: int = 20,
    visualization_preset: str = "slow",
    sleap_batch_size: int = 128,
    muscle_vrange: tuple[int, int] | None = None,
    num_muscle_samples: int = 100,
    use_shm: bool = False,
    missing_muscle_frames_tolerance: int = 3,
    num_workers: int = -1,
    log_level: str = "INFO",
    use_homography: bool = False,
) -> None:
    """High-level post-processing pipeline for a single Spotlight recording.

    This function executes a complete processing pipeline that includes:
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
       be consistent with behavior images (this is based on Spotlight calibration
       parameters), and apply the same alignment transforms used for behavior frames to
       maintain pixel-wise correspondence. Note that depending on whether `align_fly` is
       True, the output muscle frames are either full-sized or aligned/cropped.
    4. Generate summary video (showing behavior frames, 2D pose, and optionally muscle
       frames). If `with_muscle` is True, also generate muscle-upon-behavior overlays
       for a subset of frames for visual inspection.

    Args:
        recording_dir (Path | str): Path to the recording directory created by the
            Spotlight recorder program.
        crop_dim (int): Output frame dimensions for aligned frames (so that the output
            is crop_dim x crop_dim pixels).
        overwrite (bool): Whether to overwrite existing processed outputs.
        align_fly (bool): Whether to align and crop behavior frames based on fly pose.
        with_muscle (bool): Whether to process muscle images and align with behavior.
        make_visualizations (bool): Whether to generate summary videos and overlays.
        play_fps (int): Frame rate for generated videos. This is for visualization only.
            It has no impact on the actual data saved. It merely sets the metadata that
            tells video players how fast "1x speed" is. For example, if behavior frames
            are recorded at 330 FPS, and play_fps is 33, then the default playback speed
            ("1x" as far as your video player is concerned) will be 0.1x speed.
        behavior_video_crf (int): Constant Rate Factor for video encoding quality. Lower
            is better. 12-17 is visually lossless for most purposes. <10 is overkill.
        behavior_video_preset (str): ffmpeg preset for encoding speed vs compression.
            Slower setting = better compression. "slow" or "slower" is recommended.
        visualization_crf (int): Same as `behavior_video_crf` but for summary video.
        visualization_preset (str): Same as `behavior_video_preset` but for summary video.
        sleap_batch_size (int): Batch size for SLEAP pose estimation.
        muscle_vrange (tuple[int, int] | None): Value range for muscle visualization.
            If None, the range will be determined automatically based on muscle image
            statistics.
        num_muscle_samples (int | None): Number of sample muscle-behavior pairs to
            generate overlays for. Default is 100.
        use_shm (bool): Whether to use shared memory (/dev/shm) for temporary files.
            Doing so will avoid duplicated disk read and write, but it is extremely
            sketchy - if the process runs out of shared memory, the entire OS will
            likely crash. Default is False.
        missing_muscle_frames_tolerance (int): Maximum allowed consecutive missing
            frames (the first one is always missing due to rolling shutter; the last
            few might be missing due to nondeterministic hardware timing when recording
            stops).
        num_workers (int): Number of parallel workers (-1 for all available cores).
        log_level (str): Logging level for the processing pipeline. Options are:
            "DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL". Default is "INFO".
        use_homography (bool): If True, use homography transformation for muscle-to-behavior
            alignment instead of the stage-dependent affine transformation from Spotlight
            calibration. This requires a homography calibration to have been performed.
            Default is False.
        homography_path (Path | str | None): Path to homography calibration YAML file.
            If None and use_homography is True, will look for homography_result.yaml in
            the standard profile calibration location. Default is None.
    """
    # Set up logging with the specified level
    numeric_level = getattr(logging, log_level.upper(), None)
    if not isinstance(numeric_level, int):
        raise ValueError(f"Invalid log level: {log_level}")

    logging.basicConfig(
        level=numeric_level,
        format="%(asctime)s - %(levelname)s - %(message)s",
        force=True,  # Override any existing logging configuration
    )
    logger = logging.getLogger(__name__)

    # Validate recording directory
    recording_dir = Path(recording_dir)
    processed_dir = recording_dir / "processed"
    if processed_dir.exists() and not overwrite:
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
    muscle_calib_path = metadata_dir / "calibration_parameters_muscle.yaml"
    homography_path = metadata_dir / "homography_parameters.yaml"
    behavior_calib_path = metadata_dir / "calibration_parameters_behavior.yaml"
    experiment_parameters_path = metadata_dir / "experiment_parameters.yaml"
    if with_muscle:
        raw_muscle_images_dir = recording_dir / "muscle_images/"
    # Output
    behavior_frames_metadata_path = processed_dir / "behavior_frames_metadata.csv"
    stage_trajectory_viz_path = processed_dir / "stage_trajectory.png"
    processed_muscle_frames_dir = None
    muscle_frames_metadata_path = None
    overlay_samples_dir = None
    if align_fly:
        processed_behavior_video_path = processed_dir / "aligned_behavior_video.mkv"
        alignment_metadata_path = processed_dir / "behavior_alignment_transforms.h5"
        summary_video_path = processed_dir / "aligned_summary_video.mp4"
        if with_muscle:
            processed_muscle_frames_dir = processed_dir / "aligned_muscle_images/"
            overlay_samples_dir = processed_dir / "aligned_2camera_overlay_samples/"
    else:
        processed_behavior_video_path = processed_dir / "fullsize_behavior_video.mkv"
        alignment_metadata_path = None
        summary_video_path = processed_dir / "fullsize_summary_video.mp4"
        if with_muscle:
            processed_muscle_frames_dir = processed_dir / "fullsize_muscle_images/"
            overlay_samples_dir = processed_dir / "fullsize_2camera_overlay_samples/"
    if with_muscle:
        muscle_frames_metadata_path = processed_dir / "muscle_frames_metadata.csv"

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
        align_fly=align_fly,
        use_shm=use_shm,
        sleap_batch_size=sleap_batch_size,
        crop_dim=crop_dim,
        play_fps=play_fps,
        behavior_video_crf=behavior_video_crf,
        behavior_video_preset=behavior_video_preset,
        num_workers=num_workers,
    )

    # Process muscle frames (if requested)
    # 1. Warp muscle images to align with behavior frames
    # 2. Apply the same alignment transforms used for behavior frames
    if with_muscle:
        logger.info("Mapping muscle frames to behavior frames...")
        if homography_path is not None:
            homography_path = Path(homography_path)
        warp_all_muscle_frames_to_behavior(
            muscle_calib_path=muscle_calib_path,
            behavior_calib_path=behavior_calib_path,
            experiment_parameters_path=experiment_parameters_path,
            processed_behavior_frame_metadata_path=behavior_frames_metadata_path,
            raw_muscle_images_dir=raw_muscle_images_dir,
            transformed_muscle_images_output_dir=processed_muscle_frames_dir,
            muscle_metadata_output_path=muscle_frames_metadata_path,
            align_fly=align_fly,
            behavior_alignment_metadata_path=alignment_metadata_path,
            processed_behavior_video_path=processed_behavior_video_path,
            missing_muscle_frames_tolerance=missing_muscle_frames_tolerance,
            num_workers=num_workers,
            use_homography=use_homography,
            homography_path=homography_path,
        )

    # Generate visualizations (if requested)
    if make_visualizations:
        logger.info("Generating summary video...")
        generate_summary_video(
            behavior_video_path=processed_behavior_video_path,
            output_path=summary_video_path,
            with_muscle=with_muscle,
            draw_2dpose=align_fly,
            muscle_images_dir=processed_muscle_frames_dir,
            experiment_parameters_path=experiment_parameters_path,
            pose_2d_path=alignment_metadata_path,
            muscle_vrange=muscle_vrange,
            play_fps=play_fps,
            crf=visualization_crf,
            preset=visualization_preset,
        )

        visualize_stage_trajectory(
            behavior_frame_metadata_path=behavior_frames_metadata_path,
            output_path=stage_trajectory_viz_path,
        )

        if with_muscle:
            logger.info("Generating muscle-behavior overlay samples...")
            generate_overlay_samples(
                behavior_video_path=processed_behavior_video_path,
                muscle_images_dir=processed_muscle_frames_dir,
                muscle_metadata_path=muscle_frames_metadata_path,
                experiment_parameters_path=experiment_parameters_path,
                output_dir=overlay_samples_dir,
                muscle_vrange=muscle_vrange,
                num_samples=num_muscle_samples,
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
    #     with_muscle=True,
    #     overwrite=True,
    #     align_fly=True,
    # )
    # # With muscle, full-size
    # postprocess_recording_data(
    #     recording_dir=Path("~/data/spotlight/20250613-fly1b-002/").expanduser(),
    #     with_muscle=True,
    #     overwrite=True,
    #     align_fly=False,
    # )
    # # Without muscle, aligned
    # postprocess_recording_data(
    #     recording_dir=Path("~/data/spotlight/20250613-fly1b-002/").expanduser(),
    #     with_muscle=False,
    #     overwrite=True,
    #     align_fly=True,
    # )
    # # Without muscle, full-size
    # postprocess_recording_data(
    #     recording_dir=Path("~/data/spotlight/20250613-fly1b-002/").expanduser(),
    #     with_muscle=False,
    #     overwrite=True,
    #     align_fly=False,
    # )
