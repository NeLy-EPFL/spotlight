import sys
import os
import tyro
import logging
from pathlib import Path

from spotlight_tools.common import load_spotlight_tools_config
from spotlight_tools.postprocessing.stage import interp_stage_pos_at_behavior_frames
from spotlight_tools.postprocessing.behavior import decode_and_transform_behavior_frames
from spotlight_tools.postprocessing.muscle import map_muscle_frames_to_behavior
from spotlight_tools.postprocessing.visualize import (
    generate_summary_video,
    generate_overlay_samples,
)


# Reopen STDOUT in unbuffered mode to ensure that print statements print immediately
# This is useful when we run this script from a bash script and redirect the output to
# a non-TTY file.
sys.stdout = os.fdopen(sys.stdout.fileno(), "w", buffering=1)


def postprocess_recording_data(
    recording_dir: Path | str,
    crop_dim: int = 900,
    overwrite: bool = False,
    with_muscle: bool = False,
    make_visualizations: bool = True,
    play_fps: int = 30,
    behavior_video_crf: int = 12,
    behavior_video_preset: str = "slow",
    visualization_crf: int = 20,
    visualization_preset: str = "slow",
    sleap_batch_size: int = 128,
    muscle_vrange: tuple[int, int] | None = None,
    num_muscle_samples: int | None = 100,
    use_shm: bool = False,
    missing_muscle_frames_tolerance: int = 3,
    num_workers: int = -1,
) -> None:
    """High-level post-processing pipeline for a single Spotlight recording.

    This function runs (a subset of) a sequence of post-processing steps.
    Use the ``interpolate_stage_position``, ``merge_behavior_video``,
    ``estimate_2dpose``, ``warp_muscle_images``, and ``make_visualizations``
    flags to control which steps are executed. The steps are executed in
    the order listed below, and dependencies between steps are handled
    automatically.
      1. Interpolate stage positions per behavior frame and save
         ``processed/behavior_frames_metadata.csv``.
      2. Merge behavior frame JPEGs into a single MKV video
         (``processed/behavior_video.mkv``) using the configured encoding
         parameters.
      3. Run 2D pose estimation (SLEAP) on the behavior video and save
         results to ``processed/pose_2d.npz``.
      4. Warp muscle images so they align with behavior frames and produce
         processed muscle images.
      5. Create visualizations: a summary video and overlay
         sample grid combining behavior and muscle images.

    Args:
        recording_dir (Path | str): Path to the recording directory (the
            directory created by the Spotlight acquisition software).
        overwrite (bool): If True existing output files may be overwritten.
            Otherwise the function will raise if outputs already exist.
        interpolate_stage_position (bool): Whether to interpolate stage
            positions for each behavior frame.
        merge_behavior_video (bool): Whether to merge behavior frame JPEGs
            into a single video.
        estimate_2dpose (bool): Whether to run SLEAP to estimate 2D keypoints
            keypoints for each frame.
        warp_muscle_images (bool): Whether to run muscle image warping so
            muscle frames align with behavior frames.
        make_visualizations (bool): Whether to generate a summary video and
            overlay samples after processing is complete.
        play_fps (int): FPS used for the generated summary/preview videos
            (for display purposes only).
        behavior_video_crf (int): Encoder CRF value used when creating the
            merged behavior video (lower = higher quality).
        behavior_video_preset (str): ffmpeg libx264 preset controlling
            encode speed vs. compression.
        sleap_batch_size (int): Batch size passed to the SLEAP runner.
        muscle_transform_num_workers (int): Number of parallel workers
            to use when warping muscle images. If -1, use all available cores.
        muscle_vrange (tuple[int,int] | None): Optional (vmin, vmax) to use
            when visualizing muscle images. If None an adaptive range is
            computed when needed.
        num_frames (int | None): If set, limit processing to the first
            ``num_frames`` behavior frames (useful for quick tests).

    Returns:
        None: The function writes processed outputs into
        ``<recording_dir>/processed`` and does not return a value.
    """
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

    # Interpolate stage positions for behavior frames
    logger.info("Interpolating stage positions for behavior frames...")
    interp_stage_pos_at_behavior_frames(
        frames_dir=recording_dir / "behavior_images",
        stage_positions_path=recording_dir / "stage_position/stage_position.csv",
        output_path=processed_dir / "behavior_frames_metadata.csv",
    )

    # Process behavior frames:
    # 1. Decode pseudo-BGR JPEGs into single frames
    # 2. Run SLEAP to detect fly position and orientation for each frame
    # 3. Rotate and crop each frame to align the fly (centered, facing up)
    # fmt: off
    config = load_spotlight_tools_config()
    raw_behavior_frame_paths = sorted(recording_dir.glob("behavior_images/behavior_frame_*.jpg"))
    logger.info("Decoding and transforming behavior frames...")
    decode_and_transform_behavior_frames(
        raw_behavior_frame_paths=raw_behavior_frame_paths,
        sleap_model_dir=Path(config["pose2d"]["sleap_model_dir"]).expanduser(),
        output_video_path=processed_dir / "aligned_behavior_video.mkv",
        output_metadata_path=processed_dir / "behavior_alignment_transforms.h5",
        keypoints_code2name=config["pose2d"]["keypoint_names"],
        use_shm=use_shm,
        sleap_batch_size=sleap_batch_size,
        crop_dim=crop_dim,
        play_fps=play_fps,
        behavior_video_crf=behavior_video_crf,
        behavior_video_preset=behavior_video_preset,
        num_workers=num_workers,
    )
    # fmt: on

    # Process muscle frames (if requested)
    # 1. Warp muscle images to align with behavior frames
    # 2. Apply the same alignment transforms used for behavior frames
    if with_muscle:
        logger.info("Mapping muscle frames to behavior frames...")
        # fmt: off
        map_muscle_frames_to_behavior(
            muscle_calib_path=recording_dir / "metadata/calibration_parameters_muscle.yaml",
            behavior_calib_path=recording_dir / "metadata/calibration_parameters_behavior.yaml",
            dual_recording_timing_path=recording_dir / "metadata/dual_recording_timing.yaml",
            processed_behavior_frame_metadata_path=processed_dir / "behavior_frames_metadata.csv",
            behavior_alignment_metadata_path=processed_dir / "behavior_alignment_transforms.h5",
            raw_muscle_images_dir=recording_dir / "muscle_images",
            transformed_muscle_images_output_dir=processed_dir / "aligned_muscle_images/",
            muscle_metadata_output_path=processed_dir / "muscle_frames_metadata.csv",
            missing_muscle_frames_tolerance=missing_muscle_frames_tolerance,
            num_workers=num_workers,
        )
        # fmt: on

    # Generate visualizations (if requested)
    if make_visualizations:
        logger.info("Generating summary video...")
        # fmt: off
        generate_summary_video(
            behavior_video_path=processed_dir / "aligned_behavior_video.mkv",
            muscle_images_dir=processed_dir / "aligned_muscle_images",
            dual_recording_timing_path=recording_dir / "metadata/dual_recording_timing.yaml",
            pose_2d_path=processed_dir / "behavior_alignment_transforms.h5",
            output_path=processed_dir / "summary_video.mp4",
            with_muscle=with_muscle,
            muscle_vrange=muscle_vrange,
            play_fps=play_fps,
            crf=visualization_crf,
            preset=visualization_preset,
        )
        
        generate_overlay_samples(
            behavior_video_path=processed_dir / "aligned_behavior_video.mkv",
            muscle_images_dir=processed_dir / "aligned_muscle_images",
            muscle_metadata_path=processed_dir / "muscle_frames_metadata.csv",
            dual_recording_timing_path=recording_dir / "metadata/dual_recording_timing.yaml",
            output_dir=processed_dir / "overlay_samples",
            muscle_vrange=muscle_vrange,
            num_samples=num_muscle_samples,
        )
        # fmt: on


if __name__ == "__main__":
    # * CLI
    # tyro.cli(postprocess_recording_data)

    # * Example
    logging.basicConfig(
        level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
    )
    postprocess_recording_data(
        recording_dir="/home/sibwang/Data/spotlight/20250613-fly1b-002/",
        with_muscle=True,
        overwrite=True,
    )
