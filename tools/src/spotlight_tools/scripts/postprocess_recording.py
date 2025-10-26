import sys
import os
import tyro
from pathlib import Path

from spotlight_tools.postprocessing.behavior_video import jpeg_to_mkv
from spotlight_tools.postprocessing.frame_metadata import (
    interpolate_stage_position_for_behavior_images,
)
from spotlight_tools.postprocessing.io import check_is_directory_valid
from spotlight_tools.postprocessing.warp_muscle_image import process_muscle_data
from spotlight_tools.postprocessing.estimate_pose import run_sleap
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
    overwrite: bool = False,
    interpolate_stage_position: bool = True,
    merge_behavior_video: bool = True,
    estimate_2dpose: bool = False,
    warp_muscle_images: bool = False,
    make_visualizations: bool = True,
    play_fps: int = 30,
    behavior_video_crf: int = 5,
    behavior_video_preset: str = "slow",
    sleap_batch_size: int = 128,
    muscle_transform_num_workers: int = -1,
    muscle_vrange: tuple[int, int] | None = None,
    num_frames: int | None = None,
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
    # Handle dependencies of processing stages
    if warp_muscle_images:
        interpolate_stage_position = True
    if estimate_2dpose:
        merge_behavior_video = True

    # IO setup
    recording_dir = Path(recording_dir).expanduser()
    check_is_directory_valid(recording_dir)
    processed_dir = recording_dir / "processed"
    processed_dir.mkdir(exist_ok=True)

    # Interpolate stage position for each behavior frame
    if interpolate_stage_position:
        print("Interpolating stage positions for behavior frames")
        behavior_frames_dir = recording_dir / "behavior_images"
        stage_positions_path = recording_dir / "stage_position/stage_position.csv"
        behavior_timestamps_output_path = processed_dir / "behavior_frames_metadata.csv"
        interpolate_stage_position_for_behavior_images(
            behavior_frames_dir,
            stage_positions_path,
            behavior_timestamps_output_path,
            overwrite,
        )

    # Merge behavior video
    if merge_behavior_video:
        print("Merging behavior frames into a single video")
        behavior_video_path = processed_dir / "behavior_video.mkv"
        jpeg_to_mkv(
            behavior_frames_dir,
            behavior_video_path,
            overwrite,
            play_fps,
            behavior_video_crf,
            behavior_video_preset,
            num_frames=num_frames,
        )

    # Run 2D pose estimation
    if estimate_2dpose:
        print("Detecting fly skeleton (neck-thorax-abdomen) using SLEAP")
        pose_output_path = processed_dir / "pose_2d.npz"
        run_sleap(
            behavior_video_path,
            pose_output_path,
            overwrite=overwrite,
            num_frames=num_frames,
            batch_size=sleap_batch_size,
        )

    # Warp muscle images to match behavior images
    if warp_muscle_images:
        print("Warping muscle images to match behavior images")
        process_muscle_data(
            recording_dir,
            overwrite=overwrite,
            num_frames=num_frames,
            num_workers=muscle_transform_num_workers,
        )

    # Generate visualizations
    if make_visualizations:
        # Make summary video
        print("Generating summary video with behavior, pose, and muscle data...")
        generate_summary_video(
            recording_dir,
            draw_pose=estimate_2dpose,
            draw_muscle=warp_muscle_images,
            num_frames=num_frames,
            overwrite=overwrite,
        )

        # Overlay muscle on top of behavior image for randomly selected samples
        if warp_muscle_images:
            print("Generating overlay samples of behavior and muscle data...")
            generate_overlay_samples(
                recording_dir,
                muscle_vrange=muscle_vrange,
                overwrite=overwrite,
            )


def main():
    tyro.cli(postprocess_recording_data)


if __name__ == "__main__":
    # * CLI
    main()

    # * Example
    # postprocess_recording_data(
    #     recording_dir="/home/sibwang/Data/spotlight/20250613-fly1b-002/",
    #     overwrite=True,
    #     interpolate_stage_position=True,
    #     merge_behavior_video=True,
    #     estimate_2dpose=True,
    #     warp_muscle_images=True,
    #     make_visualizations=True,
    #     play_fps=30,
    #     num_frames=300,
    # )
