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
    estimate_2dpose: bool = True,
    warp_muscle_images: bool = True,
    make_visualizations: bool = True,
    play_fps: int = 30,
    behavior_video_crf: int = 5,
    behavior_video_preset: str = "slow",
    sleap_batch_size: int = 128,
    muscle_vrange: tuple[int, int] | None = None,
    num_frames: int | None = None,
) -> None:
    """Postprocess data recorded by the Spotlight setup.

    First, this function consolidate metadata for each behavior frame,
    generating a single CSV file containing the acquisition time (from
    camera's internal clock), image-received time (in UNIX epoch time), and
    the estimated positions of the motion stages (their positions are
    logged at a frequency typically lower than the behavior recording
    frequency, so some interpolation is needed.

    Second, this function merges the behavior camera frames into a single
    video containing monochrome frames using the H.264 codec. Note: during
    recording, the behavior frames are originally saved in 'pseudo-BGR'
    files. Namely, each file actually contains three consecutive frames in
    the three channels. The output of this function no longer employs this
    trick - each frame is just an actual monochrame frame.

    For more information on the video compression parameters, see
    https://trac.ffmpeg.org/wiki/Encode/H.264

    Args:
        recording_dir (Path or str):
            Root directory of the recording. This is the path that you set
            in the Spotlight recording GUI.
        overwrite (bool):
            If True, this function will overwrite existing files.
            Otherwise, an exception is raised if the output file(s) already
            exists. Default is False.
        play_fps (int):
            Frames per second for the output video. Note that this value is
            only used for *displaying* the data. The real FPS is set when
            the data is collected. For example, if data is collected at 300
            FPS, and `play_fps` here is set to 30, then the video will be
            played at 0.1x speed when opened by a video player (even if the
            video player thinks it's playing at 1x speed). This can be
            handy sometimes. Default is 30.
        behavior_video_crf (int):
            Constant Rate Factor (CRF) to be used for video compression.
            CRF is a quality-based encoding method that lets users target a
            specific quality level rather than bitrate. CRF values range
            from 0 to 51. Lower values = higher quality and larger files,
            vice versa. Generally, 0 is mathematically lossless; 1-5 are
            visually lossless; 15-18 are very high quality. Default is 5.
        behavior_video_preset (str):
            A preset is a collection of options that will provide a certain
            encoding speed to compression ratio. A slower preset will
            provide better compression (compression is quality per
            filesize). Choose from: "veryslow", "slower", "slow", "medium",
            "fast", "faster", "veryfast", "superfast", "ultrafast". Default
            is "slow".
        num_frames (int, optional):
            If set, the video will contain only the first `num_frames`
            frames. This is useful if you want to generate a very short
            video just to make sure that the data pipeline is working.
            Default is None.
        muscle_camera (bool):
            If True, muscle images are warped to be consistent with
            behavior images. Furthermore, a video of the behavior-muscle
            overlay will be generated.
        sleap_batch_size (int):
            Batch size for SLEAP pose estimation. This is the number of
            frames to process in a single `sleap-track` run. Default is 128.
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
        process_muscle_data(recording_dir, overwrite=overwrite, num_frames=num_frames)

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
    # main()
    postprocess_recording_data(
        recording_dir="/home/sibwang/Data/spotlight/20250613-fly1b-002/",
        overwrite=True,
        interpolate_stage_position=True,
        merge_behavior_video=True,
        estimate_2dpose=True,
        warp_muscle_images=True,
        make_visualizations=True,
        num_frames=300,
    )
