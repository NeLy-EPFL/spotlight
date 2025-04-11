import tyro
from pathlib import Path

from spotlight_tools.postprocessing.video import jpeg_to_mkv
from spotlight_tools.postprocessing.frame_metadata import (
    consolidate_behavior_acquisition_times_and_stage_positions,
)
from spotlight_tools.postprocessing.io import (
    check_is_directory_valid,
)


def postprocess_recording_data(
    recording_dir: Path,
    overwrite: bool = False,
    play_fps: int = 30,
    behavior_video_crf: int = 5,
    behavior_video_preset: str = "slow",
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
    
    Example:
        poetry run python scripts/postprocess_recording.py \
            --recording_dir /mnt/spotlight-data/DM/250407-t66-mdn-chrimson/fly0_pw150/ \
            --behavior-video-crf 16 \
            --behavior-video-preset slow
        
        This will create a folder under the recording directory called
        "processed" and save the postprocessed files there. The video will
        have a CRF of 16 and using the "slow" preset parameters.

    Args:
        recording_dir (Path):
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
    """
    check_is_directory_valid(recording_dir)
    processed_dir = recording_dir / "processed"
    processed_dir.mkdir(exist_ok=True)

    behavior_frames_dir = recording_dir / "behavior_images"
    stage_positions_path = recording_dir / "stage_position/stage_position.csv"
    behavior_video_path = processed_dir / "behavior_video.mkv"
    behavior_timestamps_path = processed_dir / "behavior_frames_metadata.csv"
    consolidate_behavior_acquisition_times_and_stage_positions(
        behavior_frames_dir, stage_positions_path, behavior_timestamps_path, overwrite
    )
    jpeg_to_mkv(
        behavior_frames_dir,
        behavior_video_path,
        overwrite,
        play_fps,
        behavior_video_crf,
        behavior_video_preset,
        num_frames,
    )


if __name__ == "__main__":
    tyro.cli(postprocess_recording_data)
