import logging
import numpy as np
import yaml
from pathlib import Path
from subprocess import run
from tqdm import tqdm

import spotlight_tools
from sleap_utils.fly37 import preprocess_fly37, node_names
from spotlight_tools.common.video import get_video_info


# Load config
spotlight_package_dir = Path(spotlight_tools.__path__[0]).expanduser()
config_path = spotlight_package_dir.parent / "config/config.yaml"
with open(config_path, "r") as f:
    config = yaml.safe_load(f)
sleap_centroid_model_dir = Path(
    config["pose2d"]["sleap_centroid_model_dir"]
).expanduser()
sleap_centered_instance_model_dir = Path(
    config["pose2d"]["sleap_centered_instance_model_dir"]
).expanduser()
sleap_conda_env_name = config["pose2d"]["sleap_conda_env_name"]


def run_sleap(
    behavior_video_path: Path,
    output_dir: Path,
    sleap_centroid_model_dir: Path = sleap_centroid_model_dir,
    sleap_centered_instance_model_dir: Path = sleap_centered_instance_model_dir,
    sleap_conda_env_name: str = sleap_conda_env_name,
    batch_size: int = 4096,
    num_frames: int | None = None,
    overwrite: bool = False,
) -> tuple[np.ndarray, list[str]]:
    """Run SLEAP on the behavior video.

    Args:
        behavior_video_path (Path):
            Path to the behavior video.
        output_dir (Path):
            Path to save the output files.
        sleap_centroid_model_dir (Path):
            Path to the SLEAP centroid model directory.
        sleap_centered_instance_model_dir (Path):
            Path to the SLEAP centered instance model directory.
        sleap_conda_env_name (str):
            Name of the conda environment to run SLEAP in.
        batch_size (int):
            Number of frames to process in a single `sleap-track` run.
        num_frames (int | None):
            If set, the video will contain only the first `num_frames`. Useful for
            testing. Default is None.
        overwrite (bool):
            If True, overwrite existing files. Default is False.

    Returns:
        np.ndarray:
            2D pose data in the format (num_frames, num_keypoints, 2). The last
            dimension is the x and y coordinates.
        list[str]:
            List of node names (keypoints) used in the pose estimation.
            The names include:
            - Leg keypoints: "{leg}_{keypoint}" where `leg` is from
              {"LF", "LM", "LH", "RF", "RM", "RH"} (L/F for left and right, F/M/H for
              front/middle/hind legs) and `keypoint` is from
              {"ThC", "CTr", "FTi", "TiTa", "Cl"} for thorax-coxa, coxa-trochanter,
              femur-tibia, tibia-tarsus, and claw respectively.
            - Special keypoints: "Th" (thorax), "N" (neck), "A" (abdomen), "LA"/"RA"
              (left/right antenna), "LW"/"RW" (left/right wing).
    """
    if output_dir.exists() and not overwrite:
        logging.error(
            f"Output directory {output_dir} already exists. Use another directory "
            "or set `overwrite=True`."
        )
        raise FileExistsError(
            f"Output directory {output_dir} already exists. Use another directory "
            "or set `overwrite=True`."
        )

    output_dir.mkdir(parents=True, exist_ok=True)
    output_path_2dpose = output_dir / "pose_2d.npz"

    width, height, num_frames_total = get_video_info(behavior_video_path)
    if num_frames is not None:
        num_frames_total = num_frames

    batch_schedule = []
    for start in range(0, num_frames_total, batch_size):
        end = min(start + batch_size, num_frames_total)
        batch_schedule.append((start, end))
    logging.info(
        f"Splitting {num_frames_total} frames into {len(batch_schedule)} batches."
    )

    # Run sleap-track
    all_slp_output_paths = [
        output_dir / f"sleap_output_part{i:03d}.slp"
        for i, _ in enumerate(batch_schedule)
    ]
    for i, (start, end) in tqdm(
        enumerate(batch_schedule),
        desc="Running SLEAP by batch",
        total=len(batch_schedule),
        disable=None,
    ):
        slp_output_path = all_slp_output_paths[i]
        args = [
            "conda",
            "run",
            "-n",
            sleap_conda_env_name,
            "sleap-track",
            str(behavior_video_path),
            "--output",
            str(slp_output_path),
            "--model",
            str(sleap_centroid_model_dir),
            "--model",
            str(sleap_centered_instance_model_dir),
            "--max_instances",
            "1",
            "--tracking.target_instance_count",
            "1",
            "--tracking.max_tracks",
            "1",
            "--gpu",
            "auto",
            "--frames",
            f"{start}-{end - 1}",
            "--verbosity",
            "none",
        ]
        logging.info(f"Calling `{' '.join(args)}`")
        with open(output_dir / f"sleap_track_part{i:03d}.log", "w") as log_file:
            run(args, check=True, stdout=log_file, stderr=log_file)

    # Extract 2D pose data from SLEAP output files
    nodes_xy_all_list = []
    for slp_output_path in all_slp_output_paths:
        nodes_xy = preprocess_fly37(
            slp_path=slp_output_path,
            n_target_tracks=1,
            interpolation_limit=35,
        )
        if nodes_xy.shape[1] != 1:
            raise ValueError(
                f"Expected 1 target track (i.e. 1 fly), but got {nodes_xy.shape[1]} tracks."
            )
        nodes_xy = nodes_xy.squeeze(axis=1)  # Remove the track dimension
        nodes_xy_all_list.append(nodes_xy)
    nodes_xy_all = np.concatenate(nodes_xy_all_list, axis=0)

    np.savez(
        output_path_2dpose,
        nodes_xy=nodes_xy_all,
        node_names=node_names,
        width=width,
        height=height,
    )

    return nodes_xy_all, node_names
