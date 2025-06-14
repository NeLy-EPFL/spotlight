import logging
import h5py
import numpy as np
import yaml
from pathlib import Path
from subprocess import run
from tqdm import tqdm

from sleap_utils.fly37 import preprocess_fly37
from spotlight_tools.common.video import get_video_info


# fmt: off
_sleap_model_base_dir = Path("~/Data/sleap/models/tl_bottom_cam_1024_250327_002024/").expanduser()
_sleap_centroid_model_dir = _sleap_model_base_dir / "250327_002024.centroid"
_sleap_centered_instance_model_dir = (_sleap_model_base_dir / "250327_002024.centered_instance")
_sleap_conda_env_name = "sleap"
# fmt: on


def run_sleap(
    behavior_video_path: Path,
    output_dir: Path,
    sleap_centroid_model_dir: Path = _sleap_centroid_model_dir,
    sleap_centered_instance_model_dir: Path = _sleap_centered_instance_model_dir,
    sleap_conda_env_name: str = _sleap_conda_env_name,
    batch_size: int = 4096,
    num_frames: int | None = None,
    overwrite: bool = False,
) -> np.ndarray:
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
        num_frames (int | None):
            If set, the video will contain only the first `num_frames`. Useful for
            testing. Default is None.
        overwrite (bool):
            If True, overwrite existing files. Default is False.

    Returns:
        np.ndarray:
            2D pose data in the format (2, num_nodes, num_frames). The first dimension
            is the x and y coordinates.
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
    for i, (start, end) in tqdm(
        enumerate(batch_schedule),
        desc="Running SLEAP by batch",
        total=len(batch_schedule),
        disable=None,
    ):
        raw_output_path = output_dir / f"sleap_output_part{i:03d}.slp"
        analyzed_output_path = output_dir / f"sleap_output_analyzed_part{i:03d}.h5"
        args = [
            "conda",
            "run",
            "-n",
            sleap_conda_env_name,
            "sleap-track",
            str(behavior_video_path),
            "--output",
            str(raw_output_path),
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
        print(" ".join(args))
        run(args, check=True)

        nodes_xy = preprocess_fly37(
            slp_path=raw_output_path,
            n_target_tracks=1,
            interpolation_limit=35,
        )
        np.savez(output_dir / f"pose_2d_part{i:03d}.npz", nodes_xy=nodes_xy)

    #     # Run sleap-convert
    #     args = [
    #         "conda",
    #         "run",
    #         "-n",
    #         sleap_conda_env_name,
    #         "sleap-convert",
    #         str(raw_output_path),
    #         "--output",
    #         str(analyzed_output_path),
    #         "--format",
    #         "analysis",
    #     ]
    #     run(args, check=True)

    #     # Check if the output file exists
    #     if not analyzed_output_path.exists():
    #         raise RuntimeError(
    #             f"SLEAP analysis failed. Output file {analyzed_output_path} not found."
    #         )

    # # Merge the output files
    # with h5py.File(analyzed_output_path, "r") as f:
    #     node_names = [x.decode("utf-8") for x in f["node_names"]]
    #     edge_names = [[y.decode("utf-8") for y in x] for x in f["edge_names"]]
    #     edge_indices = f["edge_inds"][:, :]

    # pose_data_2d = np.empty((2, len(node_names), num_frames_total), dtype=np.float32)

    # for i, (start, end) in enumerate(batch_schedule):
    #     raw_output_path = output_dir / f"sleap_output_part{i:03d}.slp"
    #     analyzed_output_path = output_dir / f"sleap_output_analyzed_part{i:03d}.h5"
    #     with h5py.File(analyzed_output_path, "r") as f:
    #         node_names = [x.decode("utf-8") for x in f["node_names"]]
    #         edge_names = [[y.decode("utf-8") for y in x] for x in f["edge_names"]]
    #         edge_indices = f["edge_inds"][:, :]
    #         track_id = 0  # max_tracks set to 1, therefore this is the only track
    #         data_block = f["tracks"][0, :, :, :].shape

    #         if i == 0:
    #             pose_data_2d = np.empty(
    #                 (2, len(node_names), num_frames_total), dtype=np.float32
    #             )

    #         pose_data_2d[:, :, start:end] = f["tracks"][track_id, :, :, :]

    # # Save the output files
    # np.savez(
    #     output_path_2dpose,
    #     pose_data_2d=pose_data_2d,
    #     node_names=node_names,
    #     edge_names=edge_names,
    #     edge_indices=edge_indices,
    #     batch_schedule=batch_schedule,
    # )

    # return pose_data_2d
