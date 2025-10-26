import logging
import numpy as np
import yaml
from pathlib import Path
from subprocess import run
from tqdm import tqdm
from sleap_nn.predict import run_inference

import spotlight_tools
from spotlight_tools.common.video import get_video_info


def _load_config() -> dict:
    spotlight_package_dir = Path(spotlight_tools.__path__[0]).expanduser()
    config_path = spotlight_package_dir.parent.parent / "config/config.yaml"
    if not config_path.exists():
        raise FileNotFoundError(
            f"Configuration file {config_path} does not exist. Make sure the "
            "spotlight-tools package is installed correctly."
        )
    with open(config_path, "r") as f:
        config = yaml.safe_load(f)
    return config


def run_sleap(
    behavior_video_path: Path,
    output_path: Path | None = None,
    sleap_model_dir: Path | None = None,
    batch_size: int = 128,
    num_frames: int | None = None,
    overwrite: bool = False,
) -> tuple[np.ndarray, list[str]]:
    config = _load_config()

    # Validate SLEAP model directory
    if sleap_model_dir is None:
        sleap_model_dir = Path(config["pose2d"]["sleap_model_dir"]).expanduser()
        logging.info(f"Using default SLEAP model from config: {sleap_model_dir}")
    if not sleap_model_dir.exists():
        raise FileNotFoundError(
            f"SLEAP model directory {sleap_model_dir} does not exist. Please "
            "provide a valid SLEAP model directory."
        )

    # Validate output path
    if output_path is None:
        output_path = behavior_video_path.parent / "pose_2d.npz"
    if output_path.exists() and not overwrite:
        raise FileExistsError(
            f"Output file {output_path} already exists. To overwrite, set "
            "overwrite=True."
        )

    # Run SLEAP inference
    sleap_output = run_inference(
        data_path=str(behavior_video_path),
        model_paths=[str(sleap_model_dir)],
        batch_size=batch_size,
        queue_maxsize=batch_size * 2,
        frames=None if num_frames is None else list(range(num_frames)),
    )

    # Extract keypoint coordinates from SLEAP output
    num_frames = len(sleap_output)
    num_keypoints = config["pose2d"]["num_keypoints"]
    node_code2name = config["pose2d"]["keypoint_names"]
    node_names = list(node_code2name.values())
    node_code2idx = {code: i for i, code in enumerate(node_code2name.keys())}
    nodes_xy = np.full((num_frames, num_keypoints, 2), np.nan, dtype=np.float32)
    for i, sleap_label in enumerate(sleap_output):
        num_flies_detected = len(sleap_label.predicted_instances)
        if num_flies_detected == 0:
            continue  # leave as NaN
        elif num_flies_detected == 1:
            fly_instance = sleap_label.predicted_instances[0]
            for j in range(num_keypoints):
                # .points[j] alone gives an np.void array. Last [0] needed to get values
                xy = fly_instance.points[j]["xy"]
                name = fly_instance.points[j]["name"]
                keypoint_idx = node_code2idx[name]
                nodes_xy[i, keypoint_idx, :] = xy
        else:
            logging.error(
                f"Multiple flies detected in frame {i}; this shouldn't happen."
            )

    # Save output to NPZ file
    output_path.parent.mkdir(parents=True, exist_ok=True)
    width, height, num_frames_total = get_video_info(behavior_video_path)
    np.savez(
        output_path,
        nodes_xy=nodes_xy,
        node_names=node_names,
        width=width,
        height=height,
    )

    return nodes_xy, node_names
