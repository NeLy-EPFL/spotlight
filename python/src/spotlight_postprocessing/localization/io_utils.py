"""Shared I/O for `TinyLocalizationModel` predictions -- the dense per-frame
`.h5` schema `scripts/postprocessing/model_training/localization/infer.py` writes and
`scripts/postprocessing/model_training/localization/visualize_predictions.py` reads.
"""

from pathlib import Path

import h5py
import numpy as np


def save_localization_predictions_h5(
    output_path: Path,
    keypoints: np.ndarray,
    flipped_prob: np.ndarray,
    node_names: list[str],
    video_path: Path,
) -> None:
    """Save one trial's dense per-frame `TinyLocalizationModel` predictions.

    Args:
        output_path: Where to save the `.h5` file.
        keypoints: `(n_frames, n_nodes, 2)`, in the raw fullsize camera
            frame's own pixel domain (`dataset.NATIVE_FRAME_SIZE`), NaN for
            any frame that wasn't predicted (e.g. no cached input frame).
        flipped_prob: `(n_frames,)`, `sigmoid` of the model's flip logit;
            NaN where `keypoints` is NaN.
        node_names: Node name per column of `keypoints` (see
            `dataset.COARSE_KEYPOINTS`).
        video_path: Absolute path to this trial's raw fullsize video.
    """
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with h5py.File(output_path, "w") as f:
        f.create_dataset("keypoints", data=keypoints, compression="gzip")
        f.create_dataset("flipped_prob", data=flipped_prob, compression="gzip")
        f.attrs["node_names"] = node_names
        f.attrs["video_path"] = str(Path(video_path).resolve())


def load_localization_predictions_h5(input_path: Path) -> dict:
    """Load an `.h5` file written by `save_localization_predictions_h5` back
    into a plain dict.

    Args:
        input_path: `.h5` file to load.

    Returns:
        Dict with keys `keypoints`, `flipped_prob`, `node_names`
        (`list[str]`), `video_path` (`Path`).
    """
    with h5py.File(input_path, "r") as f:
        data = {
            "keypoints": f["keypoints"][:],
            "flipped_prob": f["flipped_prob"][:],
            "node_names": [str(n) for n in f.attrs["node_names"]],
            "video_path": Path(str(f.attrs["video_path"])),
        }
    return data
