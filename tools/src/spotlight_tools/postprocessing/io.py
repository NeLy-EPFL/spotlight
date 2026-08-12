"""Low-level I/O helpers for the postprocessing pipeline."""

import logging
from pathlib import Path
from typing import Literal

import h5py
import numpy as np


def find_files_per_frame_by_suffix(
    frames_dir: Path, suffix: str, stride: int = 1
) -> dict[int, Path]:
    """
    Find files in the given directory with the specified suffix and
    return a dictionary mapping frame numbers to file paths. The dictionary
    keys are sorted. This function also checks that the frame numbers are
    consecutive at the specified stride (ie. no frame is missing).
    """
    logger = logging.getLogger(__name__)
    print(f"Checking {suffix} files in {frames_dir}...")
    files = list(frames_dir.glob(f"*{suffix}"))
    files_by_frame = {}
    for file in files:
        try:
            frame_number = int(file.stem.split("_")[-1])
            files_by_frame[frame_number] = file
        except ValueError:
            logger.warning(f"Could not recognize file name '{file}'. Skipping file.")
    sorted_files_by_frame = dict(sorted(files_by_frame.items()))
    last_frame_id = list(sorted_files_by_frame.keys())[-1]
    for frame_id in range(0, last_frame_id + stride, stride):
        if frame_id not in sorted_files_by_frame.keys():
            logger.error(
                f"Frame {frame_id} is missing in the directory {frames_dir}. "
                f"Skipping it."
            )
            continue
    logger.info(
        f"Checked: {suffix} files are continuous from frame 0 to "
        f"frame {last_frame_id} in interval of {stride}."
    )

    return sorted_files_by_frame


class MuscleH5Writer:
    """Appends muscle frames to a resizable HDF5 dataset, one chunk at a
    time, following `~/projects/spotlight-tiff2h5/tifs_to_h5.py`'s on-disk
    convention (uint16, `chunks=(1, H, W)`, gzip, shuffle) -- except written
    directly during warping instead of via a TIFF-directory conversion pass,
    resizable (`maxshape`) since the final frame count isn't known until
    drop-detection/orphan-counting has run, and gzip level 1 rather than
    that script's level 6.

    Level 6 measured at 54.7ms/frame (900x900 uint16, one frame per HDF5
    chunk) -- profiling a real pipeline run found this was the single
    largest cost in the whole streaming pass, bigger than decoding, both
    models' inference, and the actual `cv2.warpPerspective` combined. Level
    1 measured at 8.9ms/frame (6.1x faster) for only ~7% larger output
    (425KB/frame vs 398KB/frame) -- `tifs_to_h5.py` pays the level-6 cost
    once, offline, well after the fact; this pipeline pays it inline, per
    trial, every run, where the tradeoff favors speed.
    """

    def __init__(self, output_path: Path, dataset_name: str, height: int, width: int):
        self.output_path = Path(output_path)
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self._file = h5py.File(self.output_path, "w")
        self._dataset = self._file.create_dataset(
            dataset_name,
            shape=(0, height, width),
            maxshape=(None, height, width),
            dtype=np.uint16,
            chunks=(1, height, width),
            compression="gzip",
            compression_opts=1,
            shuffle=True,
        )

    def append(self, frames: np.ndarray) -> None:
        """`frames`: `(n, H, W)` uint16."""
        if frames.size == 0:
            return
        start = self._dataset.shape[0]
        self._dataset.resize(start + frames.shape[0], axis=0)
        self._dataset[start:] = frames

    def close(self) -> None:
        self._file.close()


def check_output_path_against_alignment_flag(
    output_path: Path, alignment: Literal["aligned", "fullsize", "both"]
):
    """Check if the output path suggests an alignment mode inconsistent with
    `alignment`. If so, log an error message."""
    logger = logging.getLogger(__name__)

    out_path_no_delim = str(output_path).replace("_", "").replace("-", "").lower()
    if alignment == "aligned" and "fullsize" in out_path_no_delim:
        logger.error(
            f"Output path ({output_path}) suggests full-size frames, "
            "but `alignment` is 'aligned'. The output frames WILL be cropped/aligned."
        )
    if alignment == "fullsize" and "aligned" in out_path_no_delim:
        logger.error(
            f"Output path ({output_path}) suggests aligned frames, "
            "but `alignment` is 'fullsize'. The output frames WILL NOT be aligned."
        )
