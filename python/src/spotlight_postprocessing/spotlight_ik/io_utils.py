"""I/O for `tools/spotlight_ik/solve_ik.py`'s periods+IK/FK `.h5` schema.

One file per trial, one group per re-segmented "good" period (see
`solve_ik.py`'s module docstring), each holding 6 datasets: the period's raw
prediction (`pred_2d_px`, `pred_2d_mm`, `confidence`) and its IK/FK fit
(`ik_dofangles_rad`, `fk_3d_mm`, `fk_2d_px`).
"""

from dataclasses import dataclass, fields
from pathlib import Path

import h5py
import numpy as np

RAW_DATASET_NAMES = ("pred_2d_px", "pred_2d_mm", "confidence")
IK_DATASET_NAMES = ("ik_dofangles_rad", "fk_3d_mm", "fk_2d_px")


@dataclass
class Period:
    """One re-segmented "good" period's raw prediction and IK/FK fit.

    `start_idx`/`end_idx` index into the original trial's full frame range
    (`end_idx` exclusive), not this period's own arrays.
    """

    start_idx: int
    end_idx: int
    pred_2d_px: np.ndarray
    """`(period_length, n_nodes, 2)`, raw prediction, aligned pixel domain."""
    pred_2d_mm: np.ndarray
    """`(period_length, n_nodes, 2)`, raw prediction, physical mm."""
    confidence: np.ndarray
    """`(period_length, n_nodes)`, SLEAP keypoint scores."""
    ik_dofangles_rad: np.ndarray
    """`(period_length, n_dofs)`, solved joint angles."""
    fk_3d_mm: np.ndarray
    """`(period_length, n_nodes, 3)`, FK at the converged pose (NaN for
    nodes with no body-plan joint)."""
    fk_2d_px: np.ndarray
    """`(period_length, n_nodes, 2)`, `fk_3d_mm`'s xy mapped back to the
    aligned pixel domain."""


def save_ikfk_h5(
    output_path: Path,
    periods: list[Period],
    node_names: list[str],
    dof_names: list[str],
    video_path: Path,
    extra_attrs: dict | None = None,
) -> None:
    """Save one trial's periods+IK/FK results to the shared `.h5` schema.

    Args:
        output_path: Where to save the `.h5` file.
        periods: One trial's re-segmented periods, in chronological order.
        node_names: SLEAP node names, matching every dataset's node axis
            except `ik_dofangles_rad`.
        dof_names: DOF names, matching `ik_dofangles_rad`'s last axis.
        video_path: Absolute path to the video these periods belong to,
            saved as an attr for downstream tools (e.g. `make_videos.py`)
            that need to re-open it.
        extra_attrs: Additional root `.attrs` to write (e.g. the parameters
            `solve_ik.py` ran with), merged in as-is.
    """
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with h5py.File(output_path, "w") as f:
        f.attrs["node_names"] = node_names
        f.attrs["dof_names"] = dof_names
        f.attrs["video_path"] = str(Path(video_path).resolve())
        for key, value in (extra_attrs or {}).items():
            f.attrs[key] = value

        for period_id, period in enumerate(periods):
            group = f.create_group(str(period_id))
            group.attrs["start_idx"] = period.start_idx
            group.attrs["end_idx"] = period.end_idx
            for name in RAW_DATASET_NAMES + IK_DATASET_NAMES:
                dset = group.create_dataset(
                    name, data=getattr(period, name), compression="gzip"
                )
                if name == "ik_dofangles_rad":
                    dset.attrs["dof_names"] = dof_names
                elif name != "confidence":
                    dset.attrs["node_names"] = node_names


def load_ikfk_h5(input_path: Path) -> dict:
    """Load an `.h5` file written by `save_ikfk_h5` back into a plain dict.

    Args:
        input_path: `.h5` file to load.

    Returns:
        Dict with keys `periods` (`list[Period]`, in chronological order),
        `node_names` (`list[str]`), `dof_names` (`list[str]`), `video_path`
        (`Path`).
    """
    period_fields = [
        f.name for f in fields(Period) if f.name not in ("start_idx", "end_idx")
    ]
    with h5py.File(input_path, "r") as f:
        node_names = [str(n) for n in f.attrs["node_names"]]
        dof_names = [str(n) for n in f.attrs["dof_names"]]
        video_path = Path(str(f.attrs["video_path"]))

        periods = []
        for period_id in sorted(f.keys(), key=int):
            group = f[period_id]
            periods.append(
                Period(
                    start_idx=int(group.attrs["start_idx"]),
                    end_idx=int(group.attrs["end_idx"]),
                    **{name: group[name][:] for name in period_fields},
                )
            )
    return {
        "periods": periods,
        "node_names": node_names,
        "dof_names": dof_names,
        "video_path": video_path,
    }
