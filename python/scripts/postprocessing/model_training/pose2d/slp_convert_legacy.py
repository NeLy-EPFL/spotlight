#!/usr/bin/env python
"""Convert a `.slp` file between the legacy and current SLEAP file formats.

`sleap_io` (used everywhere else in this pipeline) can already *read* either
format transparently, but can only ever *write* the current format. The
legacy format can only be written by the older, isolated `sleap==1.4.1` conda
environment (see this directory's README for how to create it), so
`--current-to-legacy` shells out to it; `--legacy-to-current` runs entirely
in-process via `sleap_io`.

The two on-disk formats are otherwise nearly identical HDF5 layouts; the only
structural difference found (comparing a real legacy-era `.slp` against a
freshly `sleap_io`-saved one) is a `sessions_json` root group present only in
the current format, used here to detect which format a file is already in.

Usage:
    python slp_convert_legacy.py --legacy-to-current \
        --input-path old.slp --output-path new.slp
    python slp_convert_legacy.py --current-to-legacy \
        --input-path new.slp --output-path old.slp
"""

import subprocess
from pathlib import Path

import h5py
import sleap_io as sio
import tyro
from loguru import logger

from spotlight.postprocessing.pose2d.io_utils import check_output_path

# The `sleap==1.4.1a2` conda env described in this directory's README; the
# only installed environment able to write the legacy format.
LEGACY_SLEAP_PYTHON = Path("/home/sibwang/miniconda3/envs/sleap/bin/python")

_CURRENT_TO_LEGACY_SCRIPT = """
import sys
import sleap

input_path, output_path = sys.argv[1], sys.argv[2]
labels = sleap.load_file(input_path)
labels.save(output_path)
"""


def is_legacy_format(input_path: Path) -> bool:
    """Whether a `.slp` file is in the legacy (pre-`sessions_json`) format.

    Args:
        input_path: `.slp` file to check.

    Returns:
        True if `input_path` has no root `sessions_json` group.
    """
    with h5py.File(input_path, "r") as f:
        return "sessions_json" not in f


def convert_legacy_to_current(input_path: Path, output_path: Path) -> None:
    """Convert a legacy-format `.slp` file to the current format, in-process."""
    labels = sio.load_file(str(input_path))
    sio.save_file(labels, str(output_path))


def convert_current_to_legacy(input_path: Path, output_path: Path) -> None:
    """Convert a current-format `.slp` file to the legacy format, via the
    legacy `sleap` conda environment.
    """
    if not LEGACY_SLEAP_PYTHON.is_file():
        raise SystemExit(
            f"Legacy sleap conda env python not found at {LEGACY_SLEAP_PYTHON}; "
            "--current-to-legacy needs its `sleap==1.4.1` install (see this "
            "directory's README) since sleap_io can only write the current format."
        )
    subprocess.run(
        [str(LEGACY_SLEAP_PYTHON), "-c", _CURRENT_TO_LEGACY_SCRIPT,
         str(input_path), str(output_path)],
        check=True,
    )  # fmt: skip


def main(
    input_path: Path,
    output_path: Path,
    legacy_to_current: bool = False,
    current_to_legacy: bool = False,
    override: bool = False,
) -> None:
    """Convert a `.slp` file between the legacy and current SLEAP formats.

    Args:
        input_path: `.slp` file to convert.
        output_path: Where to save the converted `.slp` file. Aborts if this
            already exists, unless `override` is set.
        legacy_to_current: Convert a legacy-format file to the current
            format. Exactly one of `legacy_to_current`/`current_to_legacy`
            must be set.
        current_to_legacy: Convert a current-format file to the legacy
            format. Exactly one of `legacy_to_current`/`current_to_legacy`
            must be set.
        override: If True, overwrite `output_path` if it already exists.
    """
    if legacy_to_current == current_to_legacy:
        raise SystemExit(
            "Specify exactly one of --legacy-to-current or --current-to-legacy."
        )
    if not input_path.is_file():
        raise SystemExit(f"Input file does not exist: {input_path}")
    check_output_path(output_path, override)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    is_legacy = is_legacy_format(input_path)
    if legacy_to_current:
        if not is_legacy:
            raise SystemExit(f"{input_path} is already in the current format.")
        convert_legacy_to_current(input_path, output_path)
    else:
        if is_legacy:
            raise SystemExit(f"{input_path} is already in the legacy format.")
        convert_current_to_legacy(input_path, output_path)

    direction = "current" if legacy_to_current else "legacy"
    logger.info(f"Saved {direction}-format file to {output_path}")


if __name__ == "__main__":
    tyro.cli(main)
