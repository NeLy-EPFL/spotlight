#!/usr/bin/env python
"""Run SLEAP inference on a video, via `sleap-track`.

Thin wrapper: handles both top-down (centroid + centered-instance) and
single-instance models transparently, since `sleap-track` itself already
auto-detects which pipeline a model directory implements from its own
training config: pass one `--model` for a single-instance model, two
(centroid, then centered-instance) for a top-down one.

Also handles both model generations: current, sleap-nn (PyTorch) models
(`training_config.yaml`) run via this environment's own `sleap-track`, but
older, pre-sleap-nn (TensorFlow) models (`training_config.json`, e.g. the
original two-stage LM model) can't: this environment has no TensorFlow at
all, and the checkpoint format itself (Keras `best_model.h5`, not a PyTorch
state dict) is different, so there's no simple load-and-convert. Those are
instead dispatched to the isolated `sleap==1.4.1` conda env (same one
`slp_convert_legacy.py --current-to-legacy` uses), which still has
TensorFlow and can load them directly. All `--model` dirs passed together
must be the same generation (mixing them makes no sense: a legacy top-down
pair's centroid and centered-instance stages are always trained together).

`--input-path` may be a video file, or a `.slp` file that itself references
the target video (`sleap-track` supports both as its `DATA_PATH`). A `.slp`
input's format doesn't need to match the model's own generation: the legacy
conda env's `sleap-track` reads both current- and legacy-format `.slp` input
fine (only its own *output* stays legacy, being that env's native format);
this environment's `sleap-track`, however, is only confirmed to work with
current-format input, so that path still requires it.

Usage:
    python slp_run_inference.py \\
        --input-path seed.slp --output-path predictions.slp \\
        --model models/lmport_v000_run000.single_instance.n=467875
"""

import subprocess
from pathlib import Path

import tyro
from loguru import logger
from slp_convert_legacy import (
    LEGACY_SLEAP_PYTHON,
    is_legacy_format,
)

from spotlight.postprocessing.pose2d.io_utils import check_output_path


def is_legacy_model_dir(model_dir: Path) -> bool:
    """Whether `model_dir` is a pre-sleap-nn (TensorFlow) model.

    Args:
        model_dir: Trained model directory, as passed to `sleap-track -m`.

    Returns:
        True if `model_dir` has a `training_config.json` (the old format;
        sleap-nn's own models have `training_config.yaml` instead).
    """
    return (model_dir / "training_config.json").is_file()


def main(
    input_path: Path,
    output_path: Path,
    model: list[Path],
    batch_size: int = 4,
    override: bool = False,
) -> None:
    """Run SLEAP inference (`sleap-track`) on `input_path`.

    Args:
        input_path: Video or `.slp` file to run inference on. If a `.slp`
            file and every `model` is sleap-nn-native, must be in the
            current format (see module docstring); legacy-model dirs accept
            either `.slp` format.
        output_path: Where to save the resulting predictions `.slp`. Aborts
            if this already exists, unless `override` is set.
        model: One or more trained model directories, in `sleap-track`'s own
            `-m` order (single-instance: one; top-down: centroid then
            centered-instance). Must all be the same generation.
        batch_size: Frames per inference batch.
        override: If True, overwrite `output_path` if it already exists.
    """
    if not input_path.is_file():
        raise SystemExit(f"Input file does not exist: {input_path}")
    if not model:
        raise SystemExit("At least one --model is required.")

    legacy_flags = [is_legacy_model_dir(m) for m in model]
    if len(set(legacy_flags)) > 1:
        raise SystemExit(
            f"--model dirs are a mix of legacy and current generations: {model}"
        )
    use_legacy = legacy_flags[0]

    if not use_legacy and input_path.suffix == ".slp" and is_legacy_format(input_path):
        raise SystemExit(
            f"{input_path} is in the legacy SLEAP format; convert it first "
            "with `slp_convert_legacy.py --legacy-to-current`."
        )
    check_output_path(output_path, override)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    sleap_track = (
        str(LEGACY_SLEAP_PYTHON.parent / "sleap-track") if use_legacy else "sleap-track"
    )
    if use_legacy and not LEGACY_SLEAP_PYTHON.is_file():
        raise SystemExit(
            f"Legacy sleap conda env python not found at {LEGACY_SLEAP_PYTHON}; "
            "needed to run inference with a pre-sleap-nn (TensorFlow) model."
        )

    cmd = [
        sleap_track,
        str(input_path),
        "-o",
        str(output_path),
        "--batch_size",
        str(batch_size),
    ]
    for m in model:
        cmd += ["-m", str(m)]
    logger.info(f"Running: {' '.join(cmd)}")

    subprocess.run(cmd, check=True)
    logger.info(f"Saved predictions to {output_path}")


if __name__ == "__main__":
    tyro.cli(main)
