#!/usr/bin/env python
# /// script
# requires-python = ">=3.13"
# dependencies = [
#     "tensorflow",
#     "tf2onnx",
#     "onnx",
#     "tyro",
#     "loguru",
# ]
# ///
"""Export a legacy (TensorFlow/Keras) SLEAP model to ONNX.

`sleap==1.6.3` (this project's version) runs on `sleap-nn` (PyTorch), so the
main `.venv` has no TensorFlow; this script instead declares its own
dependencies above and is meant to be run standalone via `uv run`, e.g.:

    uv run tools/spotlight_pose2d/slp_export_onnx.py \\
        --model-dir bulk_data/.../my_model.centroid \\
        --output-path bulk_data/.../my_model.centroid/best_model.onnx

`uv run` builds a throwaway environment for these dependencies (~600 MB,
mostly TensorFlow) the first time this runs, then reuses it.
"""

import os

# Set before importing tensorflow. On this machine, letting TensorFlow probe
# a partially-installed CUDA stack makes the ONNX exporter's graph-optimizer
# step crash outright instead of falling back to CPU.
os.environ.setdefault("CUDA_VISIBLE_DEVICES", "-1")

from pathlib import Path

import tyro
from loguru import logger


def main(
    model_dir: Path, output_path: Path | None = None, override: bool = False
) -> None:
    """Export `model_dir/best_model.h5` to ONNX.

    Args:
        model_dir: SLEAP model directory (contains `best_model.h5`).
        output_path: Where to write the ONNX model. Defaults to
            `model_dir/best_model.onnx`.
        override: If True, overwrite `output_path` if it already exists.
    """
    import tensorflow as tf

    h5_path = model_dir / "best_model.h5"
    if not h5_path.is_file():
        raise SystemExit(f"No best_model.h5 found in {model_dir}")

    output_path = output_path or model_dir / "best_model.onnx"
    if output_path.exists() and not override:
        raise SystemExit(
            f"{output_path} already exists; pass --override to overwrite it."
        )

    model = tf.keras.models.load_model(h5_path, compile=False)

    # Keras 3's ONNX exporter requires the model to have been called at
    # least once (to build its graph); the batch dimension is the only one
    # left unspecified, so fill it in with 1.
    input_shape = [dim if dim is not None else 1 for dim in model.inputs[0].shape]
    model(tf.zeros(input_shape, dtype=tf.float32))

    model.export(str(output_path), format="onnx")
    logger.info(f"Exported {h5_path} to {output_path}")


if __name__ == "__main__":
    tyro.cli(main)
