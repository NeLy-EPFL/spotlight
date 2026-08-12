#!/usr/bin/env python
"""Exports a trained pose2d checkpoint to ONNX and TorchScript, each at fp32
and fp16. The TorchScript files (`*.torchscript.pt`) are for loading in a
different codebase's own PyTorch inference code without that codebase
needing this project's `RepVGGPoseModel` class at all (unlike a plain
`state_dict` checkpoint, which needs the exact same class already defined
and imported wherever it's loaded); the ONNX files (`*.onnx`) are for
running outside of PyTorch entirely, e.g. in a real-time application.
Mirrors `slp_export_onnx.py`'s role for the LM model.

Fuses RepVGG's multi-branch train-time blocks into a single conv per block
first (`timm.utils.reparameterize_model`), the whole point of RepVGG's
design: a plain conv stack at inference time, no branches to pay for.

Usage:
    python tools/spotlight_pose2d/export_model.py \\
        --checkpoint-path bulk_data/.../best.pt --output-stem bulk_data/.../best
"""

from pathlib import Path

import torch
import tyro

from spotlight_postprocessing.pose2d.dataset import INPUT_SIZE
from spotlight_postprocessing.pose2d.model import RepVGGPoseModel, import_timm
from spotlight_postprocessing.common import export_onnx_and_torchscript


def main(
    checkpoint_path: Path,
    output_stem: Path,
    n_keypoints: int = 37,
    override: bool = False,
) -> None:
    """Export a trained checkpoint to ONNX and TorchScript, at fp32 and fp16.

    Args:
        checkpoint_path: Trained `RepVGGPoseModel` state dict.
        output_stem: Base path (no extension) for the exported files, e.g.
            `bulk_data/.../best`; saves `<output_stem>.fp32.onnx`,
            `<output_stem>.fp16.onnx`, `<output_stem>.fp32.torchscript.pt`,
            and `<output_stem>.fp16.torchscript.pt`. Aborts if any already
            exists, unless `override` is set.
        n_keypoints: Must match the checkpoint's `n_keypoints`.
        override: If True, overwrite existing output files.
    """
    model = RepVGGPoseModel(n_keypoints, pretrained_backbone=False)
    model.load_state_dict(torch.load(checkpoint_path, map_location="cpu"))
    model.eval()
    model = import_timm().utils.reparameterize_model(model)

    export_onnx_and_torchscript(
        model_fp32=model,
        checkpoint_path=checkpoint_path,
        output_stem=output_stem,
        dummy_input_shape=(1, 3, INPUT_SIZE, INPUT_SIZE),
        onnx_output_names=["heatmaps"],
        override=override,
    )


if __name__ == "__main__":
    tyro.cli(main)
