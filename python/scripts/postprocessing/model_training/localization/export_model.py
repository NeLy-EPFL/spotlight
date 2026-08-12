#!/usr/bin/env python
"""Exports a trained `TinyLocalizationModel` checkpoint to ONNX and TorchScript,
each at fp32 and fp16. The TorchScript files (`*.torchscript.pt`) are for
loading in a different codebase's own PyTorch inference code without that
codebase needing this project's `TinyLocalizationModel` class at all (unlike a
plain `state_dict` checkpoint, which needs the exact same class already
defined and imported wherever it's loaded); the ONNX files (`*.onnx`) are
for running outside of PyTorch entirely. Mirrors `spotlight_pose2d/
export_model.py`'s role.

Usage:
    python scripts/postprocessing/model_training/localization/export_model.py \\
        --checkpoint-path bulk_data/.../best.pt --output-stem bulk_data/.../best
"""

from pathlib import Path

import torch
import tyro

from spotlight_postprocessing.localization.dataset import OUTPUT_SIZE
from spotlight_postprocessing.localization.model import TinyLocalizationModel
from spotlight_postprocessing.common import export_onnx_and_torchscript


def main(
    checkpoint_path: Path,
    output_stem: Path,
    use_global_context: bool = True,
    override: bool = False,
) -> None:
    """Export a trained `TinyLocalizationModel` checkpoint to ONNX and
    TorchScript, at fp32 and fp16.

    Args:
        checkpoint_path: Trained `TinyLocalizationModel` state dict.
        output_stem: Base path (no extension) for the exported files, e.g.
            `bulk_data/.../best`; saves `<output_stem>.fp32.onnx`,
            `<output_stem>.fp16.onnx`, `<output_stem>.fp32.torchscript.pt`,
            and `<output_stem>.fp16.torchscript.pt`. Aborts if any already
            exists, unless `override` is set.
        use_global_context: Must match what `checkpoint_path` was actually
            trained with -- False for v1-v5 checkpoints (trained before
            `model.GlobalContextBlock` existed), True from v6 on.
        override: If True, overwrite existing output files.
    """
    model = TinyLocalizationModel(use_global_context=use_global_context)
    model.load_state_dict(torch.load(checkpoint_path, map_location="cpu"))
    model.eval()

    width, height = OUTPUT_SIZE
    export_onnx_and_torchscript(
        model_fp32=model,
        checkpoint_path=checkpoint_path,
        output_stem=output_stem,
        dummy_input_shape=(1, 3, height, width),
        onnx_output_names=["keypoints", "flipped_logit"],
        override=override,
    )


if __name__ == "__main__":
    tyro.cli(main)
