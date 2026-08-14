"""Exports a trained `TinyLocalizationModel` checkpoint to a plain fp16
`state_dict` checkpoint (`*.fp16.pt`, this project's own runtime format,
see `behavior._load_localization_model`), fp16 TorchScript
(`*.fp16.torchscript.pt`, for loading in a different codebase's own
PyTorch inference code without that codebase needing this project's
`TinyLocalizationModel` class at all), and fp16 ONNX (`*.fp16.onnx`, for
running outside of PyTorch entirely). Mirrors `pose2d.export`'s role.
"""

from pathlib import Path

import torch

from spotlight.postprocessing.common import export_neural_network_model
from spotlight.postprocessing.localization.constants import OUTPUT_SIZE
from spotlight.postprocessing.localization.model import TinyLocalizationModel


def export_checkpoint(
    checkpoint_path: Path,
    output_stem: Path,
    use_global_context: bool = True,
    override: bool = False,
) -> None:
    """Export a trained `TinyLocalizationModel` checkpoint to a plain fp16
    `state_dict`, fp16 TorchScript, and fp16 ONNX.

    Args:
        checkpoint_path: Trained `TinyLocalizationModel` state dict.
        output_stem: Base path (no extension) for the exported files, e.g.
            `bulk_data/.../best`; saves `<output_stem>.fp16.pt`,
            `<output_stem>.fp16.torchscript.pt`, and `<output_stem>.fp16.onnx`.
            Aborts if any already exists, unless `override` is set.
        use_global_context: Must match what `checkpoint_path` was actually
            trained with: False for v1-v5 checkpoints (trained before
            `model.GlobalContextBlock` existed), True from v6 on.
        override: If True, overwrite existing output files.
    """
    model = TinyLocalizationModel(use_global_context=use_global_context)
    model.load_state_dict(torch.load(checkpoint_path, map_location="cpu"))
    model.eval()

    width, height = OUTPUT_SIZE
    export_neural_network_model(
        model_fp32=model,
        checkpoint_path=checkpoint_path,
        output_stem=output_stem,
        dummy_input_shape=(1, 3, height, width),
        onnx_output_names=["keypoints", "flipped_logit"],
        override=override,
    )
