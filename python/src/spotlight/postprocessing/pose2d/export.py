"""Exports a trained pose2d checkpoint to a plain fp16 `state_dict`
checkpoint (`*.fp16.pt`, this project's own runtime format, see
`behavior._load_pose2d_model`), fp16 TorchScript (`*.fp16.torchscript.pt`,
for loading in a different codebase's own PyTorch inference code without
that codebase needing this project's `RepVGGPoseModel` class at all), and
fp16 ONNX (`*.fp16.onnx`, for running outside of PyTorch entirely, e.g. in
a real-time application). Mirrors `localization.export`'s role.

Fuses RepVGG's multi-branch train-time blocks into a single conv per block
first (`timm.utils.reparameterize_model`), the whole point of RepVGG's
design: a plain conv stack at inference time, no branches to pay for.
"""

from pathlib import Path

import torch

from spotlight.postprocessing.common import export_neural_network_model
from spotlight.postprocessing.pose2d.constants import INPUT_SIZE, N_KEYPOINTS
from spotlight.postprocessing.pose2d.model import RepVGGPoseModel, import_timm


def export_checkpoint(
    checkpoint_path: Path,
    output_stem: Path,
    n_keypoints: int = N_KEYPOINTS,
    override: bool = False,
) -> None:
    """Export a trained checkpoint to a plain fp16 `state_dict`, fp16
    TorchScript, and fp16 ONNX.

    Args:
        checkpoint_path: Trained `RepVGGPoseModel` state dict.
        output_stem: Base path (no extension) for the exported files, e.g.
            `bulk_data/.../best`; saves `<output_stem>.fp16.pt`,
            `<output_stem>.fp16.torchscript.pt`, and `<output_stem>.fp16.onnx`.
            Aborts if any already exists, unless `override` is set.
        n_keypoints: Must match the checkpoint's `n_keypoints`.
        override: If True, overwrite existing output files.
    """
    model = RepVGGPoseModel(n_keypoints, pretrained_backbone=False)
    model.load_state_dict(torch.load(checkpoint_path, map_location="cpu"))
    model.eval()
    model = import_timm().utils.reparameterize_model(model)

    export_neural_network_model(
        model_fp32=model,
        checkpoint_path=checkpoint_path,
        output_stem=output_stem,
        dummy_input_shape=(1, 3, INPUT_SIZE, INPUT_SIZE),
        onnx_output_names=["heatmaps"],
        override=override,
    )
