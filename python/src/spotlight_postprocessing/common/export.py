"""Shared helper for exporting a trained model to ONNX and TorchScript, at
both fp32 and fp16, following the `<output_stem>.{fp32,fp16}.{onnx,
torchscript.pt}` naming convention used by `scripts/postprocessing/model_training/pose2d/
export_model.py` and `scripts/postprocessing/model_training/localization/export_model.py`.
"""

import copy
from pathlib import Path

import torch
from loguru import logger
from torch import nn

from spotlight_postprocessing.pose2d.io_utils import check_output_path

PRECISIONS = ("fp32", "fp16")


def export_onnx_and_torchscript(
    model_fp32: nn.Module,
    checkpoint_path: Path,
    output_stem: Path,
    dummy_input_shape: tuple[int, ...],
    onnx_output_names: list[str],
    override: bool,
) -> None:
    """Exports `model_fp32` to ONNX and TorchScript, at fp32 and fp16.

    Saves `<output_stem>.fp32.onnx`, `<output_stem>.fp16.onnx`,
    `<output_stem>.fp32.torchscript.pt`, and
    `<output_stem>.fp16.torchscript.pt`. The TorchScript files are
    self-contained: loadable via `torch.jit.load`, with no dependency on
    `model_fp32`'s own class (unlike a plain `state_dict` checkpoint). The
    ONNX files are for running outside of PyTorch entirely.

    Args:
        model_fp32: Trained model, already `eval()`'d and otherwise ready
            to run (e.g. RepVGG-reparameterized), at fp32.
        checkpoint_path: Where `model_fp32`'s weights came from, for logging.
        output_stem: Base path (no extension), e.g. `checkpoint_dir / "best"`.
        dummy_input_shape: Shape for `torch.jit.trace`/`torch.onnx.export`'s
            example input, e.g. `(1, 3, height, width)`.
        onnx_output_names: `torch.onnx.export`'s `output_names`.
        override: If True, overwrite any output file that already exists.
    """
    output_paths = {
        (precision, ext): output_stem.with_name(f"{output_stem.name}.{precision}.{ext}")
        for precision in PRECISIONS
        for ext in ("onnx", "torchscript.pt")
    }
    for path in output_paths.values():
        check_output_path(path, override)
        path.parent.mkdir(parents=True, exist_ok=True)

    models_by_precision = {"fp32": model_fp32, "fp16": copy.deepcopy(model_fp32).half()}

    for precision in PRECISIONS:
        model = models_by_precision[precision]
        dtype = torch.float16 if precision == "fp16" else torch.float32
        dummy_input = torch.zeros(*dummy_input_shape, dtype=dtype)

        torchscript_path = output_paths[(precision, "torchscript.pt")]
        with torch.no_grad():
            traced = torch.jit.trace(model, dummy_input)
        traced.save(str(torchscript_path))
        logger.info(
            f"Exported {checkpoint_path} -> {torchscript_path} (TorchScript, {precision})"
        )

        onnx_path = output_paths[(precision, "onnx")]
        torch.onnx.export(
            model,
            dummy_input,
            str(onnx_path),
            input_names=["image"],
            output_names=onnx_output_names,
            dynamic_shapes={"x": {0: torch.export.Dim("batch")}},
            external_data=False,
        )
        logger.info(f"Exported {checkpoint_path} -> {onnx_path} (ONNX, {precision})")
