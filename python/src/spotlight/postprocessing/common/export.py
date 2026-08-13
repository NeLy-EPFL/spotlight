"""Shared helper for exporting a trained model to a plain fp16 `state_dict`,
fp16 TorchScript, and fp16 ONNX, following the `<output_stem>.{pt,
torchscript.pt, onnx}` naming convention used by `pose2d.export.
export_checkpoint` and `localization.export.export_checkpoint`. fp16 only:
that's this project's own runtime precision (see `behavior._load_
localization_model`/`_load_pose2d_model`) and the only precision any other
consumer of these exports has actually needed either.
"""

from pathlib import Path

import torch
from loguru import logger
from torch import nn

from spotlight.postprocessing.pose2d.io_utils import check_output_path


def export_model(
    model_fp32: nn.Module,
    checkpoint_path: Path,
    output_stem: Path,
    dummy_input_shape: tuple[int, ...],
    onnx_output_names: list[str],
    override: bool,
) -> None:
    """Exports `model_fp32` to a plain fp16 `state_dict`, fp16 TorchScript,
    and fp16 ONNX.

    Saves `<output_stem>.pt`, `<output_stem>.torchscript.pt`, and
    `<output_stem>.onnx`. `<output_stem>.pt` is this project's own runtime
    format (see `behavior._load_localization_model`/`_load_pose2d_model`):
    a plain `state_dict`, loadable only with `model_fp32`'s own class
    already defined and instantiated. `<output_stem>.torchscript.pt` is
    self-contained (loadable via `torch.jit.load`, no model class needed),
    for a different codebase's own PyTorch inference code.
    `<output_stem>.onnx` is for running outside of PyTorch entirely.

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
    plain_path = output_stem.with_name(f"{output_stem.name}.pt")
    torchscript_path = output_stem.with_name(f"{output_stem.name}.torchscript.pt")
    onnx_path = output_stem.with_name(f"{output_stem.name}.onnx")
    for path in (plain_path, torchscript_path, onnx_path):
        check_output_path(path, override)
        path.parent.mkdir(parents=True, exist_ok=True)

    # Non-floating buffers (e.g. BatchNorm's `num_batches_tracked`) are left
    # as-is; halving an integer tensor would corrupt it.
    half_state_dict = {
        k: v.half() if v.is_floating_point() else v
        for k, v in model_fp32.state_dict().items()
    }
    torch.save(half_state_dict, plain_path)
    logger.info(f"Exported {checkpoint_path} -> {plain_path} (plain fp16 state_dict)")

    # `torch.jit.trace` (unlike `torch.jit.script`) records the literal
    # tensor ops seen for one concrete example run, so anything a traced
    # model computes from `some_tensor.device` (e.g. a coordinate grid
    # built fresh each call) gets baked in as a constant tied to whatever
    # device tracing happened on: `.to()`/`map_location` on the loaded
    # module can't undo that afterward. Tracing on the real deployment
    # device (GPU, this project's own tuned/expected path) avoids that
    # trap for the common case; a model like this run purely on CPU would
    # need its own separate CPU-traced export instead.
    device = "cuda" if torch.cuda.is_available() else "cpu"
    model = model_fp32.half().to(device)
    dummy_input = torch.zeros(*dummy_input_shape, dtype=torch.float16, device=device)

    with torch.no_grad():
        traced = torch.jit.trace(model, dummy_input)
    traced.save(str(torchscript_path))
    logger.info(f"Exported {checkpoint_path} -> {torchscript_path} (TorchScript, fp16)")

    torch.onnx.export(
        model,
        dummy_input,
        str(onnx_path),
        input_names=["image"],
        output_names=onnx_output_names,
        dynamic_shapes={"x": {0: torch.export.Dim("batch")}},
        external_data=False,
    )
    logger.info(f"Exported {checkpoint_path} -> {onnx_path} (ONNX, fp16)")
