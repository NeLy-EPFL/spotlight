"""RepVGG-A0 backbone (via `timm`) with a heatmap regression head, for
37-keypoint pose estimation on 450x450 aligned-domain crops.

RepVGG's multi-branch train-time blocks fuse into a single conv per block
for inference; see `export_model.py`, which calls `timm.utils.reparameterize_model`
on a trained checkpoint before exporting.
"""

import warnings

import torch
from torch import nn


def import_timm():
    """Imports `timm`, suppressing a harmless warning from its `wandb`
    optional dependency: `timm.utils.summary` imports `wandb` unconditionally
    (we never call the logging utilities that need it), and `wandb`'s own
    pydantic models trip an unrelated pydantic-version compatibility warning.
    """
    with warnings.catch_warnings():
        warnings.filterwarnings(
            "ignore", message=r".*was provided to the `Field\(\)` function.*"
        )
        import timm

    return timm


class HeatmapHead(nn.Module):
    """Upsamples the backbone's feature map to a per-keypoint heatmap."""

    def __init__(self, in_channels: int, n_keypoints: int) -> None:
        super().__init__()
        self.deconv = nn.Sequential(
            nn.ConvTranspose2d(in_channels, 256, kernel_size=4, stride=2, padding=1),
            nn.BatchNorm2d(256),
            nn.ReLU(inplace=True),
            nn.ConvTranspose2d(256, 256, kernel_size=4, stride=2, padding=1),
            nn.BatchNorm2d(256),
            nn.ReLU(inplace=True),
        )
        self.head = nn.Conv2d(256, n_keypoints, kernel_size=1)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.head(self.deconv(x))


class RepVGGPoseModel(nn.Module):
    """RepVGG-A0 backbone (32x downsample) + a 4x-upsampling heatmap head,
    so output heatmaps are at 1/8 of the input resolution (e.g. 450x450
    input -> 56x56 heatmaps, backbone's 15x15 upsampled 4x with one pixel
    of rounding slack).
    """

    def __init__(self, n_keypoints: int = 37, pretrained_backbone: bool = True) -> None:
        super().__init__()
        timm = import_timm()

        self.backbone = timm.create_model(
            "repvgg_a0",
            pretrained=pretrained_backbone,
            features_only=True,
            out_indices=(-1,),
        )
        out_channels = self.backbone.feature_info.channels()[-1]
        self.head = HeatmapHead(out_channels, n_keypoints)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        features = self.backbone(x)[-1]
        return self.head(features)
