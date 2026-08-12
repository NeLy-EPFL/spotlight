"""A small CNN for coarse fly localization directly on the raw (unaligned,
uncropped) camera frame, at 0.25x resolution: head/thorax/abdomen position
plus a binary "is the fly upside down or sideways" flag.

Distinct from `spotlight_postprocessing.spotlight_pose2d`'s RepVGG-A0 model,
which needs the fly already identified, rotated, and cropped -- this one
runs earlier in the pipeline, so it has to be small and cheap enough to run
on the full frame every frame.

The keypoint head predicts a small heatmap per keypoint and extracts its
position via a differentiable soft-argmax (spatial softmax, then the
expected coordinate under that distribution) rather than global-average-
pooling the trunk down to one vector and regressing coordinates from that
directly. The first version of this model did exactly that (GAP -> FC ->
sigmoid coordinates), and it never learned anything: global average
pooling destroys spatial information before the keypoint head ever sees
it, so the best it can do is predict the training set's mean position
regardless of the input image (confirmed empirically -- its predictions
barely changed across completely different random inputs). The flip head
doesn't have this problem (whether the fly is flipped is a genuinely
global property of the frame, not a location to point to), so it keeps
the original GAP -> FC -> logit design.

`GlobalContextBlock` feeds the keypoint head a second signal alongside its
local features: measured empirically (backprop from one heatmap pixel),
the plain 3-ConvBlock trunk's receptive field is only ~92x92 raw camera
pixels, well under a fifth of the fly's own head-to-abdomen-tip span
(~460px, measured from spotlight_pose2d's own canonical aligned points).
From any single heatmap location the network can only ever see a local
patch of the fly's body -- nowhere near enough to tell, from local texture
alone, "is this near the head end or the abdomen end," which is exactly
the kind of thing that would produce the centroid-shrinkage bias
`heatmap_variance`/`gaussian_heatmap_loss` were built to fight, rather
than being purely a training-signal problem. Global-average-pooling the
trunk and broadcasting the result back to every spatial location (as an
extra set of channels, concatenated before the heatmap head) gives every
output location the whole frame's own context (unbounded receptive field)
on top of its existing local detail, without changing the heatmap's own
spatial resolution.
"""

import torch
from torch import nn

N_KEYPOINTS = 3  # head, thorax, abdomen -- see dataset.COARSE_KEYPOINTS


class ConvBlock(nn.Module):
    """3x3 conv, stride 2, BatchNorm, ReLU -- halves spatial resolution."""

    def __init__(self, in_channels: int, out_channels: int) -> None:
        super().__init__()
        self.conv = nn.Conv2d(
            in_channels, out_channels, kernel_size=3, stride=2, padding=1, bias=False
        )
        self.bn = nn.BatchNorm2d(out_channels)
        self.relu = nn.ReLU(inplace=True)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.relu(self.bn(self.conv(x)))


class GlobalContextBlock(nn.Module):
    """Global-average-pools the trunk's feature map into one context
    vector per sample, projects it, and broadcasts it back to every
    spatial location, concatenated onto the trunk's own local features --
    gives every location whole-frame context (unbounded receptive field)
    alongside its existing local detail, without changing spatial
    resolution. See the module docstring for why this matters here.
    """

    def __init__(self, in_channels: int, context_channels: int) -> None:
        super().__init__()
        self.pool = nn.AdaptiveAvgPool2d(1)
        self.project = nn.Sequential(
            nn.Linear(in_channels, context_channels), nn.ReLU(inplace=True)
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """`(batch, in_channels + context_channels, height, width)`: `x`
        with its own global-context vector appended as extra channels,
        broadcast to every spatial location.
        """
        context = self.project(self.pool(x).flatten(1))
        _, _, height, width = x.shape
        broadcast = context[:, :, None, None].expand(-1, -1, height, width)
        return torch.cat([x, broadcast], dim=1)


class HeatmapHead(nn.Module):
    """Upsamples the trunk's feature map 2x, then a 1x1 conv produces one
    raw heatmap per keypoint (no activation -- `soft_argmax` applies its
    own softmax).
    """

    def __init__(
        self, in_channels: int, n_keypoints: int, hidden_channels: int
    ) -> None:
        super().__init__()
        self.upsample = nn.ConvTranspose2d(
            in_channels, hidden_channels, kernel_size=4, stride=2, padding=1, bias=False
        )
        self.bn = nn.BatchNorm2d(hidden_channels)
        self.relu = nn.ReLU(inplace=True)
        self.head = nn.Conv2d(hidden_channels, n_keypoints, kernel_size=1)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.head(self.relu(self.bn(self.upsample(x))))


def heatmap_probs(heatmaps: torch.Tensor) -> torch.Tensor:
    """Turns each keypoint's raw heatmap into a spatial probability
    distribution: a softmax over the flattened heatmap, reshaped back to
    a 2D map. Also useful on its own (not just as `soft_argmax`/
    `heatmap_variance`'s shared first step) for visualizing what the
    model is actually attending to -- see `visualize_predictions.py`.

    Args:
        heatmaps: `(batch, n_keypoints, height, width)` raw (pre-softmax)
            per-keypoint heatmaps.

    Returns:
        `(batch, n_keypoints, height, width)`, each keypoint's own map
        summing to 1 over its `height * width` pixels.
    """
    batch, n_keypoints, height, width = heatmaps.shape
    probs = torch.softmax(heatmaps.view(batch, n_keypoints, -1), dim=-1)
    return probs.view(batch, n_keypoints, height, width)


def _heatmap_marginals(
    heatmaps: torch.Tensor,
) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor, torch.Tensor]:
    """`heatmap_probs`'s `x`/`y` marginals and the `[0, 1]`-normalized
    coordinate grids they're defined over -- the shared next step of both
    `soft_argmax` and `heatmap_variance`.

    Returns:
        `marginal_x`: `(batch, n_keypoints, width)`.
        `marginal_y`: `(batch, n_keypoints, height)`.
        `xs`: `(width,)`, `[0, 1]`-normalized.
        `ys`: `(height,)`, `[0, 1]`-normalized.
    """
    probs = heatmap_probs(heatmaps)
    _, _, height, width = heatmaps.shape
    xs = torch.linspace(0, 1, width, device=heatmaps.device)
    ys = torch.linspace(0, 1, height, device=heatmaps.device)
    return probs.sum(dim=2), probs.sum(dim=3), xs, ys


def soft_argmax(heatmaps: torch.Tensor) -> torch.Tensor:
    """Differentiable spatial argmax: a softmax over each keypoint's
    flattened heatmap turns it into a probability distribution, then the
    expected `(x, y)` under that distribution is the predicted point.

    Unlike a hard argmax (e.g. `spotlight_pose2d.predict_unlabeled_frames.
    heatmaps_to_points`'s `unravel_index`), this is differentiable, so the
    model can be trained end to end from a coordinate loss directly,
    without needing to rasterize Gaussian heatmap targets.

    Args:
        heatmaps: `(batch, n_keypoints, height, width)` raw (pre-softmax)
            per-keypoint heatmaps.

    Returns:
        `(batch, n_keypoints, 2)`, `(x, y)` normalized to `[0, 1]` by
        `(width, height)`.
    """
    marginal_x, marginal_y, xs, ys = _heatmap_marginals(heatmaps)
    expected_x = (marginal_x * xs).sum(dim=2)
    expected_y = (marginal_y * ys).sum(dim=2)
    return torch.stack([expected_x, expected_y], dim=-1)


def heatmap_variance(heatmaps: torch.Tensor) -> torch.Tensor:
    """Per-keypoint spatial variance of the heatmap's own softmax
    distribution around its own expected coordinate (`soft_argmax`'s
    output) -- a training-time regularizer, not something `soft_argmax`
    itself needs.

    `soft_argmax`'s coordinate loss alone doesn't penalize a diffuse
    heatmap that happens to average out to the right place; it only stops
    being "right on average" once the true point sits off-center within
    the spread, which biases predictions toward the heatmap's own
    centroid -- worse the farther a keypoint sits from that centroid
    (confirmed empirically: thorax, near the head/thorax/abdomen
    centroid, was accurate; head/abdomen, the outer two points, were
    pulled inward by tens of pixels). Penalizing this variance during
    training encourages peaked heatmaps instead, the standard DSNT
    (Nibali et al. 2018) fix for the same failure mode.

    Args:
        heatmaps: `(batch, n_keypoints, height, width)` raw (pre-softmax)
            per-keypoint heatmaps.

    Returns:
        `(batch, n_keypoints)`, `x` and `y` variance summed, in squared
        `[0, 1]`-normalized-coordinate units (same units as
        `soft_argmax`'s own output).
    """
    marginal_x, marginal_y, xs, ys = _heatmap_marginals(heatmaps)
    expected_x = (marginal_x * xs).sum(dim=2, keepdim=True)
    expected_y = (marginal_y * ys).sum(dim=2, keepdim=True)
    variance_x = (marginal_x * (xs - expected_x) ** 2).sum(dim=2)
    variance_y = (marginal_y * (ys - expected_y) ** 2).sum(dim=2)
    return variance_x + variance_y


class TinyOrientModel(nn.Module):
    """3 conv blocks, a global-context block (see module docstring), then
    two independent heads: `keypoints` (a small heatmap per head/thorax/
    abdomen, reduced to a `(x, y)` coordinate via `soft_argmax`) and
    `flipped` (global average pool -> FC -> a single logit for "upside
    down or sideways", off the conv trunk directly -- it doesn't need the
    context block's own broadcast copy of the same information).

    ~230k parameters at the default channel widths.
    """

    def __init__(
        self,
        n_keypoints: int = N_KEYPOINTS,
        channels: tuple[int, int, int] = (32, 64, 112),
        context_channels: int = 64,
        heatmap_hidden_channels: int = 64,
        flip_trunk_dim: int = 112,
        use_global_context: bool = True,
    ) -> None:
        super().__init__()
        c1, c2, c3 = channels
        self.conv1 = ConvBlock(3, c1)
        self.conv2 = ConvBlock(c1, c2)
        self.conv3 = ConvBlock(c2, c3)
        self.use_global_context = use_global_context
        # use_global_context=False reconstructs v1-v5's own architecture
        # exactly (same shapes, same state_dict keys), so their checkpoints
        # still load correctly under the current code.
        heatmap_in_channels = c3
        if use_global_context:
            self.global_context = GlobalContextBlock(c3, context_channels)
            heatmap_in_channels = c3 + context_channels
        self.heatmap_head = HeatmapHead(
            heatmap_in_channels, n_keypoints, heatmap_hidden_channels
        )
        self.pool = nn.AdaptiveAvgPool2d(1)
        self.flip_trunk = nn.Sequential(
            nn.Linear(c3, flip_trunk_dim), nn.ReLU(inplace=True)
        )
        self.flipped_head = nn.Linear(flip_trunk_dim, 1)
        self.n_keypoints = n_keypoints

    def forward(
        self, x: torch.Tensor, return_heatmaps: bool = False
    ) -> (
        tuple[torch.Tensor, torch.Tensor]
        | tuple[torch.Tensor, torch.Tensor, torch.Tensor]
    ):
        """Returns `(keypoints, flipped_logit)`, or with `return_heatmaps`,
        `(keypoints, flipped_logit, heatmaps)`.

        Args:
            x: `(batch, 3, height, width)` image batch.
            return_heatmaps: If True, also return the keypoint head's raw
                heatmaps -- only `train.py` needs these (for
                `model.heatmap_variance`'s regularizer); `infer.py`/
                `export_model.py` use the default 2-tuple.

        Returns:
            keypoints: `(batch, n_keypoints, 2)`, `[0, 1]`-normalized
                `(x, y)` image-fraction coordinates (see `soft_argmax`).
            flipped_logit: `(batch, 1)`, pass through `sigmoid`/
                `binary_cross_entropy_with_logits` for the "upside down or
                sideways" probability/loss.
            heatmaps: `(batch, n_keypoints, height, width)` raw
                (pre-softmax) per-keypoint heatmaps, only if
                `return_heatmaps`.
        """
        features = self.conv3(self.conv2(self.conv1(x)))
        heatmap_input = (
            self.global_context(features) if self.use_global_context else features
        )
        heatmaps = self.heatmap_head(heatmap_input)
        keypoints = soft_argmax(heatmaps)
        flip_trunk = self.flip_trunk(self.pool(features).flatten(1))
        flipped_logit = self.flipped_head(flip_trunk)
        if return_heatmaps:
            return keypoints, flipped_logit, heatmaps
        return keypoints, flipped_logit
