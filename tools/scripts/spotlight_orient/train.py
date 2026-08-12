#!/usr/bin/env python
"""Trains a `TinyOrientModel` on one or more trials' `final_predictions.h5`
files (RepVGG-A0's own exhaustive per-frame output, used as pseudo-labels --
see `dataset.py`). Plain PyTorch training loop, same rationale as
`tools/spotlight_pose2d/train.py`: a single-model, single-GPU setup doesn't
need more than that.

Trains under autocast (bf16 if the GPU supports it, else fp16, with a
`GradScaler` in the fp16 case) for speed, same as `spotlight_pose2d`.
`evaluate` always runs in fp16 regardless of the training dtype, since fp16
is this tiny model's target inference precision. Logs to TensorBoard under
`checkpoint_dir/tensorboard`, and exports `last.pt`/`best.pt` to float16
ONNX and TorchScript (see `export_model.py`) whenever training stops,
however it stops.

Usage:
    python tools/spotlight_orient/train.py \\
        --train-h5-paths bulk_data/.../final_predictions/trial_a..._pose2d_final_predictions.h5 \\
        --val-h5-paths bulk_data/.../final_predictions/trial_b..._pose2d_final_predictions.h5 \\
        --checkpoint-dir bulk_data/motion_prior/orient_model/checkpoints/v1 \\
        --frame-cache-root bulk_data/motion_prior/orient_model/frame_cache
"""

import sys
import time
from pathlib import Path
from typing import Literal

import numpy as np
import torch
import torch.nn.functional as F
import tyro
from export_model import main as export_checkpoint
from loguru import logger
from torch.utils.data import DataLoader, get_worker_info
from torch.utils.tensorboard import SummaryWriter
from tqdm import tqdm

from spotlight_tools.spotlight_orient.dataset import (
    COARSE_KEYPOINTS,
    NATIVE_FRAME_SIZE,
    OUTPUT_SIZE,
    SCALE_FACTOR,
    TinyOrientDataset,
)
from spotlight_tools.spotlight_orient.model import (
    TinyOrientModel,
    heatmap_probs,
    heatmap_variance,
)


def gaussian_heatmap_loss(
    heatmaps: torch.Tensor,
    keypoints: torch.Tensor,
    target_sigma_native_px: float,
    native_frame_size: tuple[int, int] = NATIVE_FRAME_SIZE,
) -> torch.Tensor:
    """Cross-entropy between the predicted heatmap distribution
    (`model.heatmap_probs`) and a target isotropic Gaussian centered at
    each keypoint's own true (label) location, with a fixed real-world
    sigma -- the analytical alternative to rasterizing/caching a target
    heatmap image the way `spotlight_pose2d.dataset.points_to_heatmaps`
    does: the target's log-density is evaluated directly at the (small,
    e.g. 124x92) heatmap's own grid coordinates from the closed-form
    Gaussian formula, on the fly from each sample's own label, then
    turned into a proper discrete distribution the same way the
    prediction is (a softmax) -- so no separate normalization/epsilon
    handling is needed.

    Directly supervises the heatmap's whole shape (position and spread)
    against `target_sigma_native_px`, rather than the coordinate MSE loss
    (which only supervises the distribution's mean) or a plain variance
    regularizer (which only supervises its second moment).

    Args:
        heatmaps: `(batch, n_keypoints, height, width)` raw (pre-softmax)
            per-keypoint heatmaps.
        keypoints: `(batch, n_keypoints, 2)`, `[0, 1]`-normalized `(x, y)`
            true label coordinates (`soft_argmax`'s own convention).
        target_sigma_native_px: Desired heatmap spread, in raw camera
            pixels.
        native_frame_size: `(width, height)`, see `dataset.NATIVE_FRAME_SIZE`.
            The target sigma is converted to normalized coordinates
            per-axis (not by a single shared factor), since the native
            frame isn't square -- an isotropic real-world sigma is NOT
            isotropic in normalized-coordinate units.

    Returns:
        Scalar mean cross-entropy, in nats.
    """
    _, _, height, width = heatmaps.shape
    native_width, native_height = native_frame_size
    sigma_x = target_sigma_native_px / native_width
    sigma_y = target_sigma_native_px / native_height
    xs = torch.linspace(0, 1, width, device=heatmaps.device).view(1, 1, 1, width)
    ys = torch.linspace(0, 1, height, device=heatmaps.device).view(1, 1, height, 1)
    mu_x = keypoints[..., 0].unsqueeze(-1).unsqueeze(-1)
    mu_y = keypoints[..., 1].unsqueeze(-1).unsqueeze(-1)
    target_log_density = -((xs - mu_x) ** 2) / (2 * sigma_x**2) - ((ys - mu_y) ** 2) / (
        2 * sigma_y**2
    )
    batch, n_keypoints = heatmaps.shape[:2]
    target = torch.softmax(target_log_density.reshape(batch, n_keypoints, -1), dim=-1)
    # softmax underflows to exactly 0 far from a sharp peak often enough
    # in practice (autocast runs softmax in fp32, but fp32 still has a
    # floor) that log() alone risks -inf * 0 = nan; the epsilon costs
    # nothing where it doesn't matter and avoids that failure mode.
    log_pred = torch.log(
        heatmap_probs(heatmaps).reshape(batch, n_keypoints, -1) + 1e-12
    )
    return -(target * log_pred).sum(dim=-1).mean()


def seed_worker(_worker_id: int) -> None:
    """Reseeds a `DataLoader` worker's `TinyOrientDataset.rng`.

    Forking a worker process copies its parent's `rng` object (and internal
    state) verbatim, so without this every worker would independently
    replay the exact same augmentation draws instead of each drawing its
    own deterministic-but-distinct stream. `torch.initial_seed()` is
    already set per-worker by `DataLoader` (derived from its own
    `generator`), so reusing it here keeps the whole run reproducible from
    one `seed`.
    """
    worker_info = get_worker_info()
    worker_info.dataset.rng = np.random.default_rng(torch.initial_seed() % 2**32)


def make_loader(
    h5_paths: list[Path],
    frame_cache_root: Path,
    output_size: tuple[int, int],
    max_frames_per_trial: int | None,
    train: bool,
    batch_size: int,
    num_workers: int,
    seed: int,
) -> DataLoader:
    missing = [p for p in h5_paths if not p.is_file()]
    if missing:
        raise SystemExit(f"Missing .h5 file(s): {missing}")
    dataset = TinyOrientDataset(
        h5_paths,
        frame_cache_root,
        train,
        output_size=output_size,
        max_frames_per_trial=max_frames_per_trial,
        seed=seed,
    )
    return DataLoader(
        dataset,
        batch_size=batch_size,
        shuffle=train,
        num_workers=num_workers,
        drop_last=train,
        generator=torch.Generator().manual_seed(seed) if train else None,
        worker_init_fn=seed_worker if num_workers > 0 else None,
    )


def mean_keypoint_pixel_error(
    predicted: torch.Tensor, target: torch.Tensor, output_size: tuple[int, int]
) -> float:
    """Mean Euclidean distance between predicted/target head/thorax/abdomen,
    in `dataset.NATIVE_FRAME_SIZE`'s own raw fullsize pixel domain
    (denormalizing by `output_size` then `dataset.SCALE_FACTOR` -- the
    inverse of `dataset.points_to_model_space`).

    Unlike the raw MSE loss (in squared-normalized-coordinate units, not
    human-interpretable, and not comparable in scale to the flip BCE loss),
    this is directly interpretable and is what actually answers "is
    keypoint regression converging."
    """
    pred = predicted.detach().float().cpu().numpy()
    targ = target.detach().float().cpu().numpy()
    scale = np.array(output_size, dtype=np.float32) * SCALE_FACTOR
    return float(np.linalg.norm((pred - targ) * scale, axis=-1).mean())


# The model's own sigmoid decision boundary -- unrelated to
# flip_label.FLIPPED_THRESHOLD, which derives *training* labels from a
# different model's (RepVGG-A0's) confidence, not this model's own output.
FLIP_DECISION_THRESHOLD = 0.5


def flip_confusion_counts(
    pred_flip_logit: torch.Tensor, flipped: torch.Tensor
) -> tuple[int, int, int]:
    """Returns `(true_positives, false_positives, false_negatives)` for the
    "flipped" class, at `FLIP_DECISION_THRESHOLD`, summed over the batch.
    """
    predicted_positive = torch.sigmoid(pred_flip_logit) >= FLIP_DECISION_THRESHOLD
    actual_positive = flipped.bool()
    true_positives = (predicted_positive & actual_positive).sum().item()
    false_positives = (predicted_positive & ~actual_positive).sum().item()
    false_negatives = (~predicted_positive & actual_positive).sum().item()
    return true_positives, false_positives, false_negatives


def compute_loss(
    model: TinyOrientModel,
    images: torch.Tensor,
    keypoints: torch.Tensor,
    flipped: torch.Tensor,
    flip_loss_weight: float,
    pos_weight: torch.Tensor | None,
    heatmap_loss_weight: float,
    target_sigma_native_px: float,
) -> tuple[
    torch.Tensor,
    torch.Tensor,
    torch.Tensor,
    torch.Tensor,
    torch.Tensor,
    torch.Tensor,
    torch.Tensor,
]:
    """Returns `(total_loss, keypoint_loss, flip_loss, heatmap_loss,
    mean_heatmap_variance, pred_keypoints, pred_flip_logit)`.

    `heatmap_loss` (see `gaussian_heatmap_loss`) is a regularizer against
    soft-argmax's centroid-shrinkage bias: a diffuse heatmap can still
    average out to the right coordinate on the training set, but its
    predictions get pulled toward the heatmap's own centroid whenever the
    true point sits off-center, worse for points far from that centroid.
    Supervising the heatmap's whole shape against a fixed-sigma Gaussian
    label keeps it peaked instead. `mean_heatmap_variance` (see
    `model.heatmap_variance`) isn't part of the loss -- it's a cheap,
    interpretable "how peaked is the heatmap actually" diagnostic to
    watch alongside `heatmap_loss` (e.g. 0.17 for a uniform heatmap,
    ~0.001-0.003 for a well-converged peaked one -- see v1-v3's own
    TensorBoard logs).
    """
    pred_keypoints, pred_flip_logit, heatmaps = model(images, return_heatmaps=True)
    keypoint_loss = F.mse_loss(pred_keypoints, keypoints)
    flip_loss = F.binary_cross_entropy_with_logits(
        pred_flip_logit, flipped, pos_weight=pos_weight
    )
    heatmap_loss = gaussian_heatmap_loss(heatmaps, keypoints, target_sigma_native_px)
    variance = heatmap_variance(heatmaps).mean()
    return (
        keypoint_loss
        + flip_loss_weight * flip_loss
        + heatmap_loss_weight * heatmap_loss,
        keypoint_loss,
        flip_loss,
        heatmap_loss,
        variance,
        pred_keypoints,
        pred_flip_logit,
    )


@torch.no_grad()
def evaluate(
    model: TinyOrientModel,
    loader: DataLoader,
    device: str,
    flip_loss_weight: float,
    pos_weight: torch.Tensor | None,
    heatmap_loss_weight: float,
    target_sigma_native_px: float,
    output_size: tuple[int, int],
) -> tuple[float, float, float, float, float, float, float, float]:
    """Returns `(mean_loss, mean_keypoint_loss, mean_flip_loss,
    mean_heatmap_loss, mean_heatmap_variance, mean_keypoint_pixel_error,
    flip_precision, flip_recall)` over `loader`.

    Precision/recall are computed once from confusion counts summed over
    every batch (not averaged per-batch), which matters here: with the
    "flipped" class this rare, most batches have 0-1 positive examples, so
    a per-batch precision/recall would be dominated by noise.

    Runs in fp16 (regardless of the training run's own autocast dtype --
    see `main`'s `amp_dtype`), since fp16 is this tiny model's target
    inference precision.
    """
    model.eval()
    total_loss = total_kp_loss = total_flip_loss = 0.0
    total_heatmap_loss = total_variance = total_pixel_error = 0.0
    total_tp = total_fp = total_fn = 0
    n_batches = 0
    for images, keypoints, flipped in loader:
        images, keypoints, flipped = (
            images.to(device),
            keypoints.to(device),
            flipped.to(device),
        )
        with torch.autocast(
            device_type=device, dtype=torch.float16, enabled=device == "cuda"
        ):
            (
                loss,
                kp_loss,
                flip_loss,
                heatmap_loss,
                variance,
                pred_keypoints,
                pred_flip_logit,
            ) = compute_loss(
                model,
                images,
                keypoints,
                flipped,
                flip_loss_weight,
                pos_weight,
                heatmap_loss_weight,
                target_sigma_native_px,
            )
        total_loss += loss.item()
        total_kp_loss += kp_loss.item()
        total_flip_loss += flip_loss.item()
        total_heatmap_loss += heatmap_loss.item()
        total_variance += variance.item()
        total_pixel_error += mean_keypoint_pixel_error(
            pred_keypoints, keypoints, output_size
        )
        tp, fp, fn = flip_confusion_counts(pred_flip_logit, flipped)
        total_tp += tp
        total_fp += fp
        total_fn += fn
        n_batches += 1
    n_batches = max(n_batches, 1)
    precision = total_tp / max(total_tp + total_fp, 1)
    recall = total_tp / max(total_tp + total_fn, 1)
    return (
        total_loss / n_batches,
        total_kp_loss / n_batches,
        total_flip_loss / n_batches,
        total_heatmap_loss / n_batches,
        total_variance / n_batches,
        total_pixel_error / n_batches,
        precision,
        recall,
    )


def export_checkpoints(checkpoint_dir: Path, use_global_context: bool) -> None:
    """Exports whichever of `last.pt`/`best.pt` exist under `checkpoint_dir`
    to ONNX and TorchScript (fp32 and fp16 each), via `export_model.py`'s
    own `main` (called directly, not as a subprocess).
    """
    for name in ("last", "best"):
        checkpoint_path = checkpoint_dir / f"{name}.pt"
        if not checkpoint_path.is_file():
            continue
        export_checkpoint(
            checkpoint_path=checkpoint_path,
            output_stem=checkpoint_dir / name,
            use_global_context=use_global_context,
            override=True,
        )


def backbone_modules(model: TinyOrientModel) -> list[torch.nn.Module]:
    """Every keypoint-relevant submodule -- i.e. all of `model` except
    `flip_trunk`/`flipped_head` -- for `freeze_backbone` to freeze (weights
    and BatchNorm running stats) and to re-pin to eval mode each epoch
    (`model.train()` would otherwise flip them back to train mode).
    """
    modules = [model.conv1, model.conv2, model.conv3, model.heatmap_head]
    if model.use_global_context:
        modules.append(model.global_context)
    return modules


def main(
    train_h5_paths: list[Path],
    val_h5_paths: list[Path],
    checkpoint_dir: Path,
    frame_cache_root: Path,
    output_size: tuple[int, int] = OUTPUT_SIZE,
    max_frames_per_trial: int | None = None,
    batch_size: int = 32,
    n_epochs: int = 20,
    learning_rate: float = 1e-3,
    flip_loss_weight: float = 1.0,
    heatmap_loss_weight: float = 1e-4,
    target_sigma_native_px: float = 15.0,
    lr_schedule: Literal["constant", "cosine"] = "constant",
    use_global_context: bool = True,
    init_checkpoint: Path | None = None,
    freeze_backbone: bool = False,
    num_workers: int = 4,
    patience: int = 5,
    max_steps: int | None = None,
    seed: int = 0,
) -> None:
    """Train a `TinyOrientModel`.

    Args:
        train_h5_paths: Training trials' `final_predictions.h5` files (see
            `scripts/spotlight_pose2d/run_inference_final.sh`).
        val_h5_paths: Validation trials' `final_predictions.h5` files.
        checkpoint_dir: Where to save checkpoints.
        frame_cache_root: Root directory
            `scripts/spotlight_orient/cache_fullsize_frames.py` wrote into;
            must already have every train/val trial cached at `output_size`.
        output_size: `(width, height)` this model's input frames are resized
            to; see `dataset.OUTPUT_SIZE`.
        max_frames_per_trial: Randomly sample at most this many frames per
            trial instead of every frame. Unset uses every frame.
        batch_size: Training batch size.
        n_epochs: Maximum number of passes over the training set.
        learning_rate: AdamW learning rate.
        flip_loss_weight: Weight of the binary cross-entropy flip loss
            relative to the keypoint MSE loss.
        heatmap_loss_weight: Weight of `gaussian_heatmap_loss` (a
            cross-entropy against a fixed-sigma Gaussian label centered at
            the true point), a regularizer against soft-argmax's
            centroid-shrinkage bias. `gaussian_heatmap_loss` itself sits in
            a much bigger, differently-scaled range than the keypoint MSE
            loss it's added to: empirically, ~9.3 nats for a uniform
            (untrained) heatmap, ~24-27 for an overly sharp one that
            doesn't match the target's own spread, and ~2.7-3 (its
            theoretical floor -- the target Gaussian's own entropy) for a
            well-matched one, versus keypoint_loss converging to
            ~0.0003-0.0006. The default keeps its weighted contribution
            comparable to (not dominant over) keypoint_loss at
            convergence, following the same reasoning `flip_loss_weight`
            needed after its own raw BCE magnitude turned out to dominate.
            Watch `train/heatmap_loss` and `val/mean_keypoint_error_px`
            together on TensorBoard, and `heatmap_variance` for a
            complementary, more intuitive read on peakiness (0.17 =
            uniform, ~0.001-0.003 = v3's own well-converged range); raise
            this weight if heatmaps stay diffuse, lower it if keypoint
            error regresses.
        target_sigma_native_px: `gaussian_heatmap_loss`'s target label
            sigma, in raw camera pixels -- half of `spotlight_pose2d`'s
            own effective Gaussian heatmap sigma (`HEATMAP_SIGMA=2.0` at
            its 60x60 output, 30px once converted through to raw pixels).
        lr_schedule: `"constant"` keeps `learning_rate` fixed throughout
            (every round before this one); `"cosine"` decays it to ~0 via
            `CosineAnnealingLR` over `n_epochs`, stepped once per epoch --
            worth it mainly when `learning_rate` itself is on the higher
            side, to keep late-training updates from overshooting once
            the loss is already close to converged.
        use_global_context: See `model.GlobalContextBlock` -- broadcasts a
            global-average-pooled context vector back to every heatmap
            location. False reconstructs v1-v5's own architecture (their
            receptive field, measured empirically, covers only ~20% of the
            fly's own body length -- likely why keypoint predictions never
            got much past ~28-32px and looked multi-modal rather than
            single-peaked). True from v6 on.
        init_checkpoint: Optional trained `TinyOrientModel` state dict to
            warm-start from instead of random initialization -- e.g. a
            keypoint-only checkpoint's already-converged trunk, when
            adding a new loss term (like flip detection) that would
            otherwise have to re-learn the keypoint task from scratch
            alongside the new one, at real risk of the new term's own
            gradients disturbing it (see v9's own regression: keypoint
            error went from v8's 19.4px to 40.6px after adding flip
            detection from a random init). Must match this run's own
            `n_keypoints`/`use_global_context` (any sub-module not
            actually trained by the checkpoint's own run, e.g. an unused
            flip head, is still present in its state dict at whatever
            its random initialization was, so this loads cleanly either
            way).
        freeze_backbone: Freeze `conv1`/`conv2`/`conv3`/`global_context`/
            `heatmap_head` (everything but `flip_trunk`/`flipped_head`) --
            weights AND BatchNorm running stats, so the keypoint-relevant
            trunk truly cannot move at all, no matter how `flip_loss_weight`
            is set. This is the actual fix for `init_checkpoint`'s own
            failure mode: shrinking `flip_loss_weight` to protect the
            trunk (see v10's own run) shrinks the flip head's gradient by
            the same factor, which starves a freshly-initialized flip
            head of the signal it needs (v10: `val_flip_loss` was still
            slowly falling, 1.76 -> 1.32, after 21 epochs -- learning,
            just far too slowly to be useful). Freezing decouples the two
            completely: the trunk is now protected structurally, not by a
            delicate loss-weight balance, so `flip_loss_weight` can go
            back to a normal (not artificially tiny) value and the flip
            head can train at full speed. Only meaningful together with
            `init_checkpoint` (freezing a randomly-initialized trunk
            would leave the keypoint task permanently broken).
        num_workers: `DataLoader` worker processes.
        patience: Stop early if val loss hasn't improved in this many epochs.
        max_steps: Stop after this many optimizer steps regardless of
            `n_epochs`/`patience`, for a quick smoke test; leave unset for a
            real training run.
        seed: Random seed for model init, data sampling, and shuffling.
    """
    torch.manual_seed(seed)
    device = "cuda" if torch.cuda.is_available() else "cpu"
    checkpoint_dir.mkdir(parents=True, exist_ok=True)
    tensorboard_dir = checkpoint_dir / "tensorboard"
    logger.info(f"Device: {device}")
    logger.info(f"Checkpoint dir: {checkpoint_dir}")
    logger.info(f"TensorBoard dir: {tensorboard_dir}")
    logger.info(f"Output size: {output_size}")
    logger.info(f"Keypoints: {COARSE_KEYPOINTS}")

    model = TinyOrientModel(
        n_keypoints=len(COARSE_KEYPOINTS), use_global_context=use_global_context
    ).to(device)
    if init_checkpoint is not None:
        model.load_state_dict(torch.load(init_checkpoint, map_location=device))
        logger.info(f"Warm-started from {init_checkpoint}")
    if freeze_backbone:
        for module in backbone_modules(model):
            module.eval()
            for p in module.parameters():
                p.requires_grad = False
        n_frozen = sum(
            p.numel() for m in backbone_modules(model) for p in m.parameters()
        )
        logger.info(
            f"Backbone frozen ({n_frozen:,} params) -- only flip_trunk/"
            "flipped_head will train"
        )
    n_params = sum(p.numel() for p in model.parameters())
    n_trainable = sum(p.numel() for p in model.parameters() if p.requires_grad)
    logger.info(f"Model parameters: {n_params:,} ({n_trainable:,} trainable)")

    train_loader = make_loader(
        train_h5_paths,
        frame_cache_root,
        output_size,
        max_frames_per_trial,
        True,
        batch_size,
        num_workers,
        seed,
    )
    val_loader = make_loader(
        val_h5_paths,
        frame_cache_root,
        output_size,
        max_frames_per_trial,
        False,
        batch_size,
        num_workers,
        seed,
    )
    n_iters_per_epoch = len(train_loader)
    log_every = max(1, int(0.1 * n_iters_per_epoch))
    is_tty = sys.stdout.isatty()
    logger.info(f"{n_iters_per_epoch} iterations/epoch (batch size {batch_size})")

    train_flipped = train_loader.dataset.flipped
    n_pos = int(train_flipped.sum())
    n_neg = len(train_flipped) - n_pos
    pos_weight = torch.tensor(n_neg / max(n_pos, 1), device=device)
    logger.info(
        f"Train flip class balance: {n_pos} flipped / {n_neg} not-flipped "
        f"(pos_weight={pos_weight.item():.2f})"
    )

    optimizer = torch.optim.AdamW(
        (p for p in model.parameters() if p.requires_grad), lr=learning_rate
    )
    scheduler = (
        torch.optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=n_epochs)
        if lr_schedule == "cosine"
        else None
    )
    logger.info(f"LR schedule: {lr_schedule} (initial lr={learning_rate})")
    amp_dtype = torch.bfloat16
    if device == "cuda" and not torch.cuda.is_bf16_supported():
        amp_dtype = torch.float16
    scaler = torch.amp.GradScaler(
        enabled=(device == "cuda" and amp_dtype == torch.float16)
    )
    logger.info(
        f"Training autocast dtype: {amp_dtype if device == 'cuda' else 'disabled (cpu)'}"
    )
    writer = SummaryWriter(str(tensorboard_dir))

    best_tracked_metric = float("inf")
    epochs_without_improvement = 0
    step = 0
    for epoch in range(n_epochs):
        model.train()
        if freeze_backbone:
            for module in backbone_modules(model):
                module.eval()
        epoch_start = time.time()
        running_loss = 0.0
        iterator = (
            tqdm(train_loader, desc=f"epoch {epoch}", leave=False, mininterval=1.0)
            if is_tty
            else train_loader
        )
        for i, (images, keypoints, flipped) in enumerate(iterator):
            images, keypoints, flipped = (
                images.to(device),
                keypoints.to(device),
                flipped.to(device),
            )
            with torch.autocast(
                device_type=device, dtype=amp_dtype, enabled=device == "cuda"
            ):
                (
                    loss,
                    kp_loss,
                    flip_loss,
                    heatmap_loss,
                    variance,
                    _pred_keypoints,
                    _pred_flip_logit,
                ) = compute_loss(
                    model,
                    images,
                    keypoints,
                    flipped,
                    flip_loss_weight,
                    pos_weight,
                    heatmap_loss_weight,
                    target_sigma_native_px,
                )
            optimizer.zero_grad()
            scaler.scale(loss).backward()
            scaler.step(optimizer)
            scaler.update()

            running_loss += loss.item()
            writer.add_scalar("train/loss", loss.item(), step)
            writer.add_scalar("train/keypoint_loss", kp_loss.item(), step)
            writer.add_scalar("train/flip_loss", flip_loss.item(), step)
            writer.add_scalar("train/heatmap_loss", heatmap_loss.item(), step)
            writer.add_scalar("train/heatmap_variance", variance.item(), step)
            step += 1

            if is_tty:
                iterator.set_postfix(loss=f"{loss.item():.5f}")
            elif i % log_every == 0:
                logger.info(
                    f"epoch {epoch}: {i}/{n_iters_per_epoch} iterations, "
                    f"loss={loss.item():.5f} (kp={kp_loss.item():.5f}, "
                    f"flip={flip_loss.item():.5f})"
                )

            if max_steps is not None and step >= max_steps:
                logger.info(f"Reached max_steps={max_steps}, stopping")
                torch.save(model.state_dict(), checkpoint_dir / "last.pt")
                export_checkpoints(checkpoint_dir, use_global_context)
                return

        epoch_time = time.time() - epoch_start
        (
            val_loss,
            val_kp_loss,
            val_flip_loss,
            val_heatmap_loss,
            val_variance,
            val_pixel_error,
            val_precision,
            val_recall,
        ) = evaluate(
            model,
            val_loader,
            device,
            flip_loss_weight,
            pos_weight,
            heatmap_loss_weight,
            target_sigma_native_px,
            output_size,
        )
        writer.add_scalar("train/epoch_loss", running_loss / n_iters_per_epoch, epoch)
        writer.add_scalar("train/learning_rate", optimizer.param_groups[0]["lr"], epoch)
        writer.add_scalar("val/loss", val_loss, epoch)
        writer.add_scalar("val/keypoint_loss", val_kp_loss, epoch)
        writer.add_scalar("val/flip_loss", val_flip_loss, epoch)
        writer.add_scalar("val/heatmap_loss", val_heatmap_loss, epoch)
        writer.add_scalar("val/heatmap_variance", val_variance, epoch)
        writer.add_scalar("val/mean_keypoint_error_px", val_pixel_error, epoch)
        writer.add_scalar("val/flip_precision", val_precision, epoch)
        writer.add_scalar("val/flip_recall", val_recall, epoch)
        logger.info(
            f"Epoch {epoch} ({epoch_time:.0f}s): "
            f"train_loss={running_loss / n_iters_per_epoch:.5f}, "
            f"val_loss={val_loss:.5f} (kp={val_kp_loss:.5f}, flip={val_flip_loss:.5f}, "
            f"heatmap_loss={val_heatmap_loss:.5f}, heatmap_var={val_variance:.5f}), "
            f"val_pixel_error={val_pixel_error:.1f}px, "
            f"val_flip_precision={val_precision:.3f}, val_flip_recall={val_recall:.3f}"
        )

        torch.save(model.state_dict(), checkpoint_dir / "last.pt")
        # With freeze_backbone, val_pixel_error is bit-for-bit constant
        # (the trunk that produces it literally cannot change), so it
        # would never register an "improvement" past epoch 0 and trigger
        # early stopping almost immediately -- track val_flip_loss
        # instead in that case, the only thing actually still training.
        tracked_metric = val_flip_loss if freeze_backbone else val_pixel_error
        if tracked_metric < best_tracked_metric:
            best_tracked_metric = tracked_metric
            epochs_without_improvement = 0
            torch.save(model.state_dict(), checkpoint_dir / "best.pt")
        else:
            epochs_without_improvement += 1
            if epochs_without_improvement >= patience:
                metric_name = "val_flip_loss" if freeze_backbone else "val_pixel_error"
                logger.info(
                    f"No {metric_name} improvement for {patience} epochs, "
                    "stopping early"
                )
                break

        if scheduler is not None:
            scheduler.step()

    export_checkpoints(checkpoint_dir, use_global_context)


if __name__ == "__main__":
    tyro.cli(main)
