#!/usr/bin/env python
"""Trains a `RepVGGPoseModel` on one or more trials' dense pose `.h5` files
(see `dataset.py`). Plain PyTorch training loop, deliberately not using
PyTorch Lightning: a single-model, single-GPU setup doesn't need Lightning's
multi-GPU/callback machinery, and staying plain keeps every step (the loop,
checkpointing, AMP) visible and easy to change, which matters more here
since the whole point of this model (vs. reusing SLEAP's own training) is
architecture/training control.

Usage:
    python tools/spotlight_pose2d/train.py \\
        --train-h5-paths bulk_data/.../trial_a_pose.h5 bulk_data/.../trial_b_pose.h5 \\
        --val-h5-paths bulk_data/.../trial_c_pose.h5 \\
        --checkpoint-dir bulk_data/motion_prior/2dpose_model/checkpoints/iter0c \\
        --frame-cache-root bulk_data/motion_prior/2dpose_model/frame_cache
"""

import sys
import time
from pathlib import Path

import numpy as np
import torch
import torch.nn.functional as F
import tyro
from loguru import logger
from torch.utils.data import DataLoader, get_worker_info
from torch.utils.tensorboard import SummaryWriter
from tqdm import tqdm

from spotlight.postprocessing.pose2d.constants import (
    HEATMAP_SIGMA,
    INPUT_SIZE,
    N_KEYPOINTS,
)
from spotlight.postprocessing.pose2d.dataset import PoseDataset
from spotlight.postprocessing.pose2d.export import export_checkpoint
from spotlight.postprocessing.pose2d.model import RepVGGPoseModel


def build_model(
    n_keypoints: int, pretrained_backbone: bool
) -> tuple[RepVGGPoseModel, int]:
    """Builds the model and probes its heatmap output size via a dummy pass."""
    model = RepVGGPoseModel(n_keypoints, pretrained_backbone)
    with torch.no_grad():
        dummy = torch.zeros(1, 3, INPUT_SIZE, INPUT_SIZE)
        heatmap_size = model(dummy).shape[-1]
    logger.info(f"Heatmap output size: {heatmap_size}x{heatmap_size}")
    return model, heatmap_size


def seed_worker(_worker_id: int) -> None:
    """Reseeds a `DataLoader` worker's `PoseDataset.rng`.

    Forking a worker process copies its parent's `rng` object (and internal
    state) verbatim, so without this every worker would independently replay
    the exact same augmentation draws instead of each drawing its own
    deterministic-but-distinct stream. `torch.initial_seed()` is already set
    per-worker by `DataLoader` (derived from its own `generator`), so reusing
    it here keeps the whole run reproducible from one `seed`.
    """
    worker_info = get_worker_info()
    worker_info.dataset.rng = np.random.default_rng(torch.initial_seed() % 2**32)


def make_loader(
    h5_paths: list[Path],
    frame_cache_root: Path,
    train: bool,
    heatmap_size: int,
    heatmap_sigma: float,
    batch_size: int,
    num_workers: int,
    seed: int,
) -> DataLoader:
    missing = [p for p in h5_paths if not p.is_file()]
    if missing:
        raise SystemExit(f"Missing .h5 file(s): {missing}")
    dataset = PoseDataset(
        h5_paths,
        frame_cache_root,
        train=train,
        heatmap_size=heatmap_size,
        heatmap_sigma=heatmap_sigma,
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
    predicted: torch.Tensor, target: torch.Tensor, heatmap_size: int, input_size: int
) -> float:
    """Mean Euclidean distance, in `input_size`-resolution pixels, between
    predicted and target heatmap peak locations.

    More interpretable than heatmap MSE (which conflates localization error
    with heatmap sharpness/magnitude), so this is what early stopping and
    `best.pt` selection are based on, not the loss. Keypoints missing from
    `target` (an all-zero heatmap, see `dataset.points_to_heatmaps`) are
    excluded, since their peak location is meaningless.
    """
    pred = predicted.detach().float().cpu().numpy().reshape(*predicted.shape[:2], -1)
    targ = target.detach().float().cpu().numpy().reshape(*target.shape[:2], -1)
    pred_y, pred_x = np.unravel_index(
        pred.argmax(axis=-1), (heatmap_size, heatmap_size)
    )
    targ_y, targ_x = np.unravel_index(
        targ.argmax(axis=-1), (heatmap_size, heatmap_size)
    )
    dist = np.sqrt((pred_x - targ_x) ** 2 + (pred_y - targ_y) ** 2)

    valid = targ.max(axis=-1) > 0
    if not valid.any():
        return float("nan")
    return float(dist[valid].mean()) * (input_size / heatmap_size)


def export_checkpoints(checkpoint_dir: Path, n_keypoints: int) -> None:
    """Exports whichever of `last.pt`/`best.pt` exist under `checkpoint_dir`
    to a plain fp16 state_dict, TorchScript, and ONNX, via
    `pose2d.export.export_checkpoint`.
    """
    for name in ("last", "best"):
        checkpoint_path = checkpoint_dir / f"{name}.pt"
        if not checkpoint_path.is_file():
            continue
        export_checkpoint(
            checkpoint_path=checkpoint_path,
            output_stem=checkpoint_dir / name,
            n_keypoints=n_keypoints,
            override=True,
        )


@torch.no_grad()
def evaluate(
    model: RepVGGPoseModel,
    loader: DataLoader,
    device: str,
    amp_dtype: torch.dtype,
    heatmap_size: int,
    input_size: int,
) -> tuple[float, float]:
    """Returns `(mean_loss, mean_keypoint_pixel_error)` over `loader`."""
    model.eval()
    total_loss, total_pixel_error, n_batches = 0.0, 0.0, 0
    for images, heatmaps in loader:
        images, heatmaps = images.to(device), heatmaps.to(device)
        with torch.autocast(
            device_type=device, dtype=amp_dtype, enabled=device == "cuda"
        ):
            predicted = model(images)
            loss = F.mse_loss(predicted, heatmaps)
        total_loss += loss.item()
        total_pixel_error += mean_keypoint_pixel_error(
            predicted, heatmaps, heatmap_size, input_size
        )
        n_batches += 1
    return total_loss / max(n_batches, 1), total_pixel_error / max(n_batches, 1)


def main(
    train_h5_paths: list[Path],
    val_h5_paths: list[Path],
    checkpoint_dir: Path,
    frame_cache_root: Path,
    n_keypoints: int = N_KEYPOINTS,
    batch_size: int = 16,
    n_epochs: int = 20,
    learning_rate: float = 1e-3,
    backbone_lr_scale: float = 0.1,
    warmup_steps: int = 1500,
    pretrained_backbone: bool = True,
    init_checkpoint_path: Path | None = None,
    num_workers: int = 4,
    patience: int = 5,
    max_steps: int | None = None,
    heatmap_sigma: float = HEATMAP_SIGMA,
    seed: int = 0,
) -> None:
    """Train a `RepVGGPoseModel`.

    Args:
        train_h5_paths: Training trials' dense pose `.h5` files.
        val_h5_paths: Validation trials' dense pose `.h5` files.
        checkpoint_dir: Where to save checkpoints and TensorBoard logs.
        frame_cache_root: Root directory `cache_video_frames.py` wrote into
            (see `cache_all_trial_frames.py`); must already have every
            train/val trial cached at `INPUT_SIZE`.
        n_keypoints: Number of keypoints to predict.
        batch_size: Training batch size.
        n_epochs: Maximum number of passes over the training set. With
            193k+ labeled frames in this project's actual training set and
            batch_size=16, one epoch is already ~12k iterations, so treat
            this as a safety ceiling; `patience` (early stopping) is what
            actually decides when to stop in practice.
        learning_rate: AdamW learning rate for the heatmap head, reached at
            the end of warmup.
        backbone_lr_scale: The pretrained RepVGG-A0 backbone trains at
            `learning_rate * backbone_lr_scale`, not `learning_rate` itself:
            a uniform LR sized for the randomly-initialized head is too
            aggressive for a pretrained backbone and destabilizes its
            BatchNorm running statistics. 1.0 disables the split (backbone
            == head LR).
        warmup_steps: Linearly ramp both LRs from 10% to 100% of their
            target over this many optimizer steps, then hold constant:
            matters most here since the head's early gradients (random
            init) are large and noisy. 0 disables warmup.
        pretrained_backbone: Start the RepVGG-A0 backbone from ImageNet weights.
            Pointless (and wastes a download) if `init_checkpoint_path` is
            also set, since that overwrites the whole model right after:
            pass `--no-pretrained-backbone` alongside it.
        init_checkpoint_path: If set, load this full model state dict (e.g.
            a previous round's `best.pt`) right after building the model,
            before training, for fine-tuning an already-trained checkpoint
            on a new, smaller dataset instead of starting from ImageNet
            weights and a randomly-initialized head. Unset trains from
            scratch (`pretrained_backbone` alone), as every round through
            iter0c did.
        num_workers: `DataLoader` worker processes.
        patience: Stop early if `val`'s mean keypoint pixel error (see
            `mean_keypoint_pixel_error`) hasn't improved in this many epochs.
        max_steps: Stop after this many optimizer steps regardless of
            `n_epochs`/`patience`, for a quick smoke test; leave unset for a
            real training run.
        heatmap_sigma: Standard deviation (in heatmap-output pixels, e.g. at
            a 60x60 output that's `INPUT_SIZE / 60` input pixels) of each
            keypoint's Gaussian training target: see `dataset.HEATMAP_SIGMA`.
        seed: Random seed for model init (when not using
            `init_checkpoint_path`), data shuffling, and augmentation, plus
            `cudnn.deterministic=True`/`cudnn.benchmark=False` below, so a
            run is exactly reproducible.
    """
    torch.manual_seed(seed)
    torch.backends.cudnn.deterministic = True
    torch.backends.cudnn.benchmark = False
    device = "cuda" if torch.cuda.is_available() else "cpu"
    checkpoint_dir.mkdir(parents=True, exist_ok=True)
    tensorboard_dir = checkpoint_dir / "tensorboard"
    logger.info(f"Device: {device}")
    logger.info(f"Checkpoint dir: {checkpoint_dir}")
    logger.info(f"TensorBoard dir: {tensorboard_dir}")
    logger.info(f"Frame cache root: {frame_cache_root}")
    logger.info(f"Heatmap sigma: {heatmap_sigma}")
    logger.info(f"Train .h5 files ({len(train_h5_paths)}):")
    for p in train_h5_paths:
        logger.info(f"  {p}")
    logger.info(f"Val .h5 files ({len(val_h5_paths)}):")
    for p in val_h5_paths:
        logger.info(f"  {p}")

    model, heatmap_size = build_model(n_keypoints, pretrained_backbone)
    model = model.to(device)
    if init_checkpoint_path is not None:
        if pretrained_backbone:
            logger.warning(
                "init_checkpoint_path is set; pretrained_backbone's ImageNet "
                "init will be immediately overwritten. Pass "
                "--no-pretrained-backbone to skip the wasted download."
            )
        logger.info(f"Initializing from checkpoint: {init_checkpoint_path}")
        model.load_state_dict(torch.load(init_checkpoint_path, map_location=device))
    train_loader = make_loader(
        train_h5_paths,
        frame_cache_root,
        True,
        heatmap_size,
        heatmap_sigma,
        batch_size,
        num_workers,
        seed,
    )
    val_loader = make_loader(
        val_h5_paths,
        frame_cache_root,
        False,
        heatmap_size,
        heatmap_sigma,
        batch_size,
        num_workers,
        seed,
    )
    n_iters_per_epoch = len(train_loader)
    log_every = max(1, int(0.01 * n_iters_per_epoch))
    is_tty = sys.stdout.isatty()
    logger.info(f"{n_iters_per_epoch} iterations/epoch (batch size {batch_size})")

    backbone_lr = learning_rate * backbone_lr_scale
    logger.info(f"Backbone LR: {backbone_lr}, head LR: {learning_rate}")
    optimizer = torch.optim.AdamW(
        [
            {"params": model.backbone.parameters(), "lr": backbone_lr},
            {"params": model.head.parameters(), "lr": learning_rate},
        ]
    )

    def warmup_lr_factor(step: int) -> float:
        if warmup_steps <= 0 or step >= warmup_steps:
            return 1.0
        return 0.1 + 0.9 * (step / warmup_steps)

    scheduler = torch.optim.lr_scheduler.LambdaLR(optimizer, lr_lambda=warmup_lr_factor)
    amp_dtype = torch.bfloat16
    if device == "cuda" and not torch.cuda.is_bf16_supported():
        amp_dtype = torch.float16
    scaler = torch.amp.GradScaler(
        enabled=(device == "cuda" and amp_dtype == torch.float16)
    )
    writer = SummaryWriter(str(tensorboard_dir))

    best_pixel_error = float("inf")
    epochs_without_improvement = 0
    step = 0
    for epoch in range(n_epochs):
        model.train()
        epoch_start = time.time()
        running_loss = 0.0
        iterator = (
            tqdm(train_loader, desc=f"epoch {epoch}", leave=False, mininterval=1.0)
            if is_tty
            else train_loader
        )
        for i, (images, heatmaps) in enumerate(iterator):
            images, heatmaps = images.to(device), heatmaps.to(device)
            with torch.autocast(
                device_type=device, dtype=amp_dtype, enabled=device == "cuda"
            ):
                loss = F.mse_loss(model(images), heatmaps)

            optimizer.zero_grad()
            scaler.scale(loss).backward()
            scaler.step(optimizer)
            scaler.update()
            scheduler.step()

            running_loss += loss.item()
            writer.add_scalar("train/loss", loss.item(), step)
            lr_backbone, lr_head = scheduler.get_last_lr()
            writer.add_scalar("train/lr_backbone", lr_backbone, step)
            writer.add_scalar("train/lr_head", lr_head, step)
            step += 1

            if is_tty:
                iterator.set_postfix(loss=f"{loss.item():.5f}")
            elif i % log_every == 0:
                logger.info(
                    f"epoch {epoch}: {i}/{n_iters_per_epoch} iterations, "
                    f"loss={loss.item():.5f}"
                )

            if max_steps is not None and step >= max_steps:
                logger.info(f"Reached max_steps={max_steps}, stopping")
                torch.save(model.state_dict(), checkpoint_dir / "last.pt")
                export_checkpoints(checkpoint_dir, n_keypoints)
                return

        epoch_time = time.time() - epoch_start
        val_loss, pixel_error = evaluate(
            model, val_loader, device, amp_dtype, heatmap_size, INPUT_SIZE
        )
        writer.add_scalar("train/epoch_loss", running_loss / n_iters_per_epoch, epoch)
        writer.add_scalar("val/loss", val_loss, epoch)
        writer.add_scalar("val/mean_keypoint_error_px", pixel_error, epoch)
        logger.info(
            f"Epoch {epoch} ({epoch_time:.0f}s): "
            f"train_loss={running_loss / n_iters_per_epoch:.5f}, "
            f"val_loss={val_loss:.5f}, val_pixel_error={pixel_error:.2f}px"
        )

        torch.save(model.state_dict(), checkpoint_dir / "last.pt")
        if pixel_error < best_pixel_error:
            best_pixel_error = pixel_error
            epochs_without_improvement = 0
            torch.save(model.state_dict(), checkpoint_dir / "best.pt")
        else:
            epochs_without_improvement += 1
            if epochs_without_improvement >= patience:
                logger.info(
                    f"No val_pixel_error improvement for {patience} epochs, "
                    "stopping early"
                )
                break

    export_checkpoints(checkpoint_dir, n_keypoints)


if __name__ == "__main__":
    tyro.cli(main)
