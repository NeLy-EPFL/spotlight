#!/bin/bash -l
# Fourth training round for the TinyLocalizationModel, keypoint-only again: same
# architecture/data/split as run_training_v1.sh/run_training_v3.sh, but
# replaces v3's heatmap regularizer with a more principled one.
#
# v3's model.heatmap_variance regularizer just minimized the heatmap's own
# spatial variance toward zero, unbounded -- a simplification of the DSNT
# paper's actual regularization, which targets a specific spread (sigma),
# not zero. Unbounded minimization risks over-sharpening (an extremely
# peaked heatmap can lose useful gradient signal) and has no natural
# floor. train.gaussian_heatmap_loss fixes this properly: cross-entropy
# between the predicted heatmap distribution and a target isotropic
# Gaussian centered at the true label location with a fixed real-world
# sigma (TARGET_SIGMA_NATIVE_PX), computed analytically on the fly (no
# rasterized/cached target heatmap needed, unlike
# pose2d.dataset.points_to_heatmaps). This directly supervises
# the heatmap's whole shape (position AND spread), and is well-behaved at
# both extremes: empirically, ~9.3 nats for a uniform (untrained) heatmap,
# ~24-27 for an overly sharp one that doesn't match the target's own
# spread, and ~2.7-3 (the target Gaussian's own entropy) at best.
#
# TARGET_SIGMA_NATIVE_PX=15.0: half of spotlight_pose2d's own effective
# Gaussian heatmap sigma (HEATMAP_SIGMA=2.0 at its 60x60 output, 30px once
# converted through its 450px input -> 900px aligned domain -> raw pixels,
# using box.py's verified RAW_TO_ALIGNED_SCALE=1.0).
#
# HEATMAP_LOSS_WEIGHT=0.0001: gaussian_heatmap_loss sits in a very
# differently-scaled range (~3-27 nats, see above) than the keypoint MSE
# loss it's added to (converges to ~0.0003-0.0006) -- this weight keeps
# its contribution comparable to (not dominant over) keypoint_loss at
# convergence, the same reasoning FLIP_LOSS_WEIGHT needed in v2 after its
# own raw BCE magnitude turned out to dominate. Watch train/heatmap_loss
# and val/mean_keypoint_error_px together on TensorBoard, and
# val/heatmap_variance for a more intuitive peakiness read (0.17 =
# uniform, v3's own well-converged range was ~0.001-0.003); raise this
# weight if heatmaps stay diffuse, lower it if keypoint error regresses.
#
# LEARNING_RATE=3e-3: 3x v1/v2/v3's own 1e-3, since BATCH_SIZE=256 is
# fairly large for this tiny (~210k param) model -- larger batches give
# lower-variance gradient estimates, which generally tolerate (and
# benefit from) a higher LR. AdamW itself is fairly forgiving of this
# range; watch train/loss for the first few hundred steps for spikiness
# or divergence, and drop back toward 1e-3 if it shows up.
#
# FLIP_LOSS_WEIGHT=0.0: staged rollout again, same as v1/v3 -- confirm
# this regularizer improves keypoint accuracy before re-enabling flip
# detection in a follow-up round.
set -euo pipefail

MAX_FRAMES_PER_TRIAL=500
BATCH_SIZE=256
N_EPOCHS=50
LEARNING_RATE=3e-3
FLIP_LOSS_WEIGHT=0.0
HEATMAP_LOSS_WEIGHT=0.0001
TARGET_SIGMA_NATIVE_PX=15.0
NUM_WORKERS=0
PATIENCE=10

repo_root="$(pwd)"
data_root="$repo_root/bulk_data/motion_prior/2dpose_model/labels/final_predictions"
split_path="$repo_root/bulk_data/motion_prior/2dpose_model/labels/train_val_split_29trials.json"
checkpoint_dir="$repo_root/bulk_data/motion_prior/localization_model/checkpoints/v4"
frame_cache_root="$repo_root/bulk_data/motion_prior/localization_model/frame_cache"

source "$repo_root/.venv/bin/activate"

train_paths=$(python -c "
import json
stems = json.load(open('$split_path'))['train']
print(' '.join(f'$data_root/{s}_pose2d_final_predictions.h5' for s in stems))
")
val_paths=$(python -c "
import json
stems = json.load(open('$split_path'))['validation']
print(' '.join(f'$data_root/{s}_pose2d_final_predictions.h5' for s in stems))
")

# shellcheck disable=SC2086 -- intentional word-splitting: tyro needs each
# path as its own --train-h5-paths/--val-h5-paths argument, not one string.
python "$repo_root/scripts/postprocessing/model_training/localization/train.py" \
    --train-h5-paths $train_paths \
    --val-h5-paths $val_paths \
    --checkpoint-dir "$checkpoint_dir" \
    --frame-cache-root "$frame_cache_root" \
    --max-frames-per-trial "$MAX_FRAMES_PER_TRIAL" \
    --batch-size "$BATCH_SIZE" \
    --n-epochs "$N_EPOCHS" \
    --learning-rate "$LEARNING_RATE" \
    --flip-loss-weight "$FLIP_LOSS_WEIGHT" \
    --heatmap-loss-weight "$HEATMAP_LOSS_WEIGHT" \
    --target-sigma-native-px "$TARGET_SIGMA_NATIVE_PX" \
    --num-workers "$NUM_WORKERS" \
    --patience "$PATIENCE"
