#!/bin/bash -l
# Thirteenth training round for the TinyLocalizationModel: a single from-scratch
# run (no warm-start, no frozen backbone) instead of v8->v11/v12's
# two-stage pipeline (keypoint-only checkpoint, then warm-start + freeze
# the trunk to add flip detection). For reproducibility: one script, one
# random init, one training run reconstructs the whole model, without
# depending on an earlier artifact whose own recipe (dataset.py's flip
# label, keypoint-set handling, etc.) has since changed underneath it.
#
# FLIP_LOSS_WEIGHT=0.0002: reused directly from run_training_v10.sh's own
# derivation (converged kp_loss ~0.0003 / flip's raw BCE ~1.3), NOT
# v9's original 0.1. v9 is the direct precedent for what happens without
# this: an earlier from-scratch run (run_training_v9.sh, FLIP_LOSS_WEIGHT
# =0.1, no init checkpoint) regressed keypoint error to 40.6px, versus
# v8's keypoint-only 19.4px, because the raw flip BCE loss (~1.2-1.7
# nats) so outweighs keypoint MSE (~0.0003-0.0006 at convergence) that an
# under-scaled flip weight dominates the shared trunk's gradient. That
# derivation doesn't depend on warm-starting -- it's about matching the
# WEIGHTED flip contribution to keypoint_loss's own scale at convergence
# -- so it should protect keypoint accuracy here too, without freezing
# anything.
#
# LEARNING_RATE=1e-3 (v9's own from-scratch rate, not v10's reduced
# 3e-4 -- that reduction was specifically for gently fine-tuning an
# already-converged checkpoint, which doesn't apply starting from random
# init).
#
# N_EPOCHS=150 (3x v11/v12's 50): a small FLIP_LOSS_WEIGHT means the
# flip head's own gradient is correspondingly small each step (this is
# exactly what made v10's flip head converge so slowly -- precision
# stuck ~0.15-0.18 after just 50 epochs at a reduced 3e-4 learning rate).
# Without freeze_backbone's structural shortcut, the only way to give the
# flip head enough total training signal to converge at this safe weight
# is more epochs, at the higher 1e-3 rate.
#
# PATIENCE=150 (== N_EPOCHS, effectively disables early stopping):
# train.py's early-stopping/best-checkpoint criterion tracks
# val_pixel_error whenever freeze_backbone is False (see train.py's own
# tracked_metric logic), which plateaus early (~19-20px within 20-30
# epochs, per v6-v8's own history) -- if patience were left at the usual
# 15, training would likely stop right around there, before the flip
# head (deliberately slow-learning here) has had anywhere near its full
# N_EPOCHS budget to converge. Disabling early stopping keeps this run's
# length fixed and deterministic regardless of metric noise, which is
# also simply more reproducible than a variable-length early-stopped run.
#
# Everything else (USE_GLOBAL_CONTEXT, HEATMAP_LOSS_WEIGHT=0.0001,
# TARGET_SIGMA_NATIVE_PX=15.0, LR_SCHEDULE=cosine, BATCH_SIZE=256,
# MAX_FRAMES_PER_TRIAL=500) unchanged from every round since v6/v8.
#
# No --init-checkpoint, no --freeze-backbone: this is the whole point of
# this round.
set -euo pipefail

MAX_FRAMES_PER_TRIAL=500
BATCH_SIZE=256
N_EPOCHS=150
LEARNING_RATE=1e-3
LR_SCHEDULE=cosine
FLIP_LOSS_WEIGHT=0.0002
HEATMAP_LOSS_WEIGHT=0.0001
TARGET_SIGMA_NATIVE_PX=15.0
NUM_WORKERS=0
PATIENCE=150

repo_root="$(pwd)"
data_root="$repo_root/bulk_data/motion_prior/2dpose_model/labels/final_predictions"
split_path="$repo_root/bulk_data/motion_prior/2dpose_model/labels/train_val_split_29trials.json"
checkpoint_dir="$repo_root/bulk_data/motion_prior/localization_model/checkpoints/v13"
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
    --lr-schedule "$LR_SCHEDULE" \
    --flip-loss-weight "$FLIP_LOSS_WEIGHT" \
    --heatmap-loss-weight "$HEATMAP_LOSS_WEIGHT" \
    --target-sigma-native-px "$TARGET_SIGMA_NATIVE_PX" \
    --use-global-context \
    --num-workers "$NUM_WORKERS" \
    --patience "$PATIENCE"
