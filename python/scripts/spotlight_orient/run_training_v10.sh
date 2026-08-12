#!/bin/bash -l
# Tenth training round for the TinyOrientModel: fixes v9's real
# regression (val_pixel_error 19.4px -> 40.6px after adding flip
# detection) with two changes, both directly evidenced by v9's own
# TensorBoard logs -- this isn't a class-imbalance problem (pos_weight
# already handles that), it's a loss-scale one.
#
# 1. FLIP_LOSS_WEIGHT=0.0002 (down from v9's 0.1, ~500x smaller): at
# v9's last epoch, kp_loss was 0.000734 but the WEIGHTED flip
# contribution (0.1 * ~1.2 raw BCE nats) was ~0.12 -- about 160x LARGER
# than the keypoint loss it was supposedly not supposed to dominate.
# v9's 0.1 was v2's own old value, calibrated back when keypoint_loss was
# ~10x larger (pre-global-context, pre-sigma-tuning); nobody re-derived
# it as keypoint accuracy improved through v6-v8, so it became wildly
# oversized. New value derived the same way heatmap_loss_weight was:
# v8's own converged kp_loss (~0.00032) / flip's raw BCE magnitude
# (~1.3) =~ 0.00024, rounded down slightly (consistent with this
# project's own bias toward "weaken it further" when correcting an
# overshoot).
#
# 2. INIT_CHECKPOINT=v8/best.pt (new): warm-starts from v8's already-
# converged keypoint-only weights instead of random initialization, so
# the new flip term doesn't have to re-learn the keypoint task from
# scratch alongside itself -- standard practice when adding an auxiliary
# head to an already-good model. v8's own flip head was never trained
# (FLIP_LOSS_WEIGHT=0.0 that round), so it's still at its own random
# init in that checkpoint; only the keypoint-relevant trunk is actually
# warm-started. Confirmed empirically: a 1-epoch smoke test's val_pixel_error
# was 178px from random init vs. 81px warm-started from v8, on the same
# tiny data subset.
#
# LEARNING_RATE=3e-4 (down from v9's 1e-3) and N_EPOCHS=50 (down from
# v9's 200): fine-tuning an already-converged model calls for a gentler
# rate and less time than learning everything from scratch did -- 200
# epochs of a fresh 1e-3-initial cosine schedule risks unnecessarily
# drifting away from v8's good initialization. LR_SCHEDULE stays cosine.
# Adjust back up if 50 epochs isn't enough for the flip head itself to
# converge (watch val/flip_precision and val/flip_recall).
#
# Everything else (USE_GLOBAL_CONTEXT, HEATMAP_LOSS_WEIGHT=0.0001,
# TARGET_SIGMA_NATIVE_PX=15.0, KEYPOINT_SET=neck, PATIENCE=15) unchanged
# from run_training_v9.sh/v8.sh.
set -euo pipefail

MAX_FRAMES_PER_TRIAL=500
BATCH_SIZE=256
N_EPOCHS=50
LEARNING_RATE=3e-4
LR_SCHEDULE=cosine
FLIP_LOSS_WEIGHT=0.0002
HEATMAP_LOSS_WEIGHT=0.0001
TARGET_SIGMA_NATIVE_PX=15.0
KEYPOINT_SET=neck
NUM_WORKERS=0
PATIENCE=15
INIT_CHECKPOINT_REL="bulk_data/motion_prior/orient_model/checkpoints/v8/best.pt"

repo_root="$(pwd)"
data_root="$repo_root/bulk_data/motion_prior/2dpose_model/labels/final_predictions"
split_path="$repo_root/bulk_data/motion_prior/2dpose_model/labels/train_val_split_29trials.json"
checkpoint_dir="$repo_root/bulk_data/motion_prior/orient_model/checkpoints/v10"
frame_cache_root="$repo_root/bulk_data/motion_prior/orient_model/frame_cache"
init_checkpoint="$repo_root/$INIT_CHECKPOINT_REL"

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
python "$repo_root/tools/spotlight_orient/train.py" \
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
    --keypoint-set "$KEYPOINT_SET" \
    --use-global-context \
    --init-checkpoint "$init_checkpoint" \
    --num-workers "$NUM_WORKERS" \
    --patience "$PATIENCE"
