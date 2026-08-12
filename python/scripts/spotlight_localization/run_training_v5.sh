#!/bin/bash -l
# Fifth training round for the TinyLocalizationModel, keypoint-only again: same
# architecture/data/split as run_training_v4.sh, with three changes.
#
# 1. LR_SCHEDULE=cosine: decays LEARNING_RATE to ~0 over N_EPOCHS via
# CosineAnnealingLR, stepped once per epoch. Pairs naturally with v4's
# already-higher LEARNING_RATE (3e-3, vs. v1-v3's 1e-3): fast early
# progress from the higher rate, without it overshooting once the loss is
# already close to converged late in training. v1-v4 all used a constant
# LR; this is the first round to decay it.
#
# 2. TARGET_SIGMA_NATIVE_PX=30.0: double v4's 15.0, i.e. back to
# spotlight_pose2d's own effective Gaussian heatmap sigma exactly
# (HEATMAP_SIGMA=2.0 at its 60x60 output, 30px once converted through to
# raw pixels -- see box.py's verified RAW_TO_ALIGNED_SCALE=1.0).
# HEATMAP_LOSS_WEIGHT is left at v4's own 0.0001 -- watch train/heatmap_loss
# (its own scale shifts with a different target: a wider target Gaussian
# has higher entropy, so both the uniform-heatmap and well-matched-heatmap
# loss values will sit higher than v4's own ~9.3/~2.7-3 floor+ceiling) and
# val/mean_keypoint_error_px on TensorBoard.
#
# 3. KEYPOINT_SET=neck: predicts neck/thorax/abdomen instead of v1-v4's
# head (antenna midpoint)/thorax/abdomen -- swaps the derived
# antenna-midpoint "head" point for the skeleton's own directly-labeled
# "N" (neck) node (see dataset.KEYPOINT_SPECS). Not comparable to v1-v4's
# own val_pixel_error numbers point-for-point (different target
# distribution and difficulty), so use val_pixel_error mainly to compare
# against future neck-keypoint-set rounds, not head-keypoint-set ones.
#
# FLIP_LOSS_WEIGHT=0.0: staged rollout again, same as v1/v3/v4 -- confirm
# these changes still land accuracy before re-enabling flip detection in
# a follow-up round.
set -euo pipefail

MAX_FRAMES_PER_TRIAL=500
BATCH_SIZE=256
N_EPOCHS=50
LEARNING_RATE=3e-3
LR_SCHEDULE=cosine
FLIP_LOSS_WEIGHT=0.0
HEATMAP_LOSS_WEIGHT=0.0001
TARGET_SIGMA_NATIVE_PX=30.0
KEYPOINT_SET=neck
NUM_WORKERS=0
PATIENCE=10

repo_root="$(pwd)"
data_root="$repo_root/bulk_data/motion_prior/2dpose_model/labels/final_predictions"
split_path="$repo_root/bulk_data/motion_prior/2dpose_model/labels/train_val_split_29trials.json"
checkpoint_dir="$repo_root/bulk_data/motion_prior/localization_model/checkpoints/v5"
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
python "$repo_root/scripts/spotlight_localization/train.py" \
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
    --num-workers "$NUM_WORKERS" \
    --patience "$PATIENCE"
