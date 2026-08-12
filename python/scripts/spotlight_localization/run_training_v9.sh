#!/bin/bash -l
# Ninth training round for the TinyLocalizationModel: same hyperparameters/
# data/split as run_training_v8.sh (kept as-is -- v8's 19.4px was judged
# good enough to stop keypoint-only tuning), with flip detection turned
# back on.
#
# FLIP_LOSS_WEIGHT=0.1: same value and reasoning as v2's own
# (run_training_v2.sh) -- train.py applies pos_weight internally to
# correct for the flip label's own class imbalance (~4.5% flipped
# frames, pos_weight ~= 21), which inflates flip_loss's raw magnitude
# 10-20x; 0.1 brings its weighted contribution back down to something
# comparable to (not dominant over) the keypoint task's own loss. Watch
# val/mean_keypoint_error_px alongside val/flip_precision and
# val/flip_recall on TensorBoard: if keypoint error regresses back up
# from v8's ~19.4px, lower this further; if flip precision/recall look
# poor, that's more likely pos_weight itself needing a second look (see
# flip_label.py) than this weight.
#
# Everything else (USE_GLOBAL_CONTEXT, LEARNING_RATE=1e-3, LR_SCHEDULE=
# cosine, HEATMAP_LOSS_WEIGHT=0.0001, TARGET_SIGMA_NATIVE_PX=15.0,
# KEYPOINT_SET=neck, PATIENCE=15) unchanged from run_training_v8.sh.
set -euo pipefail

MAX_FRAMES_PER_TRIAL=500
BATCH_SIZE=256
N_EPOCHS=200
LEARNING_RATE=1e-3
LR_SCHEDULE=cosine
FLIP_LOSS_WEIGHT=0.1
HEATMAP_LOSS_WEIGHT=0.0001
TARGET_SIGMA_NATIVE_PX=15.0
KEYPOINT_SET=neck
NUM_WORKERS=0
PATIENCE=15

repo_root="$(pwd)"
data_root="$repo_root/bulk_data/motion_prior/2dpose_model/labels/final_predictions"
split_path="$repo_root/bulk_data/motion_prior/2dpose_model/labels/train_val_split_29trials.json"
checkpoint_dir="$repo_root/bulk_data/motion_prior/localization_model/checkpoints/v9"
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
    --use-global-context \
    --num-workers "$NUM_WORKERS" \
    --patience "$PATIENCE"
