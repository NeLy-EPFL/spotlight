#!/bin/bash -l
# Second training round for the TinyLocalizationModel: same as
# run_training_v1.sh (fixed heatmap+soft-argmax architecture, same
# data/split), but with flip classification turned back on now that
# v1 has confirmed keypoint regression converges well on its own.
#
# FLIP_LOSS_WEIGHT=0.1: train.py applies pos_weight internally to correct
# for the flip label's own class imbalance (~4.5% flipped frames, so
# pos_weight ~= 21), which inflates flip_loss's raw magnitude roughly
# 10-20x; 0.1 brings its weighted contribution back down to something
# comparable to (not dominant over) the keypoint task's own loss -- an
# empirical starting point. Watch val/mean_keypoint_error_px alongside
# val/flip_precision and val/flip_recall on TensorBoard: if keypoint error
# regresses back toward v1's level, lower this further; if flip
# precision/recall look poor, that's more likely pos_weight itself needing
# a second look (see flip_label.py) than this weight.
set -euo pipefail

MAX_FRAMES_PER_TRIAL=500
BATCH_SIZE=256
N_EPOCHS=50
LEARNING_RATE=1e-3
FLIP_LOSS_WEIGHT=0.1
NUM_WORKERS=0
PATIENCE=10

repo_root="$(pwd)"
data_root="$repo_root/bulk_data/motion_prior/2dpose_model/labels/final_predictions"
split_path="$repo_root/bulk_data/motion_prior/2dpose_model/labels/train_val_split_29trials.json"
checkpoint_dir="$repo_root/bulk_data/motion_prior/localization_model/checkpoints/v2"
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
    --num-workers "$NUM_WORKERS" \
    --patience "$PATIENCE"
