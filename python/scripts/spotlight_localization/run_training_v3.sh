#!/bin/bash -l
# Third training round for the TinyLocalizationModel, keypoint-only again: same
# architecture/data/split as run_training_v1.sh, but adds
# HEATMAP_VARIANCE_WEIGHT to fix a real accuracy bug found by inspecting
# v1/v2's own predictions against ground truth on real frames.
#
# Soft-argmax's coordinate loss alone doesn't penalize a diffuse heatmap
# that happens to average out to the right place on the training set --
# it only breaks once the true point sits off-center within the spread,
# which pulls predictions toward the heatmap's own centroid. That's
# exactly what was observed: thorax (near the head/thorax/abdomen
# centroid) landed correctly, but head/abdomen (the outer two points)
# were pulled inward by tens of pixels, and separately, the model's
# overall predicted point spread was only 60-70% of the real physical
# spread (see box.py's docstring). model.heatmap_variance penalizes each
# keypoint heatmap's own spatial variance around its own expected
# coordinate, encouraging peaked heatmaps instead (the standard DSNT fix
# for this failure mode).
#
# HEATMAP_VARIANCE_WEIGHT=0.01 is a deliberately small starting point, the
# same way FLIP_LOSS_WEIGHT was tuned down in v2 after its raw
# magnitude turned out to dominate the keypoint loss -- watch
# val/heatmap_variance (should trend down from a diffuse heatmap's
# ~0.167 baseline) alongside val/mean_keypoint_error_px on TensorBoard;
# raise this weight if heatmaps stay diffuse, lower it if keypoint error
# regresses.
#
# FLIP_LOSS_WEIGHT=0.0: staged rollout again, same as v1 -- confirm
# the variance fix improves keypoint accuracy before re-enabling flip
# detection in a follow-up round.
set -euo pipefail

MAX_FRAMES_PER_TRIAL=500
BATCH_SIZE=256
N_EPOCHS=50
LEARNING_RATE=1e-3
FLIP_LOSS_WEIGHT=0.0
HEATMAP_VARIANCE_WEIGHT=0.01
NUM_WORKERS=0
PATIENCE=10

repo_root="$(pwd)"
data_root="$repo_root/bulk_data/motion_prior/2dpose_model/labels/final_predictions"
split_path="$repo_root/bulk_data/motion_prior/2dpose_model/labels/train_val_split_29trials.json"
checkpoint_dir="$repo_root/bulk_data/motion_prior/localization_model/checkpoints/v3"
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
    --flip-loss-weight "$FLIP_LOSS_WEIGHT" \
    --heatmap-variance-weight "$HEATMAP_VARIANCE_WEIGHT" \
    --num-workers "$NUM_WORKERS" \
    --patience "$PATIENCE"
