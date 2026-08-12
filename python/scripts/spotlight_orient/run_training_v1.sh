#!/bin/bash -l
# First real training round for the TinyOrientModel, keypoint-only:
# coarse head/thorax/abdomen localization directly on the raw (unaligned,
# uncropped) camera frame at 0.25x resolution -- see
# src/poseforge/motion_prior/spotlight_orient/{model,dataset}.py.
#
# An earlier round (not kept -- see git history if needed) used a
# global-average-pool + FC regression head for keypoints, which turned out
# to be a real architectural bug: GAP destroys spatial information before
# the head ever sees it, so the model could only ever learn the dataset's
# mean position, confirmed empirically (its predictions barely changed
# across completely different random inputs, and val/mean_keypoint_error_px
# sat dead flat around 210px regardless of loss-weight tuning). model.py
# now predicts a small heatmap per keypoint and extracts the point via a
# differentiable soft-argmax instead, which actually lets gradient reach a
# real spatial signal. A 3-trial/60-epoch smoke test with this fixed
# architecture dropped val_pixel_error from ~210px to ~33px, still clearly
# decreasing when the test ended -- this run uses all 23 train trials, so
# should do at least as well.
#
# FLIP_LOSS_WEIGHT=0.0: flip classification is deliberately left disabled
# for this round -- staged rollout, keypoint regression first. Once this
# round's accuracy is confirmed good, run_training_v2.sh repeats it with
# flip detection turned on.
#
# Reuses spotlight_pose2d's own 23/6 train/val split over the same 29
# trials (train_val_split_29trials.json), just pointed at
# labels/final_predictions/ instead of labels/ported_lm_predictions/, since
# both this model's keypoint targets and its flip-confidence proxy label
# (flip_label.py) come from RepVGG-A0's own exhaustive per-frame
# predictions there.
#
# MAX_FRAMES_PER_TRIAL bounds how much of each trial's cached frames
# TinyOrientDataset loads into memory at once (see
# cache_fullsize_frames.py, which must be run first) -- 500/trial keeps the
# in-memory dataset small; raise it for better coverage once the pipeline's
# confirmed to work end to end, or leave unset (drop the flag) to use every
# cached frame.
set -euo pipefail

MAX_FRAMES_PER_TRIAL=500
BATCH_SIZE=256
N_EPOCHS=50
LEARNING_RATE=1e-3
FLIP_LOSS_WEIGHT=0.0
NUM_WORKERS=0
PATIENCE=10

repo_root="$(pwd)"
data_root="$repo_root/bulk_data/motion_prior/2dpose_model/labels/final_predictions"
split_path="$repo_root/bulk_data/motion_prior/2dpose_model/labels/train_val_split_29trials.json"
checkpoint_dir="$repo_root/bulk_data/motion_prior/orient_model/checkpoints/v1"
frame_cache_root="$repo_root/bulk_data/motion_prior/orient_model/frame_cache"

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
    --flip-loss-weight "$FLIP_LOSS_WEIGHT" \
    --num-workers "$NUM_WORKERS" \
    --patience "$PATIENCE"
