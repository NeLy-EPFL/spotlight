#!/bin/bash -l
# Sixth training round for the TinyLocalizationModel, keypoint-only again: same
# data/split as run_training_v5.sh, with two changes.
#
# 1. USE_GLOBAL_CONTEXT (default True, model.GlobalContextBlock): v5's
# receptive field, measured empirically (backprop from one heatmap
# pixel), was only ~92x92 raw camera pixels -- under a fifth of the fly's
# own head-to-abdomen-tip span (~460px, from spotlight_pose2d's own
# canonical aligned points). From any single heatmap location the network
# could only ever see a local patch of the fly's body, nowhere near
# enough to disambiguate "which end is this" from local texture alone --
# visually, v5's own heatmaps were multi-modal (several plausible local
# peaks) rather than one clean peak per keypoint, exactly what you'd
# expect from that. GlobalContextBlock global-average-pools the trunk and
# broadcasts the result back to every heatmap location (concatenated as
# extra channels before the heatmap head), giving the whole frame's own
# context (empirically confirmed: receptive field now covers ~100% of the
# input) on top of existing local detail, without changing the heatmap's
# own spatial resolution. v1-v5's checkpoints were all trained without
# this and need `--no-use-global-context` to load (see run_inference_v5.sh).
#
# 2. LEARNING_RATE=1e-3: back down from v5's 3e-3 -- this round's own
# change (the architecture) is significant enough on its own; not worth
# conflating with a second simultaneous change to rule out if something
# goes wrong. LR_SCHEDULE stays cosine (decaying a higher initial LR
# still seems like a reasonable default; only the initial value itself
# changes here).
#
# TARGET_SIGMA_NATIVE_PX, HEATMAP_LOSS_WEIGHT, KEYPOINT_SET carried over
# unchanged from run_training_v5.sh.
#
# FLIP_LOSS_WEIGHT=0.0: staged rollout again -- confirm the architecture
# fix actually lands accuracy (target: val_pixel_error ~10-15px) before
# re-enabling flip detection in a follow-up round.
set -euo pipefail

MAX_FRAMES_PER_TRIAL=500
BATCH_SIZE=256
N_EPOCHS=50
LEARNING_RATE=1e-3
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
checkpoint_dir="$repo_root/bulk_data/motion_prior/localization_model/checkpoints/v6"
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
