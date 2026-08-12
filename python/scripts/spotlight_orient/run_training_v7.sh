#!/bin/bash -l
# Seventh training round for the TinyOrientModel, keypoint-only again:
# same hyperparameters/data/split as run_training_v6.sh, with two changes
# to train.py itself (not this script's own flags) plus a longer patience.
#
# 1. Best-checkpoint/early-stopping criterion switched from val_loss to
# val_pixel_error directly. v6 illustrated exactly why this matters: its
# best-by-val_loss epoch (32) was 19.89px, but its actual lowest-pixel-
# error epoch (42, right where patience finally triggered) was 19.32px --
# val_loss (a weighted composite of keypoint MSE + gaussian_heatmap_loss)
# doesn't perfectly track val_pixel_error, the metric that actually
# matters, so tracking it directly for both "best.pt" and early stopping
# should recover checkpoints val_loss was silently passing over.
#
# 2. PATIENCE=15 (up from v6's 10): v6's val_pixel_error was still
# trending down (noisily) when its old val_loss-based patience=10
# triggered at epoch 42 -- more room to keep improving before giving up,
# now that the criterion being watched is the one we actually care about.
#
# Everything else (USE_GLOBAL_CONTEXT, LEARNING_RATE=1e-3, LR_SCHEDULE=
# cosine, TARGET_SIGMA_NATIVE_PX=30.0, HEATMAP_LOSS_WEIGHT=0.0001,
# KEYPOINT_SET=neck) is unchanged from run_training_v6.sh, which measured
# val_pixel_error 19.3px (best epoch by the new criterion) -- clearly
# better than v5's 23.7px (confirming model.GlobalContextBlock's
# receptive-field fix helped, and visually cleaning up v5's own
# multi-modal heatmaps into single coherent blobs per keypoint), but not
# yet at the ~10-15px target.
#
# FLIP_LOSS_WEIGHT=0.0: still keypoint-only -- if this round lands near
# the ~10-15px target, the next round keeps these hyperparameters and
# re-enables flip detection; if not, further keypoint-only tuning
# continues instead.
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
PATIENCE=15

repo_root="$(pwd)"
data_root="$repo_root/bulk_data/motion_prior/2dpose_model/labels/final_predictions"
split_path="$repo_root/bulk_data/motion_prior/2dpose_model/labels/train_val_split_29trials.json"
checkpoint_dir="$repo_root/bulk_data/motion_prior/orient_model/checkpoints/v7"
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
    --lr-schedule "$LR_SCHEDULE" \
    --flip-loss-weight "$FLIP_LOSS_WEIGHT" \
    --heatmap-loss-weight "$HEATMAP_LOSS_WEIGHT" \
    --target-sigma-native-px "$TARGET_SIGMA_NATIVE_PX" \
    --keypoint-set "$KEYPOINT_SET" \
    --use-global-context \
    --num-workers "$NUM_WORKERS" \
    --patience "$PATIENCE"
