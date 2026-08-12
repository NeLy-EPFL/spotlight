#!/bin/bash -l
# Eighth training round for the TinyLocalizationModel, keypoint-only again: same
# hyperparameters/data/split as run_training_v7.sh, with one change.
#
# TARGET_SIGMA_NATIVE_PX=15.0 (down from v5/v6/v7's 30.0, back to v4's
# original value): v7 ran the FULL 50 epochs (patience=15 never
# triggered) and landed on exactly the same 19.32px best as v6 -- since
# both share seed=0 and identical hyperparameters through that epoch,
# this is a deterministic confirmation that 19.3px is a genuine plateau
# for this recipe, not an early-stopping artifact. More epochs alone
# don't help further.
#
# The 30px target was chosen (doubled from 15px) before the receptive
# field was understood as v1-v5's real bottleneck (see
# model.GlobalContextBlock's docstring) -- back when a wider, easier
# target seemed safer given the network couldn't yet reliably localize
# anything. Now that global context has fixed that (v6/v7's own achieved
# pixel error, ~19px, is already SMALLER than the 30px target spread),
# the loss has no incentive to sharpen the heatmap any further than
# "spread ~30px" -- v6/v7's own heatmap_variance plateaued around 0.005,
# nowhere near v3's own much peakier ~0.001-0.003 range under a smaller
# target. Tightening the target back to 15px should push the model to
# commit to sharper, more precise heatmaps now that it has the context to
# actually do so.
#
# Everything else (USE_GLOBAL_CONTEXT, LEARNING_RATE=1e-3, LR_SCHEDULE=
# cosine, HEATMAP_LOSS_WEIGHT=0.0001, KEYPOINT_SET=neck, PATIENCE=15)
# unchanged from run_training_v7.sh.
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
TARGET_SIGMA_NATIVE_PX=15.0
KEYPOINT_SET=neck
NUM_WORKERS=0
PATIENCE=15

repo_root="$(pwd)"
data_root="$repo_root/bulk_data/motion_prior/2dpose_model/labels/final_predictions"
split_path="$repo_root/bulk_data/motion_prior/2dpose_model/labels/train_val_split_29trials.json"
checkpoint_dir="$repo_root/bulk_data/motion_prior/localization_model/checkpoints/v8"
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
    --keypoint-set "$KEYPOINT_SET" \
    --use-global-context \
    --num-workers "$NUM_WORKERS" \
    --patience "$PATIENCE"
