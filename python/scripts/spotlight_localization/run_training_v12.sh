#!/bin/bash -l
# Twelfth training round for the TinyLocalizationModel: identical setup to v11
# (warm-start from v8/best.pt, FREEZE_BACKBONE=true, same hyperparameters),
# retrained only because the flip label itself changed.
#
# dataset.py now derives the "flipped" target from flip_label.is_flipped
# applied to flip_label.weighted_confidence (weighted mean over EVERY node,
# proximal ThC/CTr leg joints at 5x, everything else at 1x) instead of
# proximal_leg_confidence (mean over just the 12 proximal nodes). Intuition
# and validation: scripts/spotlight_localization/flip_proxy_analysis_weighted.py.
# At FLIPPED_THRESHOLD=0.5 the two metrics label almost the same frames
# (see that script's own analysis for why), so results here are expected to
# closely match v11's (precision ~0.42, val_pixel_error ~19.4px, frozen).
#
# Also: the keypoint-set customization (--keypoint-set default/neck) has
# been removed from the codebase entirely -- neck/thorax/abdomen is now
# the only supported target (dataset.COARSE_KEYPOINTS), so there's no
# --keypoint-set flag to pass here anymore.
set -euo pipefail

MAX_FRAMES_PER_TRIAL=500
BATCH_SIZE=256
N_EPOCHS=50
LEARNING_RATE=1e-3
LR_SCHEDULE=cosine
FLIP_LOSS_WEIGHT=1.0
HEATMAP_LOSS_WEIGHT=0.0001
TARGET_SIGMA_NATIVE_PX=15.0
NUM_WORKERS=0
PATIENCE=15
INIT_CHECKPOINT_REL="bulk_data/motion_prior/localization_model/checkpoints/v8/best.pt"

repo_root="$(pwd)"
data_root="$repo_root/bulk_data/motion_prior/2dpose_model/labels/final_predictions"
split_path="$repo_root/bulk_data/motion_prior/2dpose_model/labels/train_val_split_29trials.json"
checkpoint_dir="$repo_root/bulk_data/motion_prior/localization_model/checkpoints/v12"
frame_cache_root="$repo_root/bulk_data/motion_prior/localization_model/frame_cache"
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
    --use-global-context \
    --init-checkpoint "$init_checkpoint" \
    --freeze-backbone \
    --num-workers "$NUM_WORKERS" \
    --patience "$PATIENCE"
