#!/bin/bash -l
# Eleventh training round for the TinyOrientModel: fixes v10's real
# problem (flip head learning far too slowly to be useful -- val_flip_loss
# only crept from 1.76 to ~1.32 over 21 epochs, precision stuck ~0.15-0.18)
# with the actual right tool for the job.
#
# v10 shrank FLIP_LOSS_WEIGHT ~500x (0.1 -> 0.0002) to stop flip_loss's
# gradient from dominating the shared trunk and regressing keypoint
# accuracy (v9: 19.4px -> 40.6px). That protected the trunk, but shrinking
# the loss weight shrinks the gradient for EVERY parameter equally,
# including flip_trunk/flipped_head -- which started from a random init
# (v8 never trained them, FLIP_LOSS_WEIGHT=0.0 that round) and so actually
# needed a strong, not weak, gradient to learn anything in a reasonable
# number of epochs. Shrinking the weight solved keypoint regression by
# starving the flip head instead.
#
# FREEZE_BACKBONE=true (new) decouples the two properly: freezes
# conv1/conv2/conv3/global_context/heatmap_head (weights AND BatchNorm
# running stats) so the keypoint-relevant trunk cannot move even a
# little, no matter what FLIP_LOSS_WEIGHT is -- confirmed empirically,
# val_pixel_error and val_heatmap_variance were bit-for-bit IDENTICAL
# across 20 smoke-test epochs. With the trunk structurally protected,
# FLIP_LOSS_WEIGHT goes back to a normal 1.0 (pos_weight alone handles
# the class imbalance) and LEARNING_RATE back up to 1e-3, since only
# flip_trunk/flipped_head's ~12.7k params are actually training -- full
# gradient signal, no risk.
#
# train.py's own best-checkpoint/early-stopping criterion switches to
# val_flip_loss automatically when freeze_backbone is set (val_pixel_error
# can't be used as a stopping signal anymore -- it's now constant).
#
# INIT_CHECKPOINT=v8/best.pt (not v10's own checkpoint): v8 is the clean,
# unperturbed keypoint-only checkpoint; v10's own trunk had already (even
# if only slightly, given how weak its flip gradient was) been nudged by
# 21 epochs of joint training, so restarting from v8 is the more
# defensible "known good" starting point now that this round is
# structurally guaranteed not to disturb it further.
#
# Everything else (HEATMAP_LOSS_WEIGHT=0.0001, TARGET_SIGMA_NATIVE_PX=15.0,
# KEYPOINT_SET=neck, LR_SCHEDULE=cosine, N_EPOCHS=50, PATIENCE=15)
# unchanged from run_training_v10.sh.
set -euo pipefail

MAX_FRAMES_PER_TRIAL=500
BATCH_SIZE=256
N_EPOCHS=50
LEARNING_RATE=1e-3
LR_SCHEDULE=cosine
FLIP_LOSS_WEIGHT=1.0
HEATMAP_LOSS_WEIGHT=0.0001
TARGET_SIGMA_NATIVE_PX=15.0
KEYPOINT_SET=neck
NUM_WORKERS=0
PATIENCE=15
INIT_CHECKPOINT_REL="bulk_data/motion_prior/orient_model/checkpoints/v8/best.pt"

repo_root="$(pwd)"
data_root="$repo_root/bulk_data/motion_prior/2dpose_model/labels/final_predictions"
split_path="$repo_root/bulk_data/motion_prior/2dpose_model/labels/train_val_split_29trials.json"
checkpoint_dir="$repo_root/bulk_data/motion_prior/orient_model/checkpoints/v11"
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
    --freeze-backbone \
    --num-workers "$NUM_WORKERS" \
    --patience "$PATIENCE"
