#!/bin/bash -l
# Starts real pose2d training on the 29 ported LM trials, split 80/20 by
# train_val_split_29trials.json (a fixed, seeded split over the trials
# available after copy_lm_predictions_locally.py; not stratified by
# genotype). Run from the repo root.
#
# PRETRAINED_BACKBONE=true downloads ImageNet weights on first run, so this
# needs network access once. Requires cache_all_trial_frames.py to have
# already cached every train/val trial (train.py's Dataset reads frames
# from that cache, not the source videos).
#
# BATCH_SIZE=128 was chosen to use ~85% of a 12 GB GPU's memory.
# LEARNING_RATE is scaled from a 64/4e-3 baseline via the standard
# linear-scaling rule; that's the HEAD's LR only -- see train.py's
# `backbone_lr_scale`/`warmup_steps` docstrings for why the backbone needs
# its own, much lower LR.
set -euo pipefail

N_EPOCHS=20
PATIENCE=5
BATCH_SIZE=128
LEARNING_RATE=8e-3
BACKBONE_LR_SCALE=0.1
WARMUP_STEPS=1500
NUM_WORKERS=8
PRETRAINED_BACKBONE=true

repo_root="$(pwd)"
data_root="$repo_root/bulk_data/motion_prior/2dpose_model/labels/ported_lm_predictions"
split_path="$repo_root/bulk_data/motion_prior/2dpose_model/labels/train_val_split_29trials.json"
checkpoint_dir="$repo_root/bulk_data/motion_prior/2dpose_model/checkpoints/iter0c"
frame_cache_root="$repo_root/bulk_data/motion_prior/2dpose_model/frame_cache"

source "$repo_root/.venv/bin/activate"

train_paths=$(python -c "
import json
stems = json.load(open('$split_path'))['train']
print(' '.join(f'$data_root/{s}_pose.h5' for s in stems))
")
val_paths=$(python -c "
import json
stems = json.load(open('$split_path'))['validation']
print(' '.join(f'$data_root/{s}_pose.h5' for s in stems))
")

pretrained_flag="--no-pretrained-backbone"
if [ "$PRETRAINED_BACKBONE" = true ]; then
    pretrained_flag="--pretrained-backbone"
fi

# shellcheck disable=SC2086 -- intentional word-splitting: tyro needs each
# path as its own --train-h5-paths/--val-h5-paths argument, not one string.
python "$repo_root/tools/spotlight_pose2d/train.py" \
    --train-h5-paths $train_paths \
    --val-h5-paths $val_paths \
    --checkpoint-dir "$checkpoint_dir" \
    --frame-cache-root "$frame_cache_root" \
    --n-epochs "$N_EPOCHS" \
    --patience "$PATIENCE" \
    --batch-size "$BATCH_SIZE" \
    --learning-rate "$LEARNING_RATE" \
    --backbone-lr-scale "$BACKBONE_LR_SCALE" \
    --warmup-steps "$WARMUP_STEPS" \
    --num-workers "$NUM_WORKERS" \
    "$pretrained_flag"
