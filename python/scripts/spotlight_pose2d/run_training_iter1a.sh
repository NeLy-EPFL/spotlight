#!/bin/bash -l
# Fine-tunes iter0c on manually-corrected labels only (no confidence-promoted
# pseudo-labels): warm-starts from iter0c/best.pt instead of ImageNet weights
# and a random head, since iter0c already learned reasonable general fly-pose
# features from the (noisy) pseudo-labeled set it trained on. Run from the
# repo root.
#
# Rebuilds its own training data every time from CORRECTED_SLP (a merged,
# multi-video .slp, hand-corrected in the SLEAP GUI): slp_split_by_video.py
# splits it into one single-video .slp per trial (temp dir, discarded after),
# slp_convert_h5.py --slp-to-h5 converts each to DATA_DIR's *_pose.h5, then
# split_train_val.py rebuilds DATA_DIR's train_val_split.json. Cheap and
# CPU-only, so always redoing this is simpler than caching it.
#
# LEARNING_RATE/BACKBONE_LR_SCALE are far lower than iter0c's: this is a
# warm start on a much smaller (~300-frame), hand-corrected-only dataset, so
# both the head and backbone should move conservatively. BATCH_SIZE drops
# to 16 to match; N_EPOCHS/PATIENCE compensate for the smaller, noisier
# validation set.
set -euo pipefail

CORRECTED_SLP="bulk_data/motion_prior/2dpose_model/labels/rounds/iter0c/merged_pose2d_iter0c_sample_predictions_corrected.slp"
METADATA_JSON="bulk_data/motion_prior/2dpose_model/labels/metadata.json"
DATA_DIR="bulk_data/motion_prior/2dpose_model/labels/rounds/iter1a"
MIN_HAND_LABELED_FRAMES=20
VAL_FRACTION=0.2
SPLIT_SEED=0

N_EPOCHS=300
PATIENCE=15
BATCH_SIZE=16
LEARNING_RATE=5e-4
BACKBONE_LR_SCALE=0.01
WARMUP_STEPS=30
NUM_WORKERS=4
INIT_CHECKPOINT_PATH="bulk_data/motion_prior/2dpose_model/checkpoints/iter0c/best.pt"

repo_root="$(pwd)"
tools_dir="$repo_root/tools/spotlight_pose2d"
data_dir="$repo_root/$DATA_DIR"
split_path="$data_dir/train_val_split.json"
checkpoint_dir="$repo_root/bulk_data/motion_prior/2dpose_model/checkpoints/iter1a"
frame_cache_root="$repo_root/bulk_data/motion_prior/2dpose_model/frame_cache"

split_dir="$(mktemp -d)"
trap 'rm -rf "$split_dir"' EXIT

source "$repo_root/.venv/bin/activate"

echo "=== Splitting corrected .slp by video (temp: $split_dir) at $(date) ==="
python "$tools_dir/slp_split_by_video.py" \
    --input-path "$repo_root/$CORRECTED_SLP" \
    --output-dir "$split_dir"

echo "=== Converting each trial's split .slp -> dense pose .h5 at $(date) ==="
mkdir -p "$data_dir"
for slp_path in "$split_dir"/*.slp; do
    stem="$(basename "$slp_path" .slp)"
    python "$tools_dir/slp_convert_h5.py" --slp-to-h5 \
        --input-path "$slp_path" \
        --output-path "$data_dir/${stem}_pose.h5" \
        --override
done

echo "=== Building train/val split at $(date) ==="
python "$tools_dir/split_train_val.py" \
    --data-root "$data_dir" \
    --metadata-json-path "$repo_root/$METADATA_JSON" \
    --output-path "$split_path" \
    --min-hand-labeled-frames "$MIN_HAND_LABELED_FRAMES" \
    --val-fraction "$VAL_FRACTION" \
    --seed "$SPLIT_SEED" \
    --override

train_paths=$(python -c "
import json
stems = json.load(open('$split_path'))['train']
print(' '.join(f'$data_dir/{s}_pose.h5' for s in stems))
")
val_paths=$(python -c "
import json
stems = json.load(open('$split_path'))['validation']
print(' '.join(f'$data_dir/{s}_pose.h5' for s in stems))
")

echo "=== Training at $(date) ==="
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
    --init-checkpoint-path "$repo_root/$INIT_CHECKPOINT_PATH" \
    --no-pretrained-backbone
