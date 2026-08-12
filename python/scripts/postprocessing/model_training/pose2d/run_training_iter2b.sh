#!/bin/bash -l
# Sibling to run_training_iter2a.sh: same warm start (iter1a/best.pt), same
# corrected-labels-only training data, same hyperparameters, except a
# tighter Gaussian heatmap target (HEATMAP_SIGMA=1.0 vs iter2a's 2.0) --
# isolates the effect of that one change rather than chaining forward from
# iter2a itself. See run_training_iter2a.sh's header for the rest of the
# pipeline/hyperparameter rationale, unchanged here. Run from the repo root.
#
# Finishes with the same QA pass as run_training_iter2a.sh: exhaustive
# inference on one small trial with the resulting best.pt, then a quick
# annotated summary video, so the two sigmas' outputs can be compared
# directly on the same trial.
set -euo pipefail

CORRECTED_SLP="bulk_data/motion_prior/2dpose_model/labels/rounds/iter1a/merged_pose2d_iter1a_sample_predictions_corrected.slp"
METADATA_JSON="bulk_data/motion_prior/2dpose_model/labels/metadata.json"
DATA_DIR="bulk_data/motion_prior/2dpose_model/labels/rounds/iter2b"
MIN_HAND_LABELED_FRAMES=20
VAL_FRACTION=0.2
SPLIT_SEED=0

N_EPOCHS=300
PATIENCE=15
BATCH_SIZE=16
LEARNING_RATE=5e-4
BACKBONE_LR_SCALE=0.01
WARMUP_STEPS=30
HEATMAP_SIGMA=1.0
NUM_WORKERS=4
INIT_CHECKPOINT_PATH="bulk_data/motion_prior/2dpose_model/checkpoints/iter1a/best.pt"

QA_TRIAL_STEM="G213xOGL16_260709__fly005_trial000"
QA_DATA_ROOT="bulk_data/motion_prior/2dpose_model/labels/ported_lm_predictions"

repo_root="$(pwd)"
tools_dir="$repo_root/tools/spotlight_pose2d"
data_dir="$repo_root/$DATA_DIR"
split_path="$data_dir/train_val_split.json"
checkpoint_dir="$repo_root/bulk_data/motion_prior/2dpose_model/checkpoints/iter2b"
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
    --heatmap-sigma "$HEATMAP_SIGMA" \
    --num-workers "$NUM_WORKERS" \
    --init-checkpoint-path "$repo_root/$INIT_CHECKPOINT_PATH" \
    --no-pretrained-backbone

echo "=== QA: full-trial inference on $QA_TRIAL_STEM at $(date) ==="
python "$tools_dir/predict_unlabeled_frames.py" \
    --checkpoint-path "$checkpoint_dir/best.pt" \
    --input-path "$repo_root/$QA_DATA_ROOT/${QA_TRIAL_STEM}_pose.h5" \
    --frame-cache-root "$frame_cache_root" \
    --skeleton-json-path "$repo_root/$METADATA_JSON" \
    --output-h5-path "$data_dir/${QA_TRIAL_STEM}_qa_full.h5" \
    --output-slp-path "$data_dir/${QA_TRIAL_STEM}_qa_full.slp" \
    --override

echo "=== QA: rendering summary video at $(date) ==="
python "$tools_dir/visualize_predictions.py" \
    --input-path "$data_dir/${QA_TRIAL_STEM}_qa_full.h5" \
    --checkpoint-path "$checkpoint_dir/best.pt" \
    --frame-cache-root "$frame_cache_root" \
    --skeleton-json-path "$repo_root/$METADATA_JSON" \
    --output-path "$data_dir/${QA_TRIAL_STEM}_qa_full_viz.mp4" \
    --crf 23 \
    --override

echo "=== Done at $(date) ==="
