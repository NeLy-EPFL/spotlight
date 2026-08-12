#!/bin/bash -l
# Runs iter0c's checkpoint over a random sample of each trial's candidate
# frames (see sample_predictions_after_training.py), for a quick per-round
# quality check. When the next training pass finishes, write a new
# run_inference_iter0<x>.sh with that checkpoint/suffix rather than editing
# this one, so each round's exact invocation stays in git history as its
# own file.
set -euo pipefail

CHECKPOINT_PATH="bulk_data/motion_prior/2dpose_model/checkpoints/iter0c/best.pt"
LABELS_ROOT="bulk_data/motion_prior/2dpose_model/labels"
DATA_ROOT="$LABELS_ROOT/ported_lm_predictions"
OUTPUT_DIR="$LABELS_ROOT/rounds/iter0c"
FRAME_CACHE_ROOT="bulk_data/motion_prior/2dpose_model/frame_cache"
FIRST_ROUND_SLP="$LABELS_ROOT/first_round/labels.v005.slp"
SKELETON_JSON="$LABELS_ROOT/metadata.json"
OUTPUT_SUFFIX="_pose2d_iter0c_sample_predictions"
FRAMES_PER_TRIAL=2000
BATCH_SIZE=200

repo_root="$(pwd)"
merged_slp_path="$repo_root/$OUTPUT_DIR/merged_pose2d_iter0c_sample_predictions.slp"

source "$repo_root/.venv/bin/activate"

python "$repo_root/tools/spotlight_pose2d/sample_predictions_after_training.py" \
    --checkpoint-path "$repo_root/$CHECKPOINT_PATH" \
    --data-root "$repo_root/$DATA_ROOT" \
    --output-dir "$repo_root/$OUTPUT_DIR" \
    --frame-cache-root "$repo_root/$FRAME_CACHE_ROOT" \
    --first-round-slp-path "$repo_root/$FIRST_ROUND_SLP" \
    --skeleton-json-path "$repo_root/$SKELETON_JSON" \
    --output-suffix "$OUTPUT_SUFFIX" \
    --merged-slp-path "$merged_slp_path" \
    --frames-per-trial "$FRAMES_PER_TRIAL" \
    --batch-size "$BATCH_SIZE"
