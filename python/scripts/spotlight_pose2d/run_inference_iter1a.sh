#!/bin/bash -l
# Runs iter1a's checkpoint over a random sample of each trial's candidate
# frames (see sample_predictions_after_training.py), for a quick per-round
# quality check. When the next training pass finishes, write a new
# run_inference_iter1<x>.sh with that checkpoint/suffix rather than editing
# this one, so each round's exact invocation stays in git history as its
# own file.
#
# Protects both the original 186 hand labels (FIRST_ROUND_SLP) and every
# frame corrected during the iter0c review pass (CORRECTED_SLP) from
# re-prediction -- DATA_ROOT's own *_pose.h5 files were never updated with
# those corrections, so without CORRECTED_SLP here, iter1a's checkpoint
# would silently overwrite them.
set -euo pipefail

CHECKPOINT_PATH="bulk_data/motion_prior/2dpose_model/checkpoints/iter1a/best.pt"
LABELS_ROOT="bulk_data/motion_prior/2dpose_model/labels"
DATA_ROOT="$LABELS_ROOT/ported_lm_predictions"
OUTPUT_DIR="$LABELS_ROOT/rounds/iter1a"
FRAME_CACHE_ROOT="bulk_data/motion_prior/2dpose_model/frame_cache"
FIRST_ROUND_SLP="$LABELS_ROOT/first_round/labels.v005.slp"
CORRECTED_SLP="$LABELS_ROOT/rounds/iter0c/merged_pose2d_iter0c_sample_predictions_corrected.slp"
SKELETON_JSON="$LABELS_ROOT/metadata.json"
OUTPUT_SUFFIX="_pose2d_iter1a_sample_predictions"
FRAMES_PER_TRIAL=2000
BATCH_SIZE=200

repo_root="$(pwd)"
merged_slp_path="$repo_root/$OUTPUT_DIR/merged_pose2d_iter1a_sample_predictions.slp"

source "$repo_root/.venv/bin/activate"

python "$repo_root/tools/spotlight_pose2d/sample_predictions_after_training.py" \
    --checkpoint-path "$repo_root/$CHECKPOINT_PATH" \
    --data-root "$repo_root/$DATA_ROOT" \
    --output-dir "$repo_root/$OUTPUT_DIR" \
    --frame-cache-root "$repo_root/$FRAME_CACHE_ROOT" \
    --first-round-slp-path "$repo_root/$FIRST_ROUND_SLP" \
    --corrected-slp-paths "$repo_root/$CORRECTED_SLP" \
    --skeleton-json-path "$repo_root/$SKELETON_JSON" \
    --output-suffix "$OUTPUT_SUFFIX" \
    --merged-slp-path "$merged_slp_path" \
    --frames-per-trial "$FRAMES_PER_TRIAL" \
    --batch-size "$BATCH_SIZE"
