#!/bin/bash -l
# Runs the latest checkpoint (iter2b/best.pt) over every frame of every
# trial -- the exhaustive, production inference pass, as opposed to
# run_inference_iter2a.sh's per-round 2000-frame spot check. Leaving
# --frames-per-trial unset (see sample_predictions_after_training.py)
# predicts every frame instead of a sample.
#
# Protects the original 186 hand labels (FIRST_ROUND_SLP) and every frame
# corrected during the iter0c/iter1a GUI review passes (CORRECTED_SLP_*)
# from re-prediction -- iter2b (like iter2a) was warm-started from
# iter1a/best.pt and has no GUI-corrected round of its own yet, so there's
# nothing further to add here.
#
# Output goes to labels/final_predictions/, separate from each round's own
# rounds/<round>/ sample predictions. Safe to re-run: already-predicted
# trials, and already-rendered summary videos, are both skipped.
set -euo pipefail

CHECKPOINT_PATH="bulk_data/motion_prior/2dpose_model/checkpoints/iter2b/best.pt"
LABELS_ROOT="bulk_data/motion_prior/2dpose_model/labels"
DATA_ROOT="$LABELS_ROOT/ported_lm_predictions"
OUTPUT_DIR="$LABELS_ROOT/final_predictions"
FRAME_CACHE_ROOT="bulk_data/motion_prior/2dpose_model/frame_cache"
FIRST_ROUND_SLP="$LABELS_ROOT/first_round/labels.v005.slp"
CORRECTED_SLP_ITER0C="$LABELS_ROOT/rounds/iter0c/merged_pose2d_iter0c_sample_predictions_corrected.slp"
CORRECTED_SLP_ITER1A="$LABELS_ROOT/rounds/iter1a/merged_pose2d_iter1a_sample_predictions_corrected.slp"
SKELETON_JSON="$LABELS_ROOT/metadata.json"
OUTPUT_SUFFIX="_pose2d_final_predictions"
BATCH_SIZE=200
VIZ_CRF=23

repo_root="$(pwd)"
tools_dir="$repo_root/tools/spotlight_pose2d"
output_dir="$repo_root/$OUTPUT_DIR"
merged_slp_path="$output_dir/merged_pose2d_final_predictions.slp"

source "$repo_root/.venv/bin/activate"

echo "=== Predicting every trial at $(date) ==="
python "$tools_dir/sample_predictions_after_training.py" \
    --checkpoint-path "$repo_root/$CHECKPOINT_PATH" \
    --data-root "$repo_root/$DATA_ROOT" \
    --output-dir "$output_dir" \
    --frame-cache-root "$repo_root/$FRAME_CACHE_ROOT" \
    --first-round-slp-path "$repo_root/$FIRST_ROUND_SLP" \
    --corrected-slp-paths "$repo_root/$CORRECTED_SLP_ITER0C" "$repo_root/$CORRECTED_SLP_ITER1A" \
    --skeleton-json-path "$repo_root/$SKELETON_JSON" \
    --output-suffix "$OUTPUT_SUFFIX" \
    --merged-slp-path "$merged_slp_path" \
    --batch-size "$BATCH_SIZE"

echo "=== Rendering summary videos at $(date) ==="
for h5_path in "$output_dir"/*"$OUTPUT_SUFFIX.h5"; do
    stem="$(basename "$h5_path" "$OUTPUT_SUFFIX.h5")"
    viz_path="$output_dir/${stem}${OUTPUT_SUFFIX}_viz.mp4"
    if [ -f "$viz_path" ]; then
        echo "Skipping $stem (already rendered)"
        continue
    fi
    python "$tools_dir/visualize_predictions.py" \
        --input-path "$h5_path" \
        --checkpoint-path "$repo_root/$CHECKPOINT_PATH" \
        --frame-cache-root "$repo_root/$FRAME_CACHE_ROOT" \
        --skeleton-json-path "$repo_root/$SKELETON_JSON" \
        --output-path "$viz_path" \
        --crf "$VIZ_CRF"
done

echo "=== Done at $(date) ==="
