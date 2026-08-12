#!/bin/bash -l
# Runs v12's TinyLocalizationModel checkpoint over the same 4 trials used for
# v11's own QA pass (predicting every cached frame, then rendering an
# annotated video for each) -- see run_training_v12.sh for what changed
# between v11 and v12 (only the flip label proxy; the --keypoint-set flag
# no longer exists at all).
#
# Output filenames are suffixed "_v12", so re-running this doesn't clobber
# v11's own QA outputs for the same trials.
#
# CANONICAL_H5_PATHS covers every trial's own final_predictions.h5, not
# just the one being visualized, since it measures the RepVGG-A0
# pipeline's own long-run aligned-domain coarse-keypoint position (see
# box.measure_canonical_aligned_points) -- a global constant.
#
# MAX_VIZ_FRAMES: see run_inference_v11.sh's own comment for why this is
# capped well below a full ~20k-frame trial (memory, not compute).
set -euo pipefail

TRIAL_STEMS=(
    "Z-2419xCI55__fly000_trial000"
    "V-7xCI55_260708__fly004_trial000"
    "G213xCI55_260709__fly000_trial000"
    "G213xOGL16_260709__fly000_trial000"
)
CHECKPOINT_PATH_REL="bulk_data/motion_prior/localization_model/checkpoints/v12/best.pt"
VIZ_CRF=23
MAX_VIZ_FRAMES=3000

repo_root="$(pwd)"
tools_dir="$repo_root/scripts/spotlight_localization"
labels_root="$repo_root/bulk_data/motion_prior/2dpose_model/labels/final_predictions"
frame_cache_root="$repo_root/bulk_data/motion_prior/localization_model/frame_cache"
checkpoint_path="$repo_root/$CHECKPOINT_PATH_REL"
output_dir="$repo_root/bulk_data/motion_prior/localization_model/predictions"

source "$repo_root/.venv/bin/activate"
mkdir -p "$output_dir"

canonical_h5_paths=$(ls "$labels_root"/*_pose2d_final_predictions.h5)

for trial_stem in "${TRIAL_STEMS[@]}"; do
    predictions_h5="$output_dir/${trial_stem}_localization_predictions_v12.h5"
    viz_path="$output_dir/${trial_stem}_localization_viz_v12.mp4"

    echo "=== Predicting $trial_stem at $(date) ==="
    python "$tools_dir/infer.py" \
        --checkpoint-path "$checkpoint_path" \
        --input-path "$labels_root/${trial_stem}_pose2d_final_predictions.h5" \
        --frame-cache-root "$frame_cache_root" \
        --output-h5-path "$predictions_h5" \
        --override

    echo "=== Rendering QA video for $trial_stem at $(date) ==="
    # shellcheck disable=SC2086 -- intentional word-splitting: tyro needs
    # each path as its own --canonical-h5-paths argument, not one string.
    python "$tools_dir/visualize_predictions.py" \
        --input-path "$predictions_h5" \
        --checkpoint-path "$checkpoint_path" \
        --canonical-h5-paths $canonical_h5_paths \
        --frame-cache-root "$frame_cache_root" \
        --output-path "$viz_path" \
        --crf "$VIZ_CRF" \
        --max-frames "$MAX_VIZ_FRAMES" \
        --override

    echo "=== Done with $trial_stem at $(date): $viz_path ==="
done
