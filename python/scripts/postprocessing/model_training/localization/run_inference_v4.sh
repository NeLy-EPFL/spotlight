#!/bin/bash -l
# Runs v4's TinyLocalizationModel checkpoint over one full validation trial
# (predicting every cached frame) and renders an annotated QA video --
# same two-step shape as spotlight_pose2d's own inference+visualization
# pipeline (see scripts/postprocessing/model_training/localization/{infer,visualize_predictions}.py).
#
# v4 is v1's architecture retrained with train.gaussian_heatmap_loss (see
# run_training_v4.sh) -- a cross-entropy against a fixed-sigma
# (TARGET_SIGMA_NATIVE_PX=15px) Gaussian label, replacing v3's cruder
# unbounded-variance-minimization regularizer, plus a higher learning
# rate (3e-3 vs. 1e-3) to speed convergence. Both v3 and v4 target the
# same soft-argmax centroid-shrinkage bias v1/v2 had (worst on
# head/abdomen, negligible on thorax).
#
# TRIAL_STEM matches run_inference_v1.sh's/v2.sh's/v3.sh's own choice,
# for a direct comparison across all four rounds on the same trial.
#
# Output filenames are suffixed "_v4", matching the other rounds' own
# convention, so re-running this doesn't clobber another round's QA
# outputs for the same trial.
#
# CANONICAL_H5_PATHS covers every trial's own final_predictions.h5, not
# just this one, since it measures the RepVGG-A0 pipeline's own long-run
# aligned-domain head/thorax/abdomen position (see
# box.measure_canonical_aligned_points) -- a global constant, not something
# specific to the trial being visualized.
#
# MAX_VIZ_FRAMES: visualize_predictions.py reads the source video in
# chunks (so it doesn't need this trial's full ~20k frames natively in
# memory at once, which at this model's raw fullsize resolution would be
# ~180GB), but the final resized frame list still has to fit in memory
# before pvio.write_frames_to_video -- at scale=0.5 that's ~45GB for the
# full trial. 3000 frames (~125s of video at ~24fps) keeps this well
# within budget while still showing several real upright/flipped
# transitions.
set -euo pipefail

TRIAL_STEM="Z-2419xCI55__fly000_trial000"
CHECKPOINT_PATH_REL="bulk_data/motion_prior/localization_model/checkpoints/v4/best.pt"
VIZ_CRF=23
MAX_VIZ_FRAMES=3000

repo_root="$(pwd)"
tools_dir="$repo_root/scripts/postprocessing/model_training/localization"
labels_root="$repo_root/bulk_data/motion_prior/2dpose_model/labels/final_predictions"
frame_cache_root="$repo_root/bulk_data/motion_prior/localization_model/frame_cache"
checkpoint_path="$repo_root/$CHECKPOINT_PATH_REL"
output_dir="$repo_root/bulk_data/motion_prior/localization_model/predictions"
predictions_h5="$output_dir/${TRIAL_STEM}_localization_predictions_v4.h5"
viz_path="$output_dir/${TRIAL_STEM}_localization_viz_v4.mp4"

source "$repo_root/.venv/bin/activate"
mkdir -p "$output_dir"

canonical_h5_paths=$(ls "$labels_root"/*_pose2d_final_predictions.h5)

echo "=== Predicting $TRIAL_STEM at $(date) ==="
python "$tools_dir/infer.py" \
    --checkpoint-path "$checkpoint_path" \
    --input-path "$labels_root/${TRIAL_STEM}_pose2d_final_predictions.h5" \
    --frame-cache-root "$frame_cache_root" \
    --output-h5-path "$predictions_h5" \
    --override

echo "=== Rendering QA video at $(date) ==="
# shellcheck disable=SC2086 -- intentional word-splitting: tyro needs each
# path as its own --canonical-h5-paths argument, not one string.
python "$tools_dir/visualize_predictions.py" \
    --input-path "$predictions_h5" \
    --checkpoint-path "$checkpoint_path" \
    --canonical-h5-paths $canonical_h5_paths \
    --frame-cache-root "$frame_cache_root" \
    --output-path "$viz_path" \
    --crf "$VIZ_CRF" \
    --max-frames "$MAX_VIZ_FRAMES" \
    --override

echo "=== Done at $(date): $viz_path ==="
