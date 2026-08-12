#!/bin/bash -l
# Runs v9's TinyLocalizationModel checkpoint over one full validation trial
# (predicting every cached frame) and renders an annotated QA video --
# same two-step shape as spotlight_pose2d's own inference+visualization
# pipeline (see scripts/postprocessing/model_training/localization/{infer,visualize_predictions}.py).
#
# v9 is v8's exact setup (kept as-is -- v8's 19.4px was judged good
# enough to stop keypoint-only tuning) with FLIP_LOSS_WEIGHT=0.1 turned
# back on and N_EPOCHS raised to 200 (retrained from scratch so the
# cosine LR schedule's own T_max matched) -- see run_training_v9.sh.
# Early-stopped at epoch 40 (patience=15): best val_pixel_error 40.6px
# (epoch 25) -- a real regression from v8's 19.4px, and flip precision/
# recall (0.12/0.90 at the last epoch) show the flip head is calling
# "flipped" far too often. Since box coloring now finally reflects a
# genuinely trained flip head (see FLIP_DECISION_THRESHOLD's own
# comment), rather than v1/v3-v8's untrained one, this QA video is worth
# watching specifically for that -- but expect it to look worse than
# v8's own, both in keypoint placement and in over-eager gray (flipped)
# coloring.
#
# TRIAL_STEM matches every earlier round's own choice, for a direct
# comparison on the same trial.
#
# Output filenames are suffixed "_v9", matching the other rounds' own
# convention, so re-running this doesn't clobber another round's QA
# outputs for the same trial.
#
# CANONICAL_H5_PATHS covers every trial's own final_predictions.h5, not
# just this one, since it measures the RepVGG-A0 pipeline's own long-run
# aligned-domain coarse-keypoint position (see
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
CHECKPOINT_PATH_REL="bulk_data/motion_prior/localization_model/checkpoints/v9/best.pt"
KEYPOINT_SET=neck
VIZ_CRF=23
MAX_VIZ_FRAMES=3000

repo_root="$(pwd)"
tools_dir="$repo_root/scripts/postprocessing/model_training/localization"
labels_root="$repo_root/bulk_data/motion_prior/2dpose_model/labels/final_predictions"
frame_cache_root="$repo_root/bulk_data/motion_prior/localization_model/frame_cache"
checkpoint_path="$repo_root/$CHECKPOINT_PATH_REL"
output_dir="$repo_root/bulk_data/motion_prior/localization_model/predictions"
predictions_h5="$output_dir/${TRIAL_STEM}_localization_predictions_v9.h5"
viz_path="$output_dir/${TRIAL_STEM}_localization_viz_v9.mp4"

source "$repo_root/.venv/bin/activate"
mkdir -p "$output_dir"

canonical_h5_paths=$(ls "$labels_root"/*_pose2d_final_predictions.h5)

echo "=== Predicting $TRIAL_STEM at $(date) ==="
python "$tools_dir/infer.py" \
    --checkpoint-path "$checkpoint_path" \
    --input-path "$labels_root/${TRIAL_STEM}_pose2d_final_predictions.h5" \
    --frame-cache-root "$frame_cache_root" \
    --output-h5-path "$predictions_h5" \
    --keypoint-set "$KEYPOINT_SET" \
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
    --keypoint-set "$KEYPOINT_SET" \
    --crf "$VIZ_CRF" \
    --max-frames "$MAX_VIZ_FRAMES" \
    --override

echo "=== Done at $(date): $viz_path ==="
