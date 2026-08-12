#!/bin/bash -l
# Runs v6's TinyLocalizationModel checkpoint over one full validation trial
# (predicting every cached frame) and renders an annotated QA video --
# same two-step shape as spotlight_pose2d's own inference+visualization
# pipeline (see scripts/postprocessing/model_training/localization/{infer,visualize_predictions}.py).
#
# v6 is v5's setup with model.GlobalContextBlock enabled (see
# run_training_v6.sh) -- fixes v5's own receptive field, measured at only
# ~92x92 raw pixels (under a fifth of the fly's ~460px body length),
# which visually showed up as multi-modal (several plausible local
# peaks) heatmaps rather than one clean peak per keypoint. Also
# LEARNING_RATE back down to 1e-3 from v5's 3e-3. Finished (patience-
# stopped at epoch 42) at val_pixel_error 19.3px, clearly better than
# v5's 23.7px -- confirms the architecture fix helped, though not yet at
# the ~10-15px target.
#
# --use-global-context is left at its default True here (unlike
# run_inference_v5.sh's required --no-use-global-context) since v6 was
# actually trained with the new architecture.
#
# TRIAL_STEM matches every earlier round's own choice, for a direct
# comparison on the same trial.
#
# Output filenames are suffixed "_v6", matching the other rounds' own
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
CHECKPOINT_PATH_REL="bulk_data/motion_prior/localization_model/checkpoints/v6/best.pt"
KEYPOINT_SET=neck
VIZ_CRF=23
MAX_VIZ_FRAMES=3000

repo_root="$(pwd)"
tools_dir="$repo_root/scripts/postprocessing/model_training/localization"
labels_root="$repo_root/bulk_data/motion_prior/2dpose_model/labels/final_predictions"
frame_cache_root="$repo_root/bulk_data/motion_prior/localization_model/frame_cache"
checkpoint_path="$repo_root/$CHECKPOINT_PATH_REL"
output_dir="$repo_root/bulk_data/motion_prior/localization_model/predictions"
predictions_h5="$output_dir/${TRIAL_STEM}_localization_predictions_v6.h5"
viz_path="$output_dir/${TRIAL_STEM}_localization_viz_v6.mp4"

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
