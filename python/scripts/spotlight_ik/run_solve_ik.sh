#!/bin/bash -l
# Fits IK/FK to every trial's final 2D pose predictions (see
# ../spotlight_pose2d/run_inference_final.sh), one output `.h5` per trial.
# Safe to re-run: trials with an existing output are skipped.
set -euo pipefail

INPUT_DIR="bulk_data/motion_prior/2dpose_model/labels/final_predictions"
INPUT_SUFFIX="_pose2d_final_predictions.h5"
OUTPUT_DIR="bulk_data/motion_prior/2dpose_model/inverse_kinematics/ikfk"
OUTPUT_SUFFIX="_ikfk.h5"

repo_root="$(pwd)"
tools_dir="$repo_root/scripts/spotlight_ik"

source "$repo_root/.venv/bin/activate"

for h5_path in "$repo_root/$INPUT_DIR"/*"$INPUT_SUFFIX"; do
    stem="$(basename "$h5_path" "$INPUT_SUFFIX")"
    output_path="$repo_root/$OUTPUT_DIR/${stem}${OUTPUT_SUFFIX}"
    if [ -f "$output_path" ]; then
        echo "Skipping $stem (already fit)"
        continue
    fi
    echo "=== Fitting IK for $stem at $(date) ==="
    python "$tools_dir/solve_ik.py" --input-path "$h5_path" --output-path "$output_path"
done

echo "=== Done at $(date) ==="
