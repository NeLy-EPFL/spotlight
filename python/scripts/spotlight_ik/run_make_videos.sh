#!/bin/bash -l
# Renders one QA clip (with the IK/FK overlay) per trial from
# run_solve_ik.sh's output. Safe to re-run: already-rendered clips are
# skipped (see make_videos.py's own --override).
#
# TODO: make_videos.py now requires an explicit --video-path (kinematics.h5
# no longer carries this itself, see spotlight_ik.io_utils' module
# docstring); this script hasn't been updated to resolve each trial's own
# aligned video path yet, so it will fail to run as-is.
set -euo pipefail

IK_ROOT="bulk_data/motion_prior/2dpose_model/inverse_kinematics"
INPUT_DIR="$IK_ROOT/ikfk"
OUTPUT_DIR="$IK_ROOT/qa_videos"

repo_root="$(pwd)"
tools_dir="$repo_root/scripts/spotlight_ik"

source "$repo_root/.venv/bin/activate"

for h5_path in "$repo_root/$INPUT_DIR"/*_ikfk.h5; do
    stem="$(basename "$h5_path" .h5)"
    echo "=== Rendering QA clip for $stem at $(date) ==="
    python "$tools_dir/make_videos.py" \
        --input-path "$h5_path" \
        --video-path "TODO" \
        --output-dir "$repo_root/$OUTPUT_DIR" \
        --with-ik \
        --videos-per-trial 1
done

echo "=== Done at $(date) ==="
