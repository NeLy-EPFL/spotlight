#!/bin/bash -l
# Validates the pipeline end to end on the original hand-labeled batch
# (first_round): inference -> legacy-to-current conversion -> alignment ->
# promotion. Run from the repo root. Each step writes a new file; nothing
# gets overwritten.
#
# Step 0 also writes labels/metadata.json (skeleton + hand-labeled-frame
# index), which the rest of the pipeline depends on -- rerun it if the
# original hand labels ever change, even though the rest of this script is
# just a smoke test.
set -euo pipefail

tools_dir="tools/spotlight_pose2d"
labels_root="bulk_data/motion_prior/2dpose_model/labels"
data_root="$labels_root/first_round"
models_root="bulk_data/motion_prior/2dpose_model/models/spotlight_lm"

echo "=== 0. Extract skeleton + hand-labeled-frame metadata at $(date) ==="
uv run python "$tools_dir/extract_metadata_from_initial_slp.py" \
    --input-path "$data_root/labels.v005.slp" \
    --output-path "$labels_root/metadata.json"

echo "=== 1. Inference (legacy TF two-stage LM model) at $(date) ==="
uv run python "$tools_dir/slp_run_inference.py" \
    --input-path "$data_root/labels.v005.slp" \
    --output-path "$data_root/labels.v005_inferred.slp" \
    --model "$models_root/250327_002024.centroid" \
             "$models_root/260701_180342.centered_instance.n=215"

echo "=== 2. Legacy -> current format at $(date) ==="
uv run python "$tools_dir/slp_convert_legacy.py" --legacy-to-current \
    --input-path "$data_root/labels.v005_inferred.slp" \
    --output-path "$data_root/labels.v005_inferred_slp16.slp"

echo "=== 3. Align (raw fullsize -> aligned domain) at $(date) ==="
uv run python "$tools_dir/slp_apply_alignment.py" --align \
    --input-path "$data_root/labels.v005_inferred_slp16.slp" \
    --output-path "$data_root/labels.v005_inferred_slp16_aligned.slp"

echo "=== 4. Promote confident predictions to labels at $(date) ==="
uv run python "$tools_dir/slp_promote_by_confidence.py" \
    --input-path "$data_root/labels.v005_inferred_slp16_aligned.slp" \
    --output-path "$data_root/labels.v005_inferred_slp16_aligned_promoted.slp" \
    --aligned-input

echo "=== Done at $(date) ==="
