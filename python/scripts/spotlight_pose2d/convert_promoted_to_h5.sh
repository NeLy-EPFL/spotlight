#!/bin/bash -l
# Converts each trial's final promoted `.slp` into the dense, one-row-per-
# frame `.h5` schema (via `slp_convert_h5.py --slp-to-h5`) that
# `spotlight_postprocessing.spotlight_pose2d`'s training `Dataset` reads.
# `slp_convert_h5.py` is single-video only, so this runs once per trial
# rather than on the combined `merged_promoted.slp`.
#
# Prefers each trial's `*_slp16_aligned_promoted_handlabeled.slp` (see
# port_preexisting_handlabels.py) when one exists, since it has real
# hand-labeled frames in place of confidence-promoted ones for those three
# trials; falls back to the plain `*_slp16_aligned_promoted.slp` otherwise.
# Neither source file is touched either way. Run from the repo root.
set -euo pipefail

repo_root="$(pwd)"
tools_dir="$repo_root/tools/spotlight_pose2d"
data_root="$repo_root/bulk_data/motion_prior/2dpose_model/labels/ported_lm_predictions"

declare -A source_for_stem

for source in "$data_root"/*_slp16_aligned_promoted.slp; do
    [ -e "$source" ] || continue
    stem="$(basename "$source" _slp16_aligned_promoted.slp)"
    source_for_stem["$stem"]="$source"
done
for source in "$data_root"/*_slp16_aligned_promoted_handlabeled.slp; do
    [ -e "$source" ] || continue
    stem="$(basename "$source" _slp16_aligned_promoted_handlabeled.slp)"
    source_for_stem["$stem"]="$source"  # takes priority over the plain version
done

for stem in "${!source_for_stem[@]}"; do
    source="${source_for_stem[$stem]}"
    output="$data_root/${stem}_pose.h5"
    if [ -f "$output" ]; then
        echo "=== Skipping $stem (already converted) ==="
        continue
    fi

    echo "=== $stem at $(date) ==="
    uv run --project "$repo_root" python "$tools_dir/slp_convert_h5.py" --slp-to-h5 \
        --input-path "$source" \
        --output-path "$output"
done

echo "=== All trials converted at $(date) ==="
