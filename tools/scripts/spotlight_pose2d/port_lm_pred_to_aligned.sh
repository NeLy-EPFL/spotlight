#!/bin/bash -l
# Ports every locally-copied LM prediction .slp (see
# copy_lm_predictions_locally.py) through legacy-to-current conversion ->
# alignment -> confidence-based promotion, mirroring run_test_round.sh's
# steps 2/3/4 -- step 1 (inference) is already done, these predictions were
# copied pre-computed from the NAS. Run from the repo root. Each step
# writes a new file; nothing gets overwritten. One file per trial; merge
# the final `*_slp16_aligned_promoted.slp` files with `slp_merge.py` once
# they're all ready (not done here).
#
# Legacy conversion and alignment write into intermediates_dir (throwaway,
# not read by anything downstream); the final promoted output goes to
# final_dir, the directory the rest of the pipeline actually reads from.
#
# Each prediction's video path is stored relative to its own trial
# directory ("processed/fullsize_behavior_video.mkv"), which is different
# per file, so the alignment step (the only one that resolves the video
# path, to find that trial's own transforms) runs with its cwd temporarily
# set there -- everything else, and all --input-path/--output-path
# arguments, stay local.
set -euo pipefail

repo_root="$(pwd)"
tools_dir="$repo_root/tools/spotlight_pose2d"
intermediates_dir="$repo_root/bulk_data/motion_prior/2dpose_model/labels/pipeline_intermediates"
final_dir="$repo_root/bulk_data/motion_prior/2dpose_model/labels/ported_lm_predictions"
nas_root="/mnt/upramdya_data/VAS/poseforge_paper_data"

for source in "$intermediates_dir"/*.slp; do
    # Only the original copied predictions match this (no "_slp16" in the
    # name); this directory also holds every step's own output by the time
    # this script has run once, which the glob above would otherwise also
    # pick up and (mis)treat as a fresh original on a second run.
    case "$source" in
        *_slp16*) continue ;;
    esac

    stem="$(basename "$source" .slp)"
    # stem is "<genotype>__<fly_trial>"; only strip the FIRST "__" (fly_trial
    # itself has no "__" in it, but being explicit here avoids relying on that).
    genotype="${stem%%__*}"
    fly_trial="${stem#*__}"

    # Skip files already ported (e.g. a previous partial run); still a fresh
    # file per step, so nothing here overwrites anything.
    if [ -f "$final_dir/${stem}_slp16_aligned_promoted.slp" ]; then
        echo "=== Skipping $stem (already ported) ==="
        continue
    fi

    echo "=== $stem at $(date) ==="

    echo "--- 1. Legacy -> current format ---"
    uv run --project "$repo_root" python "$tools_dir/slp_convert_legacy.py" --legacy-to-current \
        --input-path "$source" \
        --output-path "$intermediates_dir/${stem}_slp16.slp"

    echo "--- 2. Align (raw fullsize -> aligned domain) ---"
    (cd "$nas_root/$genotype/$fly_trial" && uv run --project "$repo_root" python \
        "$tools_dir/slp_apply_alignment.py" --align \
        --input-path "$intermediates_dir/${stem}_slp16.slp" \
        --output-path "$intermediates_dir/${stem}_slp16_aligned.slp")

    echo "--- 3. Promote confident predictions to labels ---"
    mkdir -p "$final_dir"
    uv run --project "$repo_root" python "$tools_dir/slp_promote_by_confidence.py" \
        --input-path "$intermediates_dir/${stem}_slp16_aligned.slp" \
        --output-path "$final_dir/${stem}_slp16_aligned_promoted.slp" \
        --aligned-input

    echo "=== Done $stem at $(date) ==="
done

echo "=== All trials ported at $(date) ==="
