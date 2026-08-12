#!/bin/bash -l
# Mirrors each genotype/trial directory from the Spotlight NAS share to
# bulk_data/motion_prior/spotlight_recordings/, so the rest of the pipeline
# never has to touch the NAS directly. Excludes the two heaviest, least
# pose-relevant categories of data per trial: raw behavior images (both the
# unpacked directory and its .zip, since both hold the same frames) and
# muscle images (the top-level .zip and its post-alignment counterparts
# under processed/). Safe to re-run: rsync skips files that already match.
#
# GENOTYPES excludes non-genotype top-level entries on the share
# (best_trials/, failed_recordings/, high_priority/, stray top-level .zip
# files) that don't follow the <genotype>/<fly_trial>/ layout.
set -euo pipefail

NAS_ROOT="/mnt/upramdya_data/VAS/poseforge_paper_data"
DEST_ROOT="bulk_data/motion_prior/spotlight_recordings"

repo_root="$(pwd)"
mkdir -p "$repo_root/$DEST_ROOT"

genotypes=$(cd "$NAS_ROOT" && find . -mindepth 1 -maxdepth 1 -type d \
    ! -name best_trials ! -name failed_recordings ! -name high_priority \
    -printf '%f\n')

for genotype in $genotypes; do
    echo "=== $genotype at $(date) ==="
    rsync -a --info=progress2 --partial \
        --exclude='behavior_images/' \
        --exclude='behavior_images.zip' \
        --exclude='muscle_images.zip' \
        --exclude='aligned_muscle_images/' \
        --exclude='aligned_muscle_images.h5' \
        --exclude='.DS_Store' \
        "$NAS_ROOT/$genotype/" "$repo_root/$DEST_ROOT/$genotype/"
done

echo "=== All genotypes synced at $(date) ==="
