#!/usr/bin/env python
"""Copy every existing LM-model prediction, for every trial of every
second_round genotype, from the NAS to a local directory, so the rest of
the pipeline can work on local files without touching the NAS.

Only the prediction `.slp` itself is copied (not the accompanying `.mp4`/
`.npz` files that live alongside it in the same `sleap/` directory), and
only for trials that actually have one (most don't). `GENOTYPES` is the
list of genotype directory names found under
`/mnt/datassd/spotlight/june2026/`; a name missing here entirely (e.g.
`V-7xC155`, an apparent typo/duplicate of `V-7xCI55` that's also empty on
that other mount) is skipped with a warning.

Hardcoded constants, no CLI args.

Usage:
    python scripts/spotlight_pose2d/copy_lm_predictions_locally.py
"""

import shutil
from pathlib import Path

from loguru import logger

GENOTYPES = [
    "G213xCI55_260708",
    "G213xCI55_260709",
    "G-213xGCaMP8m",
    "G213xOGL16_260709",
    "V-7xC155",
    "V-7xCI55",
    "V-7xCI55_260708",
    "Z-2419xCI55",
    "Z-2419xMC3",
    "Z-2419xOGS12",
]
REPO_ROOT = Path(__file__).resolve().parents[2]
NAS_ROOT = Path("/mnt/upramdya_data/VAS/poseforge_paper_data")
PREDICTION_RELPATH = Path("sleap/prediction_lm_full_behavior_video.slp")
LOCAL_DEST_ROOT = (
    REPO_ROOT / "bulk_data/motion_prior/2dpose_model/labels/pipeline_intermediates"
)


def main() -> None:
    LOCAL_DEST_ROOT.mkdir(parents=True, exist_ok=True)

    n_copied = 0
    n_skipped_existing = 0
    for genotype in GENOTYPES:
        genotype_dir = NAS_ROOT / genotype
        if not genotype_dir.is_dir():
            logger.warning(f"Skipping {genotype}: no such directory under {NAS_ROOT}")
            continue

        for trial_dir in sorted(p for p in genotype_dir.iterdir() if p.is_dir()):
            source = trial_dir / PREDICTION_RELPATH
            if not source.is_file():
                continue

            dest = LOCAL_DEST_ROOT / f"{genotype}__{trial_dir.name}.slp"
            if dest.is_file():
                logger.warning(
                    f"Skipping {genotype}/{trial_dir.name}: {dest} already exists"
                )
                n_skipped_existing += 1
                continue

            shutil.copy2(source, dest)
            logger.info(f"Copied {source} -> {dest}")
            n_copied += 1

    logger.info(
        f"Copied {n_copied} files ({n_skipped_existing} already present locally)"
    )


if __name__ == "__main__":
    main()
