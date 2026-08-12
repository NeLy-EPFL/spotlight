#!/usr/bin/env python
"""Caches every frame of each of the 29 trials' aligned videos as JPEG (see
`cache_video_frames.py`'s own docstring for why: sequential decode measured
~80x faster than the per-labeled-frame random seeking
`spotlight_postprocessing.spotlight_pose2d`'s dataset previously did). Caches
ALL frames, not just currently-labeled ones, independent of any particular
labeling iteration -- which frames end up labeled changes across
train/predict/correct rounds, but the underlying video frames don't, so
every future iteration reuses this cache without touching the NAS again.

Defaults to 450x450 only (this pipeline's actual training input
resolution; ~10.8 GB total at quality 90, measured). Add "900x900" to
SIZES below if you also want full-resolution frames cached (~68 GB more).

Runs N_WORKERS trials concurrently via joblib: measured as CPU-bound, not
NAS-bandwidth-bound (2 trials at once finished in ~1.1x one trial's time),
but each trial's decode/encode already uses several threads, so 4 is close
to this machine's practical ceiling (16 cores) without over-subscribing or
opening too many concurrent connections to the shared NAS mount.

Hardcoded constants, no CLI args.

Usage:
    python scripts/spotlight_pose2d/cache_all_trial_frames.py
"""

import subprocess
from pathlib import Path

from joblib import Parallel, delayed
from loguru import logger
from tqdm import tqdm

N_WORKERS = 4
SIZES = ["450x450"]
QUALITY = 90

REPO_ROOT = Path(__file__).resolve().parents[2]
CACHE_SCRIPT = REPO_ROOT / "tools/spotlight_pose2d/cache_video_frames.py"
DATA_ROOT = (
    REPO_ROOT / "bulk_data/motion_prior/2dpose_model/labels/ported_lm_predictions"
)
NAS_ROOT = Path("/mnt/upramdya_data/VAS/poseforge_paper_data")
CACHE_ROOT = REPO_ROOT / "bulk_data/motion_prior/2dpose_model/frame_cache"


def process_trial(source: Path) -> tuple[str, bool]:
    """Caches one trial's frames.

    Returns:
        `(stem, succeeded)`.
    """
    stem = source.name.removesuffix("_pose.h5")
    genotype, fly_trial = stem.split("__", 1)
    output_dir = CACHE_ROOT / stem

    if (output_dir / SIZES[0]).is_dir():
        logger.info(f"Skipping {stem} (already cached)")
        return stem, True

    video_path = (
        NAS_ROOT / genotype / fly_trial / "processed" / "aligned_behavior_video.mkv"
    )
    try:
        subprocess.run(
            [
                "uv",
                "run",
                "--project",
                str(REPO_ROOT),
                "python",
                str(CACHE_SCRIPT),
                "--input",
                str(video_path),
                "--output",
                str(output_dir),
                "--sizes",
                *SIZES,
                "--quality",
                str(QUALITY),
            ],
            check=True,
        )
    except subprocess.CalledProcessError as e:
        logger.error(f"{stem} failed: {e}")
        return stem, False
    return stem, True


def main() -> None:
    sources = sorted(DATA_ROOT.glob("*_pose.h5"))
    results = Parallel(n_jobs=N_WORKERS)(
        delayed(process_trial)(s) for s in tqdm(sources)
    )

    failed = [stem for stem, ok in results if not ok]
    logger.info(f"Processed {len(results)} trials ({len(failed)} failed)")
    if failed:
        raise SystemExit(f"Failed trials: {failed}")


if __name__ == "__main__":
    main()
