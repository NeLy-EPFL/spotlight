#!/usr/bin/env python
"""Merge multiple `.slp` files, each with their own distinct set of videos,
into one.

This is a straightforward union: every input file must share the same
skeleton (by node name), and no video path may appear in more than one
input file (an error, not a priority-based resolution). For incrementally
combining trials where the same video might already be present and you
need to choose which version wins, see `slp_convert_h5.py --h5-to-slp
--append-to`/`--priority` instead.

Requires current-format input (see `slp_convert_legacy.py`): unlike
`sleap_io`'s own transparent read support for the legacy format, this tool
doesn't accept it, since round-tripping a legacy file's own quirks through
a merge hasn't been verified.

Usage:
    python slp_merge.py --input-paths trial_a.slp trial_b.slp trial_c.slp \\
        --output-path merged.slp
"""

from pathlib import Path

import sleap_io as sio
import tyro
from loguru import logger
from slp_convert_legacy import is_legacy_format

from spotlight.postprocessing.pose2d.io_utils import (
    check_output_path,
    rebuild_instance,
)


def main(
    input_paths: list[Path],
    output_path: Path,
    override: bool = False,
) -> None:
    """Merge multiple `.slp` files into one.

    Args:
        input_paths: Two or more `.slp` files to merge, each in the current
            format, with the same skeleton (by node name) and a disjoint
            set of videos.
        output_path: Where to save the merged `.slp` file. Aborts if this
            already exists, unless `override` is set.
        override: If True, overwrite `output_path` if it already exists.
    """
    if len(input_paths) < 2:
        raise SystemExit("Provide at least two --input-paths to merge.")
    for path in input_paths:
        if not path.is_file():
            raise SystemExit(f"Input file does not exist: {path}")
        if is_legacy_format(path):
            raise SystemExit(
                f"{path} is in the legacy SLEAP format; convert it first "
                "with `slp_convert_legacy.py --legacy-to-current`."
            )
    check_output_path(output_path, override)

    skeleton = None
    seen_video_paths: set[str] = set()
    all_videos = []
    all_labeled_frames = []
    for path in input_paths:
        labels = sio.load_file(str(path))
        if skeleton is None:
            skeleton = labels.skeleton
        elif [n.name for n in labels.skeleton.nodes] != [
            n.name for n in skeleton.nodes
        ]:
            raise SystemExit(
                f"{path}: skeleton node names differ from {input_paths[0]}'s"
            )

        for video in labels.videos:
            if video.filename in seen_video_paths:
                raise SystemExit(
                    f"{path}: video {video.filename!r} also appears in an earlier "
                    "input file; slp_merge.py only merges disjoint video sets."
                )
            seen_video_paths.add(video.filename)
            all_videos.append(video)

        for lf in labels.labeled_frames:
            new_instances = [rebuild_instance(inst, skeleton) for inst in lf.instances]
            all_labeled_frames.append(
                sio.LabeledFrame(
                    video=lf.video, frame_idx=lf.frame_idx, instances=new_instances
                )
            )

    merged = sio.Labels(
        labeled_frames=all_labeled_frames, videos=all_videos, skeletons=[skeleton]
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    sio.save_file(merged, str(output_path))
    logger.info(
        f"Merged {len(input_paths)} files, {len(all_labeled_frames)} frames, "
        f"{len(all_videos)} videos -> {output_path}"
    )


if __name__ == "__main__":
    tyro.cli(main)
