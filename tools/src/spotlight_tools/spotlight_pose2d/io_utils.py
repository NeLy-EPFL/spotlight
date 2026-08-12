"""Shared I/O helpers used by every tool/caller script in `tools/
spotlight_pose2d/` and `scripts/spotlight_pose2d/`.

Every tool takes a single trial's data (one video/trial in, one video/trial
out); `find_trial_dirs`-style multi-trial batching from the old pipeline is
gone, so an external loop (a small caller script) is what chains these across
a whole dataset.
"""

import json
import re
from pathlib import Path

import h5py
import numpy as np
import sleap_io as sio


def check_output_path(output_path: Path, override: bool) -> None:
    """Abort if `output_path` already exists, unless `override` is set.

    Args:
        output_path: Path a script is about to write to.
        override: If True, an existing file is silently overwritten; if
            False (every script's default), abort rather than clobber it.
    """
    if output_path.exists() and not override:
        raise SystemExit(
            f"{output_path} already exists; pass --override to overwrite it."
        )


def parse_trial_identity(video_path: Path) -> tuple[str, str]:
    """Parse `(genotype, fly_trial)` from a video path.

    Assumes the `.../<genotype>/<fly_trial>/processed/<video>` directory
    convention used throughout this project (e.g.
    `.../G-213xCI55_260720/fly000_trial000/processed/aligned_behavior_video.mkv`).

    Args:
        video_path: Path to a trial's video file.

    Returns:
        `(genotype, fly_trial)`, e.g. `("G-213xCI55_260720", "fly000_trial000")`.
    """
    parts = Path(video_path).parts
    return parts[-4], parts[-3]


# The original hand-labeled `labels.v005.slp`'s own video-path convention
# (distinct from `parse_trial_identity`'s aligned/fullsize project convention):
# `.../june2026/<genotype>/<fly_trial>[_full]/processed/fullsize_behavior_video*.mkv`.
FIRST_ROUND_VIDEO_PATH_PATTERN = re.compile(
    r".*/june2026/(?P<genotype>[^/]+)/(?P<trial>[^/]+)/processed/"
    r"fullsize_behavior_video[^/]*\.mkv$"
)


def parse_genotype_trial(video_path: str) -> tuple[str, str] | None:
    """Parse `(genotype, fly_trial)` from a `labels.v005.slp` video path.

    Returns None if `video_path` doesn't match `FIRST_ROUND_VIDEO_PATH_PATTERN`
    (e.g. `labels.v005.slp`'s one video from an unrelated old experiment).
    """
    match = FIRST_ROUND_VIDEO_PATH_PATTERN.match(video_path)
    if match is None:
        return None
    trial = match["trial"].removesuffix("_full")
    return match["genotype"], trial


def user_labeled_frames(labels: sio.Labels, video: sio.Video) -> list[tuple[int, list]]:
    """This video's frames that have a real (non-predicted) instance.

    Returns:
        `(frame_idx, instances)` pairs, `instances` holding only the
        non-predicted ones (any co-existing `PredictedInstance` is dropped).
    """
    frames = []
    for lf in labels.labeled_frames:
        if lf.video is not video:
            continue
        instances = [
            inst for inst in lf.instances if not isinstance(inst, sio.PredictedInstance)
        ]
        if instances:
            frames.append((lf.frame_idx, instances))
    return frames


def rebuild_instance(instance: sio.Instance, skeleton: sio.Skeleton) -> sio.Instance:
    """Copy `instance` onto a shared `skeleton` object (same node names, but
    a different file's own skeleton is otherwise a distinct Python object,
    which `sio.Labels` would otherwise treat as a second, separate skeleton).
    """
    if isinstance(instance, sio.PredictedInstance):
        points_and_scores = instance.numpy(scores=True)
        return sio.PredictedInstance.from_numpy(
            points_data=points_and_scores[:, :2],
            skeleton=skeleton,
            point_scores=points_and_scores[:, 2],
            score=instance.score,
        )
    return sio.Instance.from_numpy(points_data=instance.numpy(), skeleton=skeleton)


def try_resolve_video_path(video_path: str | Path) -> Path | None:
    """Resolve a SLEAP `Video`'s (possibly relative) filename to an absolute path.

    SLEAP stores a video's path exactly as given when the labels were
    created, which is often relative to whatever CWD was active at the time
    (e.g. `sleap-track` run from a trial directory) rather than to the
    `.slp` file's own location. Guessing a directory-depth convention here
    could silently produce a wrong path if guessed incorrectly, so a
    relative path is only ever resolved against the current working
    directory.

    Args:
        video_path: A `sio.Video.filename` value.

    Returns:
        The resolved absolute path, or None if it doesn't point to an
        existing file.
    """
    resolved = Path(video_path)
    if not resolved.is_absolute():
        resolved = (Path.cwd() / resolved).resolve()
    return resolved if resolved.is_file() else None


def resolve_video_path(video_path: str | Path, context: str) -> Path:
    """Like `try_resolve_video_path`, but raises instead of returning None.

    Args:
        video_path: A `sio.Video.filename` value.
        context: Short description of what's being resolved, for the error
            message (e.g. the input file path this came from).

    Returns:
        The resolved, existing absolute path.
    """
    resolved = try_resolve_video_path(video_path)
    if resolved is None:
        candidate = Path(video_path)
        if not candidate.is_absolute():
            candidate = (Path.cwd() / candidate).resolve()
        raise SystemExit(
            f"{context}: video path {str(video_path)!r} does not resolve to an "
            f"existing file (resolved to {candidate}). Re-run from the directory "
            "the video path is relative to, or fix the video reference."
        )
    return resolved


def trial_dir_from_video_path(video_path: Path) -> Path:
    """The trial directory (containing `metadata.zip`, `processed/`, ...)
    that owns a given video path.

    Args:
        video_path: Path to a trial's video file, two levels under the
            trial directory (see `parse_trial_identity`).

    Returns:
        The trial directory.
    """
    return Path(video_path).parents[1]


# Dense, per-trial pose `.h5` schema shared by `slp_convert_h5.py` and
# `quickik_solve.py`: one row per video frame (not just labeled/promoted
# ones), so downstream period-finding can work on a contiguous boolean mask
# rather than a sparse, index-addressed one.
POSE_H5_DATASETS = ("poses", "keypoint_scores", "instance_score", "is_label")


def save_pose_h5(
    output_path: Path,
    poses: np.ndarray,
    keypoint_scores: np.ndarray,
    instance_score: np.ndarray,
    is_label: np.ndarray,
    node_names: list[str],
    video_path: Path,
    extra_attrs: dict | None = None,
) -> None:
    """Save one trial's dense per-frame poses to the shared `.h5` schema.

    Args:
        output_path: Where to save the `.h5` file.
        poses: `(n_frames, n_nodes, 2)` keypoint coordinates, NaN where absent.
        keypoint_scores: `(n_frames, n_nodes)` per-keypoint detection scores,
            NaN where absent.
        instance_score: `(n_frames,)` overall instance detection score, NaN
            where absent (e.g. a label with no matching prediction).
        is_label: `(n_frames,)` bool, True for frames with a user-labeled
            `Instance` (not just a `PredictedInstance`).
        node_names: Skeleton node name per column of the arrays above.
        video_path: Absolute path to the video these poses belong to, saved
            as an attr so `--h5-to-slp` doesn't need a separate `--video-path`.
        extra_attrs: Additional root `.attrs` to write (e.g. the parameters a
            downstream script ran with), merged in as-is.
    """
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with h5py.File(output_path, "w") as f:
        f.create_dataset("poses", data=poses, compression="gzip")
        f.create_dataset("keypoint_scores", data=keypoint_scores, compression="gzip")
        f.create_dataset("instance_score", data=instance_score, compression="gzip")
        f.create_dataset("is_label", data=is_label, compression="gzip")
        f.attrs["node_names"] = node_names
        f.attrs["video_path"] = str(Path(video_path).resolve())
        for key, value in (extra_attrs or {}).items():
            f.attrs[key] = value


def load_pose_h5(input_path: Path) -> dict:
    """Load an `.h5` file written by `save_pose_h5` back into a plain dict.

    Args:
        input_path: `.h5` file to load.

    Returns:
        Dict with keys `poses`, `keypoint_scores`, `instance_score`,
        `is_label`, `node_names` (`list[str]`), `video_path` (`Path`).
    """
    with h5py.File(input_path, "r") as f:
        data = {name: f[name][:] for name in POSE_H5_DATASETS}
        data["node_names"] = [str(n) for n in f.attrs["node_names"]]
        data["video_path"] = Path(str(f.attrs["video_path"]))
    return data


def skeleton_json_dict(skeleton: sio.Skeleton) -> dict:
    """A skeleton's nodes/edges/symmetries (names only), as a plain dict.

    The schema `load_skeleton_json` reads back. `extract_metadata_from_
    initial_slp.py` writes this dict plus extra keys of its own merged in,
    so that script's output doubles as a `--skeleton-json-path` for tools
    that only need the skeleton part.

    Args:
        skeleton: Skeleton to extract nodes/edges/symmetries from.

    Returns:
        `{"node_names": [...], "edges": [[src, dst], ...], "symmetries":
        [[a, b], ...]}`.
    """
    return {
        "node_names": [node.name for node in skeleton.nodes],
        "edges": [[edge.source.name, edge.destination.name] for edge in skeleton.edges],
        "symmetries": [
            [node.name for node in symmetry.nodes] for symmetry in skeleton.symmetries
        ],
    }


def load_skeleton_json(input_path: Path) -> sio.Skeleton:
    """Load a `sio.Skeleton` from a JSON file written by `skeleton_json_dict`
    (whether saved on its own or, as `extract_metadata_from_initial_slp.py`
    does, alongside other keys this ignores).

    Args:
        input_path: JSON file to load.

    Returns:
        A fresh `sio.Skeleton`, with nodes in the saved order and the saved
        edges/symmetries attached.
    """
    data = json.loads(input_path.read_text())
    skeleton = sio.Skeleton(nodes=data["node_names"])
    skeleton.add_edges([tuple(edge) for edge in data["edges"]])
    skeleton.add_symmetries([tuple(symmetry) for symmetry in data["symmetries"]])
    return skeleton
