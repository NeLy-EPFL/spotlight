"""5-panel QA visualization for one postprocessed recording, cv2/pvio only
(no matplotlib): raw+orient-box overlay, cropped+aligned (blank if flipped),
muscle (if requested), pose2d skeleton overlay (if requested), synthetic 3D
IK panel (if requested). Every panel is resized to a common height and
horizontally concatenated. Reads back already-computed outputs (video files,
muscle H5, pose2d H5, IK/FK H5) -- no model re-inference here, that already
happened in `behavior.process_behavior_pipeline`.

Rendering is chunked and parallelized (joblib): each worker seeks its own
`cv2.VideoCapture` to its chunk's start frame, composites its frames, and
encodes them directly to its own small temp video via `pvio` -- no JPEG
scratch-file round trip (unlike `StreamingVideoWriter`, which needs one
since a whole trial's frames don't fit in memory at once; one chunk's worth
does). Chunk videos are concatenated at the end via ffmpeg's stream-copy
concat demuxer (no re-encode, since every chunk shares the same codec
params).
"""

import json
import logging
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import cv2
import h5py
import numpy as np
import pandas as pd
import pvio
from joblib import Parallel, delayed
from spotlight_tools.spotlight_orient.box import raw_domain_box_corners
from spotlight_tools.spotlight_pose2d.viz import (
    build_edge_colors,
    build_node_colors,
    draw_pose,
)

VISUALIZATION_HEIGHT = 450
FLIP_DECISION_THRESHOLD = 0.5
YELLOW = (0, 255, 255)  # BGR
GRAY = (128, 128, 128)
WHITE = (255, 255, 255)
DEFAULT_CHUNK_SIZE = 500

_IK_SCRIPTS_DIR = Path(__file__).resolve().parents[3] / "scripts" / "spotlight_ik"


def _round_to_multiple(value: int, multiple: int = 16) -> int:
    return max(multiple, int(round(value / multiple)) * multiple)


def _resize_to_height(frame: np.ndarray, height: int) -> np.ndarray:
    h, w = frame.shape[:2]
    new_w = _round_to_multiple(round(w * height / h))
    return cv2.resize(frame, (new_w, height))


def _render_chunk(
    *,
    chunk_start: int,
    chunk_end: int,
    tmpdir: str,
    raw_paths: list[Path],
    show_aligned: bool,
    aligned_video_path: Path | None,
    keypoints_pre: np.ndarray | None,
    canonical_points: np.ndarray | None,
    flipped: np.ndarray | None,
    with_muscle: bool,
    muscle_h5_path: Path | None,
    muscle_dataset_name: str | None,
    muscle_meta: pd.DataFrame | None,
    muscle_vrange: tuple[int, int] | None,
    with_pose2d: bool,
    pose2d_data: np.ndarray | None,
    node_names: list[str] | None,
    edges: list[tuple[int, int]] | None,
    point_colors: list[tuple[int, int, int]] | None,
    edge_colors: list[tuple[int, int, int]] | None,
    with_ik: bool,
    ik_periods_by_frame: dict,
    th_idx_ik: int | None,
    thc_idxs: tuple | None,
    play_fps: float,
    crf: int,
    preset: str | None,
) -> str:
    """Composites and encodes one contiguous frame range, entirely within
    this worker process. Returns the path to this chunk's own temp video.
    """
    aligned_cap = None
    if show_aligned:
        aligned_cap = cv2.VideoCapture(str(aligned_video_path))
        aligned_cap.set(cv2.CAP_PROP_POS_FRAMES, chunk_start)

    muscle_file = muscle_dataset = None
    if with_muscle:
        muscle_file = h5py.File(muscle_h5_path, "r")
        muscle_dataset = muscle_file[muscle_dataset_name]

    _raw_cache_path_idx, _raw_cache_channels = None, None

    def _raw_frame(i: int) -> np.ndarray:
        nonlocal _raw_cache_path_idx, _raw_cache_channels
        path_idx, channel = divmod(i, 3)
        if path_idx != _raw_cache_path_idx:
            image = cv2.imread(str(raw_paths[path_idx]))
            _raw_cache_channels = cv2.split(image)
            _raw_cache_path_idx = path_idx
        return _raw_cache_channels[channel]

    if with_ik:
        sys.path.insert(0, str(_IK_SCRIPTS_DIR))
        from make_videos import (
            compute_camera_orientation,
            compute_grid_lines,
            draw_fk_3d_panel,
            draw_grid_floor,
            project_relative_to_panel,
        )

    frames = []
    for i in range(chunk_start, chunk_end):
        panels = []
        raw_bgr = cv2.cvtColor(_raw_frame(i), cv2.COLOR_GRAY2BGR)

        is_flipped = flipped is not None and flipped[i] >= FLIP_DECISION_THRESHOLD
        if show_aligned and canonical_points is not None and keypoints_pre is not None:
            corners = raw_domain_box_corners(keypoints_pre[i], canonical_points)
            if corners is not None:
                color = GRAY if is_flipped else YELLOW
                cv2.polylines(
                    raw_bgr, [corners.astype(np.int32)], True, color, 4, cv2.LINE_AA
                )
        panels.append(_resize_to_height(raw_bgr, VISUALIZATION_HEIGHT))

        aligned_frame = None
        if show_aligned:
            ok, aligned_frame = aligned_cap.read()
            if not ok:
                aligned_frame = np.zeros_like(panels[0])
            display_aligned_frame = (
                np.zeros_like(aligned_frame) if is_flipped else aligned_frame
            )
            panels.append(
                _resize_to_height(display_aligned_frame, VISUALIZATION_HEIGHT)
            )

        if with_muscle:
            matches = muscle_meta.index[
                muscle_meta["corresponding_behavior_frame_id"] <= i
            ]
            if len(matches) > 0:
                muscle_img = muscle_dataset[int(matches[-1])]
                norm = np.clip(
                    (muscle_img.astype(np.float32) - muscle_vrange[0])
                    / max(1, muscle_vrange[1] - muscle_vrange[0]),
                    0, 1,
                )  # fmt: skip
                muscle_bgr = cv2.cvtColor(
                    (norm * 255).astype(np.uint8), cv2.COLOR_GRAY2BGR
                )
            else:
                muscle_bgr = np.zeros_like(panels[0])
            panels.append(_resize_to_height(muscle_bgr, VISUALIZATION_HEIGHT))

        period_info = ik_periods_by_frame.get(i)
        if with_pose2d:
            pose_panel = (
                aligned_frame.copy()
                if aligned_frame is not None
                else np.zeros(
                    (VISUALIZATION_HEIGHT, VISUALIZATION_HEIGHT, 3), dtype=np.uint8
                )
            )
            raw_points = pose2d_data[i].copy()
            if with_ik and period_info is not None:
                period, local_i = period_info
                draw_pose(
                    pose_panel,
                    raw_points,
                    edges,
                    [WHITE] * len(edges),
                    [WHITE] * len(node_names),
                )
                draw_pose(
                    pose_panel,
                    period.fk_2d_px[local_i],
                    edges,
                    edge_colors,
                    point_colors,
                )
            else:
                draw_pose(pose_panel, raw_points, edges, edge_colors, point_colors)
            panels.append(_resize_to_height(pose_panel, VISUALIZATION_HEIGHT))

        if with_ik:
            panel_3d = np.zeros(
                (VISUALIZATION_HEIGHT, VISUALIZATION_HEIGHT, 3), dtype=np.uint8
            )
            if (
                period_info is not None
                and th_idx_ik is not None
                and all(idx is not None for idx in thc_idxs)
            ):
                period, local_i = period_info
                fk_3d_mm_frame = period.fk_3d_mm[local_i]
                right_axis, up_axis, view_dir = compute_camera_orientation(
                    fk_3d_mm_frame, thc_idxs
                )
                draw_grid_floor(
                    panel_3d,
                    compute_grid_lines(right_axis, up_axis, VISUALIZATION_HEIGHT),
                )
                centered = fk_3d_mm_frame - fk_3d_mm_frame[th_idx_ik]
                points_2d = project_relative_to_panel(
                    centered, right_axis, up_axis, VISUALIZATION_HEIGHT
                )
                depths = centered @ view_dir
                draw_fk_3d_panel(
                    panel_3d, points_2d, depths, edges, edge_colors, point_colors
                )
            panels.append(panel_3d)

        frames.append(np.hstack(panels))

    if aligned_cap is not None:
        aligned_cap.release()
    if muscle_file is not None:
        muscle_file.close()

    chunk_path = str(Path(tmpdir) / f"chunk_{chunk_start:09d}.mp4")
    pvio.write_frames_to_video(
        chunk_path, frames, play_fps, mode="auto", quality=crf, preset=preset,
        quiet=True,
    )  # fmt: skip
    return chunk_path


def _concat_chunk_videos(chunk_paths: list[str], output_path: Path) -> None:
    """Fast stream-copy concat (no re-encode) -- every chunk shares the same
    codec/resolution/params, encoded by the same `pvio` call above."""
    with tempfile.NamedTemporaryFile("w", suffix=".txt", delete=False) as f:
        for p in chunk_paths:
            f.write(f"file '{p}'\n")
        list_path = f.name
    try:
        subprocess.run(
            [
                "ffmpeg", "-y", "-v", "error", "-f", "concat", "-safe", "0",
                "-i", list_path, "-c", "copy", str(output_path),
            ],
            check=True,
        )  # fmt: skip
    finally:
        Path(list_path).unlink(missing_ok=True)


def generate_summary_video(
    *,
    recording_dir: Path,
    postprocessed_dir: Path,
    alignment: str,
    with_muscle: bool,
    with_pose2d: bool,
    with_ik: bool,
    aligned_video_path: Path | None,
    fullsize_video_path: Path | None,
    flipped_prob: np.ndarray | None,
    muscle_h5_path: Path | None,
    muscle_dataset_name: str | None,
    pose2d_h5_path: Path | None,
    pose2d_skeleton_json_path: Path | None,
    ikfk_h5_path: Path | None,
    output_path: Path,
    muscle_vrange: tuple[int, int] | None,
    play_fps: float,
    crf: int,
    preset: str | None,
    num_workers: int = -1,
    chunk_size: int = DEFAULT_CHUNK_SIZE,
    max_frames: int | None = None,
) -> None:
    logger = logging.getLogger(__name__)
    show_aligned = alignment in ("aligned", "both")

    # Every pseudo-BGR file packs 3 consecutive monochrome behavior frames
    # (see behavior.py); n_frames counts the real, expanded frames.
    raw_paths = sorted((recording_dir / "behavior_images").glob("behavior_frame_*.jpg"))
    n_frames = len(raw_paths) * 3
    if max_frames is not None:
        n_frames = min(n_frames, max_frames)

    keypoints_pre = flipped = canonical_points = None
    alignment_metadata_path = postprocessed_dir / "behavior_alignment_transforms.h5"
    if show_aligned and alignment_metadata_path.exists():
        with h5py.File(alignment_metadata_path, "r") as f:
            keypoints_pre = f["keypoints_xy_pre_alignment"][:n_frames]
            keypoints_post = f["keypoints_xy_post_alignment"][:n_frames]
            if flipped_prob is None:
                flipped = f["flipped_prob"][:n_frames]
        canonical_points = np.nanmean(keypoints_post, axis=0)
    if flipped_prob is not None:
        flipped = flipped_prob

    pose2d_data = node_names = edges = point_colors = edge_colors = None
    if with_pose2d:
        with h5py.File(pose2d_h5_path, "r") as f:
            pose2d_data = f["poses"][:n_frames]
            node_names = [str(n) for n in f.attrs["node_names"]]
        skeleton = json.loads(pose2d_skeleton_json_path.read_text())
        name_to_idx = {name: i for i, name in enumerate(node_names)}
        edges = [
            (name_to_idx[a], name_to_idx[b])
            for a, b in skeleton["edges"]
            if a in name_to_idx and b in name_to_idx
        ]
        # `build_node_colors`/`build_edge_colors` return RGB tuples (see
        # `spotlight_pose2d.viz`'s own docstring: designed for pvio's pure-RGB
        # pipeline, "used as-is with no channel reordering"). This pipeline's
        # canvases are fed to pvio directly here (no cv2.imwrite round trip),
        # but still go through cv2.polylines/circle/line first, which write
        # into whatever channel order the array already has -- reversed here
        # once, up front, so the final colors match poseforge2's own
        # IK/pose2d QA videos.
        point_colors = [c[::-1] for c in build_node_colors(node_names)]
        edge_colors = [c[::-1] for c in build_edge_colors(edges, node_names)]

    ik_periods_by_frame = {}
    th_idx_ik = None
    thc_idxs = None
    if with_ik and ikfk_h5_path is not None and ikfk_h5_path.exists():
        from spotlight_tools.spotlight_ik.io_utils import load_ikfk_h5

        ik_data = load_ikfk_h5(ikfk_h5_path)
        ik_node_names = ik_data["node_names"]
        for period in ik_data["periods"]:
            for local_i, frame_idx in enumerate(
                range(period.start_idx, period.end_idx)
            ):
                ik_periods_by_frame[frame_idx] = (period, local_i)
        ik_name_to_idx = {name: i for i, name in enumerate(ik_node_names)}
        th_idx_ik = ik_name_to_idx.get("Th")
        thc_idxs = tuple(
            ik_name_to_idx.get(n) for n in ("LF_ThC", "RF_ThC", "LH_ThC", "RH_ThC")
        )

    muscle_meta = None
    if with_muscle:
        muscle_meta = pd.read_csv(postprocessed_dir / "muscle_frames_metadata.csv")
        if muscle_vrange is None:
            with h5py.File(muscle_h5_path, "r") as f:
                dataset = f[muscle_dataset_name]
                sample = dataset[:: max(1, len(dataset) // 50)]
            nonzero = sample[sample > 0]
            muscle_vrange = (
                (int(np.percentile(nonzero, 50)), int(np.percentile(nonzero, 99)))
                if nonzero.size
                else (0, 65535)
            )

    output_path.parent.mkdir(parents=True, exist_ok=True)
    chunk_bounds = list(range(0, n_frames, chunk_size)) + [n_frames]
    chunks = list(zip(chunk_bounds[:-1], chunk_bounds[1:]))
    logger.info(
        f"Rendering {n_frames} visualization frames across {len(chunks)} "
        f"parallel chunks of ~{chunk_size} frames each..."
    )

    t_render = time.perf_counter()
    with tempfile.TemporaryDirectory(prefix="viz_chunks_") as tmpdir:
        chunk_paths = Parallel(n_jobs=num_workers)(
            delayed(_render_chunk)(
                chunk_start=start,
                chunk_end=end,
                tmpdir=tmpdir,
                raw_paths=raw_paths,
                show_aligned=show_aligned,
                aligned_video_path=aligned_video_path,
                keypoints_pre=keypoints_pre,
                canonical_points=canonical_points,
                flipped=flipped,
                with_muscle=with_muscle,
                muscle_h5_path=muscle_h5_path,
                muscle_dataset_name=muscle_dataset_name,
                muscle_meta=muscle_meta,
                muscle_vrange=muscle_vrange,
                with_pose2d=with_pose2d,
                pose2d_data=pose2d_data,
                node_names=node_names,
                edges=edges,
                point_colors=point_colors,
                edge_colors=edge_colors,
                with_ik=with_ik,
                ik_periods_by_frame=ik_periods_by_frame,
                th_idx_ik=th_idx_ik,
                thc_idxs=thc_idxs,
                play_fps=play_fps,
                crf=crf,
                preset=preset,
            )
            for start, end in chunks
        )
        logger.info(
            f"STEP TIME viz_render_and_encode (parallel, per-chunk, no JPEG "
            f"scratch round trip): {time.perf_counter() - t_render:.1f}s"
        )

        t_concat = time.perf_counter()
        _concat_chunk_videos(chunk_paths, output_path)
        logger.info(
            f"STEP TIME viz_concat (stream copy, no re-encode): "
            f"{time.perf_counter() - t_concat:.1f}s"
        )

    logger.info(f"Saved {output_path}")
