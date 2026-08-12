"""5-panel QA visualization for one postprocessed recording, cv2/pvio only
(no matplotlib): raw+orient-box overlay, cropped+aligned (blank if flipped),
muscle (if requested), pose2d skeleton overlay (if requested), synthetic 3D
IK panel (if requested). Every panel is resized to a common height and
horizontally concatenated. Reads back already-computed outputs (video files,
muscle H5, pose2d H5, IK/FK H5) -- no model re-inference here, that already
happened in `behavior.process_behavior_pipeline`.
"""

import json
import logging
import sys
import time
from pathlib import Path

import cv2
import h5py
import numpy as np
import pandas as pd
from spotlight_tools.common.video import StreamingVideoWriter
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

_IK_SCRIPTS_DIR = Path(__file__).resolve().parents[3] / "scripts" / "spotlight_ik"


def _round_to_multiple(value: int, multiple: int = 16) -> int:
    return max(multiple, int(round(value / multiple)) * multiple)


def _resize_to_height(frame: np.ndarray, height: int) -> np.ndarray:
    h, w = frame.shape[:2]
    new_w = _round_to_multiple(round(w * height / h))
    return cv2.resize(frame, (new_w, height))


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
    _raw_cache_path_idx, _raw_cache_channels = None, None

    def _raw_frame(i: int) -> np.ndarray:
        nonlocal _raw_cache_path_idx, _raw_cache_channels
        path_idx, channel = divmod(i, 3)
        if path_idx != _raw_cache_path_idx:
            image = cv2.imread(str(raw_paths[path_idx]))
            _raw_cache_channels = cv2.split(image)
            _raw_cache_path_idx = path_idx
        return _raw_cache_channels[channel]

    aligned_cap = cv2.VideoCapture(str(aligned_video_path)) if show_aligned else None

    keypoints_pre = flipped = None
    canonical_points = None
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
        # canvases go through `cv2.imwrite` (BGR) inside `StreamingVideoWriter`
        # before pvio ever sees them, so they need reversing here to render
        # with the same colors as poseforge2's own IK/pose2d QA videos.
        point_colors = [c[::-1] for c in build_node_colors(node_names)]
        edge_colors = [c[::-1] for c in build_edge_colors(edges, node_names)]

    ik_periods_by_frame = {}
    th_idx_ik = None
    thc_idxs = None
    if with_ik and ikfk_h5_path is not None and ikfk_h5_path.exists():
        sys.path.insert(0, str(_IK_SCRIPTS_DIR))
        from make_videos import (  # noqa: E402
            compute_camera_orientation,
            compute_grid_lines,
            draw_fk_3d_panel,
            draw_grid_floor,
            project_relative_to_panel,
        )
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

    muscle_dataset = muscle_meta = muscle_file = None
    if with_muscle:
        muscle_file = h5py.File(muscle_h5_path, "r")
        muscle_dataset = muscle_file[muscle_dataset_name]
        muscle_meta = pd.read_csv(postprocessed_dir / "muscle_frames_metadata.csv")
        if muscle_vrange is None:
            sample = muscle_dataset[:: max(1, len(muscle_dataset) // 50)]
            nonzero = sample[sample > 0]
            muscle_vrange = (
                (int(np.percentile(nonzero, 50)), int(np.percentile(nonzero, 99)))
                if nonzero.size
                else (0, 65535)
            )

    output_path.parent.mkdir(parents=True, exist_ok=True)
    writer = StreamingVideoWriter(output_path, play_fps, crf, preset)
    t_render = time.perf_counter()
    for i in range(n_frames):
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
            # Panel 2 blanks specifically for flipped frames (display-only
            # flag); the underlying crop stays available in `aligned_frame`
            # for panel 4's background below, since pose2d itself ran on the
            # real best-effort crop regardless of flip status.
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

        writer.write_chunk([np.hstack(panels)])
        if (i + 1) % 500 == 0:
            logger.info(f"Rendered {i + 1}/{n_frames} visualization frames")
    logger.info(
        f"STEP TIME viz_render_loop (compositing, writing scratch JPEGs): "
        f"{time.perf_counter() - t_render:.1f}s"
    )

    if aligned_cap is not None:
        aligned_cap.release()
    if muscle_file is not None:
        muscle_file.close()
    t_encode = time.perf_counter()
    writer.close()
    logger.info(
        f"STEP TIME viz_encode (writer.close()): {time.perf_counter() - t_encode:.1f}s"
    )
    logger.info(f"Saved {output_path}")
