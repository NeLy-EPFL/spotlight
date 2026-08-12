#!/usr/bin/env python
"""Renders QA video clips of `solve_ik.py`'s periods+IK/FK `.h5` output: one
short clip per sampled period. Raw predictions (`pred_2d_px`) are drawn in
white; with `--with-ik`, the IK forward-kinematics result (`fk_2d_px`) is
drawn on top with the same per-leg-chain colors as `visualize_predictions.
py` (see `spotlight_pose2d.viz`), the thorax is omitted from the raw layer
(the IK layer already draws it there), the two SLEAP-skeleton edges from
the thorax to the midleg ThC nodes are never drawn (see
`EXCLUDED_EDGE_NAME_PAIRS`), and a second panel is added: a
synthetic 3D view of the IK reconstruction (`fk_3d_mm`, same colors) plus a
ground-plane grid, using an orthographic camera that recenters on the
thorax and tracks the fly's heading (yaw only) every frame -- the fly stays
visually fixed on screen while the grid rotates underneath it as it turns.
Ported from this project's own prior (`prior2d/scripts/old/make_videos.py`)
implementation of the same idea. cv2-only drawing (no matplotlib); video
I/O via `pvio`, GPU (NVENC) encoded.

Usage:
    python tools/spotlight_ik/make_videos.py \\
        --input-path trial_ikfk.h5 --output-dir qa_videos/ --with-ik
"""

from pathlib import Path

import cv2
import numpy as np
import pvio
import tyro
from loguru import logger

from spotlight_tools.spotlight_ik.io_utils import Period, load_ikfk_h5
from spotlight_tools.spotlight_pose2d.io_utils import load_skeleton_json
from spotlight_tools.spotlight_pose2d.viz import (
    build_edge_colors,
    build_node_colors,
    draw_pose,
)

WHITE = (255, 255, 255)
RAW_LINE_THICKNESS = 1
RAW_POINT_RADIUS = 3
IK_LINE_THICKNESS = 2
IK_POINT_RADIUS = 5
POINT_RADIUS_3D = 3

# mm from thorax the 3D panel should comfortably fit, same for every video
# (a fixed camera distance -- "equidistant from the fly's center" -- rather
# than a per-period adaptive scale) so relative fly/pose size is directly
# comparable across periods and trials.
MM_RADIUS_TO_FIT = 2.8
SCALE_MARGIN = 0.9  # fraction of half-panel-size that MM_RADIUS_TO_FIT maps to

# The free-floating root's Z never moves away from its neutral 0 (the
# orthographic-XY IK fit gives it no Z constraint), so a fixed world Z
# offset below it is as good a ground-plane approximation as any per-frame
# contact estimate.
FLOOR_Z_OFFSET_MM = -2.0
GRID_SPACING_MM = 1.0
# The grid is world-XY-aligned but the camera's right/up axes are
# fly-heading-dependent (see `compute_camera_orientation`), and the oblique
# (pitched) view foreshortens a horizontal plane's extent along the screen's
# "up" direction much more than along "right" -- so a grid sized to just the
# panel's own visible radius would often run out of the panel along one axis
# well before the other, depending on heading. A large fixed multiple of the
# panel's visible half-extent keeps the grid comfortably covering the panel
# (and visibly beyond it) at any heading.
FLOOR_HALF_SIZE_MM = 3.5 * MM_RADIUS_TO_FIT / SCALE_MARGIN
GRID_COLOR = (45, 45, 45)  # dark gray; no fill, only grid lines are drawn
GRID_LINE_THICKNESS = 1

# The real SLEAP skeleton wires the thorax ("Th") hub to the midlegs' ThC
# (leg-base) nodes only (not front/hind), but that edge would misrepresent
# the IK reconstruction here: "Th" and the body plan's thorax origin are
# different physical points (see `spotlight_ik.neuromechfly`'s module
# docstring), so drawing a line between them and a midleg ThC suggests a
# rigid connection the fit itself never assumes.
EXCLUDED_EDGE_NAME_PAIRS = {frozenset({"Th", "LM_ThC"}), frozenset({"Th", "RM_ThC"})}


def select_periods(periods: list[Period], n: int) -> list[Period]:
    """The `n` longest periods, in chronological order (or all, if fewer than
    `n` exist). Longest-first is a deterministic, reproducible choice that
    also tends to pick the most informative footage for a QA spot check.
    """
    longest = sorted(periods, key=lambda p: p.end_idx - p.start_idx, reverse=True)
    return sorted(longest[:n], key=lambda p: p.start_idx)


def compute_camera_orientation(
    fk_3d_mm_frame: np.ndarray, thc_idxs: tuple[int, int, int, int]
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Compute the orthographic camera's orientation for one frame.

    The camera is a "45-deg right-front, -30-deg pitch" oblique view relative
    to the fly's heading, derived from the front-leg vs. hind-leg ThC
    (leg-base) centroid, since the non-leg nodes (N, A, LA, RA, LW, RW) have
    no `fk_3d_mm` value to use instead. Recomputed every frame from that
    frame's own heading (not just the period's first frame), so the camera's
    azimuth tracks the fly's current yaw: the fly stays visually fixed on
    screen while the ground-plane grid (world-XY-fixed) appears to rotate
    underneath it as it turns. Pitch (elevation) is always a fixed 30
    degrees, so only yaw is tracked -- this method only ever reads the
    heading's XY projection, so it never captures the fly's own pitch/roll
    in the first place.

    Args:
        fk_3d_mm_frame: `(n_nodes, 3)` fk_3d_mm for one frame.
        thc_idxs: `(lf_idx, rf_idx, lh_idx, rh_idx)` indices of the LF/RF/LH/RH
            ThC (leg-base) nodes, into `fk_3d_mm_frame`'s node axis.

    Returns:
        right_axis: `(3,)` unit vector spanning the image plane's x axis.
        up_axis: `(3,)` unit vector spanning the image plane's y axis.
        view_dir: `(3,)` unit vector pointing from the fly toward the
            camera; used only for depth-sorting, not projection, since this
            is an orthographic camera.
    """
    lf_idx, rf_idx, lh_idx, rh_idx = thc_idxs
    front = (fk_3d_mm_frame[lf_idx] + fk_3d_mm_frame[rf_idx]) / 2
    hind = (fk_3d_mm_frame[lh_idx] + fk_3d_mm_frame[rh_idx]) / 2
    forward_xy = (front - hind)[:2]
    forward_xy = forward_xy / np.linalg.norm(forward_xy)
    forward = np.array([forward_xy[0], forward_xy[1], 0.0])

    z_hat = np.array([0.0, 0.0, 1.0])
    # The fly's "right" is `forward` rotated -90 deg about Z (viewed from
    # above, i.e. (x, y) -> (y, -x)); an arbitrary but fixed handedness
    # choice for this synthetic view, since there's no ground-truth answer.
    right_body = np.array([forward[1], -forward[0], 0.0])

    azimuth_dir = forward + right_body
    azimuth_dir = azimuth_dir / np.linalg.norm(azimuth_dir)  # 45-deg right-front

    pitch = np.radians(30.0)
    # Camera elevated, looking down (-30-deg pitch).
    view_dir = np.cos(pitch) * azimuth_dir + np.sin(pitch) * z_hat
    view_dir = view_dir / np.linalg.norm(view_dir)

    right_axis = np.cross(view_dir, z_hat)
    right_axis = right_axis / np.linalg.norm(right_axis)
    up_axis = np.cross(right_axis, view_dir)  # already unit: inputs unit + orthogonal

    return right_axis, up_axis, view_dir


def project_relative_to_panel(
    centered: np.ndarray,
    right_axis: np.ndarray,
    up_axis: np.ndarray,
    panel_size: int,
) -> np.ndarray:
    """Orthographically project points already relative to the camera origin.

    Args:
        centered: `(..., 3)`, already offset by whatever origin the camera is
            centered on this frame (see `compute_camera_orientation`), may
            contain NaN.
        right_axis: See `compute_camera_orientation`.
        up_axis: See `compute_camera_orientation`.
        panel_size: Panel width/height in pixels (square panel);
            `MM_RADIUS_TO_FIT` mm from the origin maps to `SCALE_MARGIN` of
            the panel's half-size, the same for every call regardless of
            period or frame.

    Returns:
        `(..., 2)` pixel (x, y) coordinates, NaN preserved where the input
        was NaN.
    """
    screen_x = centered @ right_axis
    screen_y = centered @ up_axis
    scale = SCALE_MARGIN * (panel_size / 2) / MM_RADIUS_TO_FIT
    px_x = panel_size / 2 + screen_x * scale
    px_y = panel_size / 2 - screen_y * scale  # flip: image row grows downward
    return np.stack([px_x, px_y], axis=-1)


def compute_grid_lines(
    right_axis: np.ndarray, up_axis: np.ndarray, panel_size: int
) -> np.ndarray:
    """Project the ground-plane grid's line segments into panel pixels.

    The grid is centered under whatever origin the camera follows each frame
    (the current thorax position), at a fixed world Z offset below it (see
    `FLOOR_Z_OFFSET_MM`), spanning `FLOOR_HALF_SIZE_MM` along world X/Y with
    lines every `GRID_SPACING_MM`. Since the camera's orientation tracks the
    fly's yaw every frame (see `compute_camera_orientation`), this must be
    recomputed every frame too, not just once per period.

    Args:
        right_axis: See `compute_camera_orientation`.
        up_axis: See `compute_camera_orientation`.
        panel_size: Panel width/height in pixels (square panel).

    Returns:
        `(n_lines, 2, 2)` int32 pixel coordinates: one `(start, end)` pair of
        `(x, y)` points per line segment.
    """
    h = FLOOR_HALF_SIZE_MM
    offsets = np.arange(-h, h + GRID_SPACING_MM / 2, GRID_SPACING_MM)
    segments_world = [
        seg
        for offset in offsets
        for seg in (
            [[-h, offset, FLOOR_Z_OFFSET_MM], [h, offset, FLOOR_Z_OFFSET_MM]],
            [[offset, -h, FLOOR_Z_OFFSET_MM], [offset, h, FLOOR_Z_OFFSET_MM]],
        )
    ]
    screen = project_relative_to_panel(
        np.array(segments_world), right_axis, up_axis, panel_size
    )
    return np.round(screen).astype(np.int32)


def draw_grid_floor(canvas: np.ndarray, grid_lines: np.ndarray) -> None:
    """Draw the ground-plane grid lines onto canvas, in place (no fill)."""
    for (x1, y1), (x2, y2) in grid_lines:
        cv2.line(
            canvas, (x1, y1), (x2, y2), GRID_COLOR, GRID_LINE_THICKNESS, cv2.LINE_AA
        )


def draw_fk_3d_panel(
    canvas: np.ndarray,
    points: np.ndarray,
    depths: np.ndarray,
    edges: list[tuple[int, int]],
    edge_colors: list[tuple[int, int, int]],
    point_colors: list[tuple[int, int, int]],
) -> None:
    """Draw the IK 3D panel's skeleton onto canvas, in place, depth-sorted.

    Unlike `spotlight_pose2d.viz.draw_pose` (which always draws all edges,
    then all points), edges and points are interleaved and drawn in
    ascending depth-away-from-camera order (farthest first), so nearer parts
    of the fly draw on top of farther ones where legs cross in this oblique
    view.

    Args:
        canvas: Panel image to draw onto.
        points: `(n_nodes, 2)` pixel coordinates (see
            `project_relative_to_panel`), may contain NaN.
        depths: `(n_nodes,)` depth-away-from-camera per node (see
            `compute_camera_orientation`'s `view_dir`), most negative farthest.
        edges: `(idx_a, idx_b)` pairs.
        edge_colors: One color per edge.
        point_colors: One color per node.
    """
    primitives = [
        (min(depths[a], depths[b]), "edge", (a, b, color))
        for (a, b), color in zip(edges, edge_colors, strict=True)
    ]
    primitives += [(depths[idx], "point", idx) for idx in range(len(points))]
    primitives.sort(key=lambda primitive: primitive[0])

    for _, kind, payload in primitives:
        if kind == "edge":
            a, b, color = payload
            pa, pb = points[a], points[b]
            if np.any(np.isnan(pa)) or np.any(np.isnan(pb)):
                continue
            cv2.line(
                canvas,
                (round(pa[0]), round(pa[1])),
                (round(pb[0]), round(pb[1])),
                color,
                IK_LINE_THICKNESS,
                cv2.LINE_AA,
            )
        else:
            pt = points[payload]
            if np.any(np.isnan(pt)):
                continue
            cv2.circle(
                canvas,
                (round(pt[0]), round(pt[1])),
                POINT_RADIUS_3D,
                point_colors[payload],
                -1,
                cv2.LINE_AA,
            )


def render_period_clip(
    period: Period,
    video_path: Path,
    edges: list[tuple[int, int]],
    edge_colors: list[tuple[int, int, int]],
    point_colors: list[tuple[int, int, int]],
    th_idx: int,
    thc_idxs: tuple[int, int, int, int],
    output_path: Path,
    video_height: int,
    with_ik: bool,
    crf: int,
) -> None:
    """Render one period's clip to `output_path`. See module docstring."""
    frame_indices = list(range(period.start_idx, period.end_idx))
    frames, fps = pvio.read_frames_from_video(video_path, frame_indices)
    fps = fps or 30.0

    left_width = round(frames[0].shape[1] * video_height / frames[0].shape[0])
    n_nodes = len(point_colors)
    white_edge_colors = [WHITE] * len(edges)
    white_point_colors = [WHITE] * n_nodes

    for i, frame in enumerate(frames):
        raw_points = period.pred_2d_px[i].copy()
        if with_ik:
            # Omitted from the raw layer: the IK layer already draws it, and
            # overlapping the two dots at (near-)identical positions read as
            # one oversized dot.
            raw_points[th_idx] = np.nan
        draw_pose(
            frame,
            raw_points,
            edges,
            white_edge_colors,
            white_point_colors,
            RAW_LINE_THICKNESS,
            RAW_POINT_RADIUS,
        )
        if with_ik:
            draw_pose(
                frame,
                period.fk_2d_px[i],
                edges,
                edge_colors,
                point_colors,
                IK_LINE_THICKNESS,
                IK_POINT_RADIUS,
            )
        frames[i] = cv2.resize(
            frame, (left_width, video_height), interpolation=cv2.INTER_AREA
        )

    if with_ik:
        for i in range(len(frames)):
            fk_3d_mm_frame = period.fk_3d_mm[i]
            # Orientation tracks yaw every frame; grid must be recomputed
            # alongside it (see `compute_camera_orientation`/
            # `compute_grid_lines`).
            right_axis, up_axis, view_dir = compute_camera_orientation(
                fk_3d_mm_frame, thc_idxs
            )
            grid_lines = compute_grid_lines(right_axis, up_axis, video_height)

            panel_3d = np.zeros((video_height, video_height, 3), dtype=np.uint8)
            draw_grid_floor(panel_3d, grid_lines)
            origin = fk_3d_mm_frame[th_idx]  # follow: recenter every frame
            centered = fk_3d_mm_frame - origin
            points_2d = project_relative_to_panel(
                centered, right_axis, up_axis, video_height
            )
            depths = centered @ view_dir
            draw_fk_3d_panel(
                panel_3d, points_2d, depths, edges, edge_colors, point_colors
            )
            frames[i] = np.hstack([frames[i], panel_3d])

    pvio.write_frames_to_video(output_path, frames, fps, mode="gpu", quality=crf)
    logger.info(f"Saved {output_path}")


def main(
    input_path: Path,
    output_dir: Path,
    skeleton_json_path: Path = Path(
        "bulk_data/motion_prior/2dpose_model/labels/metadata.json"
    ),
    with_ik: bool = False,
    videos_per_trial: int = 1,
    video_height: int = 448,
    crf: int = 23,
    override: bool = False,
) -> None:
    """Render QA video clips for one trial's `solve_ik.py` output. See module
    docstring.

    Args:
        input_path: `solve_ik.py` output `.h5` (see `spotlight_ik.io_utils.
            save_ikfk_h5`).
        output_dir: Directory to save clips into (one `.mp4` per sampled
            period, named `<input_path stem>_period<id>.mp4`); created if
            missing.
        skeleton_json_path: Real skeleton (nodes, edges), from
            `extract_metadata_from_initial_slp.py`. Its node names must
            match `input_path`'s own `node_names` exactly, order included.
        with_ik: If True, also draws `fk_2d_px` (per-leg colors) on top of
            the raw (white) skeleton, and adds a second panel with a
            synthetic 3D view of `fk_3d_mm`. Off by default: a plain QA
            look at the raw predictions within good periods.
        videos_per_trial: Number of periods to render clips for (the
            longest periods, deterministically; see `select_periods`), or
            all of them if fewer exist.
        video_height: Output panel height in pixels; the left (video) panel
            is resized to this height (aspect ratio preserved), and, with
            `with_ik`, the right (3D) panel is a `video_height` square next
            to it. A multiple of 16 avoids the encoder silently
            padding/resizing the output.
        crf: H.264 quality, 0-51 (lower is higher quality, larger files);
            passed to `pvio.write_frames_to_video` as `quality`.
        override: If True, overwrite an existing clip instead of skipping it.
    """
    data = load_ikfk_h5(input_path)
    node_names = data["node_names"]

    skeleton = load_skeleton_json(skeleton_json_path)
    skeleton_node_names = [node.name for node in skeleton.nodes]
    if skeleton_node_names != node_names:
        raise SystemExit(
            f"{skeleton_json_path}'s skeleton nodes {skeleton_node_names} don't "
            f"match {input_path}'s {node_names}."
        )
    name_to_idx = {name: i for i, name in enumerate(node_names)}
    edges = [
        (name_to_idx[edge.source.name], name_to_idx[edge.destination.name])
        for edge in skeleton.edges
        if frozenset({edge.source.name, edge.destination.name})
        not in EXCLUDED_EDGE_NAME_PAIRS
    ]
    point_colors = build_node_colors(node_names)
    edge_colors = build_edge_colors(edges, node_names)
    th_idx = name_to_idx["Th"]
    thc_idxs = (
        name_to_idx["LF_ThC"],
        name_to_idx["RF_ThC"],
        name_to_idx["LH_ThC"],
        name_to_idx["RH_ThC"],
    )

    periods = select_periods(data["periods"], videos_per_trial)
    if not periods:
        logger.warning(f"{input_path} has no periods; nothing to render.")
        return

    output_dir.mkdir(parents=True, exist_ok=True)
    for period in periods:
        output_path = output_dir / f"{input_path.stem}_period{period.start_idx}.mp4"
        if output_path.exists() and not override:
            logger.info(f"{output_path} already exists; skipping (pass --override).")
            continue
        render_period_clip(
            period,
            data["video_path"],
            edges,
            edge_colors,
            point_colors,
            th_idx,
            thc_idxs,
            output_path,
            video_height,
            with_ik,
            crf,
        )


if __name__ == "__main__":
    tyro.cli(main)
