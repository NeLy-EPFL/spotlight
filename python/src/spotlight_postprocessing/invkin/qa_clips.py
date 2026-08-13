"""Renders QA video clips of `invkin.solve_ik`'s dense `inverse_kinematics.h5`
output: one short clip per sampled IK-attempted run (a contiguous non-NaN
stretch of `dof_angles`; see `find_ik_runs`). Raw predictions (`pred_2d_px`)
are drawn in white; with `with_ik=True`, the IK forward-kinematics result
(`fk_2d_px`) is drawn on top with the same per-leg-chain colors as
`visualize_predictions.py` (see `pose2d.viz`), the thorax is omitted from the
raw layer (the IK layer already draws it there), the two SLEAP-skeleton edges
from the thorax to the midleg ThC nodes are never drawn (see
`EXCLUDED_EDGE_NAME_PAIRS`), and a second panel is added: a synthetic 3D view
of the IK reconstruction (`fk_3d_mm`, same colors) plus a ground-plane grid,
using an orthographic camera that recenters on the thorax and tracks the
fly's heading (yaw only) every frame -- the fly stays visually fixed on
screen while the grid rotates underneath it as it turns. Ported from this
project's own prior (`prior2d/scripts/old/make_videos.py`) implementation of
the same idea. cv2-only drawing (no matplotlib); video I/O via `pvio`, GPU
(NVENC) encoded.

The camera/grid helpers here (`find_ik_runs`, `compute_camera_orientation`,
`project_relative_to_panel`, `compute_grid_lines`, `draw_grid_floor`,
`draw_fk_3d_panel`) are also reused by `visualize.py`'s own QA summary
video, for the same synthetic-3D-panel look.
"""

from pathlib import Path

import cv2
import numpy as np
import pvio
from loguru import logger
from scipy import ndimage

from spotlight_postprocessing.invkin.io_utils import load_inverse_kinematics_h5
from spotlight_postprocessing.pose2d.io_utils import load_pose_h5, load_skeleton_json
from spotlight_postprocessing.pose2d.viz import (
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
# different physical points (see `invkin.neuromechfly`'s module
# docstring), so drawing a line between them and a midleg ThC suggests a
# rigid connection the fit itself never assumes.
EXCLUDED_EDGE_NAME_PAIRS = {frozenset({"Th", "LM_ThC"}), frozenset({"Th", "RM_ThC"})}


def find_ik_runs(dof_angles: np.ndarray) -> list[tuple[int, int]]:
    """Contiguous non-NaN stretches of `dof_angles` (`inverse_kinematics.h5`
    has no explicit period concept -- an IK attempt existing at all, frame
    to frame, is the only "this is one contiguous stretch" signal left), as
    `(start, end)` (`end` exclusive).
    """
    attempted = ~np.isnan(dof_angles).any(axis=-1)
    labeled, n_runs = ndimage.label(attempted)
    return [
        (int(idxs[0]), int(idxs[-1]) + 1)
        for run_id in range(1, n_runs + 1)
        for idxs in (np.flatnonzero(labeled == run_id),)
    ]


def select_runs(runs: list[tuple[int, int]], n: int) -> list[tuple[int, int]]:
    """The `n` longest runs, in chronological order (or all, if fewer than
    `n` exist). Longest-first is a deterministic, reproducible choice that
    also tends to pick the most informative footage for a QA spot check.
    """
    longest = sorted(runs, key=lambda r: r[1] - r[0], reverse=True)
    return sorted(longest[:n], key=lambda r: r[0])


def compute_forward_xy(
    fk_3d_mm_frame: np.ndarray, thc_idxs: tuple[int, int, int, int]
) -> np.ndarray:
    """The fly's raw (unsmoothed) heading, as a `(2,)` unit XY vector:
    front-leg vs. hind-leg ThC (leg-base) centroid, since the non-leg nodes
    (N, A, LA, RA, LW, RW) have no `fk_3d_mm` value to use instead. Split
    out of `compute_camera_orientation` so callers that want the camera's
    yaw-tracking smoothed over time (see `visualize.py`) can filter this
    vector across frames first, then feed the smoothed result to
    `camera_axes_from_forward_xy` instead of this frame's own raw heading.

    Args:
        fk_3d_mm_frame: `(n_nodes, 3)` fk_3d_mm for one frame.
        thc_idxs: `(lf_idx, rf_idx, lh_idx, rh_idx)` indices of the LF/RF/LH/RH
            ThC (leg-base) nodes, into `fk_3d_mm_frame`'s node axis.
    """
    lf_idx, rf_idx, lh_idx, rh_idx = thc_idxs
    front = (fk_3d_mm_frame[lf_idx] + fk_3d_mm_frame[rf_idx]) / 2
    hind = (fk_3d_mm_frame[lh_idx] + fk_3d_mm_frame[rh_idx]) / 2
    forward_xy = (front - hind)[:2]
    return forward_xy / np.linalg.norm(forward_xy)


def camera_axes_from_forward_xy(
    forward_xy: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """The orthographic camera's orientation for one frame, given its own
    (possibly time-smoothed) heading -- see `compute_forward_xy`.

    The camera is a "45-deg right-front, -30-deg pitch" oblique view
    relative to `forward_xy`. Pitch (elevation) is always a fixed 30
    degrees, so only yaw (`forward_xy`'s own angle) ever varies.

    Args:
        forward_xy: `(2,)` unit heading vector (see `compute_forward_xy`).

    Returns:
        right_axis: `(3,)` unit vector spanning the image plane's x axis.
        up_axis: `(3,)` unit vector spanning the image plane's y axis.
        view_dir: `(3,)` unit vector pointing from the fly toward the
            camera; used only for depth-sorting, not projection, since this
            is an orthographic camera.
    """
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


def compute_camera_orientation(
    fk_3d_mm_frame: np.ndarray, thc_idxs: tuple[int, int, int, int]
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """`camera_axes_from_forward_xy(compute_forward_xy(...))` -- this
    frame's own raw (unsmoothed) heading, recomputed fresh every frame (not
    just the period's first frame), so the camera's azimuth tracks the
    fly's current yaw: the fly stays visually fixed on screen while the
    ground-plane grid (world-XY-fixed) appears to rotate underneath it as
    it turns. See `compute_forward_xy` to smooth the heading across frames
    before computing camera axes from it instead.
    """
    return camera_axes_from_forward_xy(compute_forward_xy(fk_3d_mm_frame, thc_idxs))


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

    Unlike `pose2d.viz.draw_pose` (which always draws all edges,
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


def render_run_clip(
    start: int,
    end: int,
    *,
    pred_2d_px: np.ndarray,
    fk_2d_px: np.ndarray | None,
    fk_3d_mm: np.ndarray | None,
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
    """Render one run's clip to `output_path`. See module docstring.

    Args:
        start, end: This run's frame range (`end` exclusive) into
            `pred_2d_px`/`fk_2d_px`/`fk_3d_mm`, which cover the whole trial.
        pred_2d_px, fk_2d_px, fk_3d_mm: `pose2d.h5`/`inverse_kinematics.h5`'s
            own dense, whole-trial arrays (`fk_2d_px`/`fk_3d_mm` only needed
            if `with_ik`).
    """
    frame_indices = list(range(start, end))
    frames, fps = pvio.read_frames_from_video(video_path, frame_indices)
    fps = fps or 30.0

    left_width = round(frames[0].shape[1] * video_height / frames[0].shape[0])
    n_nodes = len(point_colors)
    white_edge_colors = [WHITE] * len(edges)
    white_point_colors = [WHITE] * n_nodes

    for i, frame in enumerate(frames):
        raw_points = pred_2d_px[start + i].copy()
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
                fk_2d_px[start + i],
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
            fk_3d_mm_frame = fk_3d_mm[start + i]
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


def render_qa_clips(
    input_path: Path,
    pose2d_path: Path,
    video_path: Path,
    output_dir: Path,
    skeleton_json_path: Path,
    with_ik: bool = False,
    videos_per_trial: int = 1,
    video_height: int = 448,
    crf: int = 23,
    override: bool = False,
) -> None:
    """Render QA video clips for one trial's `invkin.solve_ik` output. See
    module docstring.

    Args:
        input_path: `invkin.solve_ik` output `inverse_kinematics.h5` (see
            `invkin.io_utils.save_inverse_kinematics_h5`). Its own
            dof_angles NaN pattern is what defines a "run" to sample a
            clip from, whether or not `with_ik` below actually draws the
            FK overlay.
        pose2d_path: The `pose2d.h5` `input_path` was fit from (see
            `pose2d.io_utils.save_pose_h5`) -- raw predictions
            (`pred_2d_px`) come from here, not from `input_path`.
        video_path: The aligned behavior video these predictions came
            from (neither h5 file carries this itself).
        output_dir: Directory to save clips into (one `.mp4` per sampled
            run, named `<input_path stem>_run<start>.mp4`); created if
            missing.
        skeleton_json_path: Real skeleton (nodes, edges), from
            `extract_metadata_from_initial_slp.py`. Its node names must
            match `input_path`'s own keypoint order exactly, order included.
        with_ik: If True, also draws `fk_2d_px` (per-leg colors) on top of
            the raw (white) skeleton, and adds a second panel with a
            synthetic 3D view of `fk_3d_mm`. Off by default: a plain QA
            look at the raw predictions within IK-attempted runs.
        videos_per_trial: Number of runs to render clips for (the longest
            runs, deterministically; see `select_runs`), or all of them
            if fewer exist.
        video_height: Output panel height in pixels; the left (video) panel
            is resized to this height (aspect ratio preserved), and, with
            `with_ik`, the right (3D) panel is a `video_height` square next
            to it. A multiple of 16 avoids the encoder silently
            padding/resizing the output.
        crf: H.264 quality, 0-51 (lower is higher quality, larger files);
            passed to `pvio.write_frames_to_video` as `quality`.
        override: If True, overwrite an existing clip instead of skipping it.
    """
    ik = load_inverse_kinematics_h5(input_path)
    pose2d = load_pose_h5(pose2d_path)
    node_names = pose2d["node_names"]

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

    runs = select_runs(find_ik_runs(ik.dof_angles), videos_per_trial)
    if not runs:
        logger.warning(f"{input_path} has no IK-attempted runs; nothing to render.")
        return

    output_dir.mkdir(parents=True, exist_ok=True)
    for start, end in runs:
        output_path = output_dir / f"{input_path.stem}_run{start}.mp4"
        if output_path.exists() and not override:
            logger.info(f"{output_path} already exists; skipping (pass override=True).")
            continue
        render_run_clip(
            start,
            end,
            pred_2d_px=pose2d["poses"],
            fk_2d_px=ik.keypoint_positions_2d_px,
            fk_3d_mm=ik.keypoint_positions_3d_mm,
            video_path=video_path,
            edges=edges,
            edge_colors=edge_colors,
            point_colors=point_colors,
            th_idx=th_idx,
            thc_idxs=thc_idxs,
            output_path=output_path,
            video_height=video_height,
            with_ik=with_ik,
            crf=crf,
        )
