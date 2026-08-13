"""Shared skeleton coloring and cv2 drawing for this project's pose
visualizations. Used by `scripts/postprocessing/model_training/pose2d/visualize_predictions.py`
and `invkin.qa_clips`, so both draw the same skeleton in
the same colors.
"""

import cv2
import numpy as np

OTHER_COLOR = (200, 200, 200)  # nodes/edges with no leg/antenna color below
LINE_THICKNESS = 1
POINT_RADIUS = 2

# This project's skeleton (see extract_metadata_from_initial_slp.py) names
# leg nodes "<LEG>_<JOINT>" (e.g. "LF_ThC") and antennae "LA"/"RA"; these
# colors are given as RGB, and `pvio` reads/writes frames in RGB too (unlike
# OpenCV's usual BGR), so they're used as-is with no channel reordering.
KCHAIN_COLORS = {
    "LF": (15, 115, 153),
    "LM": (26, 141, 175),
    "LH": (117, 190, 203),
    "RF": (186, 30, 49),
    "RM": (201, 86, 79),
    "RH": (213, 133, 121),
    "LA": (50, 120, 32),
    "RA": (50, 120, 32),
}


def color_for_node(name: str) -> tuple[int, int, int] | None:
    """`KCHAIN_COLORS` entry for a leg/antenna node, or None for anything else."""
    prefix = name.split("_")[0]
    return KCHAIN_COLORS.get(prefix)


def build_node_colors(node_names: list[str]) -> list[tuple[int, int, int]]:
    """One color per node: `KCHAIN_COLORS` for leg/antenna nodes, else
    `OTHER_COLOR`.
    """
    return [color_for_node(name) or OTHER_COLOR for name in node_names]


def build_edge_colors(
    edges: list[tuple[int, int]], node_names: list[str]
) -> list[tuple[int, int, int]]:
    """One color per edge: whichever endpoint has a leg/antenna color (e.g.
    the Th-LM_ThC hub edge takes LM's color), else `OTHER_COLOR`.
    """
    colors = []
    for a, b in edges:
        colors.append(
            color_for_node(node_names[a])
            or color_for_node(node_names[b])
            or OTHER_COLOR
        )
    return colors


def draw_pose(
    frame: np.ndarray,
    points: np.ndarray,
    edges: list[tuple[int, int]],
    edge_colors: list[tuple[int, int, int]],
    point_colors: list[tuple[int, int, int]],
    line_thickness: int = LINE_THICKNESS,
    point_radius: int = POINT_RADIUS,
) -> None:
    """Draw skeleton edges and keypoint dots onto frame, in place."""
    for (a, b), color in zip(edges, edge_colors, strict=True):
        pa, pb = points[a], points[b]
        if np.any(np.isnan(pa)) or np.any(np.isnan(pb)):
            continue
        cv2.line(
            frame,
            (round(pa[0]), round(pa[1])),
            (round(pb[0]), round(pb[1])),
            color,
            line_thickness,
            cv2.LINE_AA,
        )
    for idx, pt in enumerate(points):
        if np.any(np.isnan(pt)):
            continue
        cv2.circle(
            frame,
            (round(pt[0]), round(pt[1])),
            point_radius,
            point_colors[idx],
            -1,
            cv2.LINE_AA,
        )
