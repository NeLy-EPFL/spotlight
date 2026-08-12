"""QA visualization for one postprocessed recording, cv2/pvio/numpy only --
matplotlib and cmasher are used only to borrow a font path and a colormap
(as a precomputed lookup table), never for actual plotting/rendering.
Two-row panel grid:

    row 1: raw + orient-box overlay | cropped+aligned (blank if flipped) | muscle (if requested)
    row 2: pose2d skeleton overlay (if requested)          | synthetic 3D IK (if requested)

Row 2 only exists when `with_pose2d`; its column 2 (synthetic 3D IK) is
omitted entirely when `with_ik` is off, the same way the muscle panel is
omitted (not blanked) when `with_muscle` is off. Panel 1 is padded to a
square (matching the other row-1 panels) whenever the recording has an
aligned domain at all (`alignment in ("aligned", "both")`); in
`"fullsize"` mode there is no row 2 (pose2d/IK both require an aligned
domain) and panel 1 keeps its native aspect ratio. Reads back
already-computed outputs (video files, muscle H5, pose2d H5, IK/FK H5) --
no model re-inference here, that already happened in
`behavior.process_behavior_pipeline`.

Rendering is chunked and parallelized (joblib): each worker composites its
own frame range and encodes it directly to its own small temp video via
`pvio` -- no JPEG scratch-file round trip (unlike `StreamingVideoWriter`,
which needs one since a whole trial's frames don't fit in memory at once;
one chunk's worth does). Chunk videos are concatenated at the end via
ffmpeg's stream-copy concat demuxer (no re-encode, since every chunk shares
the same codec params).
"""

import cProfile
import functools
import io
import json
import logging
import pstats
import subprocess
import sys
import tempfile
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import cmasher as cmr
import cv2
import h5py
import numpy as np
import pandas as pd
import pvio
from joblib import Parallel, delayed
from PIL import Image, ImageDraw, ImageFont
from scipy.ndimage import binary_closing, binary_opening, gaussian_filter1d
from spotlight_tools.calibration.mapper import SpotlightPositionMapper
from spotlight_tools.common.video import pad_to_macroblock
from spotlight_tools.spotlight_orient.box import (
    fit_disambiguated_direction,
    raw_domain_box_corners,
)
from spotlight_tools.spotlight_pose2d.viz import LINE_THICKNESS, POINT_RADIUS
from spotlight_tools.spotlight_pose2d.viz import (
    build_edge_colors,
    build_node_colors,
    draw_pose,
)

PANEL_SIZE = 450
FLIP_DECISION_THRESHOLD_DEFAULT = 0.5
# RGB -- frames go straight to `pvio.write_frames_to_video` (no cv2.imwrite
# in between, which is what used to make BGR the right choice here, back
# when a JPEG scratch-write step sat between drawing and pvio).
YELLOW = (255, 255, 0)
WHITE = (255, 255, 255)
TEXT_COLOR = (0xDD, 0xDD, 0xDD)
BORDER_COLOR = (0x33, 0x33, 0x33)
DEFAULT_COLORMAP = "lilac"
COLORMAP_LUT_SIZE = 256
IK_LINE_THICKNESS_SCALE = 2
IK_POINT_RADIUS_SCALE = 2
DEFAULT_CHUNK_SIZE = 500
DEFAULT_NUM_WORKERS = 8
DEFAULT_COMPOSITE_WORKERS = 4
"""Threads per chunk *process* used to composite (not encode) frames.
Compositing is CPU-bound and dominated by cv2/numpy calls that release the
GIL, so it benefits from more parallelism than `DEFAULT_NUM_WORKERS`
(which stays low to respect the GPU's concurrent-NVENC-session limit) --
this lets each of those processes composite several frames at once while
encoding stays exactly as GPU-session-limited as before."""

SCALE_BAR_UM = 500
LABEL_FONT_SIZE = 13
TITLE_FONT_SIZE = 16
OVERLAY_MARGIN = 12

PANEL_NAMES = {
    "raw": "Behavior recording (full-size)",
    "aligned": "Behavior recording (cropped)",
    "muscle": "Muscle recording (cropped)",
    "pose2d": "2D pose",
    "ik3d": "IK-reconstructed 3D pose",
}
POSE2D_IK_LEGEND_LINES = ["white: raw predictions", "colored: IK fit"]
RAW_BOX_LEGEND_LINES = [
    "yellow: accepted",
    "white: rejected (not upright, too close to edge)",
]

_IK_SCRIPTS_DIR = Path(__file__).resolve().parents[3] / "scripts" / "spotlight_ik"


def _round_to_multiple(value: int, multiple: int = 16) -> int:
    return max(multiple, int(round(value / multiple)) * multiple)


def _resize_to_height(frame: np.ndarray, height: int) -> np.ndarray:
    h, w = frame.shape[:2]
    new_w = _round_to_multiple(round(w * height / h))
    return cv2.resize(frame, (new_w, height))


def _resize_square(frame: np.ndarray, size: int) -> np.ndarray:
    """Resizes an already-square (e.g. `crop_dim` x `crop_dim`) frame to
    exactly `size` x `size` -- unlike `_resize_to_height`, which rounds its
    output width to a multiple of 16 for codec compatibility, so it must
    NOT be used here: that rounding would make this panel a couple of
    pixels narrower than `PANEL_SIZE`, mismatching every other panel's
    (and the black flip/placeholder panel's) exact `PANEL_SIZE` width and
    crashing `pvio.write_frames_to_video`'s same-dimensions check the
    moment a flipped and an unflipped frame land in the same chunk."""
    return cv2.resize(frame, (size, size))


def _pad_to_width(panel: np.ndarray, target_width: int) -> np.ndarray:
    """Left-justifies `panel` in a black canvas `target_width` wide (same
    height) -- padding goes on the right only, so panels stay flush left
    and a future additional panel can be appended there. No-op if `panel`
    is already that wide or wider."""
    width = panel.shape[1]
    if width >= target_width:
        return panel
    return cv2.copyMakeBorder(
        panel, 0, 0, 0, target_width - width, cv2.BORDER_CONSTANT, value=(0, 0, 0)
    )


@functools.lru_cache(maxsize=None)
def _find_font_path(family: str, weight: str = "normal", style: str = "normal") -> str:
    """The file path of a matching installed font (e.g. "Arial", "bold")."""
    import matplotlib.font_manager as fm

    font_props = fm.FontProperties(family=family, weight=weight, style=style)
    return fm.findfont(font_props)


@functools.lru_cache(maxsize=None)
def _load_font(size: int, bold: bool = False) -> ImageFont.FreeTypeFont:
    path = _find_font_path("Arial", weight="bold" if bold else "normal")
    return ImageFont.truetype(path, size)


def _build_colormap_lut(name: str) -> np.ndarray:
    """`(COLORMAP_LUT_SIZE, 3)` uint8 RGB lookup table for the colormap
    `name` (any cmasher colormap, e.g. "lilac", or any matplotlib-
    registered name, e.g. "viridis" or cmasher's own "cmr.lilac" form).

    Built once per trial and applied per frame via a plain numpy index
    (see its call site) -- the only use of matplotlib/cmasher here is
    borrowing the colormap's RGB values; `Colormap.__call__` itself (with
    its masking/clipping for arbitrary float input) is never run per
    frame, since it turned out to noticeably outweigh the rest of a
    frame's own compositing cost.
    """
    cmap = getattr(cmr, name, None)
    if cmap is None:
        import matplotlib

        cmap = matplotlib.colormaps[name]
    lut = cmap(np.linspace(0, 1, COLORMAP_LUT_SIZE))[:, :3]
    return (lut * 255).astype(np.uint8)


def _resolve_encode_mode() -> str:
    """Whether NVENC is available, resolved once in the main process
    instead of "auto" (which makes every chunk's own `pvio.write_frames_
    to_video` call re-probe CUDA availability). `pvio` already caches that
    probe per-process via `functools.lru_cache`, so this only saves the
    first probe in each of `num_workers` worker processes (not one per
    chunk) -- a modest but free win, and it also avoids `pvio` logging a
    "mode='gpu' requested but unavailable" warning on a machine with no
    GPU, which passing "gpu" unconditionally would risk."""
    try:
        import torch

        return "gpu" if torch.cuda.is_available() else "cpu"
    except Exception:
        return "cpu"


def _compute_um_per_raw_pixel(calibration_path: Path) -> float:
    """Local physical scale (micrometers per raw-domain pixel), from the
    behavior camera's own stage/pixel/physical calibration -- averaged
    over the x and y pixel directions (empirically near-identical, as
    expected for a camera with square, non-skewed pixels)."""
    mapper = SpotlightPositionMapper(calibration_path)
    origin = np.array([[0.0, 0.0]])
    step = 100.0
    dx = mapper.stage_and_pixel_to_physical(np.zeros((1, 2)), np.array([[step, 0.0]]))
    dy = mapper.stage_and_pixel_to_physical(np.zeros((1, 2)), np.array([[0.0, step]]))
    origin_phys = mapper.stage_and_pixel_to_physical(np.zeros((1, 2)), origin)
    mm_per_px_x = np.linalg.norm(dx[0] - origin_phys[0]) / step
    mm_per_px_y = np.linalg.norm(dy[0] - origin_phys[0]) / step
    return float(np.mean([mm_per_px_x, mm_per_px_y])) * 1000.0


def _make_panel_overlay(
    width: int,
    height: int,
    panel_name: str,
    scale_bar_px: float | None,
    scale_bar_label: str | None,
    corner_label: str | None,
    extra_title_lines: list[str] | None = None,
) -> np.ndarray:
    """A `(height, width)` float32 array in `[0, 1]`: 1 where
    text/scale-bar pixels should be blended in, 0 (or anti-aliased
    in-between values) elsewhere. Computed once (per distinct panel
    size/text combination in the trial, not per frame) and reused via
    alpha blending, since PIL text rendering isn't cheap enough to redo
    for every one of a trial's frames.
    """
    image = Image.new("L", (width, height), 0)
    draw = ImageDraw.Draw(image)
    draw.text(
        (OVERLAY_MARGIN, OVERLAY_MARGIN), panel_name, fill=255,
        font=_load_font(TITLE_FONT_SIZE, bold=True),
    )  # fmt: skip

    line_y = OVERLAY_MARGIN + TITLE_FONT_SIZE + 4
    line_font = _load_font(LABEL_FONT_SIZE)
    for line in extra_title_lines or []:
        draw.text((OVERLAY_MARGIN, line_y), line, fill=255, font=line_font)
        line_y += LABEL_FONT_SIZE + 4

    # Scale bar and its label sit on one line, vertically centered on each
    # other (`anchor="lm"`: the given point is the text's left-middle),
    # rather than the label sitting below the bar.
    bar_y = height - OVERLAY_MARGIN - LABEL_FONT_SIZE // 2
    if scale_bar_px is not None:
        bar_x0 = OVERLAY_MARGIN
        bar_x1 = bar_x0 + scale_bar_px
        draw.line([(bar_x0, bar_y), (bar_x1, bar_y)], fill=255, width=3)
        draw.text(
            (bar_x1 + 8, bar_y), scale_bar_label, fill=255,
            font=_load_font(LABEL_FONT_SIZE), anchor="lm",
        )  # fmt: skip
    elif corner_label is not None:
        draw.text(
            (OVERLAY_MARGIN, bar_y), corner_label, fill=255,
            font=_load_font(LABEL_FONT_SIZE), anchor="lm",
        )  # fmt: skip

    return np.asarray(image, dtype=np.float32) / 255.0


def _make_speed_overlay(width: int, height: int, text: str) -> np.ndarray:
    """Like `_make_panel_overlay`, but a single line right-aligned to the
    top-right corner of the whole (multi-panel) frame -- the playback-speed
    indicator isn't tied to any one panel. Same font (size and weight) as
    the panel titles, at the same `y`, so it reads as sitting on the same
    line as row 1's panel titles."""
    image = Image.new("L", (width, height), 0)
    draw = ImageDraw.Draw(image)
    draw.text(
        (width - OVERLAY_MARGIN, OVERLAY_MARGIN), text, fill=255,
        font=_load_font(TITLE_FONT_SIZE, bold=True), anchor="ra",
    )  # fmt: skip
    return np.asarray(image, dtype=np.float32) / 255.0


_TEXT_COLOR_ARR = np.array(TEXT_COLOR, dtype=np.float32)

# (row0, col0, cropped_alpha, full_shape) -- see `_crop_overlay_to_content`.
OverlayCrop = tuple[int, int, np.ndarray, tuple[int, int]]


def _crop_overlay_to_content(overlay: np.ndarray) -> OverlayCrop | None:
    """Bounding box of `overlay`'s non-zero region, cropped down to just
    that box. `_apply_overlay` blends only this (typically tiny, e.g. a
    few lines of text in a corner) region instead of running float
    blending math over an entire panel every frame when almost all of it
    is unaffected (alpha 0) -- the overlay is the same on every frame of
    the trial, so this crop is worth computing once, not per frame.

    Returns `None` if `overlay` is entirely zero (nothing to draw).
    """
    rows = np.flatnonzero(overlay.any(axis=1))
    cols = np.flatnonzero(overlay.any(axis=0))
    if len(rows) == 0 or len(cols) == 0:
        return None
    r0, r1 = int(rows[0]), int(rows[-1]) + 1
    c0, c1 = int(cols[0]), int(cols[-1]) + 1
    return r0, c0, overlay[r0:r1, c0:c1], overlay.shape


def _apply_overlay(panel: np.ndarray, overlay_crop: OverlayCrop | None) -> np.ndarray:
    """Alpha-blends `overlay_crop` (see `_crop_overlay_to_content`), in
    `TEXT_COLOR`, onto a copy of `panel`, if `overlay_crop` isn't `None`
    and its original (pre-crop) size matches `panel`'s (a size mismatch
    means this panel's actual rendered size didn't match what the overlay
    was precomputed for, e.g. a flipped-black placeholder that's still
    drawn -- skip rather than crash on a QA-only detail).

    Always returns a copy (never mutates `panel` in place): some callers
    pass a placeholder array shared across many frames/panels, which an
    in-place blend would silently corrupt for all of them.
    """
    if overlay_crop is None:
        return panel
    row0, col0, alpha, full_shape = overlay_crop
    if full_shape != panel.shape[:2]:
        return panel
    result = panel.copy()
    h, w = alpha.shape
    region = result[row0 : row0 + h, col0 : col0 + w].astype(np.float32)
    alpha = alpha[..., None]
    result[row0 : row0 + h, col0 : col0 + w] = (
        region * (1 - alpha) + _TEXT_COLOR_ARR * alpha
    ).astype(np.uint8)
    return result


def _format_speed(playback_speed: float) -> str:
    """`0.1` -> "0.1x"; `0.10101...` (a `limit_denominator` approximation
    that landed slightly off the requested value) -> "0.101x"."""
    return f"{playback_speed:.4f}".rstrip("0").rstrip(".") + "x"


def _compute_border_lines(
    row1_widths: list[int], row2_widths: list[int]
) -> list[tuple[tuple[int, int], tuple[int, int]]]:
    """Line segments, in the final composited frame's own coordinates, at
    every seam between adjacent panels (and between row 1 and row 2, if
    row 2 exists) -- computed once per trial from panel slot widths, which
    are fixed for the whole trial, not per frame."""
    lines = []
    row1_height = PANEL_SIZE
    total_width = max(sum(row1_widths), sum(row2_widths) if row2_widths else 0)
    total_height = PANEL_SIZE * (2 if row2_widths else 1)

    x = 0
    for w in row1_widths[:-1]:
        x += w
        lines.append(((x, 0), (x, row1_height)))

    if row2_widths:
        x = 0
        for w in row2_widths[:-1]:
            x += w
            lines.append(((x, row1_height), (x, total_height)))
        lines.append(((0, row1_height), (total_width, row1_height)))

    return lines


def _precompute_panel_overlays(
    *,
    recording_dir: Path,
    raw_paths: list[Path],
    show_aligned: bool,
    with_muscle: bool,
    with_pose2d: bool,
    with_row2: bool,
    with_ik: bool,
    crop_dim: int | None,
    playback_speed: float,
) -> tuple[dict, list[tuple[tuple[int, int], tuple[int, int]]]]:
    """One `_make_panel_overlay` per active panel slot, keyed "raw",
    "aligned", "muscle", "pose2d", "ik3d", plus "speed" (the whole-frame
    playback-speed indicator) -- each sized to that panel's own actual
    final rendered dimensions, computed once per trial (not per frame or
    per chunk) and reused by every `_render_chunk` worker. Also returns
    the panel-border line segments (see `_compute_border_lines`), which
    depend on the same per-panel widths.

    Scale bars need this trial's physical pixel scale, from the behavior
    camera's own calibration file; if that file is missing, panels still
    get their name label, just no scale bar.
    """
    calibration_path = (
        recording_dir / "metadata" / "calibration_parameters_behavior.yaml"
    )
    um_per_raw_px = (
        _compute_um_per_raw_pixel(calibration_path) if calibration_path.exists() else None
    )  # fmt: skip

    sample_raw = cv2.imread(str(raw_paths[0]))
    native_h, native_w = sample_raw.shape[:2]
    raw_panel = _resize_to_height(sample_raw, PANEL_SIZE)
    if show_aligned:
        raw_panel = _pad_to_width(raw_panel, PANEL_SIZE)
    raw_h, raw_w = raw_panel.shape[:2]

    def scale_bar_px(native_size: int) -> float | None:
        if um_per_raw_px is None:
            return None
        return SCALE_BAR_UM * (PANEL_SIZE / native_size) / um_per_raw_px

    raw_scale_bar_px = scale_bar_px(native_h)
    crop_scale_bar_px = scale_bar_px(crop_dim) if crop_dim else None
    scale_bar_label = f"{SCALE_BAR_UM} um"

    overlays = {
        "raw": _make_panel_overlay(
            raw_w,
            raw_h,
            PANEL_NAMES["raw"],
            raw_scale_bar_px,
            scale_bar_label,
            None,
            extra_title_lines=RAW_BOX_LEGEND_LINES if show_aligned else None,
        )  # fmt: skip
    }
    row1_widths = [raw_w]
    if show_aligned:
        overlays["aligned"] = _make_panel_overlay(
            PANEL_SIZE, PANEL_SIZE, PANEL_NAMES["aligned"],
            crop_scale_bar_px, scale_bar_label, None,
        )  # fmt: skip
        row1_widths.append(PANEL_SIZE)
    if with_muscle:
        overlays["muscle"] = _make_panel_overlay(
            PANEL_SIZE, PANEL_SIZE, PANEL_NAMES["muscle"],
            crop_scale_bar_px, scale_bar_label, None,
        )  # fmt: skip
        row1_widths.append(PANEL_SIZE)

    row2_widths = []
    if with_pose2d:
        overlays["pose2d"] = _make_panel_overlay(
            PANEL_SIZE, PANEL_SIZE, PANEL_NAMES["pose2d"],
            crop_scale_bar_px, scale_bar_label, None,
            extra_title_lines=POSE2D_IK_LEGEND_LINES if with_ik else None,
        )  # fmt: skip
        row2_widths.append(PANEL_SIZE)
    if with_row2 and with_ik:
        sys.path.insert(0, str(_IK_SCRIPTS_DIR))
        from make_videos import GRID_SPACING_MM

        grid_label = f"grid: {GRID_SPACING_MM:g} mm"
        overlays["ik3d"] = _make_panel_overlay(
            PANEL_SIZE, PANEL_SIZE, PANEL_NAMES["ik3d"], None, None, grid_label
        )
        row2_widths.append(PANEL_SIZE)

    total_width = max(sum(row1_widths), sum(row2_widths) if row2_widths else 0)
    total_height = PANEL_SIZE * (2 if row2_widths else 1)
    overlays["speed"] = _make_speed_overlay(
        total_width, total_height, f"speed: {_format_speed(playback_speed)}"
    )

    # Crop every overlay down to its own drawn-on bounding box (see
    # `_crop_overlay_to_content`) once here, rather than in `_render_chunk`
    # blending full-panel-sized float math every frame.
    overlays = {key: _crop_overlay_to_content(mask) for key, mask in overlays.items()}

    border_lines = _compute_border_lines(row1_widths, row2_widths)
    return overlays, border_lines


def _compute_smoothed_flip_mask(
    flipped_prob: np.ndarray, threshold: float, mask_window: int
) -> np.ndarray:
    """Thresholds `flipped_prob` at `threshold`, then denoises the
    resulting boolean time series with a binary opening (drops isolated
    flipped spikes) followed by a closing (fills isolated not-flipped gaps
    inside a longer flipped stretch) -- both using a `mask_window`-sized 1D
    structuring element. `mask_window == -1` skips this denoising step
    entirely (the raw per-frame threshold is used as-is)."""
    is_flipped = flipped_prob >= threshold
    if mask_window == -1:
        return is_flipped
    structure = np.ones(mask_window, dtype=bool)
    is_flipped = binary_opening(is_flipped, structure=structure)
    is_flipped = binary_closing(is_flipped, structure=structure)
    return is_flipped


def _compute_smoothed_directions(keypoints_pre: np.ndarray, sigma: float) -> np.ndarray:
    """Per-frame fitted neck/thorax/abdomen direction (see
    `box.fit_disambiguated_direction`), optionally smoothed over time.

    Smooths the unit direction VECTOR's x/y components (not the raw
    angle), which sidesteps the wraparound a naive angle average would hit
    near +/-180 degrees. NaN frames (no orient-model prediction) are
    filled from their nearest valid neighbor before smoothing, matching
    the forward/backward-fill already used elsewhere for keypoint gaps.

    Args:
        keypoints_pre: `(n_frames, 3, 2)` raw-domain neck/thorax/abdomen.
        sigma: Gaussian smoothing sigma, in frames. `-1` returns the raw
            per-frame fit unsmoothed (still disambiguated).

    Returns:
        `(n_frames, 2)` unit direction vectors, or an all-NaN row for any
        frame that had no valid neighbor to fill from at all.
    """
    n_frames = len(keypoints_pre)
    directions = np.full((n_frames, 2), np.nan, dtype=np.float64)
    for i in range(n_frames):
        points = keypoints_pre[i]
        if np.isnan(points).any():
            continue
        directions[i] = fit_disambiguated_direction(points)

    valid = ~np.isnan(directions).any(axis=1)
    if not valid.any():
        return directions
    valid_idxs = np.flatnonzero(valid)
    for axis in range(2):
        directions[:, axis] = np.interp(
            np.arange(n_frames), valid_idxs, directions[valid_idxs, axis]
        )

    if sigma != -1:
        for axis in range(2):
            directions[:, axis] = gaussian_filter1d(directions[:, axis], sigma=sigma)
        norms = np.linalg.norm(directions, axis=1, keepdims=True)
        directions = directions / np.clip(norms, 1e-8, None)

    directions[~valid] = np.nan
    return directions


def _render_chunk(
    *,
    chunk_start: int,
    chunk_end: int,
    tmpdir: str,
    raw_paths: list[Path],
    show_aligned: bool,
    crop_dim: int | None,
    transform_matrices: np.ndarray | None,
    keypoints_pre: np.ndarray | None,
    smoothed_directions: np.ndarray | None,
    canonical_points: np.ndarray | None,
    is_flipped: np.ndarray | None,
    panel_overlays: dict,
    border_lines: list[tuple[tuple[int, int], tuple[int, int]]],
    with_muscle: bool,
    muscle_h5_path: Path | None,
    muscle_dataset_name: str | None,
    muscle_meta: pd.DataFrame | None,
    muscle_vrange: tuple[int, int] | None,
    muscle_lut: np.ndarray | None,
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
    mode: str,
    composite_workers: int,
) -> str:
    """Composites and encodes one contiguous frame range, entirely within
    this worker process. Returns the path to this chunk's own temp video.

    Compositing (this function's own CPU/numpy/cv2 work) runs across
    `composite_workers` threads -- cv2's calls release the GIL, so this is
    real parallelism, not fighting the GIL -- while encoding (the final
    `pvio` call) stays single-threaded per chunk, since the number of
    chunk *processes* running concurrently is already capped to respect
    the GPU's own concurrent-NVENC-session limit. This lets compositing
    scale independently of that cap instead of being throttled down to it.

    Frames are grouped by which raw JPEG they're packed into (3 per file)
    so each thread decodes its own group's image once and composites all
    of that group's frames from it -- no cache shared across threads
    (which the previous single-cache-variable version would have needed a
    lock for) and no redundant re-decoding.
    """
    # This chunk's own muscle rows, batch-read once up front rather than
    # looked up per frame inside the threaded compositing below: h5py
    # handles aren't safe to share across threads, and one batched fancy-
    # index read is cheaper than one `__getitem__` per frame anyway.
    muscle_row_by_frame = {}
    if with_muscle:
        ids = muscle_meta["corresponding_behavior_frame_id"].to_numpy()
        frame_idxs = np.arange(chunk_start, chunk_end)
        row_pos = np.searchsorted(ids, frame_idxs, side="right") - 1
        needed = sorted({int(p) for p in row_pos if p >= 0})
        if needed:
            with h5py.File(muscle_h5_path, "r") as f:
                fetched = f[muscle_dataset_name][needed]
            slot_by_pos = {p: s for s, p in enumerate(needed)}
            muscle_row_by_frame = {
                int(fi): fetched[slot_by_pos[int(p)]]
                for fi, p in zip(frame_idxs, row_pos)
                if p >= 0
            }

    if with_ik:
        sys.path.insert(0, str(_IK_SCRIPTS_DIR))
        from make_videos import (
            compute_camera_orientation,
            compute_grid_lines,
            draw_fk_3d_panel,
            draw_grid_floor,
            project_relative_to_panel,
        )

    with_row2 = with_pose2d
    row1_black = np.zeros((PANEL_SIZE, PANEL_SIZE, 3), dtype=np.uint8)

    def composite_frame(i: int, raw_frame: np.ndarray) -> np.ndarray:
        flipped_i = is_flipped is not None and is_flipped[i]

        # --- row 1 ---
        row1 = []
        raw_bgr = cv2.cvtColor(raw_frame, cv2.COLOR_GRAY2BGR)
        if show_aligned and canonical_points is not None and keypoints_pre is not None:
            direction_override = (
                smoothed_directions[i] if smoothed_directions is not None else None
            )
            if direction_override is not None and np.isnan(direction_override).any():
                direction_override = None
            corners = raw_domain_box_corners(
                keypoints_pre[i], canonical_points, pred_direction_override=direction_override
            )  # fmt: skip
            if corners is not None:
                color = WHITE if flipped_i else YELLOW
                cv2.polylines(
                    raw_bgr, [corners.astype(np.int32)], True, color, 4, cv2.LINE_AA
                )
        raw_panel = _resize_to_height(raw_bgr, PANEL_SIZE)
        if show_aligned:
            raw_panel = _pad_to_width(raw_panel, PANEL_SIZE)
        row1.append(_apply_overlay(raw_panel, panel_overlays.get("raw")))

        aligned_frame = None
        if show_aligned:
            # Re-derive the aligned crop directly from the raw frame using
            # this exact frame's own saved transform, rather than decoding
            # `aligned_behavior_video.mp4` back out -- identical result
            # (same transform, same source pixels), no video decode at all.
            aligned_gray = cv2.warpAffine(
                raw_frame, transform_matrices[i], (crop_dim, crop_dim),
                flags=cv2.INTER_NEAREST, borderMode=cv2.BORDER_CONSTANT,
                borderValue=0,
            )  # fmt: skip
            aligned_frame = cv2.cvtColor(aligned_gray, cv2.COLOR_GRAY2BGR)
            display_aligned_frame = (
                row1_black if flipped_i else _resize_square(aligned_frame, PANEL_SIZE)
            )
            row1.append(
                _apply_overlay(display_aligned_frame, panel_overlays.get("aligned"))
            )

        if with_muscle:
            muscle_img = None if flipped_i else muscle_row_by_frame.get(i)
            if muscle_img is not None:
                norm = np.clip(
                    (muscle_img.astype(np.float32) - muscle_vrange[0])
                    / max(1, muscle_vrange[1] - muscle_vrange[0]),
                    0, 1,
                )  # fmt: skip
                lut_idx = (norm * (COLORMAP_LUT_SIZE - 1)).astype(np.uint8)
                muscle_bgr = muscle_lut[lut_idx]
                muscle_bgr = _resize_square(muscle_bgr, PANEL_SIZE)
            else:
                muscle_bgr = row1_black
            row1.append(_apply_overlay(muscle_bgr, panel_overlays.get("muscle")))

        # --- row 2 ---
        row2 = []
        period_info = ik_periods_by_frame.get(i)
        if with_pose2d:
            if flipped_i:
                pose_panel = row1_black.copy()
            else:
                # Resize the background to final display resolution
                # *before* drawing (not after -- drawing on the native
                # crop_dim canvas then shrinking the whole panel scales
                # LINE_THICKNESS/POINT_RADIUS down with it, making markers
                # sub-pixel-thin). Scale point coordinates by the same
                # ratio so they land in the right place.
                pose_panel = (
                    _resize_square(aligned_frame, PANEL_SIZE)
                    if aligned_frame is not None
                    else row1_black.copy()
                )
                point_scale = PANEL_SIZE / crop_dim
                raw_points = pose2d_data[i] * point_scale
                if with_ik and period_info is not None:
                    period, local_i = period_info
                    draw_pose(
                        pose_panel,
                        raw_points,
                        edges,
                        [WHITE] * len(edges),
                        [WHITE] * len(node_names),
                    )
                    # Colored (IK/FK) drawn after white (raw) -- already on
                    # top -- at 2x thickness/radius so it reads clearly
                    # over the white raw skeleton wherever they overlap.
                    draw_pose(
                        pose_panel,
                        period.fk_2d_px[local_i] * point_scale,
                        edges,
                        edge_colors,
                        point_colors,
                        line_thickness=LINE_THICKNESS * IK_LINE_THICKNESS_SCALE,
                        point_radius=POINT_RADIUS * IK_POINT_RADIUS_SCALE,
                    )
                else:
                    draw_pose(pose_panel, raw_points, edges, edge_colors, point_colors)
            row2.append(_apply_overlay(pose_panel, panel_overlays.get("pose2d")))

        if with_row2 and with_ik:
            if flipped_i:
                panel_3d = row1_black.copy()
            else:
                panel_3d = np.zeros((PANEL_SIZE, PANEL_SIZE, 3), dtype=np.uint8)
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
                        panel_3d, compute_grid_lines(right_axis, up_axis, PANEL_SIZE)
                    )
                    centered = fk_3d_mm_frame - fk_3d_mm_frame[th_idx_ik]
                    points_2d = project_relative_to_panel(
                        centered, right_axis, up_axis, PANEL_SIZE
                    )
                    depths = centered @ view_dir
                    draw_fk_3d_panel(
                        panel_3d, points_2d, depths, edges, edge_colors, point_colors
                    )
            row2.append(_apply_overlay(panel_3d, panel_overlays.get("ik3d")))

        row1_img = np.hstack(row1)
        if with_row2:
            row2_img = np.hstack(row2)
            width = max(row1_img.shape[1], row2_img.shape[1])
            row1_img = _pad_to_width(row1_img, width)
            row2_img = _pad_to_width(row2_img, width)
            frame = np.vstack([row1_img, row2_img])
        else:
            frame = row1_img
        frame = _apply_overlay(frame, panel_overlays.get("speed"))
        for pt1, pt2 in border_lines:
            cv2.line(frame, pt1, pt2, BORDER_COLOR, 1, cv2.LINE_AA)
        return pad_to_macroblock(frame)

    def composite_group(
        path_idx: int, group_frame_idxs: list[int]
    ) -> list[tuple[int, np.ndarray]]:
        image = cv2.imread(str(raw_paths[path_idx]))
        channels = cv2.split(image)
        return [(i, composite_frame(i, channels[i % 3])) for i in group_frame_idxs]

    groups = []
    for path_idx in range(chunk_start // 3, (chunk_end - 1) // 3 + 1):
        group_frame_idxs = [
            i
            for i in range(path_idx * 3, path_idx * 3 + 3)
            if chunk_start <= i < chunk_end
        ]
        if group_frame_idxs:
            groups.append((path_idx, group_frame_idxs))

    frame_by_idx = {}
    with ThreadPoolExecutor(max_workers=composite_workers) as executor:
        for group_result in executor.map(lambda g: composite_group(*g), groups):
            frame_by_idx.update(group_result)
    frames = [frame_by_idx[i] for i in range(chunk_start, chunk_end)]

    chunk_path = str(Path(tmpdir) / f"chunk_{chunk_start:09d}.mp4")
    pvio.write_frames_to_video(
        chunk_path, frames, play_fps, mode=mode, quality=crf, preset=preset,
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
    crop_dim: int,
    with_muscle: bool,
    with_pose2d: bool,
    with_ik: bool,
    flipped_prob: np.ndarray | None,
    muscle_h5_path: Path | None,
    muscle_dataset_name: str | None,
    pose2d_h5_path: Path | None,
    pose2d_skeleton_json_path: Path | None,
    ikfk_h5_path: Path | None,
    output_path: Path,
    muscle_vrange: tuple[int, int] | None,
    play_fps: float,
    playback_speed: float,
    crf: int,
    preset: str | None,
    flip_confidence_threshold: float = FLIP_DECISION_THRESHOLD_DEFAULT,
    flip_mask_window: int = 5,
    orientation_filter_sigma: float = 5.0,
    num_workers: int = DEFAULT_NUM_WORKERS,
    composite_workers: int = DEFAULT_COMPOSITE_WORKERS,
    chunk_size: int = DEFAULT_CHUNK_SIZE,
    max_frames: int | None = None,
    profile: bool = False,
    colormap: str = DEFAULT_COLORMAP,
) -> None:
    logger = logging.getLogger(__name__)
    muscle_lut = _build_colormap_lut(colormap) if with_muscle else None
    encode_mode = _resolve_encode_mode()
    show_aligned = alignment in ("aligned", "both")

    # Every pseudo-BGR file packs 3 consecutive monochrome behavior frames
    # (see behavior.py); n_frames counts the real, expanded frames.
    raw_paths = sorted((recording_dir / "behavior_images").glob("behavior_frame_*.jpg"))
    n_frames = len(raw_paths) * 3
    if max_frames is not None:
        n_frames = min(n_frames, max_frames)

    keypoints_pre = is_flipped = canonical_points = transform_matrices = None
    smoothed_directions = None
    alignment_metadata_path = postprocessed_dir / "behavior_alignment_transforms.h5"
    if show_aligned and alignment_metadata_path.exists():
        with h5py.File(alignment_metadata_path, "r") as f:
            keypoints_pre = f["keypoints_xy_pre_alignment"][:n_frames]
            keypoints_post = f["keypoints_xy_post_alignment"][:n_frames]
            transform_matrices = f["transform_matrices"][:n_frames]
            if flipped_prob is None:
                flipped_prob = f["flipped_prob"][:n_frames]
        canonical_points = np.nanmean(keypoints_post, axis=0)
        is_flipped = _compute_smoothed_flip_mask(
            flipped_prob, flip_confidence_threshold, flip_mask_window
        )
        smoothed_directions = _compute_smoothed_directions(
            keypoints_pre, orientation_filter_sigma
        )

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
        # `build_node_colors`/`build_edge_colors` return RGB tuples, used
        # as-is (see `spotlight_pose2d.viz`'s own docstring): frames go
        # straight to `pvio.write_frames_to_video`, which reads RGB, with
        # no cv2.imwrite (BGR) step in between to reorder for.
        point_colors = build_node_colors(node_names)
        edge_colors = build_edge_colors(edges, node_names)

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

    panel_overlays, border_lines = _precompute_panel_overlays(
        recording_dir=recording_dir,
        raw_paths=raw_paths,
        show_aligned=show_aligned,
        with_muscle=with_muscle,
        with_pose2d=with_pose2d,
        with_row2=with_pose2d,
        with_ik=with_ik,
        crop_dim=crop_dim,
        playback_speed=playback_speed,
    )

    output_path.parent.mkdir(parents=True, exist_ok=True)
    chunk_bounds = list(range(0, n_frames, chunk_size)) + [n_frames]
    chunks = list(zip(chunk_bounds[:-1], chunk_bounds[1:]))
    encode_pool = Parallel(n_jobs=num_workers, return_as="generator")
    logger.info(
        f"Rendering {n_frames} visualization frames across {len(chunks)} "
        f"parallel chunks of ~{chunk_size} frames each ({num_workers} encode workers, "
        f"effective: {encode_pool._effective_n_jobs()}, x {composite_workers} composite "
        f"threads each, mode={encode_mode})..."
    )

    t_render = time.perf_counter()
    with tempfile.TemporaryDirectory(prefix="viz_chunks_") as tmpdir:

        def chunk_kwargs(start: int, end: int) -> dict:
            return dict(
                chunk_start=start, chunk_end=end, tmpdir=tmpdir, raw_paths=raw_paths,
                show_aligned=show_aligned, crop_dim=crop_dim,
                transform_matrices=transform_matrices, keypoints_pre=keypoints_pre,
                smoothed_directions=smoothed_directions, canonical_points=canonical_points,
                is_flipped=is_flipped, panel_overlays=panel_overlays,
                border_lines=border_lines, with_muscle=with_muscle,
                muscle_h5_path=muscle_h5_path, muscle_dataset_name=muscle_dataset_name,
                muscle_meta=muscle_meta, muscle_vrange=muscle_vrange, muscle_lut=muscle_lut,
                with_pose2d=with_pose2d, pose2d_data=pose2d_data, node_names=node_names,
                edges=edges, point_colors=point_colors, edge_colors=edge_colors,
                with_ik=with_ik, ik_periods_by_frame=ik_periods_by_frame,
                th_idx_ik=th_idx_ik, thc_idxs=thc_idxs, play_fps=play_fps, crf=crf,
                preset=preset, mode=encode_mode, composite_workers=composite_workers,
            )  # fmt: skip

        # Optionally profile one real chunk (the first), run synchronously
        # here rather than inside a parallel worker so cProfile only sees
        # this one chunk's own work, not joblib's dispatch overhead.
        chunk_paths = []
        remaining_chunks = chunks
        if profile and chunks:
            first_start, first_end = chunks[0]
            profiler = cProfile.Profile()
            profiler.enable()
            chunk_paths.append(_render_chunk(**chunk_kwargs(first_start, first_end)))
            profiler.disable()
            profile_path = postprocessed_dir / "viz_chunk_profile.prof"
            profiler.dump_stats(str(profile_path))
            stream = io.StringIO()
            pstats.Stats(profiler, stream=stream).sort_stats("cumulative").print_stats(
                20
            )
            logger.info(
                f"Visualization chunk profile (chunk 0 of {len(chunks)}, top 20 by "
                f"cumulative time; full profile saved to {profile_path}):\n"
                f"{stream.getvalue()}"
            )
            remaining_chunks = chunks[1:]

        # `return_as="generator"` (order preserved) instead of the default
        # (which blocks until every chunk is done) so completions can be
        # logged as they land, in place of a raw progress bar (and,
        # unlike joblib's own `verbose=`, through our own `logger` so
        # timestamps/formatting stay consistent with the rest of the run).
        chunk_results = encode_pool(
            delayed(_render_chunk)(**chunk_kwargs(start, end))
            for start, end in remaining_chunks
        )
        log_every = max(1, len(chunks) // 20)
        for chunk_path in chunk_results:
            chunk_paths.append(chunk_path)
            n_done = len(chunk_paths)
            if n_done % log_every == 0 or n_done == len(chunks):
                logger.info(f"Rendered {n_done}/{len(chunks)} visualization chunks...")
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
