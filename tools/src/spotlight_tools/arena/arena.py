"""
ArenaConfig -- PDF-based arena configuration (mm-native public API).

PDF internally stores coordinates in points (1 pt = 1/72 in). This module
converts all values to mm at the boundary so every public attribute and
return value is in mm. The conversion is PT_TO_MM = 25.4 / 72.

Non-Python dependency: libdmtx
    Ubuntu/Debian : sudo apt-get install libdmtx0b
    macOS         : brew install libdmtx
"""

import io
from pathlib import Path

import cv2
import mmh3
import numpy as np
import pymupdf
import yaml
from PIL import Image
from pylibdmtx.pylibdmtx import encode as dmtx_encode
from scipy.spatial.distance import cdist

# -----------------------------------------------------------------------------
# Constants
# -----------------------------------------------------------------------------

COLOR_DATAMATRIX = "#00ff00"  # green -- exactly one square, marks DM location
COLOR_APRILTAG = "#ff0000"  # red   -- one square per AprilTag marker
COLOR_ACTIVE_AREA = "#000000"  # black -- maze / active area shapes
COLOR_BOARD_BORDER = "#000000"  # black -- cut border drawn on mapping board

BORDER_THICKNESS_MM = 0.1  # mapping-board cut border line width in mm

APRILTAG_FAMILY = "tag16h5"  # override in subclasses if needed

# Minimum rendered size for embedded tag/barcode images (pixels)
_MIN_IMAGE_PX = 64
_PX_PER_MM = 10  # rendering resolution for embedded images

# PDF unit conversion (points <-> mm)
PT_TO_MM = 25.4 / 72.0
MM_TO_PT = 72.0 / 25.4

CHECKSUM_FONT = "Courier"  # monospace built-in PDF font
CHECKSUM_FONT_SIZE = 6  # pt

_CV2_APRILTAG_FAMILY = {
    "tag36h11": cv2.aruco.DICT_APRILTAG_36h11,
    "tag25h9": cv2.aruco.DICT_APRILTAG_25h9,
    "tag16h5": cv2.aruco.DICT_APRILTAG_16h5,
}

# Public position dict: keys tl, tr, bl, br, center -> (x, y) in mm
Pos = dict[str, tuple[float, float]]


# -----------------------------------------------------------------------------
# Color helpers
# -----------------------------------------------------------------------------


def _hex_to_rgb(hex_color: str) -> tuple[float, float, float]:
    h = hex_color.lstrip("#")
    return (int(h[0:2], 16) / 255.0, int(h[2:4], 16) / 255.0, int(h[4:6], 16) / 255.0)


def _color_matches(fill, hex_color: str, tol: float = 1.5 / 255.0) -> bool:
    if fill is None:
        return False
    r, g, b = _hex_to_rgb(hex_color)
    return (
        abs(fill[0] - r) <= tol and abs(fill[1] - g) <= tol and abs(fill[2] - b) <= tol
    )


# -----------------------------------------------------------------------------
# Shape helpers (pymupdf drawing dicts, coordinates in points)
# -----------------------------------------------------------------------------


def _count_corners(path: dict) -> int:
    """Count unique corner vertices in a pymupdf drawing path."""
    pts: list[pymupdf.Point] = []
    for item in path["items"]:
        kind = item[0]
        if kind == "l":  # line
            pts += [item[1], item[2]]
        elif kind == "re":  # rectangle command
            r = item[1]
            pts += [
                pymupdf.Point(r.x0, r.y0),
                pymupdf.Point(r.x1, r.y0),
                pymupdf.Point(r.x1, r.y1),
                pymupdf.Point(r.x0, r.y1),
            ]
        elif kind == "qu":  # quadrilateral
            q = item[1]
            pts += [q.ul, q.ur, q.lr, q.ll]
        elif kind == "c":  # cubic bezier (start + end)
            pts += [item[1], item[4]]

    unique: list[pymupdf.Point] = []
    for p in pts:
        if not any(abs(p.x - u.x) < 0.5 and abs(p.y - u.y) < 0.5 for u in unique):
            unique.append(p)
    return len(unique)


def _is_square(rect: pymupdf.Rect, tol_pt: float = 0.5) -> bool:
    return abs(rect.width - rect.height) <= tol_pt


def _rect_to_pos(rect: pymupdf.Rect) -> Pos:
    """Convert a pymupdf Rect (points) to a public Pos dict (mm)."""
    x0 = rect.x0 * PT_TO_MM
    x1 = rect.x1 * PT_TO_MM
    y0 = rect.y0 * PT_TO_MM
    y1 = rect.y1 * PT_TO_MM
    return {
        "topleft": (x0, y0),
        "topright": (x1, y0),
        "bottomleft": (x0, y1),
        "bottomright": (x1, y1),
        "center": ((x0 + x1) / 2, (y0 + y1) / 2),
    }


# -----------------------------------------------------------------------------
# Image helpers
# -----------------------------------------------------------------------------


def _array_to_pixmap(arr: np.ndarray) -> pymupdf.Pixmap:
    """Convert a grayscale uint8 numpy array to a pymupdf Pixmap via PNG."""
    buf = io.BytesIO()
    Image.fromarray(arr.astype(np.uint8), mode="L").save(buf, format="PNG")
    return pymupdf.Pixmap(buf.getvalue())


# -----------------------------------------------------------------------------
# ArenaConfig
# -----------------------------------------------------------------------------


class ArenaConfig:
    """Arena configuration loaded from a PDF file. All public values are in mm."""

    APRILTAG_FAMILY: str = APRILTAG_FAMILY

    def __init__(self, arena_config_pdf_path: Path, raster_resolution_mm: float = 0.1):
        self.raster_resolution_mm = raster_resolution_mm
        self.pdf_path = arena_config_pdf_path

        doc = pymupdf.open(str(arena_config_pdf_path))
        self._page = doc[0]

        self.arena_dim: tuple[float, float] = self._get_arena_size()
        self.datamatrix_pos: Pos = self._get_datamatrix_pos()
        self.apriltag_pos_list: list[Pos] = self._get_apriltag_pos_list()
        self.rasterized_active_area = self._rasterize_active_area()

    # -------------------------------------------------------------------------

    def _get_arena_size(self) -> tuple[float, float]:
        return self._page.rect.width * PT_TO_MM, self._page.rect.height * PT_TO_MM

    def _drawings_by_color(self, hex_color: str) -> list[dict]:
        return [
            p
            for p in self._page.get_drawings()
            if _color_matches(p.get("fill"), hex_color)
        ]

    # -------------------------------------------------------------------------

    def _get_datamatrix_pos(self) -> Pos:
        """Find the single green square and return its position dict (mm)."""
        paths = self._drawings_by_color(COLOR_DATAMATRIX)
        if len(paths) != 1:
            raise ValueError(
                f"Expected exactly 1 {COLOR_DATAMATRIX} shape, found {len(paths)}"
            )
        path = paths[0]
        if _count_corners(path) != 4:
            raise ValueError(
                f"Datamatrix shape has {_count_corners(path)} corners (expected 4)"
            )
        rect = path["rect"]
        if not _is_square(rect):
            raise ValueError(
                f"Datamatrix shape is not square: "
                f"{rect.width * PT_TO_MM:.3f} mm x {rect.height * PT_TO_MM:.3f} mm"
            )
        return _rect_to_pos(rect)

    def _get_apriltag_pos_list(self) -> list[Pos]:
        """
        Find all red squares, verify no overlaps, and return position dicts sorted
        so that the path datamatrix_pos[center] -> tag_0 -> tag_1 -> ... is shortest.
        """
        paths = self._drawings_by_color(COLOR_APRILTAG)
        rects: list[pymupdf.Rect] = []

        for i, path in enumerate(paths):
            if _count_corners(path) != 4:
                raise ValueError(
                    f"AprilTag shape {i} has {_count_corners(path)} corners (expected 4)"
                )
            rect = path["rect"]
            if not _is_square(rect):
                raise ValueError(
                    f"AprilTag shape {i} is not square: "
                    f"{rect.width * PT_TO_MM:.3f} mm x {rect.height * PT_TO_MM:.3f} mm"
                )
            rects.append(rect)

        for i in range(len(rects)):
            for j in range(i + 1, len(rects)):
                if not (rects[i] & rects[j]).is_empty:
                    raise ValueError(f"AprilTag shapes {i} and {j} overlap")

        # Sort via MST + DFS for shortest path from DM center through all tags
        dm_cx, dm_cy = self.datamatrix_pos["center"]
        points = np.array(
            [[dm_cx, dm_cy]]
            + [
                ((r.x0 + r.x1) / 2 * PT_TO_MM, (r.y0 + r.y1) / 2 * PT_TO_MM)
                for r in rects
            ]
        )

        dist = cdist(points, points)

        # Greedy nearest-neighbor: at each step, visit closest unvisited tag
        order: list[int] = []
        visited = np.zeros(len(points), dtype=bool)
        current = 0  # Start at DataMatrix center
        for _ in range(len(points) - 1):
            visited[current] = True
            unvisited_dists = dist[current].copy()
            unvisited_dists[visited] = np.inf
            next_node = np.argmin(unvisited_dists)
            order.append(next_node - 1)
            current = next_node
        return [_rect_to_pos(rects[i]) for i in order]

    # -------------------------------------------------------------------------

    def _rasterize_active_area(self) -> np.ndarray:
        """
        Reconstruct page with only black shapes and rasterize at full page size.

        The returned array has dimensions (arenaHeight/R) x (arenaWidth/R) so
        that pixel (col, row) corresponds exactly to physical position
        (col * R, row * R) in mm. This 1-to-1 alignment is required by the
        C++ ActiveAreaMask warp, which indexes the PNG as mask_col = physical_x/R.
        Do NOT crop this image - any crop would shift the pixel-to-physical
        mapping and misplace the mask overlay.

        Returns:
            arr -- 2D uint8 numpy array; 0 = black (walls), 255 = white (accessible)
        """
        black_paths = self._drawings_by_color(COLOR_ACTIVE_AREA)

        w_px = int(round(self.arena_dim[0] / self.raster_resolution_mm))
        h_px = int(round(self.arena_dim[1] / self.raster_resolution_mm))

        if not black_paths:
            return np.full((h_px, w_px), 255, dtype=np.uint8)

        # Replay only black paths onto a fresh page
        tmp_doc = pymupdf.open()
        tmp_page = tmp_doc.new_page(
            width=self._page.rect.width, height=self._page.rect.height
        )
        shape = tmp_page.new_shape()
        for path in black_paths:
            for item in path["items"]:
                kind = item[0]
                if kind == "re":
                    shape.draw_rect(item[1])
                elif kind == "l":
                    shape.draw_line(item[1], item[2])
                elif kind == "c":
                    shape.draw_bezier(item[1], item[2], item[3], item[4])
                elif kind == "qu":
                    shape.draw_quad(item[1])
            shape.finish(fill=(0.0, 0.0, 0.0), color=None, fill_opacity=1.0)
        shape.commit()

        dpi = 25.4 / self.raster_resolution_mm
        scale = dpi / 72.0
        pix = tmp_page.get_pixmap(
            matrix=pymupdf.Matrix(scale, scale), colorspace=pymupdf.csGRAY, alpha=False
        )
        arr = np.frombuffer(pix.samples, dtype=np.uint8).reshape(pix.height, pix.width)
        tmp_doc.close()
        return arr

    # -------------------------------------------------------------------------

    @property
    def checksum_str(self) -> str:
        _corners = ["topleft", "topright", "bottomleft", "bottomright"]

        positions = []
        # Arena dimensions
        positions += list(self.arena_dim)
        # DataMatrix position
        for corner in _corners:
            positions += list(self.datamatrix_pos[corner])
        # AprilTag positions (sorted by tag ID)
        for pos in self.apriltag_pos_list:
            for corner in _corners:
                positions += list(pos[corner])
        arr_pos = np.array(positions, dtype=np.float32)
        arr_pos = (arr_pos * 1000).round().astype(np.uint32)  # avoid float issues

        # Rasterized active area (hash of the PNG bytes)
        arr_raster = self._rasterize_active_area().astype(bool).flatten()

        # Compute 32-bit checksum of all data as hex string
        arr = np.concatenate([arr_pos, arr_raster.view(np.uint32)])
        cksum_uint32 = mmh3.hash(arr.tobytes(), signed=False)
        return f"{cksum_uint32:08x}"

    # -------------------------------------------------------------------------

    def _make_apriltag_array(self, tag_id: int, size_px: int) -> np.ndarray:
        d = cv2.aruco.getPredefinedDictionary(
            _CV2_APRILTAG_FAMILY.get(
                self.APRILTAG_FAMILY, cv2.aruco.DICT_APRILTAG_36h11
            )
        )
        img = np.zeros((size_px, size_px), dtype=np.uint8)
        cv2.aruco.generateImageMarker(d, tag_id, size_px, img, borderBits=1)
        # cv2.aruco renders AprilTag markers 180deg rotated relative to the
        # reference AprilTag library used by pupil_apriltags. Rotating here
        # ensures the printed tag's design orientation matches pupil_apriltags'
        # corner ordering, so _CORNER_KEYS in registration.py is correct.
        return np.rot90(img, 2)

    @staticmethod
    def _make_datamatrix_array(data: str, size_px: int) -> np.ndarray:
        enc = dmtx_encode(data.encode("ascii"))
        img = Image.frombytes("RGB", (enc.width, enc.height), enc.pixels)
        img = img.convert("L").resize((size_px, size_px), Image.NEAREST)
        return np.array(img)

    def _draw_mapping_board(self, output_path: Path) -> None:
        """
        Write a mapping-board PDF containing:
          - A cut border (BORDER_THICKNESS_MM thick) around the full arena
          - AprilTag markers at every red square location (IDs in sorted order)
          - DataMatrix barcode at the green square encoding the 32-bit checksum
          - No active area / black maze shapes
          - 1mm padding on all sides (canvas is 2mm taller and wider)
        """
        w_pt = self._page.rect.width
        h_pt = self._page.rect.height
        padding_pt = 1 * MM_TO_PT

        doc = pymupdf.open()
        page = doc.new_page(width=w_pt + 2 * padding_pt, height=h_pt + 2 * padding_pt)

        # Cut border at original size, offset by padding
        shape = page.new_shape()
        shape.draw_rect(
            pymupdf.Rect(padding_pt, padding_pt, w_pt + padding_pt, h_pt + padding_pt)
        )
        shape.finish(
            fill=None,
            color=_hex_to_rgb(COLOR_BOARD_BORDER),
            width=BORDER_THICKNESS_MM * MM_TO_PT,
        )
        shape.commit()

        # AprilTags
        for tag_id, pos in enumerate(self.apriltag_pos_list):
            w_mm = pos["bottomright"][0] - pos["topleft"][0]
            size_px = max(_MIN_IMAGE_PX, int(w_mm * _PX_PER_MM))
            pix = _array_to_pixmap(self._make_apriltag_array(tag_id, size_px))
            rect = pymupdf.Rect(
                pos["topleft"][0] * MM_TO_PT + padding_pt,
                pos["topleft"][1] * MM_TO_PT + padding_pt,
                pos["bottomright"][0] * MM_TO_PT + padding_pt,
                pos["bottomright"][1] * MM_TO_PT + padding_pt,
            )
            page.insert_image(rect, pixmap=pix)

            # AprilTag ID below the marker
            # Approximate text width: Courier monospace ~0.6 * fontsize per char
            tag_id_str = f"{tag_id}"
            tag_id_width = len(tag_id_str) * CHECKSUM_FONT_SIZE * 0.6
            tag_id_x = (
                (pos["topleft"][0] + pos["topright"][0]) / 2 * MM_TO_PT
                - tag_id_width / 2
                + padding_pt
            )
            tag_id_y = pos["bottomleft"][1] * MM_TO_PT + CHECKSUM_FONT_SIZE + padding_pt
            page.insert_text(
                (tag_id_x, tag_id_y),
                tag_id_str,
                fontsize=CHECKSUM_FONT_SIZE,
                fontname=CHECKSUM_FONT,
                color=_hex_to_rgb(COLOR_BOARD_BORDER),
            )

        # DataMatrix
        dm_w = self.datamatrix_pos["bottomright"][0] - self.datamatrix_pos["topleft"][0]
        size_px = max(_MIN_IMAGE_PX, int(dm_w * _PX_PER_MM))
        pix = _array_to_pixmap(self._make_datamatrix_array(self.checksum_str, size_px))
        rect = pymupdf.Rect(
            self.datamatrix_pos["topleft"][0] * MM_TO_PT + padding_pt,
            self.datamatrix_pos["topleft"][1] * MM_TO_PT + padding_pt,
            self.datamatrix_pos["bottomright"][0] * MM_TO_PT + padding_pt,
            self.datamatrix_pos["bottomright"][1] * MM_TO_PT + padding_pt,
        )
        page.insert_image(rect, pixmap=pix)

        # Checksum text centered below the DataMatrix
        # Approximate text width: Courier monospace ~0.6 * fontsize per char
        cksum_text = f"cksum: {self.checksum_str}"
        text_width = len(cksum_text) * CHECKSUM_FONT_SIZE * 0.6
        text_x = (
            (self.datamatrix_pos["topleft"][0] + self.datamatrix_pos["topright"][0])
            / 2
            * MM_TO_PT
            - text_width / 2
            + padding_pt
        )
        text_y = (
            self.datamatrix_pos["bottomleft"][1] * MM_TO_PT
            + CHECKSUM_FONT_SIZE
            + padding_pt
        )
        page.insert_text(
            (text_x, text_y),
            cksum_text,
            fontsize=CHECKSUM_FONT_SIZE,
            fontname=CHECKSUM_FONT,
            color=_hex_to_rgb(COLOR_BOARD_BORDER),
        )
        doc.save(str(output_path))
        doc.close()
        print(f"Mapping board -> {output_path}  (checksum: {self.checksum_str})")

    def save(self, output_dir: Path) -> None:
        """Save metadata YAML, mapping board PDF, and rasterized active area map."""
        output_dir.mkdir(exist_ok=True, parents=True)

        self._draw_mapping_board(output_dir / "mapping_board.pdf")

        Image.fromarray(self.rasterized_active_area).convert("1").save(
            output_dir / "active_area.png"
        )

        # Convert numpy types to plain Python types and round to 0.001 mm
        def _to_python(obj):
            if isinstance(obj, (np.floating, np.integer)):
                return round(float(obj), 3)
            elif isinstance(obj, float):
                return round(obj, 3)
            elif isinstance(obj, dict):
                return {k: _to_python(v) for k, v in obj.items()}
            elif isinstance(obj, (list, tuple)):
                return [_to_python(item) for item in obj]
            return obj

        metadata = {
            "checksum": self.checksum_str,
            "arena_dim": _to_python(self.arena_dim),
            "datamatrix_pos": _to_python(self.datamatrix_pos),
            "apriltag_positions": {
                tag_id: _to_python(pos)
                for tag_id, pos in enumerate(self.apriltag_pos_list)
            },
            "active_area_raster_resolution": self.raster_resolution_mm,
            "unit": "mm",
        }
        with open(output_dir / "metadata.yaml", "w") as f:
            yaml.dump(metadata, f, sort_keys=False)

        print(f"Arena data saved to {output_dir}")
