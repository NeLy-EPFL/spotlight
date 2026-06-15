"""
spotlight_tools.arena -- arena configuration and registration fitting.

ArenaConfig
    Parses a PDF arena spec (arena146_spec.pdf) to extract AprilTag and
    DataMatrix positions, rasterise the active-area mask, and write the
    three derived assets (mapping_board.pdf, active_area.png, metadata.yaml)
    to the arena directory.

fit_arena_registration
    End-to-end pipeline: reads the `mapping_scan/` images produced by
    `run-arena-registration-scan`, detects AprilTags, rejects outliers via
    MAD filtering and RANSAC, and writes calibration_result.yaml plus
    diagnostic plots to `<arena_dir>/model/`.
"""

from .arena import ArenaConfig as ArenaConfig

from .registration import APRILTAG_FAMILY as APRILTAG_FAMILY
from .registration import detect_apriltags as detect_apriltags
from .registration import filter_per_apriltag_outliers as filter_per_apriltag_outliers
from .registration import fit_arena_registration as fit_arena_registration
from .registration import fit_ransac_model as fit_ransac_model
from .registration import gather_apriltag_points as gather_apriltag_points
