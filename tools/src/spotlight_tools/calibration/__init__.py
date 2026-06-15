"""
spotlight_tools.calibration -- legacy ArUco-based calibration (still used by postprocessing).

This package implements the original ArUco board scanning and linear mapper
that was used before the arena146 AprilTag registration pipeline. It is
retained because postprocessing/muscle.py still depends on
SpotlightPositionMapper and BehaviorMuscleCrossMapper to warp muscle frames
into the behavior camera coordinate system.

For the current arena registration workflow, use spotlight_tools.arena instead.
"""

from .aruco import ArUcoBoard as ArUcoBoard
from .aruco import detect_aruco as detect_aruco
from .aruco import plot_aruco_detections as plot_aruco_detections

from .charuco import CharucoBoard as CharucoBoard
from .charuco import get_gizem_board as get_gizem_board

from .model import ransac_filter_outliers as ransac_filter_outliers
from .model import visualize_ransac_results as visualize_ransac_results

from .mapper import SpotlightPositionMapper as SpotlightPositionMapper
from .mapper import BehaviorMuscleCrossMapper as BehaviorMuscleCrossMapper
from .mapper import HomographyMapper as HomographyMapper
