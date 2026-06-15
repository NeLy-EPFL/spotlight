"""
spotlight_tools -- offline processing and calibration tools for the Spotlight setup.

Sub-packages
------------
arena
    Arena configuration and AprilTag-based registration fitting (current pipeline).
    Key classes: ArenaConfig; key functions: fit_arena_registration.
calibration
    Legacy ArUco-based mapper; still used by postprocessing.muscle for
    muscle-to-behavior warping.
common
    Config loader and video I/O utilities.
postprocessing
    Full offline pipeline: stage interpolation, behavior alignment (SLEAP),
    muscle warping, summary video generation, and trajectory visualisation.
"""

from importlib.resources import files


def get_assets_dir():
    """Get the path to the assets directory."""
    return files("spotlight_tools") / "assets"
