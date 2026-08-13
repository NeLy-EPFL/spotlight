"""
spotlight: lightweight calibration/arena tools for the Spotlight
setup, with no heavy ML dependencies (no torch/quickik/sleap). The full
recording-postprocessing pipeline (behavior/localization alignment, pose2d,
muscle warping, IK, QA video) is the separate `spotlight.postprocessing`
package, which depends on this one but not vice versa, so installing
just this package (the default) keeps the dependency tree light for
anyone who only needs calibration/arena tools.

Sub-packages
------------
arena
    Arena configuration and AprilTag-based registration fitting (current
    pipeline). Key classes: ArenaConfig; key functions: fit_arena_registration.
calibration
    Legacy ArUco-based mapper; still used by `spotlight.postprocessing`'s
    muscle-to-behavior warping.
common
    Config loader (`get_assets_dir`, `load_spotlight_config`).
"""

from importlib.resources import files


def get_assets_dir():
    """Get the path to the assets directory."""
    return files("spotlight") / "assets"
