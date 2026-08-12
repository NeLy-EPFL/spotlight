"""
spotlight_postprocessing -- full offline processing pipeline for recordings
(behavior/localization alignment, pose2d, muscle warping, inverse kinematics, QA
video). Needs the heavy ML/IK stack (torch, quickik, sleap, ...); the
lighter `spotlight_tools` (calibration, arena registration) doesn't depend
on this package at all, so it can be installed and run without any of this.

Install this package's own deps with the "postprocessing" extra:
`uv sync --extra postprocessing` (from `python/`), or
`pip install '.[postprocessing]'`.

Modules
-------
behavior
    Decode pseudo-BGR JPEGs, run `TinyLocalizationModel` alignment, and (in the
    same streaming pass) pose2d inference on the aligned crop. Entry point:
    process_behavior_pipeline.
muscle
    Plan the muscle<->behavior frame mapping and warp muscle frames into the
    behavior-camera coordinate system, straight into an HDF5 dataset. Entry
    points: plan_muscle_behavior_mapping, warp_muscle_chunk.
stage
    Interpolate stage XY positions at each behavior-frame timestamp.
    Entry point: interp_stage_pos_at_behavior_frames.
visualize
    Generate the 5-panel QA summary video. Entry point: generate_summary_video.
io
    Low-level helpers: find per-frame files, check output path consistency,
    append muscle frames to an HDF5 dataset.
localization, pose2d, invkin
    The fly-alignment, 2D-pose, and inverse-kinematics models/pipelines
    used by the modules above.
"""

try:
    import torch  # noqa: F401
except ImportError as e:
    raise ImportError(
        'spotlight_postprocessing requires the "postprocessing" extra '
        "(torch, quickik, sleap, and other heavy inference/training "
        "dependencies), which isn't installed. Install it with "
        "`uv sync --extra postprocessing` (from the `python/` directory), "
        "or `pip install '.[postprocessing]'`."
    ) from e

# fmt: off

from .behavior import expand_single_pseudo_bgr_image as expand_single_pseudo_bgr_image
from .behavior import transform_single_frame_to_align as transform_single_frame_to_align
from .behavior import process_behavior_pipeline as process_behavior_pipeline

from .muscle import warp_single_muscle_frame_to_behavior as warp_single_muscle_frame_to_behavior
from .muscle import plan_muscle_behavior_mapping as plan_muscle_behavior_mapping
from .muscle import warp_muscle_chunk as warp_muscle_chunk
from .muscle import match_muscle_frameid_to_behavior_frameid as match_muscle_frameid_to_behavior_frameid

from .stage import interp_stage_pos_at_behavior_frames as interp_stage_pos_at_behavior_frames

from .io import find_files_per_frame_by_suffix as find_files_per_frame_by_suffix
from .io import MuscleH5Writer as MuscleH5Writer

from .visualize import generate_summary_video as generate_summary_video

# fmt: on
