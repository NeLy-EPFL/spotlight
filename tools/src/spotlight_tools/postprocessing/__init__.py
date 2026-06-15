"""
spotlight_tools.postprocessing -- full offline processing pipeline for recordings.

Modules
-------
behavior
    Decode pseudo-BGR JPEGs, run SLEAP 2-D pose estimation, and produce
    aligned / cropped behavior video.  Entry point:
    decode_and_align_all_behavior_frames.
muscle
    Warp muscle frames into the behavior-camera coordinate system and apply
    the same fly-alignment transforms.  Entry point:
    warp_all_muscle_frames_to_behavior.
stage
    Interpolate stage XY positions at each behavior-frame timestamp.
    Entry point: interp_stage_pos_at_behavior_frames.
visualize
    Generate summary video, 2-D pose overlays, muscle-behavior overlay
    samples, and stage-trajectory plot.
io
    Low-level helpers: find per-frame files, check output path consistency.
"""

# fmt: off

from .behavior import decode_and_align_all_behavior_frames as decode_and_align_all_behavior_frames
from .behavior import expand_single_pseudo_bgr_image as expand_single_pseudo_bgr_image
from .behavior import estimate_2dpose_sequence as estimate_2dpose_sequence
from .behavior import fill_gaps_in_2dpose_sequence as fill_gaps_in_2dpose_sequence
from .behavior import transform_single_frame_to_align as transform_single_frame_to_align

from .muscle import warp_single_muscle_frame_to_behavior as warp_single_muscle_frame_to_behavior
from .muscle import warp_all_muscle_frames_to_behavior as warp_all_muscle_frames_to_behavior
from .muscle import match_behavior_frameid_to_muscle_frameid as match_behavior_frameid_to_muscle_frameid
from .muscle import match_muscle_frameid_to_behavior_frameid as match_muscle_frameid_to_behavior_frameid

from .stage import interp_stage_pos_at_behavior_frames as interp_stage_pos_at_behavior_frames 

from .io import find_files_per_frame_by_suffix as find_files_per_frame_by_suffix

from .visualize import visualize_stage_trajectory as visualize_stage_trajectory
from .visualize import generate_summary_video as generate_summary_video
from .visualize import draw_2dpose_on_single_frame as draw_2dpose_on_single_frame
from .visualize import generate_overlay_samples as generate_overlay_samples

# fmt: on
