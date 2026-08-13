"""Tuned/fixed configuration values for the pose2d (`RepVGGPoseModel`)
pipeline: input/output sizes and training-target defaults.
"""

INPUT_SIZE = 450

# Every aligned-domain video this project produces is cropped/warped to
# this size; the `.h5`'s own stored points are in this native scale, not
# `INPUT_SIZE`, since alignment and JPEG-caching are independent steps.
ALIGNED_FRAME_SIZE = 900

HEATMAP_SIGMA = 2.0

N_KEYPOINTS = 37
