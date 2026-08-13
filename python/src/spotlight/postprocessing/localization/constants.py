"""Tuned/fixed configuration values for the localization (`TinyLocalizationModel`)
pipeline: input/output sizes, keypoint naming, and detection thresholds.
"""

KeypointSpec = dict[str, tuple[str, ...]]

# Maps each coarse keypoint's output name to the RepVGG-A0 skeleton node(s)
# it's derived from: one name for a direct node (e.g. "thorax" -> "Th"),
# several for a midpoint. Dict order is the model's own keypoint head
# output order.
KEYPOINT_SPEC: KeypointSpec = {
    "neck": ("N",),
    "thorax": ("Th",),
    "abdomen": ("A",),
}
COARSE_KEYPOINTS = list(KEYPOINT_SPEC.keys())
N_KEYPOINTS = len(COARSE_KEYPOINTS)  # head, thorax, abdomen

# Every trial's raw camera frame, measured (not the commonly-quoted
# 1984x1500; the height matches, but not the width).
NATIVE_FRAME_SIZE = (1472, 1984)  # (width, height)
SCALE_FACTOR = 4  # 0.25x, see model.TinyLocalizationModel's docstring
OUTPUT_SIZE = (
    NATIVE_FRAME_SIZE[0] // SCALE_FACTOR,
    NATIVE_FRAME_SIZE[1] // SCALE_FACTOR,
)

ALIGNED_BOX_SIZE = 900  # pose2d.constants.ALIGNED_FRAME_SIZE
RAW_TO_ALIGNED_SCALE = 1.0  # measured physical constant, not fit; see box.py

# Below this weighted keypoint confidence, a frame is labeled flipped.
FLIPPED_THRESHOLD = 0.5
