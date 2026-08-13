"""Training-time augmentation for `TinyLocalizationDataset`: independent
horizontal and vertical flips plus brightness/contrast jitter. No
rotation/translation jitter, unlike `pose2d.augment`: keeps
things simple for this tiny model's first real training round.

The camera looks down at the arena from a fixed viewpoint, so both image
axes are in-plane arena directions, not gravity: whether the fly is
flipped (upside down) is about its orientation relative to gravity
(roughly the camera's optical axis, perpendicular to the image plane),
which neither in-plane mirror touches. So both flips are label-preserving,
same as `pose2d`'s own horizontal-only flip is for its leg
keypoints. All three keypoints (head, thorax, abdomen) are midline points,
so a flip only needs to mirror the relevant coordinate; unlike
`pose2d`'s left/right leg pairs, there's no keypoint identity to
relabel.
"""

import numpy as np

HORIZONTAL_FLIP_PROBABILITY = 0.5
VERTICAL_FLIP_PROBABILITY = 0.5
BRIGHTNESS_RANGE = (0.8, 1.2)
CONTRAST_RANGE = (0.8, 1.2)


def augment(
    image: np.ndarray, keypoints: np.ndarray, rng: np.random.Generator
) -> tuple[np.ndarray, np.ndarray]:
    """Apply one random augmentation to one (image, keypoints) pair.

    Args:
        image: `(H, W, C)` uint8 image.
        keypoints: `(3, 2)`, normalized `[0, 1]` `(x, y)` coordinates (see
            `dataset.points_to_model_space`).
        rng: Random number generator to draw jitter from.

    Returns:
        Augmented `(image, keypoints)`, same shapes/dtypes as the input.
    """
    keypoints = keypoints.copy()
    if rng.random() < HORIZONTAL_FLIP_PROBABILITY:
        image = np.ascontiguousarray(image[:, ::-1, :])
        keypoints[:, 0] = 1.0 - keypoints[:, 0]
    if rng.random() < VERTICAL_FLIP_PROBABILITY:
        image = np.ascontiguousarray(image[::-1, :, :])
        keypoints[:, 1] = 1.0 - keypoints[:, 1]

    brightness = rng.uniform(*BRIGHTNESS_RANGE)
    contrast = rng.uniform(*CONTRAST_RANGE)
    jittered = image.astype(np.float32)
    jittered = (jittered - 127.5) * contrast + 127.5
    jittered = jittered * brightness
    jittered = np.clip(jittered, 0, 255).astype(np.uint8)

    return jittered, keypoints
