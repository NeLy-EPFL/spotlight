"""Training-time augmentation for pose2d: a small rotation/translation
jitter (same 2x3-affine convention as `geometry.py`), a horizontal
flip, plus brightness/contrast jitter.

A fly's left/right legs aren't interchangeable, so a flip alone would mislabel
every left/right keypoint (e.g. the pixel content that was `LF_ThC` becomes,
after mirroring, what `RF_ThC` should look like). `flip_pair_indices` (see
`dataset.flip_pair_indices`) corrects this by relabeling each keypoint to its
mirror's original value, not just mirroring coordinates in place.
"""

import cv2
import numpy as np

from spotlight.postprocessing.pose2d.geometry import apply_affine

MAX_TRANSLATION_PX = 30.0
MAX_ROTATION_DEG = 5.0
FLIP_PROBABILITY = 0.5
BRIGHTNESS_RANGE = (0.8, 1.2)
CONTRAST_RANGE = (0.8, 1.2)


def random_affine_matrix(image_size: int, rng: np.random.Generator) -> np.ndarray:
    """A random small rotation+translation matrix about the image center.

    Args:
        image_size: Height/width of the (square) image.
        rng: Random number generator to draw jitter from.

    Returns:
        `(2, 3)` affine matrix, forward (src -> dst), matching
        `geometry.apply_affine`'s convention.
    """
    angle = np.deg2rad(rng.uniform(-MAX_ROTATION_DEG, MAX_ROTATION_DEG))
    translation = rng.uniform(-MAX_TRANSLATION_PX, MAX_TRANSLATION_PX, size=2)
    center = np.array([image_size / 2, image_size / 2])
    cos_a, sin_a = np.cos(angle), np.sin(angle)
    rotation = np.array([[cos_a, -sin_a], [sin_a, cos_a]])

    matrix = np.zeros((2, 3))
    matrix[:, :2] = rotation
    matrix[:, 2] = center - rotation @ center + translation
    return matrix


def augment(
    image: np.ndarray,
    points: np.ndarray,
    rng: np.random.Generator,
    flip_pair_indices: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Apply one random augmentation to one (image, keypoints) pair.

    Args:
        image: `(H, W, C)` uint8 image.
        points: `(n_nodes, 2)` keypoint coordinates, may contain NaN.
        rng: Random number generator to draw jitter from.
        flip_pair_indices: `(n_nodes,)`, node i's left/right mirror index
            (itself, if unpaired); see `dataset.flip_pair_indices`.

    Returns:
        Augmented `(image, points)`, same shapes/dtypes as the input.
    """
    matrix = random_affine_matrix(image.shape[0], rng)
    warped = cv2.warpAffine(
        image, matrix, (image.shape[1], image.shape[0]), borderMode=cv2.BORDER_REPLICATE
    )
    if warped.ndim == 2:
        warped = warped[:, :, np.newaxis]
    warped_points = apply_affine(points[np.newaxis], matrix[np.newaxis])[0]

    if rng.random() < FLIP_PROBABILITY:
        warped = np.ascontiguousarray(warped[:, ::-1, :])
        warped_points = warped_points[flip_pair_indices]
        warped_points[:, 0] = (warped.shape[1] - 1) - warped_points[:, 0]

    brightness = rng.uniform(*BRIGHTNESS_RANGE)
    contrast = rng.uniform(*CONTRAST_RANGE)
    jittered = warped.astype(np.float32)
    jittered = (jittered - 127.5) * contrast + 127.5
    jittered = jittered * brightness
    jittered = np.clip(jittered, 0, 255).astype(np.uint8)

    return jittered, warped_points
