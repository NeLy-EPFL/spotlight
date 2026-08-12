# Camera homography

## What it replaces

Earlier versions calibrated the muscle-to-behavior and muscle-only mappings with an
ArUco board. That approach has been **removed**, replaced by a **camera homography**
step: a scan that captures images of a high-resolution ChArUco board, from which a
Python tool fits the camera homography matrices for both cameras.

This is independent of [arena registration](arena_registration.md), which maps the
behavior camera to stage/arena coordinates and is unaffected.

## Procedure

1. `run-charuco-homography-scan -p PROFILE_DIR` (recorder, C++): scans a grid of
   stage positions (configured under `[homography]` in `recorder_config.yaml`) and
   captures paired behavior/muscle ChArUco images at each one.
2. `fit-homography` (Python): fits the homography matrices from the scan images and
   writes `homography_parameters.yaml`, which post-processing reads for
   muscle-to-behavior warping (see `--homography-path` in `postprocess-recording
   --help`).
