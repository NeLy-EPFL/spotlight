# Camera homography

> [!WARNING]
> **TODO — not yet implemented.** This step, and the `run-homography-scan` program
> that will drive it, do not exist yet. This page is a placeholder describing the
> intent.

## What it replaces

Earlier versions calibrated the muscle-to-behavior and muscle-only mappings with an
ArUco board. That approach has been **removed**. It will be replaced by a separate
**camera homography** step: a scan that captures images of a high-resolution
ChArUco board, from which a Python tool fits the camera homography matrices for both
cameras.

This is independent of [arena registration](arena_registration.md), which maps the
behavior camera to stage/arena coordinates and is unaffected.

## Planned pieces (TODO)

- `run-homography-scan` (recorder, C++): capture ChArUco board images for both
  cameras. *Not yet implemented; not built.*
- A Python tool in `spotlight-tools` to fit the homography matrices from the scan.
  *Not yet implemented.*

Until this is implemented, the recorder no longer writes a
`calibration_parameters_muscle.yaml`, and muscle-to-behavior warping in
post-processing has no current calibration source.
