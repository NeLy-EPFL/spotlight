# `align-cameras`

> [!NOTE]
> Muscle imaging only. If you are not doing muscle imaging, skip this step.

`align-cameras` helps align (i) the behavior camera's field of view (FOV), (ii) the
muscle camera's FOV, and (iii) the spot of the blue excitation light. It is
implemented in `recorder/src/apps/align_cameras_main.cc`.

IR illumination comes from a large ring light mechanically coupled to the behavior
camera, so it needs no explicit alignment.

The blue excitation spot size is a trade-off:

1. large enough that the part covering the fly is as homogeneous as possible;
2. as small as possible to reduce overheating (which affects behavior) — a more
   focused spot lets you lower the power.

## How it works

The behavior camera streams at the ROI set in `recorder_config.yaml` (Spotlight
takes the center of the sensor). The muscle camera streams its **full** sensor. You
then click the point on the muscle image that coincides with the center of the
behavior image; the muscle FOV is cropped around that point. This aligns the two
FOVs without having to *physically* align the cameras perfectly. To define the
center reproducibly, a fixed **bullseye target** is placed on the arena and the
stage is moved so the behavior camera is centered on it.

## Usage

```
$ ./align-cameras --help
Usage: ./align-cameras [OPTIONS]
Options:
  -h, --help                 Display this help message
  -p, --profile-dir PATH     Path to profile directory (required)
  -a, --arena PATH           Path to arena directory (containing metadata.yaml and model/)
  -v, --verbose              Enable verbose output (debug level)
  --verbosity LEVEL          Set verbosity level (trace, debug, info, warn, error, critical, off)
```

Only `--profile-dir` / `-p` is needed (the arena flag is unused here). For example:

```bash
./align-cameras -p ~/Spotlight/profiles/sibo_260514
```

Two windows pop up, one per camera. Press `ESC` at any time to quit without saving.
On startup the program resets the trigger microcontroller and streams behavior-only
(no excitation light); see [Data acquisition](../data_acquisition.md).

## Procedure

1. Place the bullseye target on top of the arena. Mind the orientation: the corner
   with an arrow must align with the arrow on the arena holder. Place the acrylic
   "paper weight" plate on top to keep it pressed down.
2. Using the knobs on the Zaber X-MCC controller, move the stages so the bullseye is
   centered in the behavior camera. (Turn a knob to move; turn more to move faster;
   press the knob to stop.)
3. *Physically* adjust the blue LED — its position, distance from the sample, and
   any lenses — so the light is (i) centered on the bullseye and (ii) an appropriate
   spot size. Use the radius markers on the bullseye in the muscle view to set the
   spot size reproducibly.
4. *Physically* adjust the muscle camera so it is roughly aligned to the bullseye
   (it need not be precise).
5. If the stage/behavior camera moved during the process, re-center the behavior
   camera on the bullseye.
6. Click the center of the bullseye in the muscle window. The clicked point is shown
   in red; the nearest feasible FOV is drawn in blue with its center marked. The
   blue and red dots generally won't coincide exactly, because the PCO camera
   requires FOV boundaries to be multiples of **32 px horizontally** and **8 px
   vertically**.
7. Make sure the entire blue rectangle is within the displayed image. If not,
   *physically* adjust the muscle camera again.
8. Press `RETURN` (not the numpad Enter) to save the muscle FOV.
9. Confirm that `muscle_camera_roi.yaml` was written under the profile directory.
   This file holds the muscle FOV boundaries and is used by
   [`run-arena-registration-scan`](run_arena_registration_scan.md) and
   [`run-spotlight`](run_spotlight.md).
