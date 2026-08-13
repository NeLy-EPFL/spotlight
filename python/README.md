> [!NOTE]
> **Index of Spotlight-related repositories:** see [go.epfl.ch/spotlight-poseforge](http://go.epfl.ch/spotlight-poseforge#code).

# spotlight

Offline Python tools for the Spotlight recording system:

1. **Arena registration** (`src/spotlight/arena/`): parse the PDF arena spec, generate the mapping board and active-area mask, detect AprilTags from a registration scan, and fit the linear (stage, pixel) <-> physical mapping model.
2. **Calibration** (`src/spotlight/calibration/`): fit the behavior-to-muscle camera homography from a scan.
3. **Postprocessing** (`src/spotlight/postprocessing/`): decode pseudo-BGR behavior JPEGs, localize and align the fly, run 2D pose estimation, warp muscle images, optionally fit inverse kinematics and replay it in FlyGym, and generate summary videos and stage-trajectory plots.

## Quick start

```bash
# Install (uv recommended). `uv sync` alone installs only the
# lightweight arena/calibration tools (spotlight); postprocessing
# (spotlight.postprocessing: torch, quickik, sleap, ...) is a separate
# extra so it doesn't weigh down a plain calibration-only install:
cd python/
uv sync                          # arena/calibration tools only
uv sync --extra postprocessing   # + the full postprocessing pipeline

# Run the registration scan (C++ side) first, then fit the model:
fit-arena-registration -a ~/Spotlight/arenas/arena146x146

# Post-process a recording:
postprocess-recording ~/data/spotlight/20250613-fly1b-002/
```

## Scripts

| Entry point | Implementation | Description |
|---|---|---|
| `fit-arena-registration` | `src/spotlight/arena/cli/fit_arena_registration.py` | Fit registration model from scan images. |
| `fit-homography` | `src/spotlight/calibration/cli/fit_homography.py` | Fit behavior-to-muscle camera homography from a scan. |
| `postprocess-recording` | `src/spotlight/postprocessing/cli/postprocess_recording.py` | Full postprocessing pipeline. |
| `visualize-stage-trajectory` | `src/spotlight/arena/cli/visualize_stage_trajectory.py` | Plot stage XY path. |
| *(run directly)* | `scripts/tools/set_up_arenas.py` | Regenerate every bundled arena's config assets from its `arena_spec.pdf`. |

See the [wiki](https://github.com/NeLy-EPFL/spotlight-control/wiki) for the full procedure.
