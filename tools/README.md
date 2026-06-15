> [!NOTE]
> **Index of Spotlight-related repositories:** see [go.epfl.ch/spotlight-poseforge](http://go.epfl.ch/spotlight-poseforge#code).

# spotlight-tools

Offline Python tools for the Spotlight recording system:

1. **Arena registration** (`src/spotlight_tools/arena/`): parse the PDF arena spec, generate the mapping board and active-area mask, detect AprilTags from a registration scan, and fit the linear (stage, pixel) <-> physical mapping model.
2. **Postprocessing** (`src/spotlight_tools/postprocessing/`): decode pseudo-BGR behavior JPEGs, run SLEAP 2-D pose estimation, align/crop frames, warp muscle images, and generate summary videos and stage-trajectory plots.

## Quick start

```bash
# Install (uv recommended)
cd spotlight-tools/
uv pip install -e .

# Run the registration scan (C++ side) first, then fit the model:
fit-arena-registration -a ~/Spotlight/arenas/arena146

# Post-process a recording:
postprocess-recording --recording-dir ~/data/spotlight/20250613-fly1b-002/ --with-muscle
```

## Scripts

| Entry point | Script | Description |
|---|---|---|
| `fit-arena-registration` | `scripts/fit_arena_registration.py` | Fit registration model from scan images. |
| `postprocess-recording` | `scripts/postprocess_recording.py` | Full postprocessing pipeline. |
| `visualize-stage-trajectory` | `scripts/visualize_stage_trajectory.py` | Plot stage XY path. |
| *(run directly)* | `scripts/arena/make_arena146_config.py` | Regenerate arena146 assets from `arena146_spec.pdf`. |

See the [wiki](https://github.com/NeLy-EPFL/spotlight-control/wiki) for the full procedure.
