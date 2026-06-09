# `fit-arena-registration`

Fits the arena registration model from a completed scan. Lives in the
[`spotlight-tools`](https://github.com/NeLy-EPFL/spotlight-tools) repo
(`scripts/fit_arena_registration.py` → `spotlight_tools.arena.registration.fit_arena_registration`).
For the procedure and the fitting algorithm, see
[Arena registration](../configuration/arena_registration.md).

## Inputs (under `<arena_dir>`)

| File | Description |
|---|---|
| `metadata.yaml` | AprilTag corner positions in arena (mm) coordinates. |
| `mapping_scan/apriltag<id>_img<i>.jpg` | 10 images per AprilTag from the scan. |
| `mapping_scan/apriltag_stage_positions.csv` | Stage position per image. |

## Outputs (under `<arena_dir>/model/`)

| File | Description |
|---|---|
| `calibration_result.yaml` | Forward and inverse mapping coefficients. |
| `calibration_points.csv` | Per-corner point set used for the fit. |
| `diagnostics.png` | Predicted-vs-actual scatter, residual histograms, spatial residual map. |
| `detections/` *(optional)* | Per-image detection overlays (`--visualize-detections`). |

A good fit has RMSE ≲ 0.1 mm and R² ≈ 1. Always check `diagnostics.png`.

## Usage

```bash
fit-arena-registration -a ~/Spotlight/arenas/arena146
```

Key options (`--help` for the full list):

| Option | Default | Description |
|---|---|---|
| `-a`, `--arena-dir` | *(required)* | Arena directory path. |
| `--min-decision-margin` | 20.0 | Drop AprilTag detections below this margin. |
| `--mad-threshold` | 3.0 | Per-burst outlier rejection threshold (sigma-equivalent). |
| `--ransac-residual-threshold` | 0.5 | RANSAC inlier threshold (mm). |
| `--visualize-detections` | False | Save per-image detection overlays. |

After fitting, `run-spotlight` loads `model/calibration_result.yaml` automatically.
