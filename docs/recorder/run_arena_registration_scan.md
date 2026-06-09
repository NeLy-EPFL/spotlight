# `run-arena-registration-scan`

`run-arena-registration-scan` performs the automated scan that feeds the
(stage, pixel) ↔ physical coordinate fit. It is implemented in
`recorder/src/apps/run_arena_registration_scan_main.cc`. This page covers the
program and its output; for the end-to-end procedure (including the Python fit) see
[Arena registration](../configuration/arena_registration.md).

## Prerequisites

1. The arena directory must contain `metadata.yaml`, `mapping_board.pdf`, and
   `active_area.png` (see [Profiles and arenas](../configuration/profiles_and_arenas.md)).
2. `mapping_board.pdf` printed at 100% scale and placed in the arena.
3. The recorder built ([Building and installing](../setup/building.md)). This binary
   additionally requires `libdmtx` (`sudo apt install libdmtx-dev`).

## Usage

```bash
./run-arena-registration-scan -p ~/Spotlight/profiles/sibo_260514 -a ~/Spotlight/arenas/arena146
```

Both `-p` (profile) and `-a` (arena) are required. `-v` / `--verbosity` control
logging.

### Step 1 — DataMatrix alignment

A live behavior-camera preview opens with crosshairs. Move the stages so the
DataMatrix barcode at the center of the board is centered under the crosshairs, then
press **ENTER**. The program decodes the DataMatrix and verifies its checksum
against `metadata.yaml`; a mismatch (e.g. the wrong board) aborts with an error.

### Step 2 — Automated AprilTag scan

The program visits every AprilTag position in `metadata.yaml` in order. At each it:

1. moves the stage to the tag's arena coordinates (using the offset from the
   DataMatrix step);
2. waits for the stages to settle
   (`motion_control/apriltag_mapping_settling_frames` in `recorder_config.yaml`);
3. acquires 10 consecutive frames;
4. writes `apriltag<id>_img<i>.jpg` and appends a row to
   `apriltag_stage_positions.csv` under `<arena_dir>/mapping_scan/`.

Progress is shown in the preview window. For arena146 (8 tags × 10 frames) this
takes a few minutes.

### Step 3 — Fit the model

```bash
fit-arena-registration -a ~/Spotlight/arenas/arena146
```

See [`fit-arena-registration`](../python-tools/fit_arena_registration.md).

## Output format

All output goes to `<arena_dir>/mapping_scan/`.

### AprilTag images — `apriltag<id>_img<i>.jpg`

- `<id>` — integer AprilTag ID, matching the keys under `apriltag_positions` in
  `metadata.yaml`.
- `<i>` — zero-based image index within the burst (0–9).

Each file is a single-channel 8-bit JPEG saved at maximum quality
(`IMWRITE_JPEG_QUALITY=100`), processed with `reorientBehaviorImage` (rotate 90°
CCW, then horizontal flip) to match the live preview. Ten consecutive frames are
acquired per tag; the first frame after the stage stops is discarded (so frames
exposed during deceleration are excluded).

### Stage positions — `apriltag_stage_positions.csv`

One row per image, in ascending tag-ID order.

| Column | Type | Description |
|---|---|---|
| `apriltag_id` | integer | AprilTag ID |
| `image_id` | integer | Zero-based index within the burst (0–9) |
| `stage_x_mm` | float (6 d.p.) | X stage position at image readout, in mm |
| `stage_y_mm` | float (6 d.p.) | Y stage position at image readout, in mm |

The stage position is read immediately after each frame is received, while the
stage is stationary.

```
apriltag_id,image_id,stage_x_mm,stage_y_mm
0,0,42.318400,17.056100
0,1,42.318600,17.056200
...
7,9,138.201300,8.043700
```
