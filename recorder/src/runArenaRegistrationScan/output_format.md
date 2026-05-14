# runArenaRegistrationScan output format

All output is written to `<arena_dir>/mapping_scan/`, where `<arena_dir>` is
the path passed to `-a`/`--arena`.

## Apriltag images

**Filename pattern:** `apriltag<id>_img<i>.jpg`

- `<id>` — integer apriltag ID, matching the keys in `metadata.yaml` under `apriltag_positions`.
- `<i>` — zero-based image index within the burst (0–9).

Each file is a single-channel (grayscale) 8-bit JPEG saved at maximum quality
(`IMWRITE_JPEG_QUALITY=100`). The image is rotated 90° counter-clockwise
relative to the raw sensor output, but is **not** horizontally flipped — it
represents the actual camera view. The live preview window applies an
additional horizontal flip for user comfort, but that flip is not applied to
saved images.

Ten consecutive frames are acquired per apriltag. The first frame after the
stage stops is discarded before the burst begins, so that frames exposed during
deceleration are not included.

## Stage position CSV

**Filename:** `apriltag_stage_positions.csv`

One row per image, covering all apriltags in ascending ID order.

| Column | Type | Description |
|---|---|---|
| `apriltag_id` | integer | Apriltag ID |
| `image_id` | integer | Zero-based index within the burst (0–9) |
| `stage_x_mm` | float (6 d.p.) | X translation stage position at the moment of image readout, in mm |
| `stage_y_mm` | float (6 d.p.) | Y translation stage position at the moment of image readout, in mm |

The stage position is read from the motion controller immediately after each
frame is received, while the stage is stationary.

### Example

```
apriltag_id,image_id,stage_x_mm,stage_y_mm
0,0,42.318400,17.056100
0,1,42.318600,17.056200
...
7,9,138.201300,8.043700
```
