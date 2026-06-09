# Arena registration

Arena registration fits the mapping between three coordinate systems — behavior
camera pixels, translation-stage positions, and physical arena coordinates — so the
recorder knows where the fly is in the arena and the post-processing can place each
frame. It is a two-program procedure: a scan
([`run-arena-registration-scan`](../recorder/run_arena_registration_scan.md), C++)
followed by a fit ([`fit-arena-registration`](../python-tools/fit_arena_registration.md),
Python).

> [!IMPORTANT]
> **Prerequisites**
> - Read [Profiles and arenas](profiles_and_arenas.md) and set up the arena
>   directory (it must contain `metadata.yaml`, `mapping_board.pdf`,
>   `active_area.png`).
> - Build the recorder ([Building and installing](../setup/building.md)).
>   `run-arena-registration-scan` additionally needs `libdmtx`
>   (`sudo apt install libdmtx-dev`).
> - Install the Python tools (`spotlight-tools`).

## Procedure

### Step 1 — Generate and print the mapping board

Generate the mapping board for your arena (`mapping_board.pdf`) — see
[Profiles and arenas → Generate the arena config assets](profiles_and_arenas.md#step-2--generate-the-arena-config-assets).
Print it at **100% scale** (no fit-to-page), cut along the outer border, and place
it in the arena with the printed side facing the camera.

### Step 2 — Run the registration scan

```bash
./run-arena-registration-scan \
    -p ~/Spotlight/profiles/sibo_260514 \
    -a ~/Spotlight/arenas/arena146
```

1. A live behavior-camera preview opens with crosshairs. Move the stages so the
   DataMatrix barcode (at the center of the board) is centered under the crosshairs,
   then press **ENTER**.
2. The program decodes the DataMatrix and verifies its checksum against
   `metadata.yaml`. A mismatch (e.g. the wrong board in the arena) aborts with an
   error.
3. The stage automatically visits each AprilTag, acquires 10 frames per tag, and
   writes everything to `~/Spotlight/arenas/arena146/mapping_scan/`.

The scan takes a few minutes (8 AprilTags × 10 frames for arena146). See the
[`run-arena-registration-scan` page](../recorder/run_arena_registration_scan.md) for
details and the exact output format.

### Step 3 — Fit the registration model

```bash
fit-arena-registration -a ~/Spotlight/arenas/arena146
```

This writes `model/calibration_result.yaml`, `model/calibration_points.csv`, and
`model/diagnostics.png` to the arena directory.

Open `model/diagnostics.png` and check the fit quality: RMSE should be ≲ 0.1 mm and
R² ≈ 1.0 (see [Quality targets](#quality-targets)). See the
[`fit-arena-registration` page](../python-tools/fit_arena_registration.md) for
options.

### Step 4 — Verify with the recorder

```bash
./run-spotlight \
    -p ~/Spotlight/profiles/sibo_260514 \
    -a ~/Spotlight/arenas/arena146
```

`run-spotlight` reads `model/calibration_result.yaml` and `metadata.yaml` from the
arena directory automatically. If either is missing or malformed, it exits with an
error.

## Algorithm

Internals of the registration fit.

Let $x_p, y_p$ be the **p**hysical coordinates of a point in the recording area
(mm); $x_t, y_t$ the positions of the **t**ranslation stages (mm); and $c_b, r_b$
the column (pixel-x) and row (pixel-y) coordinates of that point as seen by the
**b**ehavior camera (px).

Registration establishes two mappings for the behavior camera:

1. Stage + pixel → physical: $f_b:\ (x_t, y_t, c_b, r_b) \rightarrow (x_p, y_p)$
2. Stage + physical → pixel: $g_b:\ (x_t, y_t, x_p, y_p) \rightarrow (c_b, r_b)$

### Strategy

1. **Arena spec.** The arena PDF (`arena146_spec.pdf`) encodes the physical
   (arena-space) positions of all AprilTag corners and the DataMatrix center, in
   mm. `ArenaConfig` parses this and writes `metadata.yaml`.
2. **Mapping board.** `make_arena146_config.py` generates a printable
   `mapping_board.pdf` with AprilTag markers at the spec's locations, plus a
   DataMatrix barcode encoding an 8-character checksum of the full spec.
3. **Registration scan.** `run-arena-registration-scan`:
   - decodes the DataMatrix to verify the correct board is mounted (checksum must
     match `metadata.yaml`);
   - computes the stage-to-arena offset from the stage position when the camera is
     centered on the DataMatrix;
   - moves the stage to each AprilTag center, acquires 10 consecutive frames, and
     records the stage position. Output: `mapping_scan/apriltag<id>_img<i>.jpg` and
     `apriltag_stage_positions.csv`.
4. **Detection.** `fit-arena-registration` detects AprilTag corners in each image
   with `pupil_apriltags` (family `tag16h5`). Each corner yields a tuple
   $(x_t, y_t, c_b, r_b, x_p, y_p)$, with $(x_p, y_p)$ known from `metadata.yaml`.
   Detections with a low decision margin are discarded (false positives are common
   with tag16h5).
5. **Outlier rejection.** Within-burst outliers (the 10 frames per tag are
   near-identical because the stage is stationary) are removed by MAD filtering per
   (tag, corner) group. Overall outliers are then removed by RANSAC.
6. **Linear fit.** Assuming an affine mapping, fit $A_{f_b} \in \mathbb{R}^{2\times5}$:

   $$\begin{pmatrix} x_p \\ y_p \end{pmatrix} = A_{f_b} \begin{pmatrix} x_t \\ y_t \\ c_b \\ r_b \\ 1 \end{pmatrix}$$

   Two RANSAC regressors are fit (one per axis); only combined inliers are used for
   the final least-squares estimate.
7. **Inverse.** $g_b$ is derived analytically by partially inverting $A_{f_b}$: with
   $A_{f_b} = [\,A_{\rm stage}\ \ A_{\rm pix}\ \ b\,]$, the pixel block $A_{\rm pix}$
   is $2\times2$ and (in practice) invertible, so

   $$\begin{pmatrix} c_b \\ r_b \end{pmatrix} = A_{\rm pix}^{-1}\left[\begin{pmatrix} x_p \\ y_p \end{pmatrix} - A_{\rm stage}\begin{pmatrix} x_t \\ y_t \end{pmatrix} - b\right].$$

   Both matrices are saved in `calibration_result.yaml`, in the schema read by
   `CalibrationParams` in the C++ recorder.

### Quality targets

A good fit has:

- RMSE ≲ 0.1 mm (Euclidean)
- R² ≈ 1.0 on both axes
- Very few RANSAC outliers (< 5%)

Check `model/diagnostics.png` after fitting to verify the residual distribution and
spatial pattern.

### Differences from the old ArUco calibration

The previous pipeline used an ArUco board scanned on a dense grid, detected with
OpenCV, and fitted per-camera. The current pipeline uses:

- a sparse set of AprilTag markers (8 for arena146) at known positions in the PDF
  spec;
- `pupil_apriltags` for detection (more robust than `cv2.aruco` for tag16h5);
- per-burst MAD filtering to exploit the repeated measurements per tag;
- a DataMatrix checksum to guard against mismatched boards;
- a single behavior-camera model (the muscle camera is aligned via a separate
  [camera homography](camera_homography.md) step, *TODO*).
