# Doing experiments

**Tl;dr: run `source init-spotlight.sh` and follow on-screen instructions.** This activates the Python virtual environment for the tools in `tools/` and remembers which profile and arena directories to use.

Alternatively, run `align-cameras`, `run-arena-registration-scan`, and `run-spotlight` with arguments specifying the profile and arena directories: e.g., `run-spotlight -p ~/Spotlight/profiles/default -a ~/Spotlight/arenas/arena146x146`. Activate the Python virtual environment manually (`tools/`) before running `fit-arena-registration`.

---

## Step 1: Align cameras

> [!IMPORTANT]
> Muscle imaging only. Skip this step if you are recording behavior only.

Aligns the behavior camera, muscle camera FOVs, and the blue excitation light spot. Redo this step if cameras or the LED are physically moved.

1. Place the bullseye target on the arena (corner arrow aligned with the arrow on the arena holder) and put the acrylic plate on top.
2. Move the stages (Zaber knobs) so the bullseye is centered in the behavior camera view.
3. Physically adjust the blue LED (position, distance, lenses) so it is centered on the bullseye at an appropriate spot size (use the radius markers in the muscle window).
4. Physically adjust the muscle camera to roughly align to the bullseye.
5. If the stage moved during steps 3–4, re-center the behavior camera on the bullseye.
6. Click the center of the bullseye in the muscle window. A red dot marks the click; a blue rectangle shows the nearest feasible FOV (PCO camera requires boundaries at multiples of 32 px × 8 px).
7. Verify the blue rectangle is fully within the displayed image; physically adjust the muscle camera if not.
8. Press **RETURN** to save. Confirm `muscle_camera_roi.yaml` was written to the profile directory.


## Step 2: Run arena registration scan and fit arena registration model

> [!IMPORTANT]
> Prerequisite: `mapping_board.pdf` must be printed at 100% scale and placed in the arena. See [Arenas](custom_arena.md).

Calibrates the mapping between camera pixels, stage positions, and physical arena coordinates. Redo if the arena or camera setup changes.

1. Run `run-arena-registration-scan`. A live preview opens with crosshairs. Move the stages so the DataMatrix barcode (center of the board) is roughly centered under the crosshairs, then press **ENTER**. The program verifies the board matches the arena metadata, then automatically visits each AprilTag (a few minutes for arena146x146).
2. Fit the registration model by running `fit-arena-registration`. This writes `model/calibration_result.yaml` and `model/diagnostics.png` to the arena directory. Check the diagnostics: a good fit has RMSE ≲ 0.1 mm and R² ≈ 1.0. See [Arena registration](../developers_manual/arena_registration.md) for details.


## Step 3: Collect experimental recordings

Run `run-spotlight`. In the GUI, configure:

- **Recording parameters** (top): behavior FPS, exposure time, and (if muscle imaging) sync ratio and muscle exposure. These take effect at the start of the next recording.
- **Muscle preview**: enable with the **Enable** checkbox. When enabled, both cameras record.
- **Save directory**: set before starting. If the directory exists and is non-empty, you will be prompted to overwrite, auto-increment, or cancel.
- **Experiment protocol** (optional): a `;`-separated string of `<frameIdx>/<channel>/<op>` steps for scheduled optogenetic channel switching and automatic stop. "Channel" can be `ch2` or `ch3` reflecting to channels 2 and 3 on the CCS light controller (channel 1 is already occupied by the IR illumination LED). "Op" can be `on` (switching light on), `off` (switching light off), or `stop` (stopping recording). For `stop`, set the channel to `x` as the operation is global. Leave the textbox blank for an open recording, where the user clicks "Stop" manually to stop recording. Valid examples of the experiment protocol string are:
  - `900/x/stop` — record 900 behavior frames, then stop.
  - `300/ch2/on;600/ch2/off;900/x/stop` — turn channel 2 on at frame 300, off at 600, stop at 900.

Then, click **Start recording**. Click **Stop** when done (or let the protocol stop automatically).

### Recording directory layout

```
<save_dir>/
├── behavior_images/         # behavior_frame_*.jpg  (pseudo-BGR JPEGs)
├── muscle_images/           # muscle_frame_*.tif    (16-bit TIFF; muscle imaging only)
├── stage_position/
│   └── stage_position.csv
└── metadata/
    ├── experiment_parameters.yaml
    ├── recorder_config.yaml
    └── calibration_parameters_behavior.yaml
```

Behavior frames are stored as *pseudo-BGR* JPEGs (three consecutive monochrome frames packed into the three color channels). `postprocess-recording` unpacks them.


## Step 4: Postprocessing

```bash
postprocess-recording --recording-dir <save_dir>/
```

The pipeline:
1. Interpolates stage positions at behavior-frame timestamps.
2. Unpacks behavior frames and encodes them into an H.264 video; merges per-frame timestamps into a single CSV.
3. Runs SLEAP pose estimation, then rotates and crops each frame so the fly is centered and upright (when `--align-fly`, the default).
4. Warps and aligns muscle frames to match (when `--with-muscle`).
5. Generates a summary video for inspection (when `--make-visualizations`, the default).

Run `postprocess-recording --help` for full descriptions of the allowed options. Output is written to `<save_dir>/processed/`.
