# `run-spotlight`

`run-spotlight` is the main GUI for recording behavior (and optionally muscle)
data. It is implemented across `recorder/src/apps/run_spotlight_main.cc` and
`gui.cc`.

## Prerequisites

- Arena registration complete: `<arena_dir>/model/calibration_result.yaml` must
  exist (see [Arena registration](../configuration/arena_registration.md)).
- For muscle imaging: a `muscle_camera_roi.yaml` in the profile (from
  [`align-cameras`](align_cameras.md)).

> [!NOTE]
> The muscle (PCO) camera is driven **in-process**; there is no separate
> `pco-camera-server` to launch. See the [architecture overview](../architecture.md).

## Starting the application

```bash
./run-spotlight -p ~/Spotlight/profiles/sibo_260514 -a ~/Spotlight/arenas/arena146
```

Both `-p` (profile, with `recorder_config.yaml` and `muscle_camera_roi.yaml`) and
`-a` (arena, with `metadata.yaml` and `model/calibration_result.yaml`) are required.
`-v` / `--verbosity` control logging. The application reads the arena files
automatically and exits with an error if either is missing or malformed.

## Startup and main window

A configuration dialog sets the recording parameters before the main window opens:

- **Record muscle** (both cameras) or behavior only.
- **Behavior FPS** and exposure time.
- **Muscle sync ratio** (the muscle camera images once every N behavior frames).
- **Muscle light-on time** (the effective muscle exposure).

The main window shows live previews (behavior, and muscle if enabled), a
motion-stage position indicator, a tracking on/off toggle (when on, the stage keeps
the fly centered), a save-directory selector, the
[experiment protocol](#experiment-protocol-string) field, and
**Start / Stop recording**.

## Recording

1. Set the save directory. (If it exists and is non-empty, you'll be prompted to
   overwrite, auto-increment to a free directory, or cancel.)
2. Optionally enter an [experiment protocol string](#experiment-protocol-string).
   Leave it empty for an open recording.
3. Click **Start recording**.

On start, the recorder writes the following metadata alongside the images:

- `metadata/experiment_parameters.yaml` — the chosen parameters (behavior FPS,
  exposures, sync ratio, light-on time, the protocol string). When muscle imaging
  is enabled it also holds the derived continuous-mode timing
  (`muscle_nominal_exposure_us`, `muscle_buffer_time_us`).
- `metadata/recorder_config.yaml` — a snapshot of the profile's
  `recorder_config.yaml`.
- `metadata/calibration_parameters_behavior.yaml` — a snapshot of the behavior-camera
  calibration.

> [!NOTE]
> The recorder no longer writes `calibration_parameters_muscle.yaml` (the muscle
> mapping moves to a separate [camera homography](../configuration/camera_homography.md)
> step, *TODO*).

## Experiment protocol string

A recording can run *open* (until you click Stop) or follow a *scheduled* protocol
that switches optogenetic channels and stops at preset frame counts. The schedule is
entered as a single string in the protocol field.

### Syntax

A protocol is a `;`-separated list of steps. Each step is:

```
<frameIdx>/<channel>/<op>
```

- `<frameIdx>` — non-negative integer: the step is applied *after* this many
  behavior frames have been collected.
- `<channel>` / `<op>`:
  - To switch an optogenetic channel: `<channel>` is `ch2` or `ch3` (channel 1 is
    reserved for the IR LED), and `<op>` is `on` or `off`.
  - To end the recording: `<channel>` is `x` and `<op>` is `stop`.

Rules:

- If **any** steps are given, the protocol must contain **exactly one** `x/stop`
  step, and it must be the **very last** step.
- Do **not** add a trailing `;`.
- An **empty** field (or a single `;`) means an *open* recording — no programmed
  stop; it runs until you press Stop.

Multiple actions may occur at the same frame, as long as steps are listed in
non-decreasing `frameIdx` order.

> [!NOTE]
> Internally the recorder parses this string into the `opSequence` of a
> `START_RECORDING` command (channels map to `2`/`3`, the stop step to the global
> channel `-1`). See the [communication protocol](../comm_protocol.md).

### Examples

| String | Meaning |
|---|---|
| *(empty)* | Open recording; stop manually. |
| `900/x/stop` | Record 900 behavior frames, then stop. |
| `300/ch2/on;600/ch2/off;900/x/stop` | Turn channel 2 on after frame 300, off after 600, stop after 900. |
| `100/ch2/on;200/ch2/off;200/ch3/on;300/ch3/off;300/x/stop` | Two channels overlapping; multiple actions share a frame index. |

## Recording directory layout

```
<save_dir>/
├── behavior_images/         # behavior_frame_*.jpg  (packed pseudo-BGR JPEGs)
├── muscle_images/           # muscle_frame_*.tif    (16-bit TIFF; muscle imaging only)
├── stage_position/
│   └── stage_position.csv
└── metadata/
    ├── experiment_parameters.yaml
    ├── recorder_config.yaml
    └── calibration_parameters_behavior.yaml
```

Behavior frames are saved as *pseudo-BGR* JPEGs (three consecutive monochrome
frames packed into the three color channels) — a recording-time trade-off between
compression and speed that post-processing later unpacks. See
[Data acquisition](../data_acquisition.md).

## Post-processing

After recording:

```bash
postprocess-recording --recording-dir <save_dir>/
```

See [`postprocess-recording`](../python-tools/postprocess_recording.md).
