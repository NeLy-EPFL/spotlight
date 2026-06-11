# `run-spotlight`

`run-spotlight` is the main GUI for recording behavior (and optionally muscle)
data. It is implemented across `recorder/src/apps/run_spotlight_main.cc` and
`gui.cc`.

## Prerequisites

- Arena registration complete: `<arena_dir>/model/calibration_result.yaml` must
  exist (see [Arena registration](../configuration/arena_registration.md)).
- For muscle imaging: a `muscle_camera_roi.yaml` in the profile (from
  [`align-cameras`](align_cameras.md)).

## Starting the application

```bash
./run-spotlight -p ~/Spotlight/profiles/default -a ~/Spotlight/arenas/arena146
```

Both `-p` (profile, with `recorder_config.yaml` and `muscle_camera_roi.yaml`) and
`-a` (arena, with `metadata.yaml` and `model/calibration_result.yaml`) are required.
`-v` / `--verbosity` control logging. The application reads the arena files
automatically and exits with an error if either is missing or malformed.

## Configure recording parameters in the GUI

The recording parameters are set inline in the main window. The numeric fields are
*buffered* — they are read when a recording starts and have no live effect on the
streaming preview:

- **Behavior FPS (Hz)** — behavior-camera frame rate during recording.
- **Behavior exposure time (ms)** — behavior-camera exposure.
- **Behavior FPS : muscle FPS** — the sync ratio; the muscle camera images once
  every N behavior frames. Editable only while muscle imaging is enabled.
- **Muscle exposure (light-on) time (ms)** — the effective muscle exposure.
  Editable only while muscle imaging is enabled.

The window is laid out as three preview columns, each with a title and (where
applicable) a checkbox right-aligned to the preview's right edge:

- **Behavior preview** — the live behavior camera image.
- **Muscle preview** — the live muscle camera image, with an **Enable** checkbox.
  When checked, both cameras record and the muscle preview, histogram, and slider
  become active; when unchecked, only the behavior camera records. The muscle-only
  fields above are enabled/disabled together with this checkbox.
- **Stage position** — a motion-stage position indicator, with a **Tracking**
  checkbox. When tracking is on, the stage keeps the fly centered.

Below the previews are the save-directory selector, the
[experiment protocol](#experiment-protocol-string) field, and
**Start / Stop recording**.

### Muscle preview histogram and range slider

Directly under the muscle preview is a histogram with a two-handle range slider.
It is shown only while muscle imaging is enabled. The histogram displays the
intensity distribution of the latest raw 16-bit muscle frame, updated live.

The two handles set the `[vmin, vmax]` window used to map the 16-bit frame to the
8-bit preview: the blue handle (labelled `min`) is the lower bound and the orange
handle (labelled `max`) is the upper bound. Pixels at or below `vmin` display as
black and pixels at or above `vmax` display as white; the regions of the histogram
outside the window are dimmed. Drag a handle to adjust its value — clicking nearer
the min handle drags the min, nearer the max handle drags the max.

This only affects how the preview is displayed; it does not change the raw frames
written to disk (muscle images are always saved as full 16-bit TIFFs). The
handles' default positions and the histogram's value range come from
`default_display_vmin` / `default_display_vmax` and `histogram_display_min` /
`histogram_display_max` in the profile's `recorder_config.yaml`.

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
