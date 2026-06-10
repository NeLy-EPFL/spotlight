# Spotlight documentation

Documentation for the Spotlight recording pipeline. The C++ recorder, the
trigger-microcontroller firmware, and the JSON communication protocol live in
this repository ([`spotlight-control`](https://github.com/NeLy-EPFL/spotlight-control)).
The offline Python tools live in the separate
[`spotlight-tools`](https://github.com/NeLy-EPFL/spotlight-tools) repository; their
usage is documented here under [python-tools/](python-tools/).

There are two reading paths. The **User's manual** is the short, task-oriented
subset needed to run an experiment end to end. The **Developer's manual** is a
superset that also covers internals, environment setup, and hardware.

---

## User's manual

A master's intern should be able to pick this up in a day. Read it in order.

- Configuration and calibration
  - [Profiles and arenas](configuration/profiles_and_arenas.md)
  - [Aligning the two cameras](recorder/align_cameras.md) (`align-cameras`) — muscle imaging only
  - [Arena registration](configuration/arena_registration.md) (`run-arena-registration-scan` + `fit-arena-registration`)
  - [Camera homography](configuration/camera_homography.md) — muscle imaging only *(TODO: not yet implemented)*
- Data collection
  - [Recording an experiment](recorder/run_spotlight.md) (`run-spotlight`), including
    [the experiment protocol string](recorder/run_spotlight.md#experiment-protocol-string)
- Post-processing
  - [Post-processing a recording](python-tools/postprocess_recording.md) (`postprocess-recording`)
  - [Visualizing the stage trajectory](python-tools/visualize_stage_trajectory.md) (`visualize-stage-trajectory`) — optional
- [Troubleshooting](troubleshooting.md)

---

## Developer's manual

Everything in the User's manual, plus:

- Architecture
  - [Architecture overview](architecture.md)
  - [Data acquisition and synchronization](data_acquisition.md)
  - [Recorder-to-microcontroller communication protocol](comm_protocol.md)
- Hardware
  - [Hardware](hardware.md)
  - [Camera frame-rate limits](frame_rate_limits.md)
- Environment setup
  - [Software dependencies](setup/dependencies.md)
  - [eGrabber and JAI camera configuration](setup/egrabber_jai_config.md)
  - [Building and installing](setup/building.md)
- Configuration and calibration
  - [Profiles and arenas](configuration/profiles_and_arenas.md)
  - [Arena registration](configuration/arena_registration.md) — procedure and algorithm
  - [Camera homography](configuration/camera_homography.md) *(TODO)*
- Recorder programs (`recorder/`)
  - [`align-cameras`](recorder/align_cameras.md)
  - [`run-arena-registration-scan`](recorder/run_arena_registration_scan.md)
  - `run-homography-scan` *(TODO: not yet implemented; see [Camera homography](configuration/camera_homography.md))*
  - [`run-spotlight`](recorder/run_spotlight.md)
    - [Experiment protocol string](recorder/run_spotlight.md#experiment-protocol-string)
  - [`reset-camera`](recorder/reset_camera.md) — recovery helper
- Python tools (`spotlight-tools`)
  - [`fit-arena-registration`](python-tools/fit_arena_registration.md)
  - [`postprocess-recording`](python-tools/postprocess_recording.md)
  - [`visualize-stage-trajectory`](python-tools/visualize_stage_trajectory.md)
- [Troubleshooting](troubleshooting.md)
- [Code style and conventions](code_style.md)
