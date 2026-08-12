# Spotlight controller

This repository contains the control software for the Spotlight system presented in [Wang-Chen et al. (2026), Precise kinematic and muscle recording in freely behaving flies enabled by closed-loop tracking and annotation-free pose estimation
](https://go.epfl.ch/spotlight-poseforge). The firmware for the triggering microcontroller is also included.

The closed-loop control and recording programs implemented in C++. A set of offline tools for calibration, postprocessing, etc. is implemented in Python and lives in [`python/`](python/). The C++ codebase contains three parts:

- **`recorder/`:** The main recorder software that runs on the recording computer.
    - User-facing programs: `align-cameras`, `run-arena-registration-scan`, `run-spotlight`, and `run-charuco-homography-scan` (see [`docs/developers_manual/camera_homography.md`](docs/developers_manual/camera_homography.md)).
        - `align-cameras`: (i) _mechanically_ align the two cameras and the blue excitation light, and (ii) define a cropped ROI on the muscle camera to align both fields of view.
        - `run-arena-registration-scan`: move the translation stages to predefined positions and capture fiducial-marker snapshots, used by a Python tool to build a camera–stage–arena coordinate mapping.
        - `run-charuco-homography-scan`: scan a grid of stage positions to collect paired behavior/muscle images, used by the Python `fit-homography` tool to fit the behavior-to-muscle camera homography.
        - `run-spotlight`: the main program to record experimental data.
    - The behavior (Euresys) camera runs in-process; the muscle (PCO) camera runs as a separate process (`pco-camera-server`) communicating over shared memory. See [`docs/developers_manual/architecture.md`](docs/developers_manual/architecture.md) for details.
- **`trigger_firmware/`**: The embedded code that runs on the triggering microcontroller.
- **`comm_protocol/`**: A lightweight JSON-based library for serial communication between the recorder and the microcontroller. Designed to be included by both sides; avoids exceptions to respect the microcontroller's linear workflow.

The repository also contains:

- **`python/`**: Offline Python tools for calibration and postprocessing (`fit-arena-registration`, `postprocess-recording`, etc.). See `python/README.md`.
- **`trigger_hardware/`**: KiCad schematic and PCB layout for the triggering microcontroller circuit board.


## Documentation & developer info

Documentation is available in the `docs/` folder. Start at the index,
[`docs/README.md`](docs/README.md), which splits the pages into a short **User's
manual** (running an experiment end to end) and a fuller **Developer's manual**
(internals, environment setup, hardware). A few entry points:

- [Architecture overview](docs/developers_manual/architecture.md): threads, cameras, and the trigger microcontroller.
- [Data acquisition workflow](docs/developers_manual/data_acquisition.md): how the cameras and lights are controlled and synchronized using continuous acquisition.
- [Installation and compilation](docs/developers_manual/installation_compilation.md): how to build the components and install their dependencies.
- [Troubleshooting](docs/users_manual/troubleshooting.md): common runtime problems (e.g. the behavior camera grabber being held by another program, and hangs on quit).

- Code style and conventions (language standards, formatting, naming, overhead, tests)
are documented in [`docs/developers_manual/code_style.md`](docs/developers_manual/code_style.md).
