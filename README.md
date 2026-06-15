# Spotlight controller

This repository contains the control software for the Spotlight system presented in [Wang-Chen et al. (2026), Precise kinematic and muscle recording in freely behaving flies enabled by closed-loop tracking and annotation-free pose estimation
](https://go.epfl.ch/spotlight-poseforge). The firmware for the triggering microcontroller is also included.

This codebase is entirely implemented in C++. A set of tools for calibration, postprocessing, etc. is implemented in Python. These tools are available in [a separate repository](https://github.com/NeLy-EPFL/spotlight-tools). The C++ codebase contains three parts:

- **`recorder/`:** The main recorder software that runs on the recording computer.
    - The recorder contains three user-facing programs (`align-cameras`, `run-arena-registration-scan`, and `run-spotlight`); a fourth, `run-homography-scan`, is planned but not yet implemented (see [`docs/configuration/camera_homography.md`](docs/configuration/camera_homography.md)):
        - `align-cameras`: a tool for the user to (i) _mechanically_ align the two cameras and the beam of the blue excitation light, and (ii) define a cropped ROI within the full sensor area of the muscle camera to precisely align the fields of view of the two cameras.
        - `run-arena-registration-scan`: a short program where (i) the translation stages move the cameras to predefined positions, and (ii) the behavior camera captures snapshots of fiducial markers at these positions. These images allow a separate Python program to build a mapping between camera pixel coordinates, translation stage physical coordinates, and physical coordinates in the behavior arena.
        - `run-spotlight`: the main program to record experimental data.
        - `run-homography-scan` _(planned; not yet implemented)_: a short program that will capture images of a high-resolution ChArUco board, which enable a separate Python program to fit the camera homography matrices for both cameras.
    - The behavior (Euresys) camera is driven **in-process** by its acquirer thread. The muscle (PCO) camera runs as a **separate process**, `pco-camera-server` (`src/apps/pco_camera_server_main.cc`): the `MuscleCamera` class (`src/peripherals/muscle_camera.cc`) `fork()`+`execl()`s the server, which opens the PCO panda 4.2 and publishes each frame over shared memory; `MuscleCamera::wait_for_one_frame()` blocks on a shared condition variable for the latest frame. Keeping the PCO SDK in its own process isolates the `PCO_LINUX` Windows-compatibility shims (`BOOL`, `WORD`, `DWORD`, `HANDLE`, ...) that its headers inject into the global namespace, confines the PCO link/runtime dependencies to one binary, and ensures a crash in the PCO SDK cannot destabilize the GUI process. See [`docs/architecture.md`](docs/architecture.md) for details. (An in-process refactor of the muscle camera was attempted and abandoned; the separate-process design is the one in use.)
- **`trigger_firmware/`**: The embedded code that runs on the triggering microcontroller.
- **`comm_protocol/`**: A JSON-based protocol for serial communication between the recorder and the microcontroller implemented as a light library.

The repository also contains the KiCad design files for the trigger circuit board:

- **`trigger_hardware/`**: KiCad schematic and PCB layout for the triggering microcontroller circuit board.
    - This library is meant to be included by both the recorder and the microcontroller. Therefore, it needs to be especially efficient. It must avoid using exceptions to respect the linear workflow on the microcontroller.
    - Serializers and deserializers are included in this library.


## Documentation & developer info

Documentation is available in the `docs/` folder. Start at the index,
[`docs/README.md`](docs/README.md), which splits the pages into a short **User's
manual** (running an experiment end to end) and a fuller **Developer's manual**
(internals, environment setup, hardware). A few entry points:

- [Architecture overview](docs/architecture.md): threads, the in-process behavior camera and out-of-process muscle camera, and the trigger microcontroller.
- [Data acquisition workflow](docs/data_acquisition.md): how the cameras and lights are controlled and synchronized using continuous acquisition.
- [Building and installing](docs/setup/building.md) and [software dependencies](docs/setup/dependencies.md): how to build the components and install their dependencies.
- [Troubleshooting](docs/troubleshooting.md): common runtime problems (e.g. the behavior camera grabber being held by another program, and hangs on quit).

- Code style and conventions (language standards, formatting, naming, overhead, tests)
are documented in [`docs/code_style.md`](docs/code_style.md).