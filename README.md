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
    - Unlike a previous version, the muscle (PCO) camera is driven **in-process** by `run-spotlight` and `align-cameras` via the `MuscleCamera` class (`src/peripherals/muscle_camera.cc`), the same way the behavior (Euresys) camera is driven — a dedicated acquirer thread calls `MuscleCamera::waitForOneFrame()` in a loop. The PCO camera SDK is compiled with `PCO_LINUX` defined, which makes its headers inject Windows-compatibility typedefs and macros (`BOOL`, `WORD`, `DWORD`, `HANDLE`, `FALSE`/`TRUE`, `far`, ...) into the global namespace of every translation unit that includes a PCO header. Those names were verified not to clash with the Qt/OpenCV/Euresys headers, and `PCO_LINUX` is scoped **per file** (only `muscle_camera.cc` and the bundled PCO SDK sources) via a pimpl, so this pollution stays out of the GUI and tracking code. The PCO camera libraries are linked into `run-spotlight`/`align-cameras` directly; the system Qt lib dir is kept ahead of the PCO lib dir on the rpath so the bundled (and unused) Qt that ships in the PCO lib dir is never resolved. (Earlier versions ran the muscle camera as a separate `pco-camera-server` process that published frames over shared memory; that has been removed.)
- **`trigger_firmware/`**: The embedded code that runs on the triggering microcontroller.
- **`comm_protocol/`**: A JSON-based protocol for serial communication between the recorder and the microcontroller implemented as a light library.
    - This library is meant to be included by both the recorder and the microcontroller. Therefore, it needs to be especially efficient. It must avoid using exceptions to respect the linear workflow on the microcontroller.
    - Serializers and deserializers are included in this library.


## Documentation & developer info

Documentation is available in the `docs/` folder. Start at the index,
[`docs/README.md`](docs/README.md), which splits the pages into a short **User's
manual** (running an experiment end to end) and a fuller **Developer's manual**
(internals, environment setup, hardware). A few entry points:

- [Architecture overview](docs/architecture.md): threads, in-process cameras, and the trigger microcontroller.
- [Data acquisition workflow](docs/data_acquisition.md): how the cameras and lights are controlled and synchronized using continuous acquisition.
- [Building and installing](docs/setup/building.md) and [software dependencies](docs/setup/dependencies.md): how to build the components and install their dependencies.
- [Troubleshooting](docs/troubleshooting.md): common runtime problems (e.g. the behavior camera grabber being held by another program, and hangs on quit).

- Code style and conventions (language standards, formatting, naming, overhead, tests)
are documented in [`docs/code_style.md`](docs/code_style.md).