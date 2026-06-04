# Spotlight controller

This repository contains the control software for the Spotlight system presented in [Wang-Chen et al. (2026), Precise kinematic and muscle recording in freely behaving flies enabled by closed-loop tracking and annotation-free pose estimation
](https://go.epfl.ch/spotlight-poseforge). The firmware for the triggering microcontroller is also included.

This codebase is entirely implemented in C++. A set of tools for calibration, postprocessing, etc. is implemented in Python. These tools are available in [a separate repository](https://github.com/NeLy-EPFL/spotlight-tools). The C++ codebase contains three parts:

- **`recorder/`:** The main recorder software that runs on the recording computer.
    - The recorder contains four user-facing programs:
        - `align-cameras`: a tool for the user to (i) _mechanically_ align the two cameras and the beam of the blue excitation light, and (ii) define a cropped ROI within the full sensor area of the muscle camera to precisely align the fields of view of the two cameras.
        - `run-arena-registration-scan`: a short program where (i) the translation stages move the cameras to predefined positions, and (ii) the behavior camera captures snapshots of fiducial markers at these positions. These images allow a separate Python program to build a mapping between camera pixel coordinates, translation stage physical coordinates, and physical coordinates in the behavior arena.
        - `run-homography-scan`: a short program that captures images of a high-resolution ChArUco board, which enable a separate Python program to fit the camera homography matrices for both cameras.
        - `run-spotlight`: the main program to record experimental data.
    - Additionally, the recorder contains an internal `pco-camera-server` program that acquires images using the muscle camera and makes them available to the other programs via shared memory. Muscle camera acquisition is implemented as a stand-alone server because it requires a different software stack, which makes compiling it with the other programs unnecessarily complicated.
- **`trigger_firmware/`**: The embedded code that runs on the triggering microcontroller.
- **`comm_protocol/`**: A JSON-based protocol for serial communication between the recorder and the microcontroller implemented as a light library.
    - This library is meant to be included by both the recorder and the microcontroller. Therefore, it needs to be especially efficient. It must avoid using exceptions to respect the linear workflow on the microcontroller.
    - Serializers and deserializers are included in this library.


## Documentation

Documentation is available in the `docs/` folder:

- [Hardware](docs/hardware.md): hardware to be controlled by this controller.
- [Data acquisition workflow](docs/data_acquisition.md): how the cameras and lights are controlled and synchronized using continuous acquisition.
- [Recorder-microcontroller communication protocol](docs/comm_protocol.md): JSON protocol for communication between the recorder and the triggering microcontroller.


## Building

The desktop components (`recorder` and `comm_protocol`) are built with CMake. Each component is configured into its own `build/` directory. CMake fetches the `ArduinoJson` dependency automatically on first configuration, so an internet connection is needed then.

**`comm_protocol`** (the shared protocol library and its unit tests):

```sh
cmake -S comm_protocol -B comm_protocol/build
cmake --build comm_protocol/build
ctest --test-dir comm_protocol/build --output-on-failure  # run the unit tests
```

**`recorder`** (also builds the `comm_protocol` dependency):

```sh
cmake -S recorder -B recorder/build
cmake --build recorder/build
```

**`trigger_firmware`** is built and uploaded with PlatformIO:

```sh
pio run -d trigger_firmware            # build
pio run -d trigger_firmware -t upload  # flash the microcontroller
```

The repository root also has an umbrella `CMakeLists.txt` that configures the desktop components together (`cmake -S . -B build`); the per-component builds above are the usual workflow.


## Developer info

- Use C++20 (highest standard supported by `arduino-esp32 3.x` by default) for `trigger_firmware` and `comm_protocol`. Use C++23 for `recorder`.
- Use code style specified in `.clang-format`. Don't use `clang-tidy`.
- Prefer low overhead for `trigger_firmware`.
- Use snake case and suffices `.cc` and `.h`.
- Use `#pragma once` instead of `#ifndef` guards in header files.
- Use CMake and PlatformIO. `recorder` and `trigger_firmware` compile on their own. `comm_protocol` is a library included by both, but it can also be configured standalone to build and run its unit tests (see [Building](#building)).
- Use only ASCII characters except in `.md` files.