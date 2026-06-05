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

The desktop components (`recorder` and `comm_protocol`) are built with CMake. Each component is configured into its own `build/` directory. CMake fetches the `ArduinoJson` dependency (and, for the tests, `GoogleTest`) automatically on first configuration, so an internet connection is needed then.

**`comm_protocol`** (the shared protocol library and its unit tests):

```sh
cmake -S comm_protocol -B comm_protocol/build
cmake --build comm_protocol/build
ctest --test-dir comm_protocol/build --output-on-failure  # run the unit tests
```

The `comm_protocol` unit tests (`comm_protocol/tests/`) are written with GoogleTest, which CMake fetches via `FetchContent` when the library is configured standalone.

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

The firmware also has on-device unit tests (`trigger_firmware/test/`) written with [AUnit](https://github.com/bxparks/AUnit). They are run by PlatformIO on the connected board, which is uploaded to and then prints the results over serial. There are three suites:

- `test_device_io`: the cached output-state bookkeeping in `DeviceIO` (idempotent setters, the `OptoChannel::ALL` aggregation, `reset()`).
- `test_serial_io`: the line framing in `SerialIO` (CR tolerance, blank-line skipping, oversized-line dropping, and lines reassembled across `update()` calls). Because the real serial port is busy carrying the test results back to the host, these feed bytes through an in-memory `Stream` injected into `SerialIO` rather than the hardware port.
- `test_protocol`: an on-target smoke check of the `comm_protocol` library (parse/serialize round-trips); the thorough coverage lives in the desktop GoogleTest suite below.

```sh
pio test -d trigger_firmware                       # all test suites
pio test -d trigger_firmware -f test_serial_io     # one suite
```

PlatformIO installs AUnit automatically (it is listed in `platformio.ini`'s `lib_deps`). Because AUnit is not PlatformIO's built-in Unity framework, `platformio.ini` sets `test_framework = custom`; each test program drives `aunit::TestRunner` from its own `setup()`/`loop()`. The firmware sources are compiled into the test programs (`test_build_src = yes`), and `src/main.cc` excludes its `setup()`/`loop()` from test builds via `#ifndef PIO_UNIT_TESTING`.

The Arduino Nano ESP32 has a native USB CDC serial port that re-enumerates every
time the board resets after a flash. In the combined `pio test` flow PlatformIO
reopens the port to read results immediately after upload, and on macOS that open
can lose the race against the re-enumeration:

```
[Errno 16] could not open port /dev/cu.usbmodem...: Resource busy
================== 0 test cases: 0 succeeded ==================
```

When this happens the suite collects no test cases. Beware that this is *not* a
pass: PlatformIO reports such a zero-case suite as `SKIPPED` in the summary table
(or prints `[PASSED]` with "0 test cases" on the per-suite line) -- neither means
the tests ran. (A suite excluded by a `-f` filter is also shown as `SKIPPED`, but
with a blank duration since it was never built.) To get reliable results, split
the upload from the serial read so the read phase opens the port after the board
has re-enumerated but while it is still inside its startup delay (the test
programs wait `kTestHostConnectDelayMs`, 20 s, at boot for exactly this):

```sh
pio test -d trigger_firmware -f test_protocol --without-testing                     # build + upload only
sleep 6                                                                             # let the USB CDC re-enumerate
pio test -d trigger_firmware -f test_protocol --without-building --without-uploading # read results only
```

The read phase must start *while the board is still in its 20 s startup delay*:
the upload plus `sleep 6` uses about half of it, which leaves ample margin. If the
read phase still reports `0 test cases`, the board finished its delay before the
port opened -- just rerun both commands (or lower the `sleep` to `3`). This race
is per upload, so a plain `pio test` (no `-f`) hits it on `test_protocol` too;
use the split sequence regardless of whether you filter. For thorough protocol
coverage there is also the desktop GoogleTest suite (`comm_protocol/tests/`, run
with `ctest`, see above); the on-device `test_protocol` is only an on-target
smoke check of the same library.

The repository root also has an umbrella `CMakeLists.txt` that configures the desktop components together (`cmake -S . -B build`); the per-component builds above are the usual workflow.


## Developer info

- Use C++20 (highest standard supported by `arduino-esp32 3.x` by default) for `trigger_firmware` and `comm_protocol`. Use C++23 for `recorder`.
- Use code style specified in `.clang-format`. Don't use `clang-tidy`.
- Prefer low overhead for `trigger_firmware`.
- Use snake case and suffices `.cc` and `.h`.
- Use `#pragma once` instead of `#ifndef` guards in header files.
- Use CMake and PlatformIO. `recorder` and `trigger_firmware` compile on their own. `comm_protocol` is a library included by both, but it can also be configured standalone to build and run its unit tests (see [Building](#building)).
- Use only ASCII characters except in `.md` files.