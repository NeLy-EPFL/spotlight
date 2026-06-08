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
    - Additionally, the recorder contains an internal `pco-camera-server` program that acquires images using the muscle camera and makes them available to the other programs via shared memory. Muscle-camera acquisition is split into its own executable because the PCO camera SDK is compiled with `PCO_LINUX` defined, which makes its headers inject Windows-compatibility typedefs and macros (`BOOL`, `WORD`, `DWORD`, `HANDLE`, `FALSE`/`TRUE`, `far`, ...) into the global namespace of every translation unit that includes a PCO header. Those names happen not to clash with the Qt/OpenCV headers used elsewhere, but isolating the server keeps this pollution -- along with the PCO link/runtime dependencies -- out of the GUI and tracking code. The server is a separate process and publishes frames over shared memory rather than linking into `run-spotlight`.
- **`trigger_firmware/`**: The embedded code that runs on the triggering microcontroller.
- **`comm_protocol/`**: A JSON-based protocol for serial communication between the recorder and the microcontroller implemented as a light library.
    - This library is meant to be included by both the recorder and the microcontroller. Therefore, it needs to be especially efficient. It must avoid using exceptions to respect the linear workflow on the microcontroller.
    - Serializers and deserializers are included in this library.


## Documentation

Documentation is available in the `docs/` folder:

- [Hardware](docs/hardware.md): hardware to be controlled by this controller.
- [Data acquisition workflow](docs/data_acquisition.md): how the cameras and lights are controlled and synchronized using continuous acquisition.
- [Recorder-microcontroller communication protocol](docs/comm_protocol.md): JSON protocol for communication between the recorder and the triggering microcontroller.
- [Troubleshooting](docs/troubleshooting.md): common runtime problems (e.g. the behavior camera grabber being held by another program, and hangs on quit).


## Building

The desktop components (`recorder` and `comm_protocol`) are built with CMake. Each component is configured into its own `build/` directory. CMake fetches the ArduinoJson dependency (and, for the tests, GoogleTest) automatically on first configuration, so an internet connection is needed then.

### `comm_protocol`

To build the shared protocol library and its unit tests:

```sh
cmake -S comm_protocol -B comm_protocol/build
cmake --build comm_protocol/build -j 16
ctest --test-dir comm_protocol/build --output-on-failure  # run the unit tests
```

The `comm_protocol` unit tests (`comm_protocol/tests/`) are written with GoogleTest, which CMake fetches via `FetchContent` when the library is configured standalone.

### `recorder`

This also builds the `comm_protocol` dependency:

```sh
cmake -S recorder -B recorder/build
cmake --build recorder/build -j 16
ctest --test-dir recorder/build --output-on-failure  # run the unit tests
```

The `recorder` unit tests (`recorder/tests/`) are written with GoogleTest and cover the hardware-independent logic (calibration, config loading, image/metadata utilities, muscle-camera ROI). They build by default when `recorder` is configured standalone (as above); set `-DRECORDER_BUILD_TESTS=ON` to opt in from the umbrella build.

### `trigger_firmware`

This is built and uploaded with PlatformIO:

```sh
pio run -d trigger_firmware            # build
pio run -d trigger_firmware -t upload  # flash the microcontroller
```

The firmware also has on-device unit tests (`trigger_firmware/test/`) written with [AUnit](https://github.com/bxparks/AUnit). They are run by PlatformIO on the connected board, which is uploaded to and then prints the results over serial. The test suites are:

- `test_device_io`: the cached output-state bookkeeping in `DeviceIO` (idempotent setters, the `OptoChannel::ALL` aggregation, `reset()`).
- `test_serial_io`: the line framing in `SerialIO` (CR tolerance, blank-line skipping, oversized-line dropping, and lines reassembled across `update()` calls). Because the real serial port is busy carrying the test results back to the host, these feed bytes through an in-memory `Stream` injected into `SerialIO` rather than the hardware port.
- `test_protocol`: an on-target smoke check of the `comm_protocol` library (parse/serialize round-trips); the thorough coverage lives in the desktop GoogleTest suite below.

Run a suite by splitting the upload from the serial read, swapping `test_protocol`
for another suite name (or dropping `-f` to run all of them):

```sh
pio test -d trigger_firmware -f test_protocol --without-testing                      # build + upload only
sleep 6                                                                              # let the USB CDC re-enumerate
pio test -d trigger_firmware -f test_protocol --without-building --without-uploading # read results only
```

The split is required because the Arduino Nano ESP32's native USB CDC port
re-enumerates on reset after a flash, and a combined `pio test` loses the race
reopening it. The read phase must start while the board is still in its 20 s
startup delay (the test programs wait `kTestHostConnectDelayMs` at boot for exactly
this); the upload plus `sleep 6` uses about half of it, leaving ample margin. If a
suite reports `0 test cases`, the port opened too late -- just rerun both commands.
(A zero-case suite is *not* a pass: PlatformIO shows it as `SKIPPED`, the same as a
`-f`-filtered suite.) For thorough protocol coverage there is also the desktop
GoogleTest suite (`comm_protocol/tests/`, run with `ctest`, see above); the
on-device `test_protocol` is only a smoke check of the same library.

PlatformIO installs AUnit automatically (it is listed in `platformio.ini`'s `lib_deps`). Because AUnit is not PlatformIO's built-in Unity framework, `platformio.ini` sets `test_framework = custom`; each test program drives `aunit::TestRunner` from its own `setup()`/`loop()`. The firmware sources are compiled into the test programs (`test_build_src = yes`), and `src/main.cc` excludes its `setup()`/`loop()` from test builds via `#ifndef PIO_UNIT_TESTING`.

The repository root also has an umbrella `CMakeLists.txt` that configures the desktop components together (`cmake -S . -B build`); the per-component builds above are the usual workflow.


## Developer info

- Use C++20 (highest standard supported by `arduino-esp32 3.x` by default) for `trigger_firmware` and `comm_protocol`. Use C++23 for `recorder`.
- Use code style specified in `.clang-format`. Don't use `clang-tidy`.
- Use snake case for file names. Use suffices `.cc` and `.h`.
- Prefer low overhead for `trigger_firmware` and `comm_protocol`. For `recorder`, be mindful of overhead since frame acquisition can run at up to 500 FPS. However, don't overoptimize at the cost of readability.
- Follow best practices in using `const` and pass arguments by reference when applicable.
- Write comments whenever the logic is unclear/untrivial. Don't hard-code "magic numbers" or "magic logics."
- Use `#pragma once` instead of `#ifndef` guards in header files.
- Use CMake and PlatformIO. `recorder` and `trigger_firmware` compile on their own. `comm_protocol` is a library included by both, but it can also be configured standalone to build and run its unit tests (see [Building](#building)).
- Use only ASCII characters except in `.md` files.
- Write _meaningful_ unit tests only. Don't write test just for the sake of it. Don't bloat the number of lines in test files just to test trivial stuff.