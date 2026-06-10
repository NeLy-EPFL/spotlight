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


## Documentation

Documentation is available in the `docs/` folder. Start at the index,
[`docs/README.md`](docs/README.md), which splits the pages into a short **User's
manual** (running an experiment end to end) and a fuller **Developer's manual**
(internals, environment setup, hardware). A few entry points:

- [Architecture overview](docs/architecture.md): threads, in-process cameras, and the trigger microcontroller.
- [Data acquisition workflow](docs/data_acquisition.md): how the cameras and lights are controlled and synchronized using continuous acquisition.
- [Building and installing](docs/setup/building.md) and [software dependencies](docs/setup/dependencies.md): how to build the components and install their dependencies.
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

The `recorder/tests/` directory contains two GoogleTest executables:

- **`recorder_tests_nohardware`**: hardware-independent logic (calibration, config loading, image/metadata utilities, muscle-camera ROI and trigger timing). No physical device needed. Builds by default when `recorder` is configured standalone; opt in from the umbrella build with `-DRECORDER_BUILD_TESTS=ON`.
- **`recorder_tests_hardware`**: init/configure/destroy cycles for each peripheral (Euresys behavior camera, PCO muscle camera, Zaber motion stages, Arduino trigger controller) plus an all-hardware integration test. Requires all four peripherals to be powered and connected. Opt in with `-DRECORDER_BUILD_HARDWARE_TESTS=ON` and set `SPOTLIGHT_PROFILE_DIR` before running.

See [Building and installing](docs/setup/building.md) for the full test commands.

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

Code style and conventions (language standards, formatting, naming, overhead, tests)
are documented in [`docs/code_style.md`](docs/code_style.md).