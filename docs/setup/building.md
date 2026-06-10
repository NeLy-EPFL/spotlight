# Building and installing

This repository has three desktop/embedded components: the `recorder` (CMake), the
shared `comm_protocol` library (CMake), and the `trigger_firmware` (PlatformIO). The
offline Python tools live in the separate
[`spotlight-tools`](https://github.com/NeLy-EPFL/spotlight-tools) repository.

Install the third-party libraries and SDKs first — see
[Software dependencies](dependencies.md).

> [!NOTE]
> CMake fetches the ArduinoJson dependency (and GoogleTest for the tests)
> automatically on first configuration, so an internet connection is needed then.

## `comm_protocol`

The shared protocol library and its unit tests:

```sh
cmake -S comm_protocol -B comm_protocol/build
cmake --build comm_protocol/build -j 16
ctest --test-dir comm_protocol/build --output-on-failure   # run the unit tests
```

The tests (`comm_protocol/tests/`) use GoogleTest, fetched via `FetchContent` when
the library is configured standalone.

## `recorder`

This also builds the `comm_protocol` dependency:

```sh
cmake -S recorder -B recorder/build
cmake --build recorder/build -j 16
ctest --test-dir recorder/build --output-on-failure        # run the unit tests
```

The binaries are produced under `recorder/build/`. The three programs are
[`run-spotlight`](../recorder/run_spotlight.md),
[`align-cameras`](../recorder/align_cameras.md), and
[`run-arena-registration-scan`](../recorder/run_arena_registration_scan.md).

The `recorder/tests/` directory contains two test executables:

**`recorder_tests_nohardware`** — hardware-independent logic (calibration,
config loading, image/metadata utilities, muscle-camera ROI and trigger timing).
No physical device needed.  Builds by default when `recorder` is configured
standalone; opt in from the umbrella build with `-DRECORDER_BUILD_TESTS=ON`.

```sh
ctest --test-dir recorder/build --label-regex nohardware --output-on-failure
# or simply (nohardware tests are the only ones registered by default):
ctest --test-dir recorder/build --output-on-failure
```

**`recorder_tests_hardware`** — init/configure/destroy cycles for each
peripheral (Euresys behavior camera, PCO muscle camera, Zaber motion stages,
Arduino trigger controller) and an all-hardware integration test.  Requires all
four peripherals to be powered and connected.  Off by default; enable with
`-DRECORDER_BUILD_HARDWARE_TESTS=ON` and set the `SPOTLIGHT_PROFILE_DIR`
environment variable to the profile directory before running.

```sh
# Re-configure to build hardware tests, then build and run:
cmake -S recorder -B recorder/build -DRECORDER_BUILD_HARDWARE_TESTS=ON
cmake --build recorder/build -j16
export SPOTLIGHT_PROFILE_DIR=~/Spotlight/profiles/sibo_260514
ctest --test-dir recorder/build --label-regex hardware --output-on-failure
```

To run **all** tests (both executables) in one go after enabling hardware tests:

```sh
ctest --test-dir recorder/build --output-on-failure
```

> [!TIP]
> If a compile error appears during a parallel build, rerun with `-j 1` to get the
> error messages in order.

The repository root also has an umbrella `CMakeLists.txt` that configures the
desktop components together (`cmake -S . -B build`); the per-component builds above
are the usual workflow.

## `trigger_firmware`

The trigger firmware is built and uploaded with **PlatformIO** (we use PlatformIO,
not the Arduino IDE). Install it either as the **PlatformIO IDE** (the official
VS Code extension, recommended) or as the standalone `pio` command-line tool
(`pip install platformio`). Both provide the `pio` CLI used below; the VS Code
extension also gives build/upload/serial-monitor buttons.

```sh
pio run -d trigger_firmware            # build
pio run -d trigger_firmware -t upload  # flash the microcontroller
```

PlatformIO resolves the board, toolchain, and library dependencies from
`trigger_firmware/platformio.ini`.

The firmware has on-device unit tests (`trigger_firmware/test/`) written with
[AUnit](https://github.com/bxparks/AUnit) (`test_device_io`, `test_serial_io`,
`test_protocol`). Because the Arduino Nano ESP32's native USB CDC port
re-enumerates on reset after a flash, split the upload from the serial read:

```sh
pio test -d trigger_firmware -f test_protocol --without-testing                      # build + upload only
sleep 6                                                                              # let the USB CDC re-enumerate
pio test -d trigger_firmware -f test_protocol --without-building --without-uploading # read results only
```

Swap `test_protocol` for another suite name, or drop `-f` to run all of them. If a
suite reports `0 test cases`, the port opened too late — just rerun both commands.
(A zero-case suite is *not* a pass; PlatformIO shows it as `SKIPPED`.) Thorough
protocol coverage lives in the desktop GoogleTest suite (`comm_protocol/tests/`);
the on-device `test_protocol` is only a smoke check.

PlatformIO installs AUnit automatically (it is listed in `platformio.ini`'s
`lib_deps`). See the [README](../../README.md) for the firmware test internals
(`test_framework = custom`, `test_build_src`, the `PIO_UNIT_TESTING` guard).

If uploads fail with a USB-permission error (`LIBUSB_ERROR_ACCESS` / no DFU device),
see the [USB-upload troubleshooting](dependencies.md#usb-upload-troubleshooting) on
the dependencies page.

## Python tools (`spotlight-tools`)

The offline tools use **uv** for package management (previously Poetry). uv is much
faster and still uses `pyproject.toml`.

```bash
# Install uv if not already installed (see https://docs.astral.sh/uv/)
cd spotlight-tools/
uv sync
```

After installation the entry points are available in your shell (or via
`uv run <command>`):

```bash
fit-arena-registration -a ~/Spotlight/arenas/arena146
postprocess-recording  --recording-dir ~/data/spotlight/20250613-fly1b-002/
visualize-stage-trajectory --recording-dir ~/data/spotlight/20250613-fly1b-002/
```

Entry points are defined in the `[project.scripts]` section of
`spotlight-tools/pyproject.toml`. See [python-tools/](../python-tools/) for per-tool
documentation.