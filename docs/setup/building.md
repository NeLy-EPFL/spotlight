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
ctest --test-dir recorder/build -LE hardware --output-on-failure   # unit tests
```

The binaries are produced under `recorder/build/`. The three programs are
[`run-spotlight`](../recorder/run_spotlight.md),
[`align-cameras`](../recorder/align_cameras.md), and
[`run-arena-registration-scan`](../recorder/run_arena_registration_scan.md).

The unit tests (`recorder/tests/`) cover the hardware-independent logic
(calibration, config loading, image/metadata utilities, muscle-camera ROI and
trigger timing). **No unit test requires a physical device to respond**, though a
test target may include vendor SDK headers and link the SDK libraries. They build
by default when `recorder` is configured standalone; set `-DRECORDER_BUILD_TESTS=ON`
to opt in from the umbrella build.

### Hardware tests

A separate set of tests **does** drive a physical device. They are all tagged with
the CTest label **`hardware`** so the normal run can skip them (the `-LE hardware`
above) and the rig can run only them:

```sh
ctest --test-dir recorder/build -L hardware --output-on-failure    # only hardware tests
ctest --test-dir recorder/build --output-on-failure                # everything (unit + hardware)
```

Current hardware tests:

| Test | What it checks | Needs |
|---|---|---|
| `BehaviorCameraHardware.ConstructsAndConfigures` | Behavior (JAI/Euresys) camera opens + GenICam configuration succeeds | Behavior camera |
| `MuscleCameraHardware.InitializesAcquiresAndCloses` | Muscle (PCO) camera initializes, produces a frame, and closes cleanly | Muscle camera |
| `BothCamerasHardware.SequentialInitBringsBothUp` | Both cameras come up using the **serialized** init order of the apps (behavior fully ready, *then* muscle) | Both cameras |

> [!IMPORTANT]
> A hardware test needs the device(s) attached **and free** to pass — e.g. close
> eGrabber Studio first, since the grabber can be opened by only one client at a
> time. Run them on the recording computer, not in CI. They still *build* (and
> CTest still *lists* them) on any machine that can build the recorder; only
> *running* them touches hardware.

These live in **two** executables (both labeled `hardware`):

- `recorder_hardware_tests` — the behavior-camera test only. It deliberately does
  **not** link the PCO/muscle SDK, so a failure here is isolated from the muscle
  camera (useful for answering "is the behavior-camera problem caused by the PCO
  SDK being loaded?").
- `recorder_hardware_tests_pco` — the muscle and both-camera tests, which link both
  the PCO and Euresys SDKs.

To add a hardware test, drop it in whichever target fits (PCO-free vs. PCO-linked)
in `recorder/tests/CMakeLists.txt`; both are already labeled `hardware`.

By default CTest only prints a test's output when it fails (`--output-on-failure`).
To see SDK messages, initialization output, and GoogleTest lines for every test
regardless of outcome, add `-V` (verbose):

```sh
ctest --test-dir recorder/build -L hardware -V
```

Alternatively, run the test binary directly to get raw GoogleTest output
(no CTest wrapper):

```sh
./recorder/build/recorder_hardware_tests          # behavior-camera tests
./recorder/build/recorder_hardware_tests_pco      # muscle + both-cameras tests
```

Pass `--gtest_filter=TestSuite.TestName` to run a single test case:

```sh
./recorder/build/recorder_hardware_tests_pco --gtest_filter=MuscleCameraHardware.InitializesAcquiresAndCloses
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