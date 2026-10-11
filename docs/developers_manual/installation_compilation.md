# Installation & compilation

## Step 1: Clone the repository

```bash
git clone git@github.com:NeLy-EPFL/spotlight-control.git
```

The Python tools (`tools/`) are part of this repository — no separate clone needed.

## Step 2: Build recorder programs

```bash
cd spotlight-control

# Generate Makefile using CMake
cmake -S recorder -B recorder/build

# Run Makefile using 16 CPU cores
cmake --build recorder/build -j 16

# Install executables to recorder/bin
cmake --install recorder/build
```

The recorder requires OpenCV 4.10 or newer; use Ubuntu's package (`sudo apt install libopencv-dev`). If an older OpenCV is also installed (e.g. a self-built one in `/usr/local`), CMake skips it, but an existing build directory remembers the old one: reconfigure it with `cmake -S recorder -B recorder/build -U OpenCV_DIR` (or delete `recorder/build`).

This installs the executables to `recorder/bin/`. When running the programs, either use paths to these executables explicitly (e.g., `/path/to/spotlight-control/recorder/bin/run-spotlight -p ... -a ...`), or add `recorder/build` to the front of the `$PATH` environment variables (add the line `export PATH="/path/to/spotlight-control/recorder/bin:$PATH"`. Note that this changes the default version of the programs when you launch them from the terminal).

### Known issue: PCO SDK's DICOM plug-in crashes the camera server

With pco.cpp 3.2.0 (`libpco_fdcm.so.1.31.0`) on Ubuntu 26.04, `run-spotlight` aborts with `PCO camera server exited during startup`, and `pco-camera-server` itself dies with a segfault right after both cameras report `setup complete`. PCO's own `/opt/pco/pco.cpp/bin/SimpleExample` crashes the same way, so this is a bug in the SDK, not in this codebase.

**Cause:** when `pco::Camera::record()` creates PCO's recorder (`PCO_RecorderCreate`), the recorder loads every image-file plug-in in `/opt/pco/pco.cpp/lib/` (`libpco_f*.so`), whether or not it is used. The DICOM plug-in, `libpco_fdcm.so`, bundles its own copy of DCMTK and has two bugs:

1. **Crash on load when the PID has 7 or more digits.** Its startup code generates a DICOM UID that includes the process ID. With a PID ≥ 1,000,000 the UID becomes 65 characters, one over DICOM's limit. DCMTK then tries to log a warning through a logger that the plug-in hasn't constructed yet, and segfaults in `dcmtk::log4cplus::Logger::isEnabledFor`. Ubuntu allows PIDs up to 4,194,304, so this starts happening once the machine has been up long enough, and it may come and go after reboots.
2. **Crash on unload.** The plug-in's unload handler dereferences a pointer that is NULL unless the plug-in was actually used, so the process segfaults on `exit()`.

A minimal reproducer and the full analysis (for reporting to PCO) are in `~/project/dev/pco_dicom_bug/` on the recording computer.

**Workaround:** we never write DICOM files, so move the plug-in out of the SDK's library folder:

```bash
sudo mkdir -p /opt/pco/disabled
sudo mv /opt/pco/pco.cpp/lib/libpco_fdcm.so* /opt/pco/disabled/
```

Reinstalling or updating the PCO SDK puts the file back, so repeat this after every PCO update until PCO fixes the bug.

## Step 3: Build and upload trigger firmware

Plug the Arduino Nano ESP32 board into the computer, then:

```bash
# Build
pio run -d trigger_firmware

# Upload to device
pio run -d trigger_firmware -t upload
```

## Step 4: Install Python tools

```bash
cd tools
uv sync  # see https://docs.astral.sh/uv/ if uv is not installed
```
