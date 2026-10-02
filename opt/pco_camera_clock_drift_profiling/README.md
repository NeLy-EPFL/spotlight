# PCO camera clock drift profiling

The two PCO muscle cameras start on the same acquire-enable edge and then run on their own clocks (see `docs/developers_manual/data_acquisition.md`). This test measures how far they drift apart. It is independent of the rest of the repository and has three parts:

- `arduino/`: PlatformIO project for the trigger board (Arduino Nano ESP32, same pins as `trigger_firmware`). Samples both cameras' status lines.
- `cpp/`: CMake project. Runs the cameras and saves the samples.
- `python/`: Analysis and plots.

## How it works

The C++ program:

1. Configures both cameras: 1120×1120 centered ROI, auto trigger, external acquire mode (acquire enable on HWIO 2, active high), common time on SMA #4, and the shutter-open time for 66 Hz.
2. Starts recording into a small ring buffer. The images are never read.
3. Sends `"<duration_us> <sample_period_us>\n"` to the Arduino, with the duration of `n_frames + 1` frames (1000 frames and 2 µs by default).

The Arduino holds acquire enable LOW for 1 s, releases it, and samples both status lines every sample period. Like `trigger_firmware`, it uses a plain loop with `micros()` and `digitalRead()`. It packs the samples (4 per byte) into PSRAM, sets acquire enable LOW again, and sends the whole sequence back (format documented in `arduino/include/pco_clock_drift/config.h`). The C++ program stops recording and saves the samples.

The sampling loop takes about 1.45 µs per sample, so the sample period must be at least 2 µs. With a shorter period the loop falls behind and the samples no longer correspond to fixed times. The PSRAM holds about 33M samples (8 MB), i.e. about 4400 frames at 2 µs. Arduino on the ESP32 runs on FreeRTOS, whose 1 kHz tick interrupt stalls the loop for about 8 µs every ms. An edge during a stall is timestamped up to about 8 µs early, which shows up as occasional outliers, as it would in `trigger_firmware`.

## Running

1. Set the camera serial numbers and the serial port in `cpp/src/main.cc` (`config::calcium_serial_number`, `config::fiducial_serial_number`, `config::serial_port`).
2. Flash the Arduino: `cd arduino && pio run -t upload`. The display should read "PCO CAM CLOCK DRIFT TEST".
3. Build and run the C++ program (no other program may hold the cameras or the serial port). It takes about 30 s:
   ```
   cd cpp
   cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build
   ./build/pco_camera_clock_drift_profiling
   ```
4. Analyze the recording (requires [uv](https://docs.astral.sh/uv/), which installs the dependencies):
   ```
   uv run python/analyze_clock_drift.py cpp/clock_drift_<timestamp>
   ```

## Output

The C++ program saves the recording to `clock_drift_<timestamp>/` in the working directory:

- `samples.bin`: the packed samples as sent by the Arduino. Sample `i` is in bits `2 * (i % 4)` (calcium) and `2 * (i % 4) + 1` (fiducial) of byte `i / 4`. In NumPy: `bits = np.unpackbits(np.fromfile(path, np.uint8), bitorder="little")`, then `bits[0::2]` (calcium) and `bits[1::2]` (fiducial), truncated to `n_samples`.
- `metadata.json`: the number of samples, sample period, operating point, and the shutter-open times applied by the cameras (which round the requested 15147.0 µs to 15143 µs).

## Analysis

`python/analyze_clock_drift.py` first removes glitches: HIGH pulses, then LOW gaps, shorter than `--min-pulse-us` (10 µs). The fiducial status line has a spike shorter than 250 ns about 1.5–2 µs before the rising edge of some frames; the calcium line has none. It then finds each camera's common-time starts (rising edges) and durations, and compares the starts with the theoretical ones, `t0 + k * T`, where `t0` is the calcium camera's first start (for both cameras) and `T` is the applied shutter-open time + readout time. Frames are paired by index.

It prints the number of frames and removed glitches, the interval and common-time statistics per camera, and a linear fit of the fiducial – calcium difference (drift in ppm, intercept, RMSE, and R²). It saves `clock_drift.png` in the recording directory with:

- (a) The deviation of each camera's common-time starts from the theoretical ones.
- (b) The difference between the two cameras' deviations, i.e. the fiducial – calcium drift, with the linear fit and its equation. Unlike (a), it does not depend on the Arduino's clock.
- (c) The distribution (violin plot per camera) of intervals between consecutive common-time starts.
- (d) The distribution of common-time durations.

In (c) and (d), the violins are horizontal, with up to 300 randomly sampled data points per camera (jittered vertically only). The dashed line is the theoretical value, the vertical bars are the minimum, median, and maximum, and the kernel width of the violins is one sample period, since the values are quantized to it.

Both lines in (a) slope down by about 80 ppm: the measured interval is about 1.2 µs shorter than the theoretical one, for both cameras alike. This is either the Arduino's clock or the theoretical interval (the readout time is from the PCO manual, and the applied shutter-open time is as reported by the SDK); this recording cannot tell them apart.
