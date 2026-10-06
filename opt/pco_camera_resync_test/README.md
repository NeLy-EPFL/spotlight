# PCO camera re-sync test

The two PCO muscle cameras start on the same acquire-enable edge and then drift apart (see `../pco_camera_clock_drift_profiling/`). `trigger_firmware` re-syncs them by holding acquire enable LOW for one muscle frame period plus 2 ms while the cameras keep recording (`TriggerController::reset_timing()`). This test checks that this works while both cameras' frames are being grabbed as in `pco-camera-server`, without stopping and restarting the recording in software. It is independent of the rest of the repository and has the same three parts as the drift test:

- `arduino/`: PlatformIO project for the trigger board (Arduino Nano ESP32, same pins as `trigger_firmware`). Drives acquire enable and samples both cameras' status lines.
- `cpp/`: CMake project. Runs and grabs from the cameras, and saves the samples and grab logs.
- `python/`: Analysis and plots.

## How it works

The C++ program:

1. Configures both cameras as `pco-camera-server` does (1120×1120 centered ROI, auto trigger, external acquire mode, common time on SMA #4, shutter-open time for 66 Hz), except for binary timestamps: these cameras only support ASCII timestamps.
2. Starts recording into a 10-image ring buffer, and grabs both cameras' frames in one thread each, like `serve_frames()` in `pco-camera-server` (wait for a new image with a 0.1 s timeout, then fetch the latest image). Errors other than timeouts are logged, not thrown.
3. Sends `"<duration_us> <sample_period_us> <resync_interval_us> <acquire_low_us>\n"` to the Arduino (40 s, 2 µs, 10 s, and one frame interval + 2 ms by default).

The Arduino holds acquire enable LOW for 1 s, releases it, and samples both status lines every sample period, as in the drift test. At every multiple of the re-sync interval, it holds acquire enable LOW for `acquire_low_us`. It then sends the whole sequence back (format documented in `arduino/include/pco_resync_test/config.h`). The C++ program stops grabbing and recording, and saves the results.

## Running

1. Set the camera serial numbers and the serial port in `cpp/src/main.cc` (`config::calcium_serial_number`, `config::fiducial_serial_number`, `config::serial_port`).
2. Flash the Arduino: `cd arduino && pio run -t upload`. The display should read "PCO CAM RESYNC TEST".
3. Build and run the C++ program (no other program may hold the cameras or the serial port). It takes about 1 min:
   ```
   cd cpp
   cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build
   ./build/pco_camera_resync_test
   ```
4. Analyze the recording (requires [uv](https://docs.astral.sh/uv/)):
   ```
   uv run python/analyze_resync.py cpp/resync_test_<timestamp>
   ```

## Output

The C++ program saves the recording to `resync_test_<timestamp>/` in the working directory:

- `samples.bin`: the packed samples, as in the drift test.
- `metadata.json`: the number of samples, sample period, re-sync interval, acquire-LOW time, operating point, and the applied shutter-open times.
- `frames_<camera>.csv`: one row per grabbed frame: the host time since the command was sent (µs) and the PCO recorder image number.
- `grab_errors.csv`: the grab errors other than timeouts.

`python/analyze_resync.py` removes glitches and finds the common-time starts as the drift analysis does. Since the common time starts one rolling time (ROI height × line time) after the first line's exposure, it computes the exposure starts from them. It prints, per camera and re-sync, when the last exposure started before acquire went LOW and the first after it was released, and the gap between them; the fiducial – calcium offset at the start and end of each segment between re-syncs; and the number of grabbed frames and grab errors. It saves `resync_test.png` in the recording directory with (a) the fiducial – calcium offset over time and (b) the host time between grabbed frames.

## Results (2026-10-02)

- Re-syncing works while grabbing: no grab errors, and the recorder image numbers have no gaps.
- Every re-sync resets the offset to -6 to -2 µs (it drifts by about +24 µs per 10 s, i.e. about 2.4 ppm). Both cameras start their first exposure 40–48 µs after the release.
- No exposure starts while acquire is LOW. An exposure in progress when acquire goes LOW completes. The gap between exposure starts across a re-sync was 1.23–1.36 frame intervals. Depending on the phase of the re-sync, it ranges from about the LOW time (no frame lost) to one frame interval plus the LOW time (one frame lost).
- The fiducial camera exposed all 2640 frames, but only 761 (about 19 Hz) reached the host, in every segment. This is unrelated to re-syncing: that camera is connected at USB 2.0 speed (480 Mbit/s in `lsusb -t`), which is too slow for 1120×1120 16-bit frames at 66 Hz.
