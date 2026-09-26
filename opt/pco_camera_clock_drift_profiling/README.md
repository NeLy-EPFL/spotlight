# PCO camera clock drift profiling

The two PCO muscle cameras start on the same acquire-enable edge and then run on their own clocks (see `docs/developers_manual/data_acquisition.md`). This test measures how far they drift apart. It is independent of the rest of the repository: `cpp/` is a standalone CMake project, and `arduino/` is a standalone PlatformIO project for the trigger board (Arduino Nano ESP32, same pins as `trigger_firmware`).

## How it works

For each of 5 repeats, the C++ program:

1. Configures both cameras: 1120×1120 centered ROI, auto trigger, external acquire mode (acquire enable on HWIO 2, active high), common time on SMA #4, and the shutter-open time for 66 Hz.
2. Starts recording into a small ring buffer. The images are never read.
3. Sends `"<shutter_open_time_us> <n_frames>\n"` (i.e. `"15147 5000\n"`) to the Arduino.

The Arduino then holds acquire enable LOW for 1 s, releases it, and busy-polls both status lines. It records the `micros()` time of the first 5000 rising and falling edges of each camera, sets acquire enable LOW again, and sends the times back in binary (format documented in `arduino/include/pco_clock_drift/config.h`). The C++ program stops recording, saves the edge times, and prints a summary.

Expected timings are derived from the operating point in `docs/users_manual/frame_rate_limits.md` (66 Hz, 12.136 µs line time, 4.5 µs readout): frame interval 15151.5 µs, shutter-open time 15147.0 µs, common time 1554.7 µs.

## Running

1. Set the camera serial numbers in `cpp/src/main.cc` (`config::calcium_serial_number`, `config::fiducial_serial_number`) and check `config::serial_port`.
2. Flash the Arduino: `cd arduino && pio run -t upload`. The display should read "PCO CAM CLOCK DRIFT TEST".
3. Build and run the C++ program (no other program may hold the cameras or the serial port):
   ```
   cd cpp
   cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build
   ./build/pco_camera_clock_drift_profiling
   ```

Each repeat takes about 80 s.

## Output

Each repeat is saved to `clock_drift_<timestamp>/repeat_<i>.csv` in the working directory, with one row per frame: the raw on/off edge times of both cameras (µs since acquire enable went HIGH) and the derived frame intervals, common times, and fiducial – calcium onset offset.

For each camera, the summary prints the mean (signed) and maximum absolute deviation of the frame interval and of the common time from their expected values. It also prints the change in the fiducial – calcium onset offset between the first and last frame (in µs and ppm).

The per-camera numbers are measured against the Arduino's crystal, which is itself only accurate to tens of ppm, so a mean interval deviation of that size can come from either clock. The between-camera drift does not depend on the Arduino's clock. Each edge time has about 1 µs resolution plus occasional jitter from other tasks on the ESP32, which only affects the maximum deviations.
