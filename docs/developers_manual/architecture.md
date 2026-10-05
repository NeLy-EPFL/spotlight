# Architecture overview

The recorder controls many hardware devices (Euresys frame grabber + JAI behavior
camera, PCO muscle camera, Zaber translation stages, the trigger microcontroller,
and several light controllers) while streaming ~1000×2000 px behavior frames at
several hundred FPS with minimal data loss. Two ideas make this tractable:
**multithreading** (to decouple independent logic and exploit multiple cores) and a
**dedicated trigger microcontroller** (to keep hardware timing off the host's
non-real-time scheduler and ensure matching muscle and behavior frames are exposed
durint the same period of time).

This page is the conceptual map. For the timing/synchronization details see
[Data acquisition](data_acquisition.md); for the host↔microcontroller messages see
the [communication protocol](comm_protocol.md).

## Multithreading

Each largely independent job runs in its own thread, communicating through
well-defined data holders rather than shared program state. This keeps, for
example, motion-control logic from entangling with image-acquisition logic, and
lets the OS schedule parallelizable work (acquisition, compression, tracking)
across cores. Image saving uses a *pool* of threads so the recorder keeps up even
when frames arrive faster than a single saver can compress and write them.

The recorder runs roughly the following threads:

- Main thread (sets up everything, owns the camera objects, runs teardown).
- GUI (Qt event loop).
- Behavior image acquirer.
- Behavior image savers (multiple).
- Muscle image acquirer.
- Muscle image savers (multiple).
- Motion-control IO thread (talks to the Zaber controller).
- Tracking-control thread (estimates the fly position, computes stage targets).
- Stage-position logger.
- Microcontroller communication thread (serial IO to the trigger firmware).

Shared data are protected with the usual synchronization primitives (mutexes and
condition variables) to avoid race conditions and busy-waiting.

## Camera drivers

The behavior camera is driven **in-process** by its acquirer thread:

- `BehaviorCamera` (`src/peripherals/behavior_camera.cc`) wraps the Euresys
  EGrabber API for the JAI CoaXPress camera and calls `waitForOneFrame()` in a
  loop.

The two muscle (PCO) cameras, calcium and fiducial, run in one **separate
process** (see [data_acquisition.md](data_acquisition.md#two-muscle-cameras)).
It has to be a single process: the PCO SDK does not support opening the two
cameras from separate processes (the first process to open a camera claims all
of them).

- `pco-camera-server` (`src/apps/pco_camera_server_main.cc`) opens both
  pco.panda 4.2 cameras one after the other, by the serial numbers configured
  in `muscle_camera/serial_number_<camera>`, sets ROI/trigger/acquire/exposure,
  starts them, and then publishes each camera's frames from its own thread over
  five shared-memory regions per camera (frame data, shutter-open time, server
  state, mutex, condvar), whose names are configured per camera in
  `muscle_camera/shared_*_name_<camera>`.
- `PcoCameraClient` (`src/peripherals/muscle_camera.cc`) maps one camera's
  shared-memory regions. `is_ready()` reports whether the server has that
  camera recording; `wait_for_next_frame()` blocks on the shared condvar and
  returns the latest frame; `set_nominal_exposure_us()` blocks until the server
  has applied the new exposure.
- `MuscleCamera` owns one `PcoCameraClient` per camera. It `fork()`+`execl()`s
  the server and blocks until both cameras are recording; `stop()` sends
  `SIGTERM` to the server and reaps it. `wait_for_next_frame_pair()` returns
  one frame of each camera, paired by acquisition time.

The separate process keeps the PCO SDK (and the `PCO_LINUX` Windows-compat shims
it injects into the global namespace) completely isolated from the Qt/Euresys
stack, and means a crash in the PCO SDK cannot destabilize the GUI process.

On shutdown, `quit_program()` calls `stop()` on the behavior and muscle cameras
(releasing the Euresys grabber and terminating the PCO server) before calling
`std::exit()`.

## Trigger microcontroller

Precise, low-latency synchronization between the cameras and the strobed lights is
handled by an Arduino Nano ESP32 running the firmware in `trigger_firmware/`, not
by the host. The host sends compact JSON commands (frame rate, exposures, the
recording schedule) over USB serial; the microcontroller generates the TTL trigger
signals. This protocol is a small library in `comm_protocol/`, shared by both the
recorder and the firmware. See [comm_protocol.md](comm_protocol.md) and
[data_acquisition.md](data_acquisition.md).

## Components in this repository

- `recorder/` — the C++ recorder and its three programs ([`align-cameras`](recorder/align_cameras.md),
  [`run-arena-registration-scan`](recorder/run_arena_registration_scan.md),
  [`run-spotlight`](recorder/run_spotlight.md)).
- `trigger_firmware/` — the microcontroller firmware (PlatformIO).
- `comm_protocol/` — the shared JSON protocol library (CMake; included by both of
  the above).
- `tools/` — the offline Python tools (`fit-arena-registration`, `postprocess-recording`, etc.).
