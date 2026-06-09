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

## In-process cameras

Both cameras are driven **in-process**, each by its own acquirer thread that calls
`waitForOneFrame()` in a loop:

- `BehaviorCamera` (`src/peripherals/behavior_camera.cc`) wraps the Euresys
  EGrabber API for the JAI CoaXPress camera.
- `MuscleCamera` (`src/peripherals/muscle_camera.cc`) wraps the PCO SDK for the
  pco.panda 4.2, pimpl'd so the PCO headers (and the `PCO_LINUX` Windows-compat
  shims they inject) stay confined to that one translation unit.

> [!NOTE]
> Earlier versions ran the muscle camera as a separate `pco-camera-server` process
> that published frames over shared memory, because the PCO SDK was assumed to be
> incompatible with the Qt/Euresys stack. That assumption did not hold; the server
> and its shared-memory plumbing have been removed. The investigation that led to
> this is preserved in [muscle_camera_inprocess_refactor.md](muscle_camera_inprocess_refactor.md).

Because the camera objects are touched only by their acquirer threads, shutdown is
**join-based**: the acquirers are joined before the camera objects are destroyed,
so the grabbers are released cleanly. See the quit section of
[Troubleshooting](troubleshooting.md) for why this ordering matters.

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

The offline Python tools are in the separate
[`spotlight-tools`](https://github.com/NeLy-EPFL/spotlight-tools) repository.
