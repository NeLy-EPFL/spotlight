# Troubleshooting

## Behavior camera: `run-spotlight` aborts at startup with `vector::_M_range_check`

Symptom — GenTL discovery takes unusually long (tens of seconds), then the
program crashes:

```
[info] Running GenTL eGrabber discovery...
[info] GenTL eGrabber discovery completed
[info] Configuring camera...
terminate called after throwing an instance of 'std::logic_error'
  what():  vector::_M_range_check: __n (which is 0) >= this->size() (which is 0)
```

Cause — discovery found **zero** cameras, so `egrabberDiscovery.cameras(0)` in
`src/peripherals/behavior_camera.cc` indexes an empty vector and throws. The
behavior camera (a CoaXPress device on the Euresys grabber, `/dev/coaxlink*`)
can only be opened by one GenTL client at a time. If another client already
holds it, discovery comes up empty.

The usual culprit is **eGrabber Studio** (`/opt/euresys/egrabber/studio/studio`)
being left open — it is easy to forget when it is minimized or on another
workspace. `run-spotlight` is the only program in this repo that uses the
grabber, so anything else holding it is external.

Check who holds the grabber and close it:

```bash
fuser /dev/coaxlink0          # prints the PID(s) holding the device, if any
ps -o pid,cmd -p <PID>        # identify it (often .../egrabber/studio/studio)
kill <PID>                    # or just close the eGrabber Studio window
```

If the camera was left in a bad state by an unclean exit (see below), a
power-cycle of the camera (and grabber) clears it.

## Behavior camera: GenTL discovery hangs when started alongside the PCO camera

Symptom — the log shows `Running GenTL eGrabber discovery...` but never the
matching `GenTL eGrabber discovery completed`. The behavior preview stays blank
(no behavior frames ever arrive), and closing the GUI then hangs forever (you
have to `kill -9`).

Cause — the behavior acquirer thread is stuck **inside the `BehaviorCamera`
constructor**, in the Euresys GenTL discovery call. It never reaches its grab
loop, so no frames are produced and `behaviorImageAcquirerThread.join()` at quit
waits on a thread that will never return. The trigger is **running the Euresys
discovery and the PCO SDK library init at the same time in the same process**:
the muscle (PCO) camera is now driven in-process, so `PCO_InitializeLib()` /
`pco::Camera` construction runs concurrently with `EGrabberDiscovery::discover()`
on a separate thread, and that concurrency wedges the Euresys discovery. (This
could not happen when the muscle camera ran as a separate `pco-camera-server`
process: the PCO init lived in that child process and never overlapped the
Euresys discovery in the recorder.)

Fix (in the code) — `run-spotlight` and `align-cameras` now **serialize the two
camera initializations**: they wait for the behavior camera to finish
initializing (`BehaviorCamera::isReady()`) before starting the muscle-camera
thread, so the Euresys discovery runs alone. If you add a new program that opens
both cameras, keep that ordering.

## `run-spotlight` quit must join the acquirer threads before destroying the cameras

Both cameras are driven in-process: the behavior (Euresys) camera by the behavior
acquirer thread, and the muscle (PCO) camera by the muscle acquirer thread (there
is no separate camera process anymore). The camera objects are touched **only**
by their acquirer threads, so they **must outlive those threads**. `run-spotlight`
shutdown is therefore **join-based**, not `std::exit()`-based:

1. `quitProgram()` only *signals* shutdown: it sets `programState->toQuit`,
   `stop()`s the muscle camera, tells the saver/motion threads to stop, switches
   off the blue excitation light, and calls `application->quit()` so the Qt event
   loop exits. It does **not** call `std::exit()` and does **not** destroy any
   camera.
2. After `application->exec()` returns, `runSpotlightMain()` runs a single
   teardown that `stop()`s **both** cameras, tells the savers/motion to stop
   (idempotent with step 1), **joins every worker thread**, and only then resets
   `muscleRecordingState->muscleCamera` and `behaviorRecordingState->behaviorCamera`
   to `nullptr`. Those destructors run on the main thread with no acquirer alive:
   `~BehaviorCamera`/`~EGrabber` releases the CoaXPress grabber, and
   `~MuscleCamera` stops the PCO camera and calls `PCO_CleanupLib()`. This same
   teardown is reached on a **startup abort** (a camera failing to initialize), so
   that path also joins cleanly instead of `std::terminate`-ing on still-joinable
   threads.

**Invariant to preserve:** the CoaXPress grabber is released only by
`~BehaviorCamera`/`~EGrabber`. Any exit that skips that destructor — a segfault,
`std::terminate` (e.g. an uncaught exception in a thread, or destroying a
joinable `std::thread`), or `kill -9` — strands the grabber mid-stream, leaving
the camera unresponsive even to eGrabber Studio until it is **power-cycled**.

This requires shutdown to be **bounded** — no step may block forever, and **both
camera grabs are interruptible**:

- `BehaviorCamera::waitForOneFrame()` uses a bounded `ScopedBuffer` pop (200 ms)
  and `BehaviorCamera::stop()` calls `cancelPop()`, so a blocked grab throws
  `GC_ERR_ABORT` (caught internally) and the call returns `std::nullopt`. The
  acquirer therefore exits even if **no frames are arriving** (e.g. the camera is
  not being triggered) — it does not depend on triggers continuing during
  shutdown.
- `MuscleCamera::waitForOneFrame()` similarly returns `std::nullopt` after
  `MuscleCamera::stop()` (its PCO grab uses a ~100 ms `waitForNewImage` timeout
  that re-checks the stop flag).
- Because of the above, the teardown must actually **call `stop()` on both
  cameras** (it does) — `toQuit` alone does not unblock a grab that is waiting for
  a frame.
- `~ArduinoCommunication` calls `stopCommunication()` before joining its comm
  thread, so the join cannot deadlock (`quitProgram()` only stops excitation,
  never the comm thread itself).
- An uncaught exception while opening either camera is caught in its acquirer
  (`behaviorImageAcquirer` / `muscleImageAcquirer`), which logs and sets `toQuit`
  instead of letting the exception escape the thread and abort the process (an
  abort would itself strand the grabber).

`SIGINT` is async-signal-safe: the handler only stores to an atomic flag. A
`QTimer` on the main thread polls it and calls `mainGUIWindow->close()`, so Ctrl-C
and the GUI close button take the **same** path (`closeEvent` → `quitProgram`).
This avoids running anything Qt-unsafe from signal context and the old hazard of a
second `^C` re-entering `quitProgram()` mid-`std::exit()`.

If the camera is already stranded by an earlier unclean exit, power-cycle the
camera (and grabber) to clear it.

---

The entries below are migrated hardware/SDK errors. Many are intermittent vendor-SDK
quirks with no known root cause; the recorded workaround is what helped in practice.

## Behavior camera (Euresys / JAI)

### `GenApi error code 8`

```
terminate called after throwing an instance of 'Euresys::genapi_error'
  what():  GenTL error -10100, GenApi error code 8
```

**Cause:** unknown. **Workaround:** in eGrabber Studio, start streaming, stop it,
close the camera tab, then retry.

### `Assertion 'TriggerMode == "On"' failed`

The GenICam parameters are in an initial state that does not allow the normal
configuration sequence to run. **Fix:** open eGrabber Studio, select the JAI camera,
"Open", go to the Script tab, load and run the setup script; the output should show
`Success`. If not, restart the computer and retry.

> [!NOTE]
> This workaround predates the recorder applying the Euresys/GenICam setup
> in-process (`BehaviorCamera`); with the current code the configuration sequence
> runs programmatically, so this assertion should no longer occur on a normal start.
> Kept for reference.

### `EGrabber has no registered event for this filter`

```
terminate called after throwing an instance of 'Euresys::client_error'
  what():  EGrabber has no registered event for this filter
```

**Cause:** acquisition was not started before grabbing. **Fix (in code):** start
acquisition (`reallocBuffers` + `start`, i.e. `BehaviorCamera::start()`) first.

## Muscle camera (PCO)

### A DLL could not be found / camera not connected

```
pco::CameraException (0x800a300d): SDK DLL error 800a300d at device 'camera sdk dll': A DLL could not be found.
```

**Cause:** the PCO camera is not connected. **Fix:** check the USB connection;
verify the camera's Status LED is solid green (not flashing); unplug/replug if
needed.

### `Value is out of range` — ROI is 1-indexed

```
pco::CameraException (0x8003101e): Firmware error 8003101e at device 'SC2 Main uP': Value is out of range.
```

This generic "out of range" usually means the ROI origin is below 1: the ROI
definition is **1-indexed**, so `x0`/`y0` must be at least 1. (Reference: pco.cpp
User Manual, sec. 2.4.3.)

### `ROI setting is wrong` — bad ROI step/size

```
pco::CameraException (0x8003103e): Firmware error 8003103e at device 'SC2 Main uP': ROI setting is wrong
```

The ROI size must be a multiple of **32 px horizontally** and **8 px vertically**,
and at least **64×16 px**.

> [!IMPORTANT]
> `x0`/`y0` are 1-indexed, but do **not** add 1 to `x1`/`y1`:
> ```c++
> config.roi.x0 = xOffset + 1;             // correct
> config.roi.y0 = yOffset + 1;             // correct
> config.roi.x1 = xOffset + imageWidth;    // correct (do NOT use +1)
> config.roi.y1 = yOffset + imageHeight;   // correct (do NOT use +1)
> ```
> (Reference: pco.panda 4.2 User Manual, Appendix A1.1.)

### `Handle is invalid`

```
pco::CameraException (0xa00a3002): SDK DLL error a00a3002 at device 'camera sdk dll': Handle is invalid.
```

**Cause:** unknown. **Workaround:** a clean rebuild (`make clean` then rebuild)
seemed to help even with no source change — possibly related to the bundled PCO SDK
sources being compiled into the recorder.

### `Arm is not possible while record active`

```
pco::CameraException (0x80031022): Firmware error 80031022 at device 'SC2 Main uP': Arm is not possible while record active.
```

**Cause:** unknown; suspected the camera was not closed cleanly last time.
**Workaround:** physically unplug/replug the camera USB cable and wait until the
Status LED is solid green.

## GUI

### GUI won't quit; prints `[heap] 0`, `[heap] 1`, …

**Cause:** unknown. **Workaround:** force-quit the application. (See the quit
section above for the intended clean-shutdown path.)

## Arduino / trigger microcontroller

### `No DFU capable USB device available`

```
dfu-util: Cannot open DFU device 2341:0070 ... (LIBUSB_ERROR_ACCESS)
dfu-util: No DFU capable USB device available
Failed uploading: uploading error: exit status 74
```

**Fix:** download and run the Arduino ESP32
[`post_install.sh`](https://github.com/arduino/ArduinoCore-mbed/blob/main/post_install.sh)
with `sudo`. See also the udev-rule fix in the
[dependencies page](setup/dependencies.md#usb-upload-troubleshooting).
(Reference: https://github.com/espressif/arduino-esp32/issues/8481#issuecomment-1665794171)
