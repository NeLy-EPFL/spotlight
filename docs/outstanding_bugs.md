# Outstanding bugs

Findings from a code + docs review of the **in-process muscle-camera refactor**
(branch `refactor260601`), recorded 2026-06-10. The refactor moved the PCO muscle
camera from a separate `pco-camera-server` process into the `MuscleCamera` class
driven in-process by an acquirer thread (see
[muscle_camera_inprocess_refactor.md](muscle_camera_inprocess_refactor.md) and
[architecture.md](architecture.md)).

Status legend: **OPEN** = not yet fixed; **FIXED** = resolved on this branch.
Bugs #1 and #2 below have been FIXED (2026-06-10); bugs #3–#7 remain OPEN. The
documentation issues at the bottom were FIXED when this file was created.

---

## Critical

### 1. Disabling muscle imaging permanently kills the muscle acquirer thread — FIXED 2026-06-10

`MuscleCamera::waitForOneFrame()` returns `std::nullopt` for **two** different
reasons, and its header contract spells this out
(`include/recorder/peripherals/muscle_camera.h:125-131`):

> Returns `std::nullopt` when there is no frame to return — because `stop()` has
> been called (shutdown) **or because the camera is currently disabled** (see
> `setEnabled`). Callers distinguish the two via their own shutdown flag (the
> muscle acquirer checks programState's `toQuit`).

The acquirer does **not** distinguish them. It treats every `nullopt` as shutdown
and breaks out of its loop (`src/common/muscle_recording.cc:136-141`):

```cpp
std::optional<FrameData> maybeFrame = muscleRecordingState->muscleCamera->waitForOneFrame();
if (!maybeFrame.has_value()) {
    // Camera was stopped (shutdown) -- exit the loop.
    break;          // BUG: also taken when merely *disabled*, not just on shutdown
}
```

Startup sequence that triggers it:

1. `MuscleCamera::Impl` defaults `desiredEnabled = true` and starts recording in
   the constructor, so the acquirer initially produces frames.
2. The GUI is built after the camera is ready, and `createMusclePreviewColumn()`
   calls `muscleRecordingState_->muscleCamera->setEnabled(muscleImagingCheckBox_->isChecked())`
   (`src/apps/gui.cc:664`). The checkbox is unchecked by default
   (`src/apps/gui.cc:628-629`), so this is **`setEnabled(false)`**.
3. The acquirer's next `waitForOneFrame()` takes the disabled path, returns
   `nullopt`, and the acquirer **`break`s — the thread exits permanently.**

Because muscle imaging is off by default, the muscle acquirer dies within a few
hundred ms of **every** launch. When the user later ticks "Enable muscle imaging",
`setEnabled(true)` only sets a flag and notifies a condition variable, but no
thread is alive to observe it. Net effect: the muscle preview stays blank, the
camera is never restarted, and a muscle recording saves an **empty**
`muscle_images/`. The GUI's own comment (`src/apps/gui.cc:660-665`, "the muscle
camera starts stopped and only runs once the user enables imaging") describes the
intended behavior that this `break` defeats.

`align-cameras` is unaffected only because it never calls `setEnabled(false)` (no
checkbox), so its `nullopt` always means shutdown.

**Fixed (2026-06-10):** `src/common/muscle_recording.cc` now does `continue`
instead of `break` on a `nullopt`, exactly as the header contract prescribes. A
disable merely pauses the acquirer (it resumes when re-enabled); shutdown still
exits via the `while (!programState->toQuit.load())` condition, which is safe
because every shutdown path sets `toQuit` *before* calling `MuscleCamera::stop()`.
Builds and the 42 hardware-independent unit tests pass. **Still needs an on-rig
retest** of enable → disable → re-enable and of a muscle recording producing the
expected frame count — the acquirer loop is not exercised by the offline tests.

---

## Medium

### 2. `align-cameras` and `run-arena-registration-scan` hang forever if a camera fails to initialize — FIXED 2026-06-10

`run-spotlight` guards its "wait for camera ready" loop with
`&& !programState->toQuit.load()` (`src/apps/run_spotlight_main.cc:348-352`), so
when an acquirer catches a camera-open failure and sets `toQuit`, the wait exits
cleanly and falls through to teardown. The other two entry points were **not**
given this guard:

- `src/apps/align_cameras_main.cc:238-241` (behavior-camera wait) and
  `:260-267` (muscle-camera wait) — neither checks `toQuit`.
- `src/apps/run_arena_registration_scan_main.cc:463-468` (behavior-camera wait) —
  does not check `toQuit`.

When the relevant camera throws during open (a stranded grabber, the PCO camera
not connected, etc. — all documented in [troubleshooting.md](troubleshooting.md)),
the acquirer sets `toQuit` and returns, but these loops spin forever, only logging
a periodic warning. The user's only escape is Ctrl-C, and since these two tools
install no SIGINT handler, that is an unclean exit that can **strand the CoaXPress
grabber** — the exact failure mode the troubleshooting page warns about (an exit
that skips `~BehaviorCamera`/`~EGrabber` leaves the grabber held until a
power-cycle).

**Fixed (2026-06-10):** all three wait loops now check
`&& !programState->toQuit.load()`. `align-cameras` gained a single `shutdown`
lambda — stop both grabs → join both acquirers → null the cameras → switch off
excitation / close the Arduino link if it was started — used for both the normal
exit and an early abort from either wait loop (its muscle-thread handle is declared
up front so the lambda can join it even on an early abort). `run-arena-registration-scan`
bails from its wait loop by stopping and joining the behavior acquirer before
returning (triggering/motion aren't set up yet). A camera-init failure now aborts
cleanly instead of hanging — which previously forced a Ctrl-C that could strand the
CoaXPress grabber. Builds. **Still needs an on-rig retest** of the failure path
(e.g. launch `align-cameras` with the PCO camera unplugged).

---

## Low

### 3. Data race on the `behaviorCamera` / `muscleCamera` shared pointers — OPEN

The acquirer threads assign `behaviorRecordingState->behaviorCamera`
(`src/common/behavior_recording.cc:26`) and
`muscleRecordingState->muscleCamera` (`src/common/muscle_recording.cc:107`) while
the main thread reads them in its wait loops (`src/apps/run_spotlight_main.cc:348`,
`:398`). These are plain `std::shared_ptr` (`include/recorder/common/muscle_recording.h:18`,
and the behavior equivalent), so concurrent read/write is a data race (UB). It
works in practice on x86 (a single aligned pointer store), but it is
ThreadSanitizer-positive, and the intended synchronization point (`isReady()`, an
atomic) cannot be reached without first racily reading the pointer.

**Fix direction:** publish readiness through a separate `std::atomic<bool>` in the
shared state (or `std::atomic<Camera*>`), and have the main thread poll that rather
than the `shared_ptr`.

### 4. `BehaviorCamera::isReady()` becomes true before `start()` runs — OPEN

The constructor sets `cameraReadyFlag_ = true` at its end
(`src/peripherals/behavior_camera.cc:55`), but the acquirer calls
`behaviorCamera->start()` only *after* construction returns
(`src/common/behavior_recording.cc:33`). So `isReady()` can report ready while the
grabber is not yet streaming, and a `start()` failure is never reflected by
`isReady()`. The GUI gates `startRecording()` on `isReady()`
(`src/apps/gui.cc:899-900`). The hazard is mostly masked by downstream `toQuit`
checks, but the readiness signal does not mean "ready to grab" as callers assume.

**Fix direction:** set the ready flag after `start()` succeeds (e.g. move
`start()` into the camera and flip the flag at the end), or expose a separate
"streaming" predicate.

### 5. Transient behavior-frame stall when enabling muscle imaging or starting a muscle recording — OPEN

`setEnabled()` and `setNominalExposureUs()` only post a flag/value; the acquirer
applies the change via stop → reconfigure → restart on its next iteration. The GUI
then tells the controller to start locking behavior frames to the muscle camera's
common-time (SMA #4) onsets — `setEnabled(enabled)` immediately followed by
`stream(...)` (`src/apps/gui.cc:647`, `:657`), and `setNominalExposureUs(...)`
immediately followed by `startRecording(...)` (`src/apps/gui.cc:936`, `:958`) —
before the camera has necessarily resumed driving SMA #4. Unlike the Stage-A
deadlock this is self-correcting (the camera spins up within a grab cycle, and the
controller's `camFlushTimeUs` delay covers part of it), but expect a brief
behavior-preview hiccup on toggle / record start.

**Fix direction:** confirm `camFlushTimeUs` comfortably exceeds a PCO
`stop → setExposureTime → record` cycle; if not, have the GUI wait for the camera
to confirm it is recording before issuing the controller command.

### 6. `computeDefaultExposureUs` can underflow — OPEN

`src/peripherals/muscle_camera.cc:300-301` computes
`defaultMuscleIntervalUs - static_cast<unsigned int>(sensorReadoutTimeUs)` in
`unsigned int`. If a config gives a streaming muscle interval shorter than the
sensor readout time, this wraps to a huge value that is then programmed as the
exposure, with no error. The analogous GUI path
(`pushMuscleCameraExposure`, `src/apps/gui.cc:1131-1138`) guards this case with a
clear log; the constructor path does not.

**Fix direction:** validate `muscleInterval > readout` in the constructor path and
fail with a clear message (mirroring `pushMuscleCameraExposure`).

### 7. (Pre-existing) Unsynchronized `ProgrammedStop` frame counts — OPEN

`ProgrammedStop::numBehaviorFramesExpected` / `numMuscleFramesExpected` are plain
`int`s (`include/recorder/common/data_types.h:78-79`) written by the GUI thread
(`src/apps/gui.cc:848-856`) and read by the acquirer threads
(`src/common/behavior_recording.cc:100-101`, `src/common/muscle_recording.cc:150-151`).
This is unsynchronized cross-thread access. It is probably safe in practice because
the GUI writes these *before* it stores `programState->isRecording` (an atomic,
release) and the acquirers read them *after* loading `isRecording` (acquire), which
establishes the needed happens-before — but the ordering is implicit and fragile.
Not introduced by this refactor.

**Fix direction:** make the counts atomic, or document/enforce the
`isRecording`-ordering invariant.

---

## Documentation issues — FIXED 2026-06-10

These were corrected in the change that added this file.

- **`README.md` listed `run-homography-scan` among "four user-facing programs"**
  with no caveat, but that program does not exist (no source, no CMake target).
  `docs/README.md` and `docs/configuration/camera_homography.md` correctly mark it
  *"TODO — not yet implemented"*, and `architecture.md` says **three** programs.
  Fixed: `README.md` now says three programs and marks `run-homography-scan` as
  planned/not-yet-implemented.
- **`docs/troubleshooting.md` stated the minimum muscle ROI is "64×18 px".** The
  code enforces 64×**16** (`src/peripherals/muscle_camera.cc:273-275`, whose own
  critical message says "64x16"), and 18 is not a multiple of 8. Fixed to 64×16.
- **`src/apps/run_spotlight_main.cc` `quitProgram()` comment** claimed the behavior
  acquirer unblocks "because the trigger controller is still streaming … so that
  loop unblocks promptly." That reasoning is stale: the behavior grab is now bounded
  (200 ms `ScopedBuffer` pop) and `cancelPop()`-interruptible, so it returns and
  observes `toQuit` regardless of whether frames arrive (as
  `troubleshooting.md` already describes). Comment updated; no behavior change.
- **`docs/recorder/run_spotlight.md`** described "a configuration dialog [that]
  sets the recording parameters before the main window opens." There is no such
  pre-launch dialog; those controls are inline in the main window. Section
  rewritten to match the actual GUI.
</content>
</invoke>
