# Bug & code-health findings

Review of `spotlight-control` on branch `refactor260601`. Focus on correctness
and multithreading. Each item lists the location, what is wrong, why it matters,
and a suggested fix. Severity is my own judgement; nothing here is a confirmed
field failure, but the high/medium items are real defects rather than style.

The documentation and naming fixes I applied while reviewing are listed at the
bottom (separate from the bugs).

## Status (all fixes applied and verified)

All items below are **fixed** except #5, which the maintainer asked to leave as
is. The recorder builds clean and all 42 hardware-independent tests pass.

| # | Item | Status |
|---|------|--------|
| 1 | Muscle lost-wakeup / dropped-frame detection | Fixed |
| 2 | Unreachable / deadlock-prone join cleanup | Fixed (committed to the `std::exit` teardown model; dead join block + helper removed) |
| 3 | `std::exit`/unsafe work in SIGINT handlers | Fixed (handlers now only set a flag) |
| 4 | Data race publishing the camera `shared_ptr`s | Fixed (`std::atomic<std::shared_ptr<...>>`) |
| 5 | Timestamp-based frame identity in arena scan | **Left as-is per maintainer** |
| 6 | `FrameData::frame_id` type/default | Fixed |
| 7 | Dead `BehaviorCamera` members | Fixed |
| 8 | `/* */` doc-comment blocks | Function-doc blocks converted to `//`; file-header banners left (the style rule is scoped to header declarations) |

---

## High / medium-high

### 1. Muscle frames can be silently lost (and signals lost) in `MuscleCamera::wait_for_one_frame()`
**Files:** `recorder/src/peripherals/muscle_camera.cc:233-274` (consumer),
`recorder/src/apps/pco_camera_server_main.cc:386-400` (producer).

The muscle camera is a single-slot shared-memory handoff: the server overwrites
one frame buffer and `pthread_cond_signal`s; the consumer waits and clones the
*latest* buffer. Two problems compound:

- **Lost wakeup.** `wait_for_one_frame()` calls `pthread_cond_wait()`
  unconditionally rather than looping on a predicate. POSIX condition variables
  do **not** latch: if the server signals in the window between the consumer
  unlocking (end of the previous iteration) and re-entering `pthread_cond_wait`,
  that signal is missed and the consumer blocks until the *next* frame. The
  `frame_count == last_frame_count_` check after waking only filters spurious
  wakeups; it cannot recover a missed signal.
- **Dropped frames.** If the consumer (acquirer + saver pipeline) ever falls
  behind the free-running camera, the server overwrites the buffer with newer
  frames before the consumer reads it. The consumer then sees `frame_count` jump
  by more than one, but the muscle acquirer assigns its *own* contiguous
  `frame_id` (`recorder/src/common/muscle_recording.cc:144`). The result during
  a recording is fewer saved muscle frames than the muscle-frame indices the
  firmware and the behavior side assume, so the behavior↔muscle frame
  correspondence drifts with no error raised.

In steady state the muscle rate is low (behavior_fps / sync_ratio) and the clone
is fast, so this is rare — but it is a data-integrity risk under disk pressure,
and the lost-wakeup pattern is a latent bug regardless of rate.

**Fix:** make the consumer wait on a predicate under the lock
(`while (frame_metadata_ptr_->frame_count == last_frame_count_) pthread_cond_wait(...)`),
which closes the lost-wakeup window. To make skips *detectable* rather than
silent, have the server expose its monotonic `frame_count` and the consumer
propagate it (or warn) when it advances by more than one, instead of renumbering
from zero.

---

## Medium

### 2. The graceful thread-join shutdown in `run-spotlight` is unreachable, and would deadlock if reached
**File:** `recorder/src/apps/run_spotlight_main.cc:140-191` (`quit_program`),
`:418-441` (join block).

`quit_program()` ends with `std::exit(0)`, and it is the only way out of the
program: it is called from `MainGUIWindow::closeEvent` (`gui.cc:1197`) and from
the `SIGINT` handler (`run_spotlight_main.cc:194`). Nothing calls
`QApplication::quit()`, so `application->exec()` never returns and the entire
`join_if_joinable(...)` cleanup block (lines 420-440) is dead code.

Were that block ever reached, it would hang:
- The muscle acquirer is parked in `wait_for_one_frame()`'s `pthread_cond_wait`
  after the PCO server is SIGTERMed; nothing signals the shared cond var again
  (already acknowledged in the comment at `:163-169`), so its join never
  returns.
- The motion-control client waits
  (`tracking_control.cc:612, 665, 694, 742, 768`) have no `to_quit` escape in
  their predicates. If the handler thread exits first, a client blocked there
  waits forever.

So shutdown works only because the process exits and abandons the threads. This
is functional but fragile: anyone who removes the `std::exit` (e.g. to "clean up
properly") gets a deadlock.

**Fix:** pick one model. Either keep `std::exit` and delete the misleading join
block, or make shutdown real: add `program_state->to_quit` to the
`response_cond_var` wait predicates, make the muscle wait interruptible (e.g.
`pthread_cond_timedwait` checking a shutdown flag), and let `exec()` return so
the joins run.

### 3. `std::exit()` and non-async-signal-safe work run inside the SIGINT handler
**Files:** `recorder/src/apps/run_spotlight_main.cc:194`,
`recorder/src/apps/run_arena_registration_scan_main.cc:47-59, 559`.

The `SIGINT` handlers call `quit_program()` directly, which logs via spdlog,
locks mutexes, calls `kill`/`waitpid`, and `std::exit()` — none of which is
async-signal-safe. If Ctrl-C arrives while a lock inside spdlog or the allocator
is held, the handler can self-deadlock or corrupt state.

**Fix:** the handler should only set an atomic flag (e.g.
`program_state->to_quit`, plus a `std::sig_atomic_t`), and let the normal
main/Qt path perform the cleanup.

### 4. Data race on the camera `shared_ptr`s during publication
**Files:** writers `recorder/src/common/behavior_recording.cc:17`,
`recorder/src/common/muscle_recording.cc:109`; readers
`recorder/src/common/tracking_control.cc:207-208`,
`recorder/src/apps/gui.cc:906`, `recorder/src/apps/run_spotlight_main.cc:170, 360`.

`behavior_recording_state->behavior_camera` and
`muscle_recording_state->muscle_camera` are plain `std::shared_ptr`s assigned by
the acquirer threads while other threads read them with no synchronization:
- behavior camera: read by the tracking thread and the GUI;
- muscle camera: polled by the init wait loop (`run_spotlight_main.cc:360`) and
  read in `quit_program`.

Concurrent read/write of a non-atomic `shared_ptr` is a data race (undefined
behavior). The `is_ready()` atomic guards the camera's *state*, not the pointer
load. Benign on x86 in practice, but formally UB and a real tear risk on the
control block.

**Fix:** use `std::atomic<std::shared_ptr<...>>` (C++20, available here) for
these handles, or publish the pointer under a mutex and only read it after
observing an atomic "ready" flag that is stored *after* the pointer.

---

## Low

### 5. `wait_for_next_frame` distinguishes frames by timestamp, not identity
**File:** `recorder/src/apps/run_arena_registration_scan_main.cc:120-131`.

A "new" frame is detected purely by `received_time` differing from the previous
value. `received_time` is a microsecond timestamp; two frames sharing the same
microsecond would be treated as one and the loop would wait an extra cycle.
Harmless at scan rates, but a monotonic frame counter would be robust.

### 6. `FrameData::frame_id` type/default
**File:** `recorder/include/recorder/common/data_types.h:7`.

`unsigned int frame_id = -1;` initializes to `UINT_MAX`, and it is assigned from
`long int current_frame_id` (`behavior_recording.cc:95`,
`muscle_recording.cc:144`). No practical impact (you would need billions of
frames to wrap), but the signed→unsigned default and the type mismatch are worth
tidying: use a consistent type and default to `0`.

### 7. Dead members in `BehaviorCamera`
**File:** `recorder/include/recorder/peripherals/behavior_camera.h:42, 36`.

`current_fps_` is declared but never initialized or used. `format_converter_ptr_`
is constructed (`behavior_camera.cc:34`) but never used afterwards. Both can be
removed (drops the otherwise-unnecessary `FormatConverter` construction).

### 8. Remaining `/* */` doc-comment blocks violate the documented style
**Files:** `recorder/src/apps/run_spotlight_main.cc:36, 128, 141`,
`recorder/src/apps/run_arena_registration_scan_main.cc:1`,
`recorder/src/apps/gui.cc:1364` (`parse_protocol_string`, also uses `@brief`-style
prose).

`docs/code_style.md:37` requires `//` comment lines, not `/* */` blocks, for
documentation. Several Doxygen-style `/** ... */` blocks remain. I converted the
one in `tracking_control.cc`; the rest are listed here rather than changed in
bulk, since you run formatting passes yourself.

---

## Documentation & naming fixes applied in this pass

These are the task-1/2/3 cleanups (not bugs), done directly:

- **`README.md` was stale on the muscle-camera architecture.** It described the
  PCO camera as driven *in-process* and claimed the separate `pco-camera-server`
  process "has been removed" — the exact opposite of the actual code
  (`muscle_camera.cc` still `fork()`+`execl()`s `pco-camera-server`, and
  `CMakeLists.txt` / `docs/architecture.md` agree). This matches the abandoned
  in-process refactor ("Stage B"). Rewrote the README section and the
  architecture-overview bullet to describe the separate-process design and point
  to `docs/architecture.md`.
- **Naming consistency** (codebase was renamed to snake_case). Updated stale
  camelCase references to renamed internal symbols in user-facing log strings and
  comments: `runSpotlight`/`alignCameras`/`arenaRegistrationScan`/`pcoCameraServer`
  startup log lines; `stageMinX/...` clip labels; and `saveDirectory`,
  `reorientBehaviorImage`, `parseProtocolString`, `camFlushTimeUs`,
  `behaviorImageAcquirer`, `buildRecordingParams`, `confirmOrResolveSaveDirectory`,
  `checkIfMotionStageIdle`, `waitUntilMotionStageIdleSync` references in comments.
  (JSON wire-format keys such as `enableMuscle`, `behFrameRate`, `frameIdx` are
  intentionally camelCase and were left unchanged.)
