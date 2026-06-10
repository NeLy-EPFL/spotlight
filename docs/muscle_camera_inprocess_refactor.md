# Can the PCO muscle camera be acquired in-process (no separate server)?

Design note / investigation. **Status: IMPLEMENTED (2026-06-09).** The muscle
camera is now driven in-process by `MuscleCamera` (`src/peripherals/muscle_camera.cc`,
pimpl'd over the PCO SDK); the `pco-camera-server` target, `pco_camera_server_main.cc`,
and the `shared_memory_utils` / `PCOSharedMemory` plumbing have been removed, and
`PCO_LINUX=1` is scoped per file. Parameter changes (exposure / muscle frame rate)
are applied by the acquirer thread via stop->reconfigure->restart, and shutdown is
join-based (the acquirer is joined before the camera is destroyed). The analysis
below is kept for context. See `README.md`, `docs/data_acquisition.md`, and the
quit section of `docs/troubleshooting.md` for the current behaviour.

## Question

Today the muscle (PCO) camera runs as its own executable, `pco-camera-server`,
which the recorder launches with `fork()`+`execl()` and talks to over shared
memory. The behavior (Euresys) camera, by contrast, is linked straight into the
recorder and driven in-process via the `BehaviorCamera` class. Can the muscle
camera be done the same way -- one binary, no separate `main`/CLI/target -- or is
the separate process actually required?

## Short answer / verdict

**Yes, it can be brought in-process. The separate process is a hygiene/packaging
choice, not a hard technical requirement.** The original justification ("requires
a different software stack, which makes compiling it with the other programs
unnecessarily complicated", see `README.md`) does not hold up as a *compile-time*
incompatibility:

- The PCO headers select their Linux paths with `PCO_LINUX`, which makes
  `pco_linux_defs.h` inject Windows-compatibility typedefs/macros
  (`BOOL`/`BYTE`/`WORD`/`DWORD`/`HANDLE`/..., `FALSE`/`TRUE`/`MAKEWORD`/`far`/
  `MAX_PATH`/...) into the global namespace of every TU that includes a PCO
  header. This is real global-namespace pollution, **but it was empirically
  verified not to clash**: the full PCO SDK headers compile cleanly in a single
  translation unit alongside Qt 6.8.2, OpenCV 4, and the Euresys EGrabber headers
  (tested with `g++ -std=c++23 -fsyntax-only`; zero errors).
- So there is no proven hard incompatibility forcing the split. What remains is
  (a) keeping that pollution out of the GUI/tracking code, and (b) a couple of
  link/runtime details below that are manageable.

There is **one genuine runtime hazard to design around** (Qt/ICU version skew in
the PCO lib dir), but it does not require a separate process -- just careful
linking. Details below.

## Current architecture (what exists today)

**Separate executable + shared memory:**

- Target `pco-camera-server` (`recorder/CMakeLists.txt:103`): links the bundled
  PCO SDK sources (`PCO_CAMERA_SOURCES` = `camera.cpp`, `image.cpp`, `xcite.cpp`,
  `cameraexception.cpp`, `stdafx.cpp`) plus `pco_sc2cam`/`pco_recorder`/
  `pco_convert`/`etc_xcite`. `PCO_LINUX=1` is scoped to this target only
  (`recorder/CMakeLists.txt:112`).
- `main` + CLI + acquisition loop: `recorder/src/apps/pco_camera_server_main.cc`
  (~467 lines). Opens the camera, sets ROI/trigger/exposure, runs a blocking
  acquire loop, and `memcpy`s each frame into shared memory under a mutex +
  condition variable.
- Client wrapper: `MuscleCamera` (`recorder/src/peripherals/muscle_camera.cc`,
  `include/recorder/peripherals/muscle_camera.h`). Its constructor `fork()`s and
  `execl()`s `pco-camera-server` (resolved next to `/proc/self/exe`), passing ROI
  and verbosity on the command line, then maps five shared-memory regions.
  `waitForOneFrame()` blocks on the shared condition variable and returns the
  latest frame; the destructor `SIGTERM`s the child.
- Shared-memory plumbing: `PCOSharedMemory` namespace in
  `recorder/src/apps/shared_memory_utils.cc` (frame data, shutter-open time,
  frame metadata, mutex, cond var). Region names come from `recorder_config.yaml`
  under `muscle_camera`.
- `MuscleCamera` is created and pumped from a dedicated **muscle image acquirer
  thread** (`recorder/src/common/muscle_recording.cc:106`, loop at line ~121),
  which already calls `waitForOneFrame()` in a loop -- structurally identical to
  how an Euresys-style in-process camera would be driven.

**Consumers of `MuscleCamera`** (everything that would be touched by a refactor):

- `recorder/src/common/muscle_recording.cc` (constructs it, runs the acquirer
  loop).
- `recorder/src/apps/run_spotlight_main.cc:48-53` (on shutdown it pulls
  `getCameraServerPID()` and `kill()`s the server -- this couples shutdown to the
  process model and would change).
- `align-cameras` (`recorder/src/apps/align_cameras_main.cc`,
  `include/recorder/apps/align_cameras.h:19`) also uses the muscle camera for
  live alignment preview, so it spawns the server too.

**Contrast -- behavior camera:** `BehaviorCamera`
(`include/recorder/peripherals/behavior_camera.h`) includes `<EGrabber.h>` and
`<FormatConverter.h>` directly, holds the grabber as a member, and exposes the
same `waitForOneFrame()` interface -- all in-process, no IPC. This is the shape
the muscle camera would take.

## Findings that matter for the refactor

### 1. No compile-time blocker (verified)
PCO SDK headers + Qt6 + OpenCV + Euresys coexist in one TU with no errors. See
the rewritten rationale in `recorder/CMakeLists.txt` (the `PCO camera SDK` block)
and `recorder/include/recorder/apps/pco_camera_server.h:3-10`.

### 2. The one real runtime hazard: bundled Qt/ICU in the PCO lib dir
`/opt/pco/pco.cpp/lib/` ships its **own** `libQt6*.so.6.5.3` and `libicu*.so.56`.
The recorder GUI links **system Qt 6.8.2** (`/home/spotlight/Qt/6.8.2/gcc_64`).

- Good news: the PCO **camera** libraries the server actually links
  (`libpco_sc2cam`, `libpco_recorder`, `libpco_convert`, `libetc_xcite`) do **not**
  depend on Qt -- their only external `NEEDED` is `libstdc++.so.6` (+ other
  `libpco*`/`libpcocam*` siblings). Verified with `objdump -p`. The bundled Qt6.5.3
  is dead weight for the camera path (it's there for PCO's own GUI tools).
- The hazard: `pco-camera-server` currently sets
  `INSTALL_RPATH "${PCO_LIB_DIR}"` (`recorder/CMakeLists.txt:119-121`). If a
  refactor naively adds `PCO_LIB_DIR` to **`run-spotlight`'s** rpath, the dynamic
  loader could resolve `libQt6Core.so.6` / `libicu*.so.56` from the PCO dir
  instead of system Qt 6.8.2 -- mixing two Qt minor versions in one process, which
  risks ODR violations and crashes.
- Mitigation (all straightforward): link the four PCO camera libs by **absolute
  path** rather than putting the whole `PCO_LIB_DIR` on the rpath; or rpath only a
  curated subdirectory that contains the camera libs but not Qt/ICU; or symlink
  the needed `libpco*`/`libpcocam*` into a dedicated dir and rpath that. The
  bundled Qt/ICU must simply never be on `run-spotlight`'s search path.

### 3. PCO uses C++ exceptions
The acquire loop catches `pco::CameraException` (e.g. timeout code `0x80004001`,
`pco_camera_server_main.cc:301-332`). In-process, the muscle acquirer thread must
catch these the same way. No issue -- just don't let them escape the thread.

### 4. `PCO_LINUX` scoping when merging
`target_compile_definitions(... PCO_LINUX=1)` applies to **every** source in a
target. If the PCO sources move into `run-spotlight`, you do **not** want
`PCO_LINUX` (and the Windows-ism shims) leaking into `gui.cc` and other Qt TUs.
Use **per-file** scoping instead:
`set_source_files_properties(<pco .cpp files> muscle_camera.cc PROPERTIES COMPILE_DEFINITIONS PCO_LINUX=1)`.
Combined with a pimpl (next section) this keeps the shims confined to exactly the
files that include PCO headers.

## Recommended approach

Make `MuscleCamera` an in-process driver that mirrors `BehaviorCamera`, and delete
the server/IPC layer.

- **Pimpl the PCO dependency.** Keep `muscle_camera.h` free of PCO headers (it is
  today). Put the `pco::Camera`, acquire loop, ROI/trigger/exposure setup, and
  `pco::CameraException` handling inside `muscle_camera.cc` (move the bulk of
  `pco_camera_server_main.cc`'s `setupPCOCamera`/`serveFrames` bodies there). Only
  `muscle_camera.cc` and the bundled PCO `.cpp` files then see `PCO_LINUX` and the
  shims.
- **Same public interface.** Keep `MuscleCamera::waitForOneFrame()`,
  `setLightOnTime()`, the ROI/timing helpers, and the `MuscleTriggerTiming` class
  exactly as they are so `muscle_recording.cc` barely changes. Replace the
  shared-memory read in `waitForOneFrame()` with a direct PCO grab + `cv::Mat`
  wrap; replace `setLightOnTime()`'s shared-memory write with a direct
  `camera.setExposureTime(...)`.
- **Drop the process plumbing.** Remove `fork`/`execl`, `getCameraServerPID()`,
  and the `kill(pcoCameraServerPid, ...)` shutdown path in
  `run_spotlight_main.cc:48-53` (replace with the normal `~MuscleCamera()` /
  `muscleCamera = nullptr` teardown already present at line 62). Delete the five
  shared-memory regions and the `PCOSharedMemory` plumbing in
  `shared_memory_utils.cc` if nothing else uses them (grep first -- currently only
  the muscle path does).
- **CMake:** add `PCO_CAMERA_SOURCES`, `PCO_INCLUDE_DIR`/`PCO_CAMERA_DIR`, and the
  four PCO libs to `run-spotlight` and `align-cameras`; scope `PCO_LINUX=1`
  per-file (finding #4); link PCO libs without exposing bundled Qt/ICU on the
  rpath (finding #2). Drop the `pco-camera-server` target, its install entry, and
  `shared_memory_utils.cc` from targets that no longer need it.

### Alternative: keep it separate (do nothing)
The current split is a legitimate, if heavier, design. Reasons one might keep it:
isolating a crash in the vendor SDK to a child process; being able to restart the
camera without restarting the GUI; and a clean firewall around the Windows-ism
namespace pollution and the bundled-Qt/ICU lib dir. None of these are required for
correctness, but they are real robustness arguments. If muscle acquisition has
ever destabilized the GUI process, weigh this before merging.

## Suggested roadmap for the implementing agent

1. **Confirm scope.** `grep -rn "PCOSharedMemory\|shared_memory_utils\|getCameraServerPID\|pco-camera-server"` to enumerate every reference; confirm shared memory is muscle-only.
2. **Introduce in-process driver.** Move `setupPCOCamera` + the acquire/convert
   loop body from `pco_camera_server_main.cc` into `muscle_camera.cc` behind the
   existing `MuscleCamera` interface (pimpl; PCO headers in the `.cc` only). Run
   the acquire loop either inside `waitForOneFrame()` (blocking grab) or on an
   internal thread feeding a 1-slot buffer -- the blocking-grab form is simplest
   and matches `BehaviorCamera`.
3. **Rewire shutdown.** Delete the PID/`kill` path in `run_spotlight_main.cc`; rely
   on `~MuscleCamera()` to `stop()` the camera. Make sure the destructor can
   interrupt a blocked grab (the SDK's `waitForNewImage` timeout pattern already
   used in the server gives you the polling hook).
4. **CMake.** Per-file `PCO_LINUX=1`; add PCO sources/includes/libs to
   `run-spotlight` and `align-cameras`; solve the rpath/Qt-ICU isolation
   (finding #2); remove the `pco-camera-server` target + install rule; prune
   `shared_memory_utils.cc` where unused.
5. **Docs.** Update `README.md` (the `pco-camera-server` bullet and the binary
   table in the top-level `CLAUDE.md`/`AGENTS.md`), and the recording/config
   references to the shared-memory `muscle_camera` keys in `recorder_config.yaml`
   if those regions are removed.
6. **Build + smoke test** on the recording machine with the real camera (this is
   hardware-dependent and cannot be validated offline): verify `run-spotlight` and
   `align-cameras` acquire muscle frames, that the GUI still uses system Qt 6.8.2
   (`ldd` the binary; confirm no `6.5.3`/PCO-dir Qt), and that shutdown is clean.

## Open decisions (for you to pick before implementation)

- **Merge vs keep separate** -- the robustness arguments above are the only thing
  pulling toward "keep separate". If you've never seen the GUI destabilized by the
  PCO SDK, merging is the simpler system.
- **Blocking grab vs internal thread** inside `MuscleCamera` -- blocking is
  simpler and the acquirer thread already exists; an internal thread only helps if
  you want the camera decoupled from the consumer's cadence.
- **Link strategy** for the four PCO libs (absolute-path link vs curated rpath
  subdir) to keep bundled Qt 6.5.3 / ICU 56 off `run-spotlight`'s search path.

## Key files

| Purpose | Path |
|---|---|
| Server `main`/CLI/acquire loop (to fold in) | `recorder/src/apps/pco_camera_server_main.cc` |
| Server header (PCO include order, `PCO_LINUX` fallback) | `recorder/include/recorder/apps/pco_camera_server.h` |
| Muscle camera client (fork/exec -> becomes in-process) | `recorder/src/peripherals/muscle_camera.cc` + `.h` |
| Shared-memory plumbing (to delete) | `recorder/src/apps/shared_memory_utils.cc` + `.h` |
| Acquirer thread / consumer | `recorder/src/common/muscle_recording.cc` |
| Shutdown coupling to server PID | `recorder/src/apps/run_spotlight_main.cc:48-53` |
| Behavior camera (the in-process template) | `recorder/src/peripherals/behavior_camera.cc` + `.h` |
| Build targets / `PCO_LINUX` scoping / rpath | `recorder/CMakeLists.txt` |
