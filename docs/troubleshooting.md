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

## `run-spotlight` quit must be a clean `std::exit(0)`, or it strands the behavior camera

**Invariant to preserve:** in `run-spotlight` the CoaXPress grabber is released
only by `~BehaviorCamera`/`~EGrabber`, which run *solely* during the clean
`std::exit(0)` at the end of `quitProgram()`. The quit path calls
`behaviorCamera->stop()`, which stops streaming but does **not** release the
device, and the `behaviorCamera` is never reset — so its destructor runs only
via static teardown at `std::exit(0)`. Any exit that is *not* that clean
`std::exit(0)` — a segfault, or a `kill -9` — skips the release and strands the
grabber mid-stream, leaving the camera unresponsive even to eGrabber Studio until
it is **power-cycled**. (`quitProgram()` is also the `SIGINT` handler, so it must
never be allowed to wedge such that a second `^C` re-enters it mid-`std::exit()`,
which is undefined behavior and segfaults.)

Keeping that exit clean requires shutdown to be **bounded** — no step may block
forever. This is exactly what the quit-robustness commit `d2b38006` ("make
program quit more robust to error") ensures, and why these three properties must
be preserved:

- `MuscleCamera::stop()` SIGTERMs the PCO camera server and then escalates to
  SIGKILL after a grace period, so an unresponsive server can never block quit.
- `~ArduinoCommunication` calls `stopCommunication()` before joining its comm
  thread, so the join cannot deadlock (`quitProgram()` only stops excitation,
  never the comm thread itself).
- `quitProgram()` does **not** reset `muscleRecordingState->muscleCamera`: the
  muscle-acquirer thread dereferences that `shared_ptr` without taking its own
  copy, so destroying the `MuscleCamera` mid-shutdown is a use-after-free.

History: `d2b38006` was briefly reverted on 2026-06-08 (to simplify), which
reintroduced a quit hang (deadlocked Arduino join) and a use-after-free segfault.
Both are unclean exits that stranded the grabber — e.g. the 2026-06-08 19:22 log,
where shutdown hangs, `^C` re-enters `quitProgram()` mid-`std::exit()`, and the
process segfaults. It was **restored on 2026-06-09**. If you simplify shutdown
again, preserve the three bounded-shutdown properties above.

If the camera is already stranded by an earlier unclean exit, power-cycle the
camera (and grabber) to clear it.
