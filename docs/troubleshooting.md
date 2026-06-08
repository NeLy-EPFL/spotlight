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

## `run-spotlight` may hang on quit (needs a manual kill)

The quit-robustness commit `d2b38006` ("make program quit more robust to
error") was **reverted on 2026-06-08** by request, to keep things simple for
now. With it reverted, shutdown is back to the older behavior:

- `ArduinoCommunication`'s destructor joins the comm thread *without* signaling
  it to stop first, and `MuscleCamera`'s destructor does an unbounded blocking
  `waitpid` on the PCO camera server.
- So if the comm thread or the PCO server does not exit promptly, `run-spotlight`
  can hang on quit and has to be killed manually:

  ```bash
  pkill -9 run-spotlight
  ```

This is accepted for now. Note that being killed (rather than quitting cleanly)
is itself an unclean exit, which can leave the behavior grabber reserved until
the next launch's discovery — see the section above. If quit-on-exit robustness
becomes worth revisiting, restore `d2b38006`.
