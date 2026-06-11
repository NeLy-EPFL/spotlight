# Quality-of-life improvements

Conveniences added in the current refactor branch that are easy to miss.

---

## `reset-camera` — OS-level camera recovery

**`recorder/scripts/reset-camera`** resets a camera at the OS level when it is
stuck after an unclean exit, without unplugging anything.

```bash
./reset-camera behavior   # PCI reset of the Euresys frame grabber
./reset-camera muscle     # disable + re-enable the PCO camera's USB link
```

Use this when `run-spotlight` (or `align-cameras`) fails to open a camera after
a crash or forced kill.  See [`recorder/reset_camera.md`](recorder/reset_camera.md)
for full details, including when a physical power-cycle is still needed instead.

---

## Trigger controller auto-reset at startup

Every host-side program that talks to the trigger controller
(`run-spotlight`, `align-cameras`, `run-arena-registration-scan`) now reboots
the controller into a clean, known state the moment it opens the serial port.
The communication thread waits for the reboot and reopens the port before
sending any commands, so the first message always reaches a freshly initialised
controller.

In practice this means you **no longer need to manually reset or power-cycle the
Arduino between runs**.  If the previous session left the firmware in a bad state
(e.g. after a crash mid-recording), the next launch cleans it up automatically.

---

## Save-directory path increment in the GUI

The save-directory row in `run-spotlight` has two new affordances for bumping the
recording number without retyping the full path.

**"Increment" button** — click it at any time to advance the trailing number in
the current path (e.g. `fly1b_001` → `fly1b_002`).  If the path has no trailing
number, `_001` is appended.  Already-occupied directories are skipped
automatically, so the button always lands on a free slot.

**"Auto increment" in the overwrite dialog** — if you click Record and the
directory already exists and is non-empty, a dialog appears offering three
choices: *Overwrite*, *Auto increment*, and *Cancel*.  Choosing *Auto increment*
applies the same skipping logic and updates the path field, after which the
recording proceeds immediately into the new directory.
