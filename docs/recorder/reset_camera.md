# `reset-camera`

A recovery helper (`recorder/scripts/reset-camera`) that resets a camera at the
OS level when it is stuck after an unclean exit, without unplugging anything. The
behavior camera is reset over PCI; the muscle camera's USB link is disabled and
re-enabled.

```bash
./reset-camera behavior   # Euresys frame grabber (PCI reset)
./reset-camera muscle     # PCO camera (USB link disable/re-enable)
```

To be absolutely sure that you're **physically** power cycling the cameras, unplug and replug the USB cable for the muscle camera, and restart the computer for the behavior camera (but don't use "restart"; instead switch the computer off and on again manually to make sure power is cut).
