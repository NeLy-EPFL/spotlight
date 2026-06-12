# Quality-of-life improvements

This page documents conveniences added in the current refactor branch that are easy to miss.

## Adjust dynamic range of muscle image during live preview

In the `run-spotlight` GUI, a histogram is now displayed under the muscle live preview widget when muscle imaging is enabled. The user can move the two sliders on the histogram to adjust the minimum and maximum pixel values that map to full-black and full-white in the live preview.


## Status display on the trigger controller

On the physical trigger control box, a display now shows the program status, behavior camera frame rate, behavior-muscle synchronization ratio, behavior camera exposure time, and muscle camera effective exposure (light-on) time.


## Functional on/off switch

The on/off switch on the trigger control box now logically disables triggering and physically cuts 24V power for triggering the CCS light controller. Consequently, at the end of an experiment, the user only needs to flip this switch. The USB-C cable and main power supply for the light controller can remain plugged in.


## Save-directory path increment in the GUI

The **Increment** button in the `run-spotlight` increments the numbers at the end of the save directory until such path is free. If the path does not end with a number ,a number is appended to the path.

When the user clicks **Record**, if the save directory exists or is non-empty, a pop-up window appears, giving the user the options of aborting, auto-incrementing the save path, or overwriting existing data.


## Consolidated behavior camera configuration

Previously certain parameters of the behavior camera and frame grabber were configured separately in the EGrabber Studio GUI, specifically by running a `config.js` script. This is no longer necessary, as the host-side C++ programs do this through the EGrabber C++ SDK.


## `reset-camera` — OS-level camera recovery

In case something goes wrong and the cameras get stuck after an unclean exit, a new `reset-camera` tool can reset the cameras at the OS level. In most cases, the user no longer needs to physically unplug/replug the cameras or restart the computer.

```bash
reset-camera behavior   # PCI reset of the Euresys frame grabber
reset-camera muscle     # disable + re-enable the PCO camera's USB link
```

Note, however, that **this does not replace hardware power cycling**.


> [!NOTE]
> 
> The following improvements are not exposed to the user.
>
> ## Trigger controller auto-reset at startup
> 
> Every host-side program that talks to the trigger controller (`run-spotlight`, `align-cameras`, `run-arena-registration-scan`) now reboots the controller into a clean, known state the moment it opens the serial port (equivalent to physically pressing the reset button on the trigger controller). This makes recovery from bad states (e.g., when host-side programs crash) more robust.
> 
> 
> ## Trigger firmware now built with PlatformIO, not the Arduino IDE
> 
> The trigger firmware (`trigger_firmware/`) is built and flashed with **PlatformIO** instead of the Arduino IDE.  PlatformIO resolves the board, toolchain, and library dependencies automatically from `trigger_firmware/platformio.ini`, which simplifies library installation. See [Installation & compilation](../developers_manual/installation_compilation.md#step-3-build-and-upload-trigger-firmware) for how to compile and upload .
> 
> 
> ## More robust state control for the muscle camera
> 
> The muscle camera server now calls `PCO_InitializeLib()` and `PCO_CleanupLib()` to ensure proper global state of the PCO camera. This reduces the frequency of having to power cycle the camera.
> 
> 
> ## Integrated motion stage configuration
> 
> Previously, certain parameters of the Zaber motion stages (e.g., speed limit, acceleration, ramp time) were configured separately in the Zaber Launch GUI. These are now set inf `recorder_config.yaml` and applied to the stages upon launch of host-side Spotlight programs, thereby improving the reproducibility of experiments.