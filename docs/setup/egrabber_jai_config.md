# eGrabber and JAI camera configuration

The Euresys frame grabber and the JAI camera communicate over the
[GenICam](https://en.wikipedia.org/wiki/GenICam) interface. There is a large set of
parameters you can tune through this interface (e.g. triggering mode, exposure).
You can set them through programmatic libraries (eGrabber, or any other GenICam
machine-vision library), by running a script (in eGrabber Studio you can run a
JavaScript file to set things up), or manually through the eGrabber Studio GUI.

> [!TIP]
> Refer to the
> [GenICam Standard Features Naming Convention (SFNC)](https://www.emva.org/wp-content/uploads/GenICam_SFNC_v2_7.pdf)
> ([Internet Archive link](https://web.archive.org/web/20250206165851/https://www.emva.org/wp-content/uploads/GenICam_SFNC_v2_7.pdf))
> if the names and values of different features prove too cryptic.

> [!CAUTION]
> With GenICam, the **sequence** of operations matters. For example, to set TTLIO12
> as an input line, you first set `LineSelector` to "TTLIO12" and *then* set
> `LineMode` to "Input". If you had set `LineSelector` to "TTLIO11" first, you'd be
> changing a different line's behavior by setting the same `LineMode` parameter.

> [!NOTE]
> The recorder applies this configuration programmatically at startup via the
> `BehaviorCamera` class (`src/peripherals/behavior_camera.cc`); these settings are
> not something you normally tweak by hand. This page documents what that
> configuration does and how to reproduce/verify it in eGrabber Studio.

## Triggering configuration

The behavior camera is **externally triggered**: the trigger microcontroller
(Arduino Nano ESP32) generates an LVTTL (3.3 V) trigger signal, which is passed to
the frame grabber via TTLIO (here TTLIO12, through the external IO connector), and
the frame grabber forwards it to the camera. The exposure time is controlled by the
trigger width (whenever the trigger signal is HIGH, the shutter is open, and vice
versa). This camera-side arrangement has not changed across versions; what changed is
the trigger controller's firmware logic (see [Data acquisition](../data_acquisition.md)),
not how the grabber and camera are configured here.

The following configuration is required.

> [!WARNING]
> The behavior can still vary depending on other parameters; these are the ones
> that are relevant.

- Tell the frame grabber to treat TTLIO12 as an input line:
  - Under Interface ("InterfaceModule" in the eGrabber C++ API), set `LineSelector`
    to "TTLIO12".
  - Under Interface, set `LineMode` to "Input".
- Set LIN1 to TTLIO12:
  - Under Interface, set `LineInputToolSelector` to "LIN1".
  - Under Interface, set `LineInputToolSource` to "TTLIO12".
- Tell the camera to rely on external control:
  - Under Device ("DeviceModule"), set `CameraControlMethod` to "EXTERNAL".
- Disable trigger control for `AcquisitionStart` and `AcquisitionEnd`.
  Counterintuitively, these must be turned off because they are separate signals
  that tell the camera whether it's "acquiring" at all. If it's not, no image is
  taken even if the `FrameStart` trigger is active.
  - Under Remote Device ("RemoteModule"), set `TriggerSelector` to
    "AcquisitionStart", then set `TriggerMode` to "Off".
  - Under Remote Device, set `TriggerSelector` to "AcquisitionEnd", then set
    `TriggerMode` to "Off".
- Enable trigger control for `FrameStart` (this controls when the camera opens the
  shutter):
  - Under Remote Device, set `TriggerSelector` to "FrameStart".
  - (You don't need to set `TriggerMode` to "On" — it's grayed out, since
    `TriggerMode` must be "On" when `CameraControlMethod` is "EXTERNAL".)
  - Under Remote Device, set `TriggerSource` to "CXPin".
- Set exposure control to trigger width (shutter open iff TTL signal is HIGH):
  - Under Remote Device, set `ExposureMode` to "TriggerWidth".

To verify: click the ▶️ button in eGrabber Studio, click the ℹ️ button on the
display for real-time statistics, and verify the real FPS agrees with what's set on
the microcontroller. Then change the trigger width on the microcontroller to see
whether the image gets brighter or dimmer.

A sample boilerplate program is available at `opt/examples/egrabber_cpp_boilerplate`:

> [!IMPORTANT]
> The camera's data stream can only be accessed by one program at a time. You must
> close eGrabber Studio before running the test C++ program (and before running
> `run-spotlight`).

```bash
cd opt/examples/egrabber_cpp_boilerplate
mkdir build
cd build
cmake ..
make
./main
# press ESC to exit
```
