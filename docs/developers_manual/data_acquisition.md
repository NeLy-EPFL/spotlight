# Data acquisition workflow

The cameras acquire frames in the following manner:

- The behavior camera generally records at a higher frame rate than the muscle camera. The synchronization ratio is an integer. In cycles where both cameras expose, they expose at the same time.
- The IR LED is on only when the behavior camera is exposing; the blue LED is on only when the muscle camera is exposing.
- An important nuance: the behavior camera has a global shutter, but the muscle camera has a rolling shutter. The amount of time it takes for the muscle camera to "roll its shutter" is generally longer than the frame interval of the behavior camera (hence the >1 sync ratio). To achieve an _effective_ global shutter for the muscle synchronized to the behavior camera, the blue LED is turned on only during the common time of the rolling shutter (i.e., period of time when all lines of pixels are scanning). In other words, the blue LED is used as the "shutter" instead of the actual camera shutter. The nominal (per-line) exposure time programmed into the muscle camera is therefore at least `rolling_time + effective_exposure_time` (in continuous mode a buffer time is added on top; see below), where `rolling_time = line_time * num_lines`.

The muscle camera always runs in **continuous mode** (defined below). To motivate why, it is useful to first consider the alternative _triggered_ modality and its limitation.

## Motivation: the limitation of triggered acquisition

In a triggered modality, the recorder would instruct the trigger microcontroller to send trigger signals to the behavior camera's frame grabber and the muscle camera at specified frame rates, and the microcontroller would do so accordingly.

The disadvantage of this approach is that the muscle camera ignores any trigger signal marking the start of a new frame cycle _unless_ the previous frame cycle is completed (i.e., the last line of pixels has completed exposure and subsequent data readout). By this time, the first line has already been idle for a period equal to `rolling_time`. This limits the frame rate to `1 / (2 * rolling_time + effective_exposure_time + readout_time)`.

## Continuous mode

To improve the frame rate limit, we always operate the muscle camera in continuous mode: it _continuously_ rolls its shutter, so that as soon as the first line finishes exposure and data readout, its exposure starts for the next cycle. There is no idle time at all.

The disadvantage of this mode is that we cannot trigger the muscle camera externally via TTL; the camera runs its own thing in open loop. This poses two challenges:

1. The triggering controller has to calibrate to the muscle camera to avoid cumulative error.
    - To mitigate this, we configure the muscle camera to produce a status signal that is ON (HIGH) during the common time of all frames, and OFF (LOW) otherwise. When this signal switches to ON from OFF, the microcontroller triggers a frame for the behavior camera, and switches the lights on for the desired exposure time. Then, the microcontroller runs `sync_ratio - 1` frame cycles for the behavior camera using its own clock, and then waits for the next signal from the muscle camera.
2. We can't set the muscle camera's frame rate independently of its exposure time, since the frame rate is always `1 / (rolling_time + exposure_time + readout_time)`.
    - To mitigate this, we lengthen the camera's nominal (per-line) exposure beyond what the effective exposure alone requires. The nominal exposure is `rolling_time + common_time`, where `common_time = effective_exposure_time + buffer_time` is the duration over which all lines expose simultaneously (and over which the camera asserts its common-time status signal on SMA #4). `buffer_time` is the slack chosen so that the resulting frame rate `1 / (rolling_time + effective_exposure_time + buffer_time + readout_time)` equals the desired muscle-camera frame rate. Equivalently, the nominal exposure is set to `muscle_interval − readout_time`, where `muscle_interval` is the desired muscle frame period. The blue LED is pulsed for `effective_exposure_time` at the onset of the common-time window.

The camera is operated in PCO's **auto-sequence** trigger mode (`TRIGGER_MODE_AUTOTRIGGER`), which is what makes it free-run continuously. It is *not* externally triggered: the trigger firmware reads the camera's common-time signal (it does not drive the camera). The nominal exposure is the only knob the recorder sets to control the muscle frame rate.

## Behavior-only acquisition

Muscle imaging is optional. When it is disabled (the `enableMuscle` flag of the trigger parameters is `false`; see the [communication protocol](serial_comm_protocol.md)), the triggering controller does not synchronize to the muscle camera at all: it ignores the muscle camera's common-time signal and triggers the behavior camera on its own clock at the requested `behFrameRate`. In this mode the blue excitation LED is never pulsed, and the muscle-related parameters (`muscEffExpTime`, `behMuscSyncRatio`, `pcoCamRollingTime`, `pcoCamReadoutTime`) are unused. The muscle camera still free-runs (it is never TTL-triggered in either mode), but the recorder does not save its frames: `muscle_images/` stays empty for a behavior-only recording.

This mode is also what the recorder uses whenever it needs the behavior camera running without excitation light (for example, during camera alignment and the calibration scans). It supersedes the earlier approach of setting the muscle effective exposure time to 0, which switched off the excitation LED but still slaved the behavior camera to the free-running muscle camera.

## Controller reset at startup

Every recorder program that talks to the trigger controller — `run-spotlight`, `align-cameras`, and `run-arena-registration-scan` — resets the controller once at startup by sending a `RESET` command (see the [communication protocol](serial_comm_protocol.md)). The firmware reboots the microcontroller with `esp_restart()`, equivalent to pressing its physical reset button.

This guarantees the controller always begins from a clean, known state — clearing any latched error or leftover configuration from a previous run — instead of inheriting whatever state it happened to be left in. Because the reboot drops the controller's USB CDC serial link, the recorder waits for the controller to reboot and re-enumerate before sending the first `STREAM`; during this brief window the controller streams with its built-in default parameters.