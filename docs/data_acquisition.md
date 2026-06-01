# Data acquisition workflow

The cameras acquire frames in the following manner:

- The behavior camera generally records at a higher frame rate than the muscle camera. The synchronization ratio is an integer. In cycles where both cameras expose, they expose at the same time.
- The IR LED is on only when the behavior camera is exposing; the blue LED is on only when the muscle camera is exposing.
- An important nuance: the behavior camera has a global shutter, but the muscle camera has a rolling shutter. The amount of time it takes for the muscle camera to "roll its shutter" is generally longer than the frame interval of the behavior camera (hence the >1 sync ratio). To achieve an _effective_ global shutter for the muscle synchronized to the behavior camera, the blue LED is turned on only during the common time of the rolling shutter (i.e., period of time when all lines of pixels are scanning). In other words, the blue LED is used as the "shutter" instead of the actual camera shutter. The exposure time of the muscle camera is therefore `rollingTime + effectiveExposureTime`, where `rollingTime = lineTime * numLines`.

There are two data acquisition modes, dictated by triggering modalities of the muscle camera:

## Frame-by-frame mode

**Frame-by-frame mode:** The recorder instructs the trigger microcontroller to send trigger signals to the behavior camera's frame grabber and the muscle camera at specified frame rates. The microcontroller does so accordingly.

The disadvantage of this mode is that the muscle camera ignores any trigger signal marking the start of a new frame cycle _unless_ the previous frame cycle is completed (i.e., the last line of pixels has completed exposure and subsequent data readout). By this time, the first line has already been idle for a period equal to `rollingTime`. This limits the frame rate to `1 / (2 * rollingTime + effectiveExposureTime + readoutTime)`.

## Continuous mode

This mode improves the frame rate limit by instructing the muscle camera to _continuously_ roll its shutter: as soon as the first line finishes exposure and data readout, its exposure starts for the next cycle. There is no idle time at all.

The disadvantage of this mode is that we cannot trigger the muscle camera externally via TTL; the camera runs its own thing in open loop. This poses two challenges:

1. The triggering controller has to calibrate to the muscle camera to avoid cumulative error.
    - To mitigate this, we configure the muscle camera to produce a status signal that is OFF during the common time of all frames, and ON otherwise. When this signal switches to OFF from ON, the microcontroller triggers a frame for the behavior camera, and switches the lights on for the desired exposure time. Then, the microcontroller runs `syncRatio - 1` frame cycles for the behavior camera using its own clock, and then waits for the next signal from the muscle camera.
2. We can't set the muscle camera's frame rate independently of its exposure time, since the frame rate is always `1 / (rollingTime + exposureTime + readoutTime)`.
    - To mitigate this, we set the muscle camera's exposure time to `effectiveExposureTime + bufferTime`, where `bufferTime` is determined so that `1 / (rollingTime + effectiveExposureTime + bufferTime + readoutTime)` is our desired frame rate for the muscle camera.