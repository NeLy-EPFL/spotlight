# Recorder-microcontroller communication protocol

There are two types of commands: RUN and LOG. The formats of these are as follows. Commands are communicated in compact JSON form and parsed/formatted using ArduinoJson.

## RUN commands

```json
{
    "cmdTyp": "RUN",  // fixed literal string
    "params": {
        "pcoCamContinuous": ...,  // boolean
        "behExpTime": ...,  // non-negative integer
        "muscEffExpTime": ...,  // non-negative integer
        "behFrameRate": ...,  // positive integer
        "behMuscSyncRatio": ...,  // positive integer
        "pcoCamRollingTime": ...,  // non-negative integer
        "pcoCamReadoutTime": ...  // non-negative integer
    },
    "recording": {
        "isRecording": ...,  // boolean
        "opSequence": [  // list of variable length (empty list allowed)
            {
                "frameIdx": ...,  // non-negative integer
                "channel": ...,  // -1, 2, or 3
                "op": ...  //  "ON", "OFF", or "STOP"
            },
            ...
        ],
    } 
}
```

Upon a RUN command, the triggering controller changes its state based on information in `params`.

If `recording/isRecording` is false, then the cameras are simply streaming frames for a live preview in the GUI. There is no special action to be taken by the triggering controller other than sending trigger signals.

If `recording/isRecording` is true, the recorder is saving frames to disk. This entails:

- If `recording/opSequence` is empty, the recording is _open_: it keeps going on until the user stops it in the GUI, at which point the recorder sends the triggering controller a new RUN command with `recording/isRecording = false` and new parameters for streaming specified in `params`.
- If `recording/opSequence` is not empty, the recording is _scheduled_: it executes steps in the `opSequence`. Each step shall be executed after the `frameIdx`-th behavior camera frame based on the following rules:
    - If `op` is ON, the controller sets the state of the digital pin corresponding to the `channel` to HIGH, vice versa. In this case, the channel must be 2 or 3 (as 1 is occupied by the IR LED already).
    - If `op` is STOP, the controller switches back to streaming mode. This entails reverting the parameters to the values immediately prior to the RUN command initiating the recording. The channel is set to -1, as the operation is global. For technical reasons, `frameIdx` must be a multiple of 3 for STOP commands. Check this upon both serialization and parsing.

Upon RUN command, the microcontroller should echo the following via the serial port: `Triggering controller received RUN command`.

## LOG commands

```json
{
  "cmdTyp": "LOG",  // fixed literal string
  "msg": ...  // string
}
```

The microcontroller simply echoes the following via the serial port: `Triggering controller received log message: <msg>`.