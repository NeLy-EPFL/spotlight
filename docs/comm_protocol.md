# Recorder-microcontroller communication protocol

There are four types of commands: `STREAM`, `START_RECORDING`, `STOP_RECORDING`, and `LOG`. The formats of these are as follows. Commands are communicated in compact JSON form and parsed/formatted using ArduinoJson.

## `STREAM`

```json
{
    "cmdType": "STREAM",  // fixed literal string
    "params": {
        "behExpTime": ...,  // non-negative integer
        "muscEffExpTime": ...,  // non-negative integer
        "behFrameRate": ...,  // positive integer
        "behMuscSyncRatio": ...,  // positive integer
        "pcoCamRollingTime": ...,  // non-negative integer
        "pcoCamReadoutTime": ...  // non-negative integer
    }
}
```

Upon a `STREAM` command, the triggering controller changes its state based on information in `params`. The images acquired by the cameras are streamed to the recorder GUI for a live preview in the GUI. There is no special action to be taken by the triggering controller other than sending trigger signals.


## `START_RECORDING`

```json
{
    "cmdType": "START_RECORDING",
    "recParams": {...},  // same as params in STREAM
    "revertToParams": {...},  // same as params in STREAM
    "opSequence": [  // list of variable length (empty list allowed)
        {
            "frameIdx": ...,  // non-negative integer
            "channel": ...,  // -1, 2, or 3
            "op": ...  //  "ON", "OFF", or "STOP"
        },
        ...
    ]
}
```

Upon a `START_RECORDING` command, the triggering controller changes its settings based on information in `recParams` and starts recording. When recording finishes, the triggering controller changes its settings to `revertToParams`.

If `opSequence` is empty, the recording is _open_: it keeps going on until the user stops it in the GUI, at which point the recorder sends a `STOP_RECORDING` command to the triggering controller. Aside from a different set of timing parameters, the behavior of the triggering controller is the same as `STREAM`.

If `opSequence` is not empty, the recording is _scheduled_: it executes steps in the `opSequence`. Each step shall be executed after the `frameIdx`-th behavior camera frame based on the following rules:

- If `op` is ON, the controller sets the state of the digital pin corresponding to the `channel` to HIGH, vice versa. In this case, the channel must be 2 or 3 (as 1 is occupied by the IR LED already).
- If `op` is STOP, the controller switches back to streaming mode automatically. The channel is set to -1 (the global `ALL` channel), as the operation is global. The triggering controller should then revert to streaming using `revertToParams`.


## `STOP_RECORDING`

```json
{
  "cmdType": "STOP_RECORDING"  // fixed literal string
}
```

Upon a `STOP_RECORDING` command, the triggering controller reverts to streaming using `revertToParams`. This command is only valid during an _open_ recording; if it is received at any other time (for example before any `START_RECORDING`, or during a scheduled recording), the controller enters an error state, drops all outputs, and logs the fault over the serial port until the next valid `STREAM` or `START_RECORDING`.

## LOG commands

```json
{
  "cmdType": "LOG",  // fixed literal string
  "msg": ...  // string
}
```

Upon a `LOG` command, the controller simply echoes the following via the serial port: `Triggering controller received log message: <msg>`.