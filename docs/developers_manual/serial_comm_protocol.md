# Recorder-microcontroller communication protocol

There are five types of commands: `STREAM`, `START_RECORDING`, `STOP_RECORDING`, `LOG`, and `RESET`. The formats of these are as follows. Commands are communicated in compact JSON form and parsed/formatted using ArduinoJson.

## `STREAM`

```json
{
    "cmdType": "STREAM",  // fixed literal string
    "params": {
        "enableMuscle": ..., // boolean
        "behExpTime": ...,  // non-negative integer
        "muscEffExpTime": ...,  // non-negative integer (ignored when enableMuscle is false)
        "behFrameRate": ...,  // positive integer
        "behMuscSyncRatio": ...,  // positive integer (ignored when enableMuscle is false)
        "pcoCamRollingTime": ...,  // non-negative integer (ignored when enableMuscle is false)
        "pcoCamReadoutTime": ...  // non-negative integer (ignored when enableMuscle is false)
    }
}
```

Upon a `STREAM` command, the triggering controller changes its state based on information in `params`. The images acquired by the cameras are streamed to the recorder GUI for a live preview in the GUI. There is no special action to be taken by the triggering controller other than sending trigger signals.

The `enableMuscle` flag selects the controller's acquisition mode (see [data acquisition](data_acquisition.md)):

- When **`true`** (muscle-synced mode), the controller synchronizes behavior acquisition to the free-running muscle (PCO) camera: it locks each group of `behMuscSyncRatio` behavior frames to the muscle camera's common-time signal and pulses the blue excitation LED for `muscEffExpTime`.
- When **`false`** (free-running mode), the muscle camera is ignored entirely. The controller triggers the behavior camera on its own clock at `behFrameRate`, never pulses the blue excitation LED, and ignores the muscle-only fields (`muscEffExpTime`, `behMuscSyncRatio`, `pcoCamRollingTime`, `pcoCamReadoutTime`). This is the mode for behavior-only acquisition. It replaces the former convention of disabling muscle imaging by setting `muscEffExpTime` to 0, which kept the behavior camera locked to the (still free-running) muscle camera.

The muscle-only fields must still be present and well-formed even when `enableMuscle` is `false`; they are simply unused.


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

Because `recParams` and `revertToParams` each have the same shape as a `STREAM` `params` object, each carries its own `enableMuscle` flag. The two may differ: for example, a recording can image muscle (`recParams.enableMuscle = true`) and then revert to a behavior-only live preview (`revertToParams.enableMuscle = false`).

If `opSequence` is empty, the recording is _open_: it keeps going on until the user stops it in the GUI, at which point the recorder sends a `STOP_RECORDING` command to the triggering controller. Aside from a different set of timing parameters, the behavior of the triggering controller is the same as `STREAM`.

If `opSequence` is not empty, the recording is _scheduled_: it executes steps in the `opSequence`. A non-empty `opSequence` must contain exactly one STOP step, and it must be the very last step. Each step shall be executed after the `frameIdx`-th behavior camera frame based on the following rules:

- If `op` is ON, the controller sets the state of the digital pin corresponding to the `channel` to HIGH, vice versa. In this case, the channel must be 2 or 3 (as 1 is occupied by the IR LED already).
- If `op` is STOP, the controller switches back to streaming mode automatically. The channel is set to -1 (the global `ALL` channel), as the operation is global. The triggering controller should then revert to streaming using `revertToParams`.


## `STOP_RECORDING`

```json
{
  "cmdType": "STOP_RECORDING"  // fixed literal string
}
```

Upon a `STOP_RECORDING` command, the triggering controller reverts to streaming using `revertToParams`. This command is only meaningful during an _open_ recording. If it is received during a _scheduled_ recording (which instead ends through its own `opSequence` STOP step), the controller enters an error state, drops all outputs, and logs the fault over the serial port until the next valid `STREAM` or `START_RECORDING`. If it is received when there is no recording to end at all (for example before any `START_RECORDING`), the controller simply logs a warning and ignores it.

## LOG commands

```json
{
  "cmdType": "LOG",  // fixed literal string
  "msg": ...  // string
}
```

Upon a `LOG` command, the controller simply echoes the following via the serial port: `Triggering controller received log message: <msg>`.

## `RESET`

```json
{
  "cmdType": "RESET"  // fixed literal string
}
```

Upon a `RESET` command, the triggering controller reboots itself by calling `esp_restart()` (from `esp_system.h`), which is equivalent to pressing the physical reset button on the board. Before rebooting, it shows `RESETTING` on the status display (with all other lines blanked) and the status LED. After the reboot the controller comes back up exactly as on a power-on: it re-initializes its peripherals and resumes streaming with the default parameters until the host sends its first `STREAM`.

Rebooting drops the controller's USB CDC serial connection, so it briefly disconnects and re-enumerates on the host. The recorder sends `RESET` once at the start of every program (see [data acquisition](data_acquisition.md)) and waits for the controller to come back before sending any further command.