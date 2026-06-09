# `visualize-stage-trajectory`

Plots the motion stages' XY trajectory over a recording — useful for debugging
tracking. Lives in the [`spotlight-tools`](https://github.com/NeLy-EPFL/spotlight-tools)
repo (`src/spotlight_tools/cli/visualize_stage_trajectory.py`). Optional.

> [!NOTE]
> Run [`postprocess-recording`](postprocess_recording.md) first: this tool reads the
> consolidated `processed/behavior_frames_metadata.csv` produced by post-processing.

## Usage

```bash
visualize-stage-trajectory --recording-dir <save_dir>/
```

| Option | Default | Description |
|---|---|---|
| `--recording-dir PATH` | *(required)* | Recording root (the path set in the GUI). |
| `--output-dir PATH` | None | Where to save the plot. If unset, it is saved as `processed/stage_position_trajectory.png` under the recording. |

Run `visualize-stage-trajectory --help` for the authoritative argument list.
