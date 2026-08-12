import tyro
import pandas as pd
from pathlib import Path
import matplotlib.pyplot as plt
from matplotlib import colormaps
from matplotlib.collections import LineCollection
from matplotlib.colors import Normalize
from matplotlib.figure import Figure
from matplotlib.axes import Axes
import numpy as np


def visualize_stage_trajectory(
    behavior_frame_metadata_path: Path, output_path: Path | None = None
) -> tuple[Figure, Axes]:
    """Create a matplotlib figure showing the XY trajectory of the stage.

    The trajectory is colored by time (seconds) starting at 0.

    Args:
        behavior_frame_metadata_path: Path to a CSV with at least the
            columns `x_pos_mm_interp`, `y_pos_mm_interp`, and
            `received_time_us`.
        output_path: Optional path to save the figure.

    Returns:
        `(fig, ax)`, so callers can further customize or save the figure.
    """
    behavior_frame_metadata_df = pd.read_csv(behavior_frame_metadata_path)

    fig, ax = plt.subplots(figsize=(7, 5))
    ax.set_title("Stage position trajectory")
    ax.set_xlabel("X Position (mm)")
    ax.set_ylabel("Y Position (mm)")
    ax.set_aspect("equal", adjustable="box")

    x_pos = behavior_frame_metadata_df["x_pos_mm_interp"].values
    y_pos = behavior_frame_metadata_df["y_pos_mm_interp"].values

    points = np.array([x_pos, y_pos]).T.reshape(-1, 1, 2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)

    cmap = colormaps["gnuplot"]

    times = behavior_frame_metadata_df["received_time_us"].values / 1e6  # us->s
    times = times - times[0]  # normalize to start at 0
    norm = Normalize(vmin=times.min(), vmax=times.max())
    lc = LineCollection(segments, cmap=cmap, norm=norm)

    lc.set_array(times[:-1])
    lc.set_linewidth(2)
    line = ax.add_collection(lc)

    cbar = fig.colorbar(line, ax=ax)
    cbar.set_label("Time (s)")
    ax.grid(True, linestyle="--", alpha=0.7)

    margin_x = 0.05 * (x_pos.max() - x_pos.min())
    margin_y = 0.05 * (y_pos.max() - y_pos.min())
    margin = max(margin_x, margin_y)
    ax.set_xlim(x_pos.min() - margin, x_pos.max() + margin)
    ax.set_ylim(y_pos.min() - margin, y_pos.max() + margin)

    if output_path is not None:
        output_path.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(output_path, dpi=300)

    return fig, ax


def generate_trajectory_plot(
    recording_dir: Path, output_dir: Path | None = None
) -> None:
    """Generate a plot of the motion stages' trajectory over the recording.

    Args:
        recording_dir (Path):
            Root directory of the recording. This is the path that you set
            in the Spotlight recording GUI.
        output_dir (Path | None):
            Path to save the generated plot. If None, the plot is saved to
            the "processed" directory under the recording directory.
    """
    recording_dir = Path(recording_dir)
    metadata_path = recording_dir / "processed/behavior_frames_metadata.csv"

    if output_dir is None:
        output_dir = recording_dir / "processed/stage_position_trajectory.png"
    visualize_stage_trajectory(metadata_path, output_dir)


def main():
    tyro.cli(generate_trajectory_plot)


if __name__ == "__main__":
    main()
