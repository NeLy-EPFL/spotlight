import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from matplotlib.axes import Axes
from matplotlib.colors import Normalize
from matplotlib.collections import LineCollection
from matplotlib import cm


def visualize_stage_trajectory(
    consolidated_metadata_df: pd.DataFrame, raw_stage_position_log_df: pd.DataFrame
) -> tuple[Figure, Axes]:
    fig, ax = plt.subplots(figsize=(7, 5))
    ax.set_title("Stage position trajectory")
    ax.set_xlabel("X Position (mm)")
    ax.set_ylabel("Y Position (mm)")
    ax.set_aspect("equal", adjustable="box")

    x_pos = consolidated_metadata_df["x_pos_mm_interp"].values
    y_pos = consolidated_metadata_df["y_pos_mm_interp"].values

    points = np.array([x_pos, y_pos]).T.reshape(-1, 1, 2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)

    colormap_name = "gnuplot"
    cmap = cm.get_cmap(colormap_name)

    times = (
        consolidated_metadata_df["receivedTimeUs"].values / 1e6
    )  # Convert to seconds
    times = times - times[0]  # Normalize to start at 0
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

    return fig, ax
