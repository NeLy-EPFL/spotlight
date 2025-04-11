import tyro
import pandas as pd
from pathlib import Path

from spotlight_tools.postprocessing.visualization import visualize_stage_trajectory


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
    consolidated_metadata_df = pd.read_csv(
        recording_dir / "processed/behavior_frames_metadata.csv"
    )
    raw_stage_position_log_df = pd.read_csv(
        recording_dir / "stage_position/stage_position.csv"
    )

    fig, ax = visualize_stage_trajectory(
        consolidated_metadata_df=consolidated_metadata_df,
        raw_stage_position_log_df=raw_stage_position_log_df,
    )

    if output_dir is None:
        output_dir = recording_dir / "processed/stage_position_trajectory.png"
    fig.savefig(output_dir)


if __name__ == "__main__":
    tyro.cli(generate_trajectory_plot)
