import tyro
from pathlib import Path
from spotlight_tools.calibration.aruco import ArUcoBoard


def generated_aruco_board(
    arena_size_x_mm: float = 48,
    arena_size_y_mm: float = 72,
    aruco_scale_mm: float = 0.3,
    aruco_spacing_unitblk: int = 2,
    output_path: str = "~/Spotlight/aruco_board/aruco_board.svg",
):
    """Generate an ArUco board with the specified parameters.

    Args:
        arena_size_x_mm (float):
            The width of the arena in mm.
        arena_size_y_mm (float):
            The height of the arena in mm.
        aruco_scale_mm (float):
            Side length of each square (i.e. "pixel") inside each aruco
            code (mm).
        aruco_spacing_unitblk (int):
            Space between each two aruco codes (in squres, i.e. "pixels").
        output_path (str):
            The path to save the generated SVG file.
    """
    aruco_board = ArUcoBoard(
        arena_dim_mm=(arena_size_x_mm, arena_size_y_mm),
        scale_mm=aruco_scale_mm,
        spacing_unitblk=aruco_spacing_unitblk,
    )
    output_path = Path(output_path).expanduser()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    aruco_board.draw_svg(output_path)


if __name__ == "__main__":
    tyro.cli(generated_aruco_board)