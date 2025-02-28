import cv2
import numpy as np
import svgwrite
from pathlib import Path


def generate_aruco_grid_svg(
    num_rows,
    num_cols,
    code_size,
    spacing_vertical,
    spacing_horizontal,
    scale,
    output_file
):
    # Define the ArUco dictionary
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)

    # Calculate the total size of each marker including the border
    marker_size_with_border = code_size + 2  # 4x4 code + 1-bit border on each side

    # Create an SVG drawing
    dwg = svgwrite.Drawing(
        output_file,
        profile='tiny',
        size=(
            f"{scale * num_cols * (marker_size_with_border + spacing_horizontal)}mm",
            f"{scale * num_rows * (marker_size_with_border + spacing_vertical)}mm"
        )
    )


    # Iterate through the grid and generate ArUco codes
    for row in range(num_rows):
        for col in range(num_cols):
            # Generate a unique ArUco marker ID
            marker_id = row * num_cols + col
            if marker_id >= aruco_dict.bytesList.shape[0]:
                raise ValueError(
                    f"Not enough markers in the dictionary for ID {marker_id}"
                )

            # Create the marker image
            marker_img = np.zeros(
                (marker_size_with_border, marker_size_with_border), dtype=np.uint8
            )
            cv2.aruco.generateImageMarker(
                aruco_dict, marker_id, marker_size_with_border, marker_img, 1
            )

            # Calculate the position of the top-left corner of the marker
            x_offset = col * (marker_size_with_border + spacing_horizontal)
            y_offset = row * (marker_size_with_border + spacing_vertical)

            # Add the marker to the SVG as a group of rectangles
            for y in range(marker_size_with_border):
                for x in range(marker_size_with_border):
                    if marker_img[y, x] == 0:  # Draw black squares
                        dwg.add(dwg.rect(
                            insert=(
                                f"{scale * (x_offset + x)}mm",
                                f"{scale *(y_offset + y)}mm"
                            ),
                            size=(f"{scale}mm", f"{scale}mm"),
                            fill="black"
                        ))

    # Save the SVG file
    dwg.save()
    print(f"Saved ArUco grid to {output_file}")

def generate_aruco_board_given_size_and_scale(arena_width, arena_height, scale):
    """Generate an ArUco board with a grid of markers that fits within the
    given arena size (mm) and scale (mm). The scale is the length of the
    smallest feature on the code (ie. side length of each black/white
    square)."""
    code_size = 4  # 4x4 ArUco code
    spacing_vertical = 1
    spacing_horizontal = 1
    num_border_blocks = 2  # one file/row of black border on each side of the aruco code
    size_with_border = code_size + num_border_blocks
    num_rows = int((arena_width / scale) / (size_with_border + spacing_vertical))
    num_cols = int((arena_height / scale) / (size_with_border + spacing_horizontal))
    generate_aruco_grid_svg(
        num_rows=num_rows,
        num_cols=num_cols,
        code_size=code_size,  
        spacing_vertical=spacing_vertical,
        spacing_horizontal=spacing_horizontal,
        scale=scale,
        output_file=output_dir / f"aruco_grid_{num_rows}x{num_cols}.svg"
    )

if __name__ == "__main__":
    output_dir = Path("./output/generated_aruco_board/")
    output_dir.mkdir(parents=True, exist_ok=True)

    arena_width = 72  # mm
    arena_height = 48  # mm
    scales = [0.6, 0.5, 0.4, 0.3]

    for scale in scales:
        generate_aruco_board_given_size_and_scale(arena_width, arena_height, scale)
