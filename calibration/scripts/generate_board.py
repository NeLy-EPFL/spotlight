from calibration.aruco import ArUcoBoard

if __name__ == "__main__":
    arena_size_x_mm = 48
    arena_size_y_mm = 72
    aruco_scale_mm = 0.3  # side length of each square/pixel inside each aruco code (mm)
    aruco_spacing_unitblk = 2  # spacing between each two aruco codes (in squres/pixels)

    aruco_board = ArUcoBoard(
        arena_dim_mm=(arena_size_x_mm, arena_size_y_mm),
        scale_mm=aruco_scale_mm,
        spacing_unitblk=aruco_spacing_unitblk,
    )
    aruco_board.draw_svg("../data/generated_aruco_board/aruco_board.svg")
