import numpy as np
import yaml
import cv2
import tyro
from pathlib import Path
from tqdm import tqdm
import matplotlib.pyplot as plt
from joblib import Parallel, delayed

from spotlight_tools.calibration.charuco import get_gizem_board


def open_tif_image_and_normalize(path: str) -> np.ndarray:
    """Read 16-bit tif image and normalize to 0-255 range based on the 5th
    and 95th percentiles."""
    image = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
    q20 = np.percentile(image, 20)
    q80 = np.percentile(image, 80)
    image = (image - q20) / (q80 - q20)
    image = np.clip(image, 0, 1)
    image = (image * 255).astype(np.uint8)
    return image


def preprocess_image(image: np.ndarray) -> np.ndarray:
    """Apply bilateral filtering and CLAHE to enhance image quality."""
    img = cv2.bilateralFilter(image, d=5, sigmaColor=50, sigmaSpace=50)
    img = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(img)
    return img


def visualize_charuco_detection(
    image: np.ndarray,
    charuco_corners: np.ndarray,
    charuco_ids: np.ndarray,
    marker_corners: np.ndarray,
    marker_ids: np.ndarray,
    camera: str,
) -> plt.Figure:
    """Visualize ChArUco detection results."""
    fig, ax = plt.subplots(figsize=(10, 8))

    # Create a copy for drawing (cv2 functions modify in-place)
    display_image = image.copy()

    # Draw ArUco markers
    if marker_ids is not None and len(marker_ids) > 0:
        # Convert to 3-channel for colored drawings
        if len(display_image.shape) == 2:
            display_image = cv2.cvtColor(display_image, cv2.COLOR_GRAY2RGB)
        cv2.aruco.drawDetectedMarkers(display_image, marker_corners, marker_ids)

    # Display image
    if len(display_image.shape) == 2:
        ax.imshow(display_image, cmap="gray")
    else:
        ax.imshow(display_image)

    # Draw ChArUco corners
    if charuco_ids is not None and len(charuco_ids) > 0:
        for i, (corner, corner_id) in enumerate(zip(charuco_corners, charuco_ids)):
            x, y = corner.ravel()
            ax.plot(x, y, "ro", markersize=2)
            ax.text(x, y, str(corner_id[0]), color="yellow", fontsize=6)

    ax.set_title(
        f"{camera}: Detected {len(charuco_ids) if charuco_ids is not None else 0} ChArUco corners"
    )
    ax.axis("off")

    return fig


def visualize_exclusion_zone(
    matching_data: dict,
    output_path: Path,
    noise_threshold_px: float = 1.0,
) -> None:
    """
    Visualize corner detection offsets from median image detection in a grid layout.
    Each subplot shows one corner ID with detections plotted as offsets from median.

    Args:
        matching_data: Dictionary with matching corners data including raw detections
        output_path: Path to save the visualization
        noise_threshold_px: Exclusion radius in pixels
    """
    cameras = ["behavior", "muscle"]

    for camera in cameras:
        # Collect all corner IDs and their detections across all stage positions
        all_corners_data = {}  # corner_id -> list of (detection, median_corner_from_median_image, stage_key)

        # Get all stage positions and assign colors
        stage_positions = sorted(
            [k for k in matching_data.keys() if k != "_rejection_stats"]
        )
        n_stages = len(stage_positions)
        if n_stages < 20:
            stage_colors = plt.cm.tab20(np.linspace(0, 1, n_stages))
        else:
            # use jet but make sure uses the whole range of colors
            stage_colors = plt.cm.jet(np.linspace(0, 1, n_stages))

        stage_to_color = {
            stage: stage_colors[i] for i, stage in enumerate(stage_positions)
        }

        for stage_key, data in matching_data.items():
            if stage_key == "_rejection_stats":
                continue

            if camera == "behavior":
                cam_surname = "beh"
            elif camera == "muscle":
                cam_surname = "muscle"
            corners_by_id_key = f"all_{cam_surname}_corners_by_id"
            median_corners_key = f"{cam_surname}_median_corners"
            median_ids_key = f"{cam_surname}_median_ids"

            if corners_by_id_key not in data or median_corners_key not in data:
                continue

            corners_by_id = data[corners_by_id_key]
            median_corners = data[median_corners_key]
            median_ids = data[median_ids_key].flatten()

            # Process each corner ID
            for idx, corner_id in enumerate(median_ids):
                if corner_id not in corners_by_id:
                    continue

                detections = corners_by_id[corner_id]
                if len(detections) == 0:
                    continue

                # Get median corner position from median image
                median_corner = median_corners[idx].ravel()

                # Store detections with their median reference and stage position
                if corner_id not in all_corners_data:
                    all_corners_data[corner_id] = []

                for det in detections:
                    all_corners_data[corner_id].append((det, median_corner, stage_key))

        # Skip if no data
        if len(all_corners_data) == 0:
            print(
                f"No corner detections found for {camera} camera. Skipping exclusion zone visualization."
            )
            continue

        # Create grid layout
        unique_corner_ids = sorted(all_corners_data.keys())
        n_corners = len(unique_corner_ids)

        # Determine grid size
        n_cols = min(8, n_corners)
        n_rows = int(np.ceil(n_corners / n_cols))

        fig, axes = plt.subplots(n_rows, n_cols, figsize=(n_cols * 2.5, n_rows * 2.5))
        if n_rows == 1 and n_cols == 1:
            axes = np.array([[axes]])
        elif n_rows == 1:
            axes = axes.reshape(1, -1)
        elif n_cols == 1:
            axes = axes.reshape(-1, 1)

        # Plot each corner
        for plot_idx, corner_id in enumerate(unique_corner_ids):
            row = plot_idx // n_cols
            col = plot_idx % n_cols
            ax = axes[row, col]

            # Compute offsets from median image detection
            detections_and_medians = all_corners_data[corner_id]
            offsets_x = []
            offsets_y = []
            distances = []
            colors = []

            for det, median_corner, stage_key in detections_and_medians:
                offset = det - median_corner
                offsets_x.append(offset[0])
                offsets_y.append(offset[1])
                distances.append(np.linalg.norm(offset))
                colors.append(stage_to_color[stage_key])

            offsets_x = np.array(offsets_x)
            offsets_y = np.array(offsets_y)
            distances = np.array(distances)
            colors = np.array(colors)

            # Plot all detections with colors representing stage positions
            ax.scatter(
                offsets_x,
                offsets_y,
                c=colors,
                s=1,
                alpha=0.7,
                edgecolors="black",
                linewidths=0.5,
            )

            # Draw origin (median position)
            ax.scatter(0, 0, c="blue", s=25, marker="+", zorder=5, label="Median")

            # Draw exclusion circle
            circle = plt.Circle(
                (0, 0),
                noise_threshold_px,
                color="blue",
                fill=False,
                linestyle="--",
                alpha=0.5,
                linewidth=1.5,
            )
            ax.add_artist(circle)

            # Axis lines
            ax.axhline(0, color="gray", linestyle="-", alpha=0.3, linewidth=0.5)
            ax.axvline(0, color="gray", linestyle="-", alpha=0.3, linewidth=0.5)

            # Set limits
            max_offset = max(
                noise_threshold_px * 2,
                (
                    np.max(np.abs(offsets_x))
                    if len(offsets_x) > 0
                    else noise_threshold_px * 2
                ),
                (
                    np.max(np.abs(offsets_y))
                    if len(offsets_y) > 0
                    else noise_threshold_px * 2
                ),
            )
            ax.set_xlim(-max_offset, max_offset)
            ax.set_ylim(-max_offset, max_offset)

            ax.set_xlabel("X offset (px)", fontsize=8)
            ax.set_ylabel("Y offset (px)", fontsize=8)
            ax.set_title(
                f"ID {corner_id} (n={len(detections_and_medians)})", fontsize=9
            )
            ax.grid(True, alpha=0.2, linewidth=0.5)
            ax.set_aspect("equal")
            ax.tick_params(labelsize=7)

        # Hide unused subplots except the last two (reserved for legend)
        for plot_idx in range(n_corners, n_rows * n_cols - 2):
            row = plot_idx // n_cols
            col = plot_idx % n_cols
            axes[row, col].axis("off")

        # Create a legend for stage positions using the last two subplots
        # Use the last two unused subplots for legend
        if n_corners < n_rows * n_cols - 1:
            # Get the last two axes for legend
            legend_ax1 = axes.flatten()[-2]
            legend_ax2 = axes.flatten()[-1]
            legend_ax1.axis("off")
            legend_ax2.axis("off")

            # Create legend handles for stage positions
            from matplotlib.patches import Patch

            legend_elements = [
                Patch(
                    facecolor=stage_to_color[stage],
                    edgecolor="black",
                    label=f"Stage ({stage[0]:.0f}, {stage[1]:.0f})",
                )
                for stage in stage_positions
            ]

            # Split legend into two axes with 5 columns each
            mid_point = len(legend_elements) // 2
            legend_ax1.legend(
                handles=legend_elements[:mid_point],
                loc="center",
                fontsize=7,
                ncol=5,
                title="Stage Positions (x, y)",
                title_fontsize=8,
            )
            legend_ax2.legend(
                handles=legend_elements[mid_point:], loc="center", fontsize=7, ncol=5
            )

        # Add overall title
        fig.suptitle(
            f"{camera.capitalize()} Camera: Corner Offsets from Median Image Detection\n"
            f"({noise_threshold_px}px exclusion radius)",
            fontsize=14,
            fontweight="bold",
        )

        plt.tight_layout(rect=[0, 0, 1, 0.97])

        # Save with camera-specific filename
        output_path_camera = (
            output_path.parent / f"{output_path.stem}_{camera}{output_path.suffix}"
        )
        fig.savefig(output_path_camera, dpi=300, bbox_inches="tight")
        plt.close(fig)

        print(
            f"{camera.capitalize()} camera exclusion zone visualization saved to {output_path_camera}"
        )


def group_images_by_position(image_dir: Path, extension: str) -> dict:
    """
    Group image paths by stage position (x, y), collecting all frames at each position.

    Args:
        image_dir: Directory containing images
        extension: File extension (e.g., '.jpg', '.tif')

    Returns:
        Dictionary mapping (x, y) -> list of image paths
    """
    images_by_pos = {}
    image_paths = sorted(list(image_dir.glob(f"*{extension}")))

    if len(image_paths) == 0:
        raise ValueError(f"No images with extension {extension} found in {image_dir}")

    for img_path in image_paths:
        parts = img_path.stem.split("_")
        stage_x = float(parts[2].replace("x", ""))
        stage_y = float(parts[3].replace("y", ""))
        stage_key = (stage_x, stage_y)

        if stage_key not in images_by_pos:
            images_by_pos[stage_key] = []
        images_by_pos[stage_key].append(img_path)

    return images_by_pos


def process_stage_position(
    stage_key: tuple,
    behavior_paths: list,
    muscle_paths: list,
    visualization_dir: Path | None = None,
    noise_threshold_px: float = 1.0,
) -> tuple:
    """
    Process all frames at a single stage position, rejecting noisy detections.

    Args:
        stage_key: (x, y) stage position
        behavior_paths: List of behavior camera image paths at this position
        muscle_paths: List of muscle camera image paths at this position
        visualization_dir: Optional directory to save detection visualizations
        noise_threshold_px: Maximum allowed deviation from median detection (pixels)

    Returns:
        Tuple of (stage_key, matching_data_dict, rejection_stats) or (None, None, None) if processing failed
    """
    # Create CharucoBoard inside worker process (can't be pickled)
    charuco_board = get_gizem_board()

    # Load all frames at this position
    behavior_frames = [
        open_tif_image_and_normalize(str(path), [10, 98]) for path in behavior_paths
    ]
    muscle_frames = [
        open_tif_image_and_normalize(str(path), [10, 95]) for path in muscle_paths
    ]

    # Compute median frames
    behavior_median = np.median(behavior_frames, axis=0).astype(np.uint8)
    muscle_median = np.median(muscle_frames, axis=0).astype(np.uint8)

    # Preprocess median frames
    behavior_median = preprocess_image(behavior_median)
    muscle_median = preprocess_image(muscle_median)

    # Detect corners in median frames
    beh_median_corners, beh_median_ids, beh_median_markers, beh_median_marker_ids = (
        charuco_board.detect_corners(
            behavior_median, camera="behavior_camera", subpixel_refinement=True
        )
    )
    (
        muscle_median_corners,
        muscle_median_ids,
        muscle_median_markers,
        muscle_median_marker_ids,
    ) = charuco_board.detect_corners(
        muscle_median, camera="muscle_camera", subpixel_refinement=True
    )

    # Skip if median detection failed
    if beh_median_ids is None or muscle_median_ids is None:
        return None, None, None
    if len(beh_median_ids) == 0 or len(muscle_median_ids) == 0:
        return None, None, None

    # Process all individual frames and collect detections
    all_beh_corners_by_id = {id_[0]: [] for id_ in beh_median_ids}
    all_muscle_corners_by_id = {id_[0]: [] for id_ in muscle_median_ids}

    for beh_frame, muscle_frame in zip(behavior_frames, muscle_frames):
        # Preprocess frames
        beh_frame = preprocess_image(beh_frame)
        muscle_frame = preprocess_image(muscle_frame)

        # Detect corners
        beh_corners, beh_ids, _, _ = charuco_board.detect_corners(
            beh_frame, camera="behavior_camera", subpixel_refinement=True
        )
        muscle_corners, muscle_ids, _, _ = charuco_board.detect_corners(
            muscle_frame, camera="muscle_camera", subpixel_refinement=True
        )

        # Collect detections by corner ID
        if beh_ids is not None:
            for corner, corner_id in zip(beh_corners, beh_ids):
                corner_id_val = corner_id[0]
                if corner_id_val in all_beh_corners_by_id:
                    all_beh_corners_by_id[corner_id_val].append(corner.ravel())

        if muscle_ids is not None:
            for corner, corner_id in zip(muscle_corners, muscle_ids):
                corner_id_val = corner_id[0]
                if corner_id_val in all_muscle_corners_by_id:
                    all_muscle_corners_by_id[corner_id_val].append(corner.ravel())

    # Filter detections based on deviation from median and compute rejection statistics
    beh_median_ids_flat = beh_median_ids.flatten()
    muscle_median_ids_flat = muscle_median_ids.flatten()

    # Track rejection statistics
    beh_rejected_distances = []
    muscle_rejected_distances = []
    beh_total_detections = 0
    muscle_total_detections = 0

    # For valid corners, compute median position across all frames
    beh_median_positions = {}
    muscle_median_positions = {}

    for corner_id in beh_median_ids_flat:
        median_idx = np.where(beh_median_ids_flat == corner_id)[0][0]
        median_corner = beh_median_corners[median_idx].ravel()

        # Get all detections for this corner ID
        detections = all_beh_corners_by_id[corner_id]
        if len(detections) == 0:
            continue

        beh_total_detections += len(detections)

        # Filter based on distance from median
        valid_detections = []
        for det in detections:
            dist = np.linalg.norm(det - median_corner)
            if dist <= noise_threshold_px:
                valid_detections.append(det)
            else:
                beh_rejected_distances.append(dist)

        # Compute median position from valid detections
        if len(valid_detections) > 0:
            beh_median_positions[corner_id] = np.median(valid_detections, axis=0)

    for corner_id in muscle_median_ids_flat:
        median_idx = np.where(muscle_median_ids_flat == corner_id)[0][0]
        median_corner = muscle_median_corners[median_idx].ravel()

        # Get all detections for this corner ID
        detections = all_muscle_corners_by_id[corner_id]
        if len(detections) == 0:
            continue

        muscle_total_detections += len(detections)

        # Filter based on distance from median
        valid_detections = []
        for det in detections:
            dist = np.linalg.norm(det - median_corner)
            if dist <= noise_threshold_px:
                valid_detections.append(det)
            else:
                muscle_rejected_distances.append(dist)

        # Compute median position from valid detections
        if len(valid_detections) > 0:
            muscle_median_positions[corner_id] = np.median(valid_detections, axis=0)

    if len(beh_median_positions) == 0 or len(muscle_median_positions) == 0:
        return None, None, None

    # Find matching corner IDs
    common_ids = set(beh_median_positions.keys()) & set(muscle_median_positions.keys())

    if len(common_ids) < 4:
        print(
            f"Warning: Only {len(common_ids)} common corners found for stage position {stage_key}"
        )
        return None, None, None

    # Extract matching corners using median positions
    matching_corners = []
    for corner_id in common_ids:
        beh_corner = beh_median_positions[corner_id]
        muscle_corner = muscle_median_positions[corner_id]
        matching_corners.append((beh_corner, muscle_corner))

    # Compute rejection statistics
    rejection_stats = {
        "behavior": {
            "total_detections": beh_total_detections,
            "rejected_count": len(beh_rejected_distances),
            "rejected_distances": beh_rejected_distances,
        },
        "muscle": {
            "total_detections": muscle_total_detections,
            "rejected_count": len(muscle_rejected_distances),
            "rejected_distances": muscle_rejected_distances,
        },
    }

    # Visualize median detections if requested
    if visualization_dir is not None:
        beh_viz_dir = visualization_dir / "behavior_camera"
        muscle_viz_dir = visualization_dir / "muscle_camera"
        beh_viz_dir.mkdir(parents=True, exist_ok=True)
        muscle_viz_dir.mkdir(parents=True, exist_ok=True)

        # Use first frame's name for visualization filename
        viz_name = f"pos_x{stage_key[0]}_y{stage_key[1]}.png"

        fig_beh = visualize_charuco_detection(
            behavior_median,
            beh_median_corners,
            beh_median_ids,
            beh_median_markers,
            beh_median_marker_ids,
            "behavior",
        )
        fig_beh.savefig(beh_viz_dir / viz_name, dpi=300, bbox_inches="tight")
        plt.close(fig_beh)

        fig_muscle = visualize_charuco_detection(
            muscle_median,
            muscle_median_corners,
            muscle_median_ids,
            muscle_median_markers,
            muscle_median_marker_ids,
            "muscle",
        )
        fig_muscle.savefig(muscle_viz_dir / viz_name, dpi=300, bbox_inches="tight")
        plt.close(fig_muscle)

    matching_data_dict = {
        "behavior_median_positions": beh_median_positions,
        "muscle_median_positions": muscle_median_positions,
        "matching_corners": matching_corners,
        "num_matches": len(matching_corners),
        "num_frames": len(behavior_paths),
        "all_beh_corners_by_id": all_beh_corners_by_id,
        "all_muscle_corners_by_id": all_muscle_corners_by_id,
        "beh_median_corners": beh_median_corners,
        "beh_median_ids": beh_median_ids,
        "muscle_median_corners": muscle_median_corners,
        "muscle_median_ids": muscle_median_ids,
    }

    return stage_key, matching_data_dict, rejection_stats


def gather_matching_corners(
    calibration_image_dir: Path,
    visualization_dir: Path | None = None,
    num_workers: int = -1,
    noise_threshold_px: float = 1.0,
) -> dict:
    """
    Gather matching ChArUco corners from behavior and muscle camera images.
    Groups multiple frames at the same stage position and rejects noisy detections.

    Args:
        calibration_image_dir: Directory containing calibration images
        visualization_dir: Optional directory to save detection visualizations
        num_workers: Number of parallel workers (-1 for all CPUs)
        noise_threshold_px: Maximum allowed deviation from median detection (pixels)

    Returns:
        Dictionary with stage positions as keys and matching corners as values
    """
    behavior_dir = calibration_image_dir / "behavior_camera"
    muscle_dir = calibration_image_dir / "muscle_camera"

    if not behavior_dir.exists():
        raise FileNotFoundError(
            f"Directory {behavior_dir} does not exist. Run calibration scan first."
        )
    if not muscle_dir.exists():
        raise FileNotFoundError(
            f"Directory {muscle_dir} does not exist. "
            "Run calibration scan first for muscle camera."
        )

    # Clean visualization directories if they exist (to avoid confusion with old visualizations)
    if visualization_dir is not None:
        beh_viz_dir = visualization_dir / "behavior_camera"
        muscle_viz_dir = visualization_dir / "muscle_camera"
        for viz_dir in [beh_viz_dir, muscle_viz_dir]:
            if viz_dir.exists():
                for file in viz_dir.iterdir():
                    if file.is_file():
                        file.unlink()

    # Group images by stage position
    print("Grouping images by stage position...")
    behavior_by_pos = group_images_by_position(behavior_dir, ".jpg")
    muscle_by_pos = group_images_by_position(muscle_dir, ".tif")

    # Find common positions
    common_positions = set(behavior_by_pos.keys()) & set(muscle_by_pos.keys())

    if len(common_positions) == 0:
        raise ValueError(
            "No matching stage positions found between behavior and muscle cameras!"
        )

    total_frames = sum(len(behavior_by_pos[pos]) for pos in common_positions)
    print(
        f"Found {len(common_positions)} stage positions with {total_frames} total frames"
    )

    # Process all stage positions in parallel
    print(
        f"Processing {len(common_positions)} stage positions with {num_workers} workers..."
    )
    results = Parallel(n_jobs=num_workers, backend="loky")(
        delayed(process_stage_position)(
            stage_key,
            behavior_by_pos[stage_key],
            muscle_by_pos[stage_key],
            visualization_dir,
            noise_threshold_px,
        )
        for stage_key in tqdm(
            sorted(common_positions), desc="Processing stage positions"
        )
    )

    # Collect results and aggregate rejection statistics
    matching_data = {}
    all_rejection_stats = {
        "behavior": {
            "rejected_distances": [],
            "total_detections": 0,
            "rejected_count": 0,
        },
        "muscle": {
            "rejected_distances": [],
            "total_detections": 0,
            "rejected_count": 0,
        },
    }

    for stage_key, data, rejection_stats in results:
        if stage_key is not None and data is not None:
            matching_data[stage_key] = data

            # Aggregate rejection statistics
            for camera in ["behavior", "muscle"]:
                all_rejection_stats[camera]["rejected_distances"].extend(
                    rejection_stats[camera]["rejected_distances"]
                )
                all_rejection_stats[camera]["total_detections"] += rejection_stats[
                    camera
                ]["total_detections"]
                all_rejection_stats[camera]["rejected_count"] += rejection_stats[
                    camera
                ]["rejected_count"]

    # Print rejection statistics
    print("\n=== Noise Rejection Statistics ===")
    for camera in ["behavior", "muscle"]:
        stats = all_rejection_stats[camera]
        total = stats["total_detections"]
        rejected = stats["rejected_count"]
        distances = stats["rejected_distances"]

        if rejected > 0:
            avg_dist = np.mean(distances)
            max_dist = np.max(distances)
            print(f"{camera.capitalize()} camera:")
            print(f"  Total detections: {total}")
            print(f"  Rejected: {rejected} ({100 * rejected / total:.1f}%)")
            print(f"  Average rejection distance: {avg_dist:.3f} px")
            print(f"  Maximum rejection distance: {max_dist:.3f} px")
        else:
            print(f"{camera.capitalize()} camera: No detections rejected")
    print("=" * 35 + "\n")

    # Store rejection stats in matching_data for visualization
    matching_data["_rejection_stats"] = all_rejection_stats

    return matching_data


def compute_homography_from_matches(matching_data: dict) -> tuple:
    """
    Compute homography matrix from matching corners across multiple images.

    Args:
        matching_data: Dictionary with matching corners data

    Returns:
        Tuple of (homography_matrix, inlier_mask, all_behavior_pts, all_muscle_pts)
    """
    # Collect all matching points
    all_behavior_pts = []
    all_muscle_pts = []

    for stage_key, data in matching_data.items():
        # Skip metadata entries
        if stage_key == "_rejection_stats":
            continue
        for beh_corner, muscle_corner in data["matching_corners"]:
            all_behavior_pts.append(beh_corner)
            all_muscle_pts.append(muscle_corner)

    all_behavior_pts = np.array(all_behavior_pts)
    all_muscle_pts = np.array(all_muscle_pts)

    print(f"Total matching corners collected: {len(all_behavior_pts)}")

    # Compute homography using RANSAC
    H_beh2muscle, inliers = cv2.findHomography(
        all_behavior_pts,
        all_muscle_pts,
        method=cv2.RANSAC,
    )
    num_inliers = np.sum(inliers)
    print(
        f"Inliers: {num_inliers}/{len(all_behavior_pts)} ({100 * num_inliers / len(all_behavior_pts):.1f}%)"
    )

    return H_beh2muscle, inliers, all_behavior_pts, all_muscle_pts


def visualize_homography_quality(
    H: np.ndarray,
    inliers: np.ndarray,
    behavior_pts: np.ndarray,
    muscle_pts: np.ndarray,
    output_path: Path,
    n_points_to_show: int = 500,
) -> None:
    """Visualize the quality of the homography fit."""
    # Transform behavior points using homography
    behavior_pts_homogeneous = np.hstack(
        [behavior_pts, np.ones((len(behavior_pts), 1))]
    )
    transformed_pts = (H @ behavior_pts_homogeneous.T).T
    transformed_pts = transformed_pts[:, :2] / transformed_pts[:, 2:3]

    # Compute reprojection errors
    errors = np.linalg.norm(transformed_pts - muscle_pts, axis=1)

    # Create visualization with 2 rows
    fig, axes = plt.subplots(2, 3, figsize=(15, 10))

    # Plot 1: Scatter of errors
    ax = axes[0, 0]
    inlier_mask = inliers.flatten().astype(bool)
    ax.scatter(
        range(len(errors)),
        errors,
        c=["green" if inlier else "red" for inlier in inlier_mask],
        alpha=0.6,
        s=10,
    )
    ax.set_xlabel("Point index")
    ax.set_ylabel("Reprojection error (pixels)")
    ax.set_title("Reprojection Errors")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Plot 2: Histogram of errors
    ax = axes[0, 1]
    ax.hist(errors[inlier_mask], bins=30, alpha=0.7, color="green", label="Inliers")
    ax.hist(errors[~inlier_mask], bins=30, alpha=0.7, color="red", label="Outliers")
    ax.set_xlabel("Reprojection error (pixels)")
    ax.set_ylabel("Count")
    ax.set_title("Error Distribution")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Plot 3: Error statistics
    ax = axes[0, 2]
    stats_text = f"""
    Total points: {len(errors)}
    Inliers: {np.sum(inlier_mask)} ({100 * np.sum(inlier_mask) / len(errors):.1f}%)
    Outliers: {np.sum(~inlier_mask)} ({100 * np.sum(~inlier_mask) / len(errors):.1f}%)
    
    Inlier errors:
      Mean: {np.mean(errors[inlier_mask]):.3f} px
      Median: {np.median(errors[inlier_mask]):.3f} px
      Std: {np.std(errors[inlier_mask]):.3f} px
      Max: {np.max(errors[inlier_mask]):.3f} px
    """
    ax.text(
        0.1,
        0.5,
        stats_text,
        fontsize=11,
        family="monospace",
        verticalalignment="center",
    )
    ax.axis("off")
    ax.set_title("Statistics")

    show_indices = np.random.choice(
        len(errors), size=min(n_points_to_show, len(errors)), replace=False
    )
    selected_inlier_mask = inlier_mask[show_indices]

    # Plot 4: Spatial distribution in behavior camera
    ax = axes[1, 0]
    scatter = ax.scatter(
        behavior_pts[show_indices, 0],
        behavior_pts[show_indices, 1],
        c=errors[show_indices],
        cmap="RdYlGn_r",
        s=4,
        alpha=0.7,
        vmin=0,
        vmax=10,
    )
    # Mark outliers with red circles
    if np.any(~selected_inlier_mask):
        ax.scatter(
            behavior_pts[show_indices][~selected_inlier_mask, 0],
            behavior_pts[show_indices][~selected_inlier_mask, 1],
            facecolors="none",
            edgecolors="red",
            s=10,
            linewidths=2,
            label="Outliers",
        )
    ax.set_xlabel("X (pixels)")
    ax.set_ylabel("Y (pixels)")
    ax.set_title("Spatial Distribution (Behavior Camera)")
    ax.invert_yaxis()  # Image coordinates have y=0 at top
    ax.grid(True, alpha=0.3)
    cbar = plt.colorbar(scatter, ax=ax)
    cbar.set_label("Reprojection error (px)")
    if np.any(~inlier_mask):
        ax.legend()

    # Plot 5: Spatial distribution in muscle camera
    ax = axes[1, 1]
    scatter = ax.scatter(
        muscle_pts[show_indices, 0],
        muscle_pts[show_indices, 1],
        c=errors[show_indices],
        cmap="RdYlGn_r",
        s=4,
        alpha=0.7,
        vmin=0,
        vmax=10,
    )
    # Mark outliers with red circles
    if np.any(~selected_inlier_mask):
        ax.scatter(
            muscle_pts[show_indices][~selected_inlier_mask, 0],
            muscle_pts[show_indices][~selected_inlier_mask, 1],
            facecolors="none",
            edgecolors="red",
            s=10,
            linewidths=2,
            label="Outliers",
        )
    ax.set_xlabel("X (pixels)")
    ax.set_ylabel("Y (pixels)")
    ax.set_title("Spatial Distribution (Muscle Camera)")
    ax.invert_yaxis()  # Image coordinates have y=0 at top
    ax.grid(True, alpha=0.3)
    cbar = plt.colorbar(scatter, ax=ax)
    cbar.set_label("Reprojection error (px)")
    if np.any(~inlier_mask):
        ax.legend()

    # Plot 6: Error vectors (showing direction of errors)
    ax = axes[1, 2]

    # Plot transformed behavior points and actual muscle points
    ax.scatter(
        muscle_pts[show_indices, 0],
        muscle_pts[show_indices, 1],
        c="blue",
        s=10,
        alpha=0.5,
        label="Actual (muscle)",
    )
    ax.scatter(
        transformed_pts[show_indices, 0],
        transformed_pts[show_indices, 1],
        c="orange",
        s=10,
        alpha=0.5,
        label="Transformed (behavior→muscle)",
    )

    # Draw error vectors for subset of points
    for idx in show_indices:
        if inlier_mask[idx]:
            color = "green"
            alpha = 0.3
        else:
            color = "red"
            alpha = 0.8
        ax.arrow(
            transformed_pts[idx, 0],
            transformed_pts[idx, 1],
            muscle_pts[idx, 0] - transformed_pts[idx, 0],
            muscle_pts[idx, 1] - transformed_pts[idx, 1],
            head_width=5,
            head_length=5,
            fc=color,
            ec=color,
            alpha=alpha,
            length_includes_head=True,
        )

    ax.set_xlabel("X (pixels)")
    ax.set_ylabel("Y (pixels)")
    ax.set_title(f"Error Vectors (showing {len(show_indices)} points)")
    ax.invert_yaxis()
    ax.grid(True, alpha=0.3)
    ax.legend()
    ax.set_aspect("equal", adjustable="datalim")

    plt.tight_layout()
    fig.savefig(output_path, dpi=150, bbox_inches="tight")
    plt.close(fig)

    print(f"Homography quality visualization saved to {output_path}")


def fit_homography(
    profile_dir: str = "~/Spotlight/default/",
    visualize_detections: bool = False,
    num_workers: int = -1,
    noise_threshold_px: float = 1.0,
) -> None:
    """
    Fit homography transformation between behavior and muscle cameras using ChArUco board.
    Processes multiple frames at each stage position and rejects noisy detections.

    Args:
        profile_dir: Path to the profile directory
        visualize_detections: Whether to visualize ChArUco detections
        num_workers: Number of parallel workers (-1 for all CPUs, 1 for no parallelization)
        noise_threshold_px: Maximum allowed deviation from median detection (pixels)
    """
    print("Fitting homography between behavior and muscle cameras...")

    # Expand profile directory
    profile_dir = Path(profile_dir).expanduser()

    # Check if calibration images exist
    calibration_image_dir = profile_dir / "calibration/charuco_homography_scan"
    if not calibration_image_dir.exists():
        raise FileNotFoundError(
            f"Directory {calibration_image_dir} does not exist. "
            "Run calibration scan first."
        )

    # Create output directories
    output_dir = profile_dir / "calibration/model/homography_consensus"
    output_dir.mkdir(parents=True, exist_ok=True)

    visualization_dir = None
    if visualize_detections:
        visualization_dir = profile_dir / "calibration/charuco_detection"
        visualization_dir.mkdir(parents=True, exist_ok=True)
    # Gather matching corners
    print("Detecting ChArUco corners in image groups...")
    matching_data = gather_matching_corners(
        calibration_image_dir,
        visualization_dir=visualization_dir,
        num_workers=num_workers,
        noise_threshold_px=noise_threshold_px,
    )

    if len(matching_data) <= 1:  # Only metadata or empty
        raise ValueError("No matching corners found in any stage position!")

    # Count actual stage positions (excluding metadata)
    num_stage_positions = len(
        [k for k in matching_data.keys() if k != "_rejection_stats"]
    )
    total_matches = sum(
        data["num_matches"]
        for k, data in matching_data.items()
        if k != "_rejection_stats"
    )
    total_frames = sum(
        data["num_frames"]
        for k, data in matching_data.items()
        if k != "_rejection_stats"
    )
    print(
        f"Found {num_stage_positions} stage positions with {total_matches} total matching corners from {total_frames} frames"
    )

    # Compute homography
    print("Computing homography transformation...")
    H_beh2muscle, inliers, behavior_pts, muscle_pts = compute_homography_from_matches(
        matching_data
    )

    # Compute inverse homography
    H_muscle2beh = np.linalg.inv(H_beh2muscle)

    # Visualize quality
    quality_viz_path = output_dir / "homography_quality.png"
    visualize_homography_quality(
        H_beh2muscle, inliers, behavior_pts, muscle_pts, quality_viz_path
    )

    # Visualize exclusion zone
    exclusion_viz_path = output_dir / "exclusion_zone.png"
    visualize_exclusion_zone(matching_data, exclusion_viz_path, noise_threshold_px)

    # Save results to YAML
    homography_results = {
        "metadata": {
            "description": "Homography transformation between behavior and muscle cameras",
            "num_stage_positions": len(
                [k for k in matching_data.keys() if k != "_rejection_stats"]
            ),
            "num_total_frames": sum(
                data["num_frames"]
                for k, data in matching_data.items()
                if k != "_rejection_stats"
            ),
            "num_matching_points": len(behavior_pts),
            "num_inliers": int(np.sum(inliers)),
            "inlier_percentage": float(100 * np.sum(inliers) / len(behavior_pts)),
            "noise_threshold_px": noise_threshold_px,
        },
        "behavior_to_muscle": {
            "matrix": [
                [
                    float(H_beh2muscle[0, 0]),
                    float(H_beh2muscle[0, 1]),
                    float(H_beh2muscle[0, 2]),
                ],
                [
                    float(H_beh2muscle[1, 0]),
                    float(H_beh2muscle[1, 1]),
                    float(H_beh2muscle[1, 2]),
                ],
                [
                    float(H_beh2muscle[2, 0]),
                    float(H_beh2muscle[2, 1]),
                    float(H_beh2muscle[2, 2]),
                ],
            ],
            "description": "Homography matrix to transform behavior camera coordinates to muscle camera coordinates",
        },
        "muscle_to_behavior": {
            "matrix": [
                [
                    float(H_muscle2beh[0, 0]),
                    float(H_muscle2beh[0, 1]),
                    float(H_muscle2beh[0, 2]),
                ],
                [
                    float(H_muscle2beh[1, 0]),
                    float(H_muscle2beh[1, 1]),
                    float(H_muscle2beh[1, 2]),
                ],
                [
                    float(H_muscle2beh[2, 0]),
                    float(H_muscle2beh[2, 1]),
                    float(H_muscle2beh[2, 2]),
                ],
            ],
            "description": "Homography matrix to transform muscle camera coordinates to behavior camera coordinates",
        },
    }

    results_path = output_dir / "homography_result.yaml"
    with open(results_path, "w") as f:
        yaml.dump(homography_results, f, default_flow_style=False, sort_keys=False)

    print(f"\nHomography results saved to {results_path}")
    print(f"Inlier ratio: {homography_results['metadata']['inlier_percentage']:.1f}%")
    print("\nHomography matrix (behavior -> muscle):")
    print(H_beh2muscle)


def main():
    tyro.cli(fit_homography)


if __name__ == "__main__":
    main()
