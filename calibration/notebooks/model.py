import numpy as np
import matplotlib.pyplot as plt
from sklearn.linear_model import LinearRegression, RANSACRegressor
from sklearn.metrics import mean_squared_error, r2_score


def fit_affine_mapping(coordinates_df):
    """
    Fit an affine transformation model to map from stage coordinates and
    pixel coordinates to physical coordinates on the ArUco board.

    Parameters:
    -----------
    coordinates_df : pd.DataFrame
        DataFrame with columns:
        - 'stage_x_mm': Stage x-coordinate in mm
        - 'stage_y_mm': Stage y-coordinate in mm
        - 'pixel_x_px': Pixel x-coordinate in pixels
        - 'pixel_y_px': Pixel y-coordinate in pixels
        - 'physical_x_mm': Physical x-coordinate in mm (on the ArUco board)
        - 'physical_y_mm': Physical y-coordinate in mm (on the ArUco board)

    Returns:
    --------
    dict
        Dictionary containing:
        - 'model_x': LinearRegression model for physical_x prediction
        - 'model_y': LinearRegression model for physical_y prediction
        - 'metrics': Dictionary with R² and RMSE metrics
    """
    # Create input features (X) and target variables (y)
    X = coordinates_df[["stage_x_mm", "stage_y_mm", "pixel_x_px", "pixel_y_px"]].values
    y_x = coordinates_df["physical_x_mm"].values
    y_y = coordinates_df["physical_y_mm"].values

    # Create and fit the models
    model_x = LinearRegression()
    model_y = LinearRegression()

    model_x.fit(X, y_x)
    model_y.fit(X, y_y)

    # Make predictions
    pred_x = model_x.predict(X)
    pred_y = model_y.predict(X)

    # Calculate metrics
    r2_x = r2_score(y_x, pred_x)
    r2_y = r2_score(y_y, pred_y)
    rmse_x = np.sqrt(mean_squared_error(y_x, pred_x))
    rmse_y = np.sqrt(mean_squared_error(y_y, pred_y))

    # Create a dictionary to store the models and metrics
    result = {
        "model_x": model_x,
        "model_y": model_y,
        "metrics": {"r2_x": r2_x, "r2_y": r2_y, "rmse_x": rmse_x, "rmse_y": rmse_y},
    }

    return result


def predict_physical_coords(model, stage_x_mm, stage_y_mm, pixel_x_px, pixel_y_px):
    """
    Predict physical coordinates using the calibration model.

    Parameters:
    -----------
    model : dict
        Dictionary containing 'model_x' and 'model_y' LinearRegression models
    stage_x_mm, stage_y_mm : float or array-like
        Stage coordinates in mm
    pixel_x_px, pixel_y_px : float or array-like
        Pixel coordinates in pixels

    Returns:
    --------
    tuple
        (physical_x_mm, physical_y_mm)
    """
    # Convert inputs to numpy arrays
    if np.isscalar(stage_x_mm):
        X = np.array([[stage_x_mm, stage_y_mm, pixel_x_px, pixel_y_px]])
    else:
        X = np.column_stack((stage_x_mm, stage_y_mm, pixel_x_px, pixel_y_px))

    # Predict
    physical_x_mm = model["model_x"].predict(X)
    physical_y_mm = model["model_y"].predict(X)

    # Convert back to scalar if input was scalar
    if len(physical_x_mm) == 1 and np.isscalar(stage_x_mm):
        return physical_x_mm[0], physical_y_mm[0]

    return physical_x_mm, physical_y_mm


def visualize_calibration_results(coordinates_df, calibration_model, vmax=None):
    """
    Visualize the results of the calibration.

    Parameters:
    -----------
    coordinates_df : pd.DataFrame
        DataFrame with the coordinates data
    calibration_model : dict
        Dictionary containing calibration models and metrics
    vmax : float, optional
        Maximum value for the error colorbar (default: None)

    Returns:
    --------
    None
    """
    # Make predictions on the training data
    X = coordinates_df[["stage_x_mm", "stage_y_mm", "pixel_x_px", "pixel_y_px"]].values
    pred_x = calibration_model["model_x"].predict(X)
    pred_y = calibration_model["model_y"].predict(X)

    # Create a figure with 2 subplots
    fig, axes = plt.subplots(1, 2, figsize=(15, 6))

    # Plot 1: Physical X - Actual vs Predicted
    axes[0].scatter(coordinates_df["physical_x_mm"], pred_x, alpha=0.5)
    axes[0].plot(
        [coordinates_df["physical_x_mm"].min(), coordinates_df["physical_x_mm"].max()],
        [coordinates_df["physical_x_mm"].min(), coordinates_df["physical_x_mm"].max()],
        "r--",
    )
    axes[0].set_xlabel("Actual Physical X (mm)")
    axes[0].set_ylabel("Predicted Physical X (mm)")
    axes[0].set_title(
        f'Physical X Calibration\nR² = {calibration_model["metrics"]["r2_x"]:.4f}, RMSE = {calibration_model["metrics"]["rmse_x"]:.4f} mm'
    )
    axes[0].grid(True)

    # Plot 2: Physical Y - Actual vs Predicted
    axes[1].scatter(coordinates_df["physical_y_mm"], pred_y, alpha=0.5)
    axes[1].plot(
        [coordinates_df["physical_y_mm"].min(), coordinates_df["physical_y_mm"].max()],
        [coordinates_df["physical_y_mm"].min(), coordinates_df["physical_y_mm"].max()],
        "r--",
    )
    axes[1].set_xlabel("Actual Physical Y (mm)")
    axes[1].set_ylabel("Predicted Physical Y (mm)")
    axes[1].set_title(
        f'Physical Y Calibration\nR² = {calibration_model["metrics"]["r2_y"]:.4f}, RMSE = {calibration_model["metrics"]["rmse_y"]:.4f} mm'
    )
    axes[1].grid(True)

    plt.tight_layout()
    plt.show()

    # Create a figure showing the residuals
    fig, axes = plt.subplots(1, 2, figsize=(15, 6))

    # Calculate residuals
    residuals_x = coordinates_df["physical_x_mm"] - pred_x
    residuals_y = coordinates_df["physical_y_mm"] - pred_y

    # Plot 1: Residuals for X
    axes[0].scatter(pred_x, residuals_x, alpha=0.5)
    axes[0].axhline(y=0, color="r", linestyle="--")
    axes[0].set_xlabel("Predicted Physical X (mm)")
    axes[0].set_ylabel("Residuals (mm)")
    axes[0].set_title("Residuals for Physical X")
    axes[0].grid(True)

    # Plot 2: Residuals for Y
    axes[1].scatter(pred_y, residuals_y, alpha=0.5)
    axes[1].axhline(y=0, color="r", linestyle="--")
    axes[1].set_xlabel("Predicted Physical Y (mm)")
    axes[1].set_ylabel("Residuals (mm)")
    axes[1].set_title("Residuals for Physical Y")
    axes[1].grid(True)

    plt.tight_layout()
    plt.show()

    # Spatial visualization of residuals
    plt.figure(figsize=(10, 8))
    plt.quiver(
        coordinates_df["physical_x_mm"],
        coordinates_df["physical_y_mm"],
        residuals_x,
        residuals_y,
        angles="xy",
        scale_units="xy",
        scale=0.1,
        width=0.002,
        color="blue",
        alpha=0.7,
    )
    plt.scatter(
        coordinates_df["physical_x_mm"],
        coordinates_df["physical_y_mm"],
        c=np.sqrt(residuals_x**2 + residuals_y**2),
        cmap="inferno",
        alpha=0.6,
        s=30,
        vmin=0,
        vmax=vmax,
    )
    plt.colorbar(label="Residual Magnitude (mm)")
    plt.title("Spatial Distribution of Calibration Residuals")
    plt.xlabel("Physical X (mm)")
    plt.ylabel("Physical Y (mm)")
    plt.grid(True)
    plt.axis("equal")
    plt.tight_layout()
    plt.show()


def ransac_filter_outliers(
    coordinates_df, max_trials=1000, residual_threshold=1.0, min_samples=None
):
    """
    Filter outliers from the coordinates data using RANSAC.

    Parameters:
    -----------
    coordinates_df : pd.DataFrame
        DataFrame with the coordinates data
    max_trials : int, optional
        Maximum number of iterations for RANSAC
    residual_threshold : float, optional
        Maximum residual for a data point to be considered an inlier (in mm)
    min_samples : int, optional
        Minimum number of samples for RANSAC to fit a model. If None, calculated automatically.

    Returns:
    --------
    dict
        Dictionary containing:
        - 'filtered_df': DataFrame with outliers removed
        - 'inlier_mask_x': Boolean mask for X inliers
        - 'inlier_mask_y': Boolean mask for Y inliers
        - 'ransac_model_x': RANSAC model for X
        - 'ransac_model_y': RANSAC model for Y
        - 'metrics': Dictionary with metrics before and after filtering
    """
    # Create input features (X) and target variables (y)
    X = coordinates_df[["stage_x_mm", "stage_y_mm", "pixel_x_px", "pixel_y_px"]].values
    y_x = coordinates_df["physical_x_mm"].values
    y_y = coordinates_df["physical_y_mm"].values

    # If min_samples is not specified, calculate it based on the number of features
    if min_samples is None:
        # For an affine model, we need at least n_features + 1 samples
        min_samples = X.shape[1] + 1

    # Apply RANSAC for X-coordinates
    ransac_x = RANSACRegressor(
        LinearRegression(),
        max_trials=max_trials,
        residual_threshold=residual_threshold,
        min_samples=min_samples,
        random_state=42,
    )
    ransac_x.fit(X, y_x)
    inlier_mask_x = ransac_x.inlier_mask_

    # Apply RANSAC for Y-coordinates
    ransac_y = RANSACRegressor(
        LinearRegression(),
        max_trials=max_trials,
        residual_threshold=residual_threshold,
        min_samples=min_samples,
        random_state=42,
    )
    ransac_y.fit(X, y_y)
    inlier_mask_y = ransac_y.inlier_mask_

    # Create combined inlier mask (points must be inliers for both X and Y)
    combined_inlier_mask = inlier_mask_x & inlier_mask_y

    # Filter the dataframe based on the combined mask
    filtered_df = (
        coordinates_df.iloc[combined_inlier_mask].copy().reset_index(drop=True)
    )

    # Calculate metrics before and after filtering
    # Before filtering
    lr_x = LinearRegression().fit(X, y_x)
    lr_y = LinearRegression().fit(X, y_y)
    pred_x_before = lr_x.predict(X)
    pred_y_before = lr_y.predict(X)
    r2_x_before = r2_score(y_x, pred_x_before)
    r2_y_before = r2_score(y_y, pred_y_before)
    rmse_x_before = np.sqrt(mean_squared_error(y_x, pred_x_before))
    rmse_y_before = np.sqrt(mean_squared_error(y_y, pred_y_before))

    # After filtering (only using inliers)
    X_filtered = X[combined_inlier_mask]
    y_x_filtered = y_x[combined_inlier_mask]
    y_y_filtered = y_y[combined_inlier_mask]
    lr_x_after = LinearRegression().fit(X_filtered, y_x_filtered)
    lr_y_after = LinearRegression().fit(X_filtered, y_y_filtered)
    pred_x_after = lr_x_after.predict(X_filtered)
    pred_y_after = lr_y_after.predict(X_filtered)
    r2_x_after = r2_score(y_x_filtered, pred_x_after)
    r2_y_after = r2_score(y_y_filtered, pred_y_after)
    rmse_x_after = np.sqrt(mean_squared_error(y_x_filtered, pred_x_after))
    rmse_y_after = np.sqrt(mean_squared_error(y_y_filtered, pred_y_after))

    # Calculate outlier percentages
    n_total = len(coordinates_df)
    n_inliers = combined_inlier_mask.sum()
    n_outliers = n_total - n_inliers
    outlier_percent = (n_outliers / n_total) * 100

    metrics = {
        "before": {
            "r2_x": r2_x_before,
            "r2_y": r2_y_before,
            "rmse_x": rmse_x_before,
            "rmse_y": rmse_y_before,
        },
        "after": {
            "r2_x": r2_x_after,
            "r2_y": r2_y_after,
            "rmse_x": rmse_x_after,
            "rmse_y": rmse_y_after,
        },
        "n_total": n_total,
        "n_inliers": n_inliers,
        "n_outliers": n_outliers,
        "outlier_percent": outlier_percent,
    }

    result = {
        "filtered_df": filtered_df,
        "inlier_mask_x": inlier_mask_x,
        "inlier_mask_y": inlier_mask_y,
        "combined_inlier_mask": combined_inlier_mask,
        "ransac_model_x": ransac_x,
        "ransac_model_y": ransac_y,
        "metrics": metrics,
        "models": {"x": lr_x_after, "y": lr_y_after},
    }

    return result


def visualize_ransac_results(coordinates_df, ransac_results):
    """
    Visualize the results of RANSAC filtering.

    Parameters:
    -----------
    coordinates_df : pd.DataFrame
        Original DataFrame with the coordinates data
    ransac_results : dict
        Dictionary containing RANSAC results from ransac_filter_outliers function

    Returns:
    --------
    None
    """
    # Extract data for visualization
    X = coordinates_df[["stage_x_mm", "stage_y_mm", "pixel_x_px", "pixel_y_px"]].values
    y_x = coordinates_df["physical_x_mm"].values
    y_y = coordinates_df["physical_y_mm"].values

    inlier_mask_x = ransac_results["inlier_mask_x"]
    inlier_mask_y = ransac_results["inlier_mask_y"]
    combined_inlier_mask = ransac_results["combined_inlier_mask"]

    ransac_model_x = ransac_results["ransac_model_x"]
    ransac_model_y = ransac_results["ransac_model_y"]

    metrics = ransac_results["metrics"]

    # ----------------------------------------------
    # Plot 1: Inliers and Outliers for Physical X
    # ----------------------------------------------
    plt.figure(figsize=(12, 10))

    # Predictions from RANSAC models
    X_line = np.array([min(y_x), max(y_x)]).reshape(-1, 1)
    y_pred_x = ransac_model_x.predict(X)

    plt.subplot(2, 2, 1)
    plt.scatter(
        y_x[inlier_mask_x],
        y_pred_x[inlier_mask_x],
        c="blue",
        marker=".",
        label="Inliers",
    )
    plt.scatter(
        y_x[~inlier_mask_x],
        y_pred_x[~inlier_mask_x],
        c="red",
        marker=".",
        label="Outliers",
    )
    plt.plot([min(y_x), max(y_x)], [min(y_x), max(y_x)], "k--")
    plt.xlabel("Actual Physical X (mm)")
    plt.ylabel("Predicted Physical X (mm)")
    plt.title("RANSAC Filtering: Physical X")
    plt.legend()
    plt.grid(True)

    # ----------------------------------------------
    # Plot 2: Inliers and Outliers for Physical Y
    # ----------------------------------------------
    y_pred_y = ransac_model_y.predict(X)

    plt.subplot(2, 2, 2)
    plt.scatter(
        y_y[inlier_mask_y],
        y_pred_y[inlier_mask_y],
        c="blue",
        marker=".",
        label="Inliers",
    )
    plt.scatter(
        y_y[~inlier_mask_y],
        y_pred_y[~inlier_mask_y],
        c="red",
        marker=".",
        label="Outliers",
    )
    plt.plot([min(y_y), max(y_y)], [min(y_y), max(y_y)], "k--")
    plt.xlabel("Actual Physical Y (mm)")
    plt.ylabel("Predicted Physical Y (mm)")
    plt.title("RANSAC Filtering: Physical Y")
    plt.legend()
    plt.grid(True)

    # ----------------------------------------------
    # Plot 3: Residuals before filtering for X and Y
    # ----------------------------------------------
    # Linear regression on all data
    lr = LinearRegression()
    lr.fit(X, y_x)
    y_pred_all_x = lr.predict(X)
    residuals_all_x = y_x - y_pred_all_x

    lr.fit(X, y_y)
    y_pred_all_y = lr.predict(X)
    residuals_all_y = y_y - y_pred_all_y

    plt.subplot(2, 2, 3)
    plt.hist(residuals_all_x, bins=50, alpha=0.5, label="X Residuals")
    plt.hist(residuals_all_y, bins=50, alpha=0.5, label="Y Residuals")
    plt.axvline(x=0, color="k", linestyle="--")
    plt.xlabel("Residual (mm)")
    plt.ylabel("Count")
    plt.title("Residuals Before Filtering")
    plt.legend()
    plt.grid(True)

    # ----------------------------------------------
    # Plot 4: Residuals after filtering for X and Y
    # ----------------------------------------------
    # Linear regression on inliers only
    X_inliers = X[combined_inlier_mask]
    y_x_inliers = y_x[combined_inlier_mask]
    y_y_inliers = y_y[combined_inlier_mask]

    lr.fit(X_inliers, y_x_inliers)
    y_pred_inliers_x = lr.predict(X_inliers)
    residuals_inliers_x = y_x_inliers - y_pred_inliers_x

    lr.fit(X_inliers, y_y_inliers)
    y_pred_inliers_y = lr.predict(X_inliers)
    residuals_inliers_y = y_y_inliers - y_pred_inliers_y

    plt.subplot(2, 2, 4)
    plt.hist(residuals_inliers_x, bins=50, alpha=0.5, label="X Residuals")
    plt.hist(residuals_inliers_y, bins=50, alpha=0.5, label="Y Residuals")
    plt.axvline(x=0, color="k", linestyle="--")
    plt.xlabel("Residual (mm)")
    plt.ylabel("Count")
    plt.title("Residuals After Filtering")
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.show()

    # ----------------------------------------------
    # Spatial Visualization of Inliers and Outliers
    # ----------------------------------------------
    plt.figure(figsize=(12, 10))

    plt.subplot(2, 1, 1)
    plt.scatter(
        coordinates_df["physical_x_mm"][combined_inlier_mask],
        coordinates_df["physical_y_mm"][combined_inlier_mask],
        c="blue",
        marker=".",
        alpha=0.5,
        label="Inliers",
    )
    plt.scatter(
        coordinates_df["physical_x_mm"][~combined_inlier_mask],
        coordinates_df["physical_y_mm"][~combined_inlier_mask],
        c="red",
        marker="x",
        alpha=0.7,
        label="Outliers",
    )
    plt.xlabel("Physical X (mm)")
    plt.ylabel("Physical Y (mm)")
    plt.title("Spatial Distribution of Inliers and Outliers")
    plt.legend()
    plt.grid(True)

    # ----------------------------------------------
    # Error magnitudes on physical space
    # ----------------------------------------------
    # Calculate error magnitudes
    error_magnitude_x = np.abs(y_x - y_pred_x)
    error_magnitude_y = np.abs(y_y - y_pred_y)
    error_magnitude = np.sqrt(error_magnitude_x**2 + error_magnitude_y**2)

    plt.subplot(2, 1, 2)
    scatter = plt.scatter(
        coordinates_df["physical_x_mm"],
        coordinates_df["physical_y_mm"],
        c=error_magnitude,
        cmap="inferno",
        alpha=0.7,
    )
    plt.colorbar(scatter, label="Error Magnitude (mm)")
    plt.xlabel("Physical X (mm)")
    plt.ylabel("Physical Y (mm)")
    plt.title("Spatial Distribution of Error Magnitudes")
    plt.grid(True)

    plt.tight_layout()
    plt.show()

    # ----------------------------------------------
    # Print RANSAC metrics
    # ----------------------------------------------
    print("\n--- RANSAC Filtering Results ---")
    print(f"Total points: {metrics['n_total']}")
    print(f"Inliers: {metrics['n_inliers']} ({100 - metrics['outlier_percent']:.1f}%)")
    print(f"Outliers: {metrics['n_outliers']} ({metrics['outlier_percent']:.1f}%)")
    print("\nMetrics Before Filtering:")
    print(f"  R² for X: {metrics['before']['r2_x']:.4f}")
    print(f"  R² for Y: {metrics['before']['r2_y']:.4f}")
    print(f"  RMSE for X: {metrics['before']['rmse_x']:.4f} mm")
    print(f"  RMSE for Y: {metrics['before']['rmse_y']:.4f} mm")
    print("\nMetrics After Filtering:")
    print(f"  R² for X: {metrics['after']['r2_x']:.4f}")
    print(f"  R² for Y: {metrics['after']['r2_y']:.4f}")
    print(f"  RMSE for X: {metrics['after']['rmse_x']:.4f} mm")
    print(f"  RMSE for Y: {metrics['after']['rmse_y']:.4f} mm")


# def fit_affine_mapping_ransac(
#     coordinates_df,
#     residual_threshold=1.0,
#     max_trials=1000,
#     min_samples=None,
#     verbose=True,
# ):
#     """
#     Fit a robust calibration model by first filtering outliers with RANSAC.

#     Parameters:
#     -----------
#     coordinates_df : pd.DataFrame
#         DataFrame with the coordinates data
#     residual_threshold : float, optional
#         Maximum residual for a data point to be considered an inlier (in mm)
#     max_trials : int, optional
#         Maximum number of iterations for RANSAC
#     min_samples : int, optional
#         Minimum number of samples for RANSAC to fit a model. If None, calculated automatically.
#     verbose : bool, optional
#         Whether to print detailed metrics

#     Returns:
#     --------
#     dict
#         Dictionary containing calibration models and metrics
#     """
#     # Filter outliers with RANSAC
#     ransac_results = ransac_filter_outliers(
#         coordinates_df,
#         max_trials=max_trials,
#         residual_threshold=residual_threshold,
#         min_samples=min_samples,
#     )

#     if verbose:
#         # Print basic metrics
#         metrics = ransac_results["metrics"]
#         print("\n--- RANSAC Filtering Results ---")
#         print(f"Total points: {metrics['n_total']}")
#         print(
#             f"Inliers: {metrics['n_inliers']} ({100 - metrics['outlier_percent']:.1f}%)"
#         )
#         print(f"Outliers: {metrics['n_outliers']} ({metrics['outlier_percent']:.1f}%)")

#     # Get the filtered dataframe
#     filtered_df = ransac_results["filtered_df"]

#     # Fit the final model on the filtered data
#     X = filtered_df[["stage_x_mm", "stage_y_mm", "pixel_x_px", "pixel_y_px"]].values
#     y_x = filtered_df["physical_x_mm"].values
#     y_y = filtered_df["physical_y_mm"].values

#     model_x = LinearRegression()
#     model_y = LinearRegression()

#     model_x.fit(X, y_x)
#     model_y.fit(X, y_y)

#     # Make predictions
#     pred_x = model_x.predict(X)
#     pred_y = model_y.predict(X)

#     # Calculate final metrics
#     r2_x = r2_score(y_x, pred_x)
#     r2_y = r2_score(y_y, pred_y)
#     rmse_x = np.sqrt(mean_squared_error(y_x, pred_x))
#     rmse_y = np.sqrt(mean_squared_error(y_y, pred_y))

#     if verbose:
#         print("\nFinal Calibration Metrics (after outlier removal):")
#         print(f"R² for X: {r2_x:.4f}")
#         print(f"R² for Y: {r2_y:.4f}")
#         print(f"RMSE for X: {rmse_x:.4f} mm")
#         print(f"RMSE for Y: {rmse_y:.4f} mm")

#     # Create a dictionary to store the models and metrics
#     result = {
#         "model_x": model_x,
#         "model_y": model_y,
#         "metrics": {
#             "r2_x": r2_x,
#             "r2_y": r2_y,
#             "rmse_x": rmse_x,
#             "rmse_y": rmse_y,
#             "n_points_after_filtering": len(filtered_df),
#             "n_points_before_filtering": len(coordinates_df),
#             "outlier_percent": ransac_results["metrics"]["outlier_percent"],
#         },
#         "ransac_results": ransac_results,
#     }

#     return result
