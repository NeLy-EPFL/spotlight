import numpy as np
import matplotlib.pyplot as plt
from sklearn.linear_model import LinearRegression, RANSACRegressor
from sklearn.metrics import mean_squared_error, r2_score


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
    print(f"  R^2 for X: {metrics['before']['r2_x']:.4f}")
    print(f"  R^2 for Y: {metrics['before']['r2_y']:.4f}")
    print(f"  RMSE for X: {metrics['before']['rmse_x']:.4f} mm")
    print(f"  RMSE for Y: {metrics['before']['rmse_y']:.4f} mm")
    print("\nMetrics After Filtering:")
    print(f"  R^2 for X: {metrics['after']['r2_x']:.4f}")
    print(f"  R^2 for Y: {metrics['after']['r2_y']:.4f}")
    print(f"  RMSE for X: {metrics['after']['rmse_x']:.4f} mm")
    print(f"  RMSE for Y: {metrics['after']['rmse_y']:.4f} mm")
    print()
