"""
Arena registration: fit the linear model mapping (stage_pos, pixel_pos) <->
physical (arena) pos from a registration scan captured by
`run-arena-registration-scan`.

The scan produces, under `<arena_dir>/mapping_scan/`:
  - `apriltag<id>_img<i>.jpg` (10 images per apriltag, processed with
    reorientBehaviorImage: rotated 90 deg CCW then horizontally flipped,
    matching the live-preview orientation)
  - `apriltag_stage_positions.csv` with stage (x, y) per image

Combined with `<arena_dir>/metadata.yaml` (which holds the arena-space
positions of every AprilTag corner) we get, per detected corner, a tuple
of (stage_pos, pixel_pos, physical_pos). A linear model of the form
    physical_pos = A * [stage_pos; pixel_pos; 1]
is fitted via RANSAC, after per-apriltag outlier rejection using the
fact that we have 10 repeated measurements for the same stage pose.

The fit is saved to `<arena_dir>/model/calibration_result.yaml` in the
same schema as the legacy `calibration/model/behavior_camera/...` output.
Per-corner inputs are saved alongside as `calibration_points.csv`.
"""

from __future__ import annotations

from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import yaml
from pupil_apriltags import Detector
from sklearn.linear_model import LinearRegression, RANSACRegressor
from sklearn.metrics import mean_squared_error, r2_score
from tqdm import tqdm

# AprilTag family used when generating the arena registration board.
APRILTAG_FAMILY = "tag16h5"

# Corner index returned by the AprilTag detector -> key in metadata.yaml
# apriltag_positions[<id>]. pupil_apriltags reports corners
# counter-clockwise starting from the marker's design bottom-left when the
# marker is viewed in its design orientation. Detection runs on the
# horizontally un-flipped (rotate-only) image so the ordering is the same
# as it was before the horizontal-flip convention was adopted.
_CORNER_KEYS = ["bottomleft", "bottomright", "topright", "topleft"]


# -----------------------------------------------------------------------------
# Detection
# -----------------------------------------------------------------------------


# pupil_apriltags has had reports of double-free / malloc errors when many
# Detector instances are created in the same process. Cache one per family.
_DETECTOR_CACHE: dict[tuple[str, float], Detector] = {}


def _get_detector(family: str, quad_decimate: float = 4.0) -> Detector:
    key = (family, quad_decimate)
    if key not in _DETECTOR_CACHE:
        _DETECTOR_CACHE[key] = Detector(families=family, quad_decimate=quad_decimate)
    return _DETECTOR_CACHE[key]


def detect_apriltags(
    image: np.ndarray,
    family: str = APRILTAG_FAMILY,
    min_decision_margin: float = 20.0,
    quad_decimate: float = 4.0,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Detect AprilTags in a grayscale image.

    Uses the pupil_apriltags detector, which handles the tag families
    actually used on the arena registration board. cv2.aruco's
    DICT_APRILTAG_* dictionaries do not detect these reliably.

    tag16h5 is small (30 codes, hamming distance 5) and has a high
    false-positive rate, so detections with decision margin below
    `min_decision_margin` are dropped.

    Args:
        image: Grayscale 8-bit image.
        family: AprilTag family name (e.g. "tag16h5").
        min_decision_margin: pupil_apriltags decision_margin floor; the
            valid detections in a registration scan have margins of
            tens to hundreds, while noise-driven false positives are
            typically below ~5.

    Returns:
        ids: shape (n,) int array of detected tag IDs.
        corners: shape (n, 4, 2) float array of corner pixel positions
            (x, y), in the input image's coordinate frame. Order matches
            pupil_apriltags' native order (counter-clockwise from the
            marker's bottom-left in design orientation).
        margins: shape (n,) float array of decision margins (useful for
            debugging / further filtering).
    """
    detector = _get_detector(family, quad_decimate)
    detections = detector.detect(image)
    kept = [d for d in detections if d.decision_margin >= min_decision_margin]

    if not kept:
        return (
            np.array([], dtype=int),
            np.zeros((0, 4, 2), dtype=float),
            np.array([], dtype=float),
        )
    ids = np.array([d.tag_id for d in kept], dtype=int)
    corners = np.array([np.asarray(d.corners, dtype=float) for d in kept])
    margins = np.array([d.decision_margin for d in kept], dtype=float)
    return ids, corners, margins


def _draw_detection(
    image: np.ndarray,
    ids: np.ndarray,
    corners: np.ndarray,
) -> np.ndarray:
    """Render an annotated BGR copy of the image for diagnostics."""
    annotated = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    _CORNER_LABELS = ["BL", "BR", "TR", "TL"]
    for tag_id, c in zip(ids, corners):
        poly = c.astype(int).reshape(-1, 1, 2)
        cv2.polylines(
            annotated, [poly], isClosed=True, color=(0, 255, 255), thickness=2
        )
        for j, (x, y) in enumerate(c):
            color = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0)][j]
            cv2.circle(annotated, (int(x), int(y)), 6, color, -1)
            cv2.putText(
                img=annotated,
                text=_CORNER_LABELS[j],
                org=(int(x) + 8, int(y) + 5),
                fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                fontScale=1,
                color=color,
                thickness=2,
                lineType=cv2.LINE_AA,
            )
        cx, cy = c.mean(axis=0).astype(int)
        cv2.putText(
            annotated,
            str(int(tag_id)),
            (cx - 10, cy + 6),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
    return annotated


# -----------------------------------------------------------------------------
# Point gathering
# -----------------------------------------------------------------------------


def gather_apriltag_points(
    arena_dir: Path,
    family: str = APRILTAG_FAMILY,
    min_decision_margin: float = 20.0,
    quad_decimate: float = 4.0,
    viz_dir: Path | None = None,
) -> pd.DataFrame:
    """
    Detect AprilTags across all registration-scan images and return a
    DataFrame, one row per detected corner, with columns:
        apriltag_id, image_id, corner_id,
        stage_x_mm, stage_y_mm,
        pixel_x_px, pixel_y_px,
        physical_x_mm, physical_y_mm,
        image_path

    For each (apriltag_id, image_id) pair only the detection whose tag
    id matches the expected one is kept; spurious detections of other
    IDs are discarded. The high false-positive rate of `tag16h5` is
    further suppressed by `min_decision_margin`.
    """
    arena_dir = Path(arena_dir)
    scan_dir = arena_dir / "mapping_scan"
    metadata = yaml.safe_load((arena_dir / "metadata.yaml").read_text())
    apriltag_positions = metadata["apriltag_positions"]
    stage_csv = pd.read_csv(scan_dir / "apriltag_stage_positions.csv")

    rows: list[dict] = []
    missing_detections: list[tuple[int, int]] = []
    n_spurious_other_ids = 0

    for _, r in tqdm(
        stage_csv.iterrows(),
        total=len(stage_csv),
        desc="Detecting AprilTags",
    ):
        tag_id = int(r["apriltag_id"])
        img_id = int(r["image_id"])
        img_path = scan_dir / f"apriltag{tag_id}_img{img_id}.jpg"
        if not img_path.exists():
            missing_detections.append((tag_id, img_id))
            continue
        image = cv2.imread(str(img_path), cv2.IMREAD_GRAYSCALE)
        if image is None:
            missing_detections.append((tag_id, img_id))
            continue

        # Images are saved after reorientBehaviorImage (rotate 90deg CCW +
        # horizontal flip). Un-flip for detection so tags are not mirrored
        # (pupil_apriltags does not reliably detect mirrored tags).
        image_for_detection = cv2.flip(image, 1)
        ids, corners_all, _margins = detect_apriltags(
            image_for_detection,
            family=family,
            min_decision_margin=min_decision_margin,
            quad_decimate=quad_decimate,
        )
        if viz_dir is not None:
            annotated = _draw_detection(image_for_detection, ids, corners_all)
            cv2.imwrite(
                str(viz_dir / f"apriltag{tag_id}_img{img_id}.jpg"),
                cv2.flip(annotated, 1),
            )

        # Discard every detection whose id does not match the one
        # implied by the filename (`apriltag<tag_id>_img<img_id>.jpg`).
        # The stage was parked over the expected AprilTag and the CSV
        # row carries that same id; any other id in the image is a
        # spurious false positive (tag16h5 is prone to these).
        match = np.where(ids == tag_id)[0]
        n_spurious_other_ids += int((ids != tag_id).sum())
        if len(match) == 0:
            missing_detections.append((tag_id, img_id))
            continue
        corners = corners_all[match[0]]

        if tag_id not in apriltag_positions:
            raise KeyError(
                f"tag id {tag_id} not present in metadata.yaml apriltag_positions"
            )
        meta = apriltag_positions[tag_id]

        image_width = image.shape[1]
        for j, key in enumerate(_CORNER_KEYS):
            phys_x, phys_y = meta[key]
            # corners[j, 0] is in the un-flipped (detect) coordinate frame;
            # flip x back to the display coordinate frame (rotate+flip image).
            pixel_x = image_width - 1 - corners[j, 0]
            rows.append(
                {
                    "apriltag_id": tag_id,
                    "image_id": img_id,
                    "corner_id": j,
                    "stage_x_mm": float(r["stage_x_mm"]),
                    "stage_y_mm": float(r["stage_y_mm"]),
                    "pixel_x_px": float(pixel_x),
                    "pixel_y_px": float(corners[j, 1]),
                    "physical_x_mm": float(phys_x),
                    "physical_y_mm": float(phys_y),
                    "image_path": str(img_path),
                }
            )

    if missing_detections:
        print(
            f"WARNING: AprilTag not detected (or expected tag id missing) in "
            f"{len(missing_detections)} of {len(stage_csv)} scan images."
        )
    if n_spurious_other_ids:
        print(
            f"Discarded {n_spurious_other_ids} detection(s) whose id did not "
            f"match the expected tag id from the filename."
        )

    return pd.DataFrame(rows)


# -----------------------------------------------------------------------------
# Outlier rejection within the 10-frame burst per AprilTag
# -----------------------------------------------------------------------------


def filter_per_apriltag_outliers(
    df: pd.DataFrame,
    mad_threshold: float = 3.0,
    min_pixel_floor: float = 1.0,
) -> pd.DataFrame:
    """
    Reject within-burst outliers using MAD on each
    (apriltag_id, corner_id) group of 10 measurements.

    The stage is stationary during the burst, so the per-corner pixel
    scatter is just sub-pixel jitter. A detection that strays well
    beyond MAD * scale from the median is a spurious match (wrong
    corner ordering, motion blur, etc.) and is dropped.

    Args:
        df: output of `gather_apriltag_points`.
        mad_threshold: cutoff in units of (1.4826 * MAD) above the
            median euclidean offset, where 1.4826 is the gaussian-
            consistent MAD scaling.
        min_pixel_floor: lower bound on the cutoff in pixels, to keep
            very tight bursts from rejecting harmless jitter.
    """
    keep = np.zeros(len(df), dtype=bool)
    for _, group in df.groupby(["apriltag_id", "corner_id"], sort=False):
        med_x = group["pixel_x_px"].median()
        med_y = group["pixel_y_px"].median()
        d = np.sqrt(
            (group["pixel_x_px"] - med_x) ** 2 + (group["pixel_y_px"] - med_y) ** 2
        ).to_numpy()
        mad = np.median(np.abs(d - np.median(d)))
        cutoff = max(mad_threshold * 1.4826 * mad, min_pixel_floor)
        idx = group.index.to_numpy()
        keep[idx[d <= cutoff]] = True
    return df.loc[keep].reset_index(drop=True)


# -----------------------------------------------------------------------------
# RANSAC fitting
# -----------------------------------------------------------------------------


def _ransac_fit(
    X: np.ndarray,
    y: np.ndarray,
    residual_threshold: float,
    max_trials: int,
    random_state: int,
) -> tuple[RANSACRegressor, np.ndarray]:
    ransac = RANSACRegressor(
        LinearRegression(),
        max_trials=max_trials,
        residual_threshold=residual_threshold,
        min_samples=X.shape[1] + 1,
        random_state=random_state,
    )
    ransac.fit(X, y)
    return ransac, ransac.inlier_mask_


def fit_ransac_model(
    df: pd.DataFrame,
    residual_threshold: float = 0.5,
    max_trials: int = 1000,
    random_state: int = 42,
) -> dict:
    """
    Fit a linear model `physical = A * [stage; pixel; 1]` with RANSAC.

    Two separate RANSAC regressors are fit (one per physical axis); a
    sample is treated as an overall inlier only if both regressors keep
    it. The final coefficients are then re-estimated on the combined
    inliers with plain least-squares for stability.
    """
    X = df[["stage_x_mm", "stage_y_mm", "pixel_x_px", "pixel_y_px"]].to_numpy()
    y_x = df["physical_x_mm"].to_numpy()
    y_y = df["physical_y_mm"].to_numpy()

    _, inlier_x = _ransac_fit(X, y_x, residual_threshold, max_trials, random_state)
    _, inlier_y = _ransac_fit(X, y_y, residual_threshold, max_trials, random_state)
    inlier_mask = inlier_x & inlier_y

    X_in = X[inlier_mask]
    lr_x = LinearRegression().fit(X_in, y_x[inlier_mask])
    lr_y = LinearRegression().fit(X_in, y_y[inlier_mask])

    pred_x = lr_x.predict(X_in)
    pred_y = lr_y.predict(X_in)
    res_x = y_x[inlier_mask] - pred_x
    res_y = y_y[inlier_mask] - pred_y
    res_euclidean = np.sqrt(res_x**2 + res_y**2)
    metrics = {
        "n_total": int(len(df)),
        "n_inliers": int(inlier_mask.sum()),
        "n_outliers": int((~inlier_mask).sum()),
        "rmse_x_mm": float(np.sqrt(mean_squared_error(y_x[inlier_mask], pred_x))),
        "rmse_y_mm": float(np.sqrt(mean_squared_error(y_y[inlier_mask], pred_y))),
        "rmse_euclidean_mm": float(np.sqrt(np.mean(res_euclidean**2))),
        "max_residual_mm": float(res_euclidean.max()),
        "r2_x": float(r2_score(y_x[inlier_mask], pred_x)),
        "r2_y": float(r2_score(y_y[inlier_mask], pred_y)),
    }
    return {
        "lr_x": lr_x,
        "lr_y": lr_y,
        "inlier_mask": inlier_mask,
        "metrics": metrics,
    }


# -----------------------------------------------------------------------------
# Linear-model inversion (matches legacy A_to_B helper)
# -----------------------------------------------------------------------------


def _invert_pixel_physical(A: np.ndarray) -> np.ndarray:
    """
    Given A such that `physical = A @ [stage_x, stage_y, pixel_x, pixel_y, 1]`,
    return B such that `pixel = B @ [stage_x, stage_y, physical_x, physical_y, 1]`.
    """
    M = A[:, 2:4]
    M_inv = np.linalg.inv(M)
    A_stage = A[:, 0:2]
    A_bias = A[:, 4:5]
    B_stage = -M_inv @ A_stage
    B_physical = M_inv
    B_bias = -M_inv @ A_bias
    return np.hstack([B_stage, B_physical, B_bias])


def _to_yaml_blocks(
    mat_pixel_to_phys: np.ndarray,
    mat_phys_to_pixel: np.ndarray,
    fit_metrics: dict,
) -> dict:
    """Reshape the two 2x5 coefficient matrices into the legacy YAML schema."""

    def row(mat: np.ndarray, keys: list[str]) -> dict:
        return {keys[i]: float(mat[i]) for i in range(len(keys))}

    px2phys_keys = ["stage_pos_x", "stage_pos_y", "pixel_pos_x", "pixel_pos_y", "bias"]
    phys2px_keys = [
        "stage_pos_x",
        "stage_pos_y",
        "physical_pos_x",
        "physical_pos_y",
        "bias",
    ]
    return {
        "stage_and_pixel_to_physical": {
            "physical_pos_x": row(mat_pixel_to_phys[0], px2phys_keys),
            "physical_pos_y": row(mat_pixel_to_phys[1], px2phys_keys),
        },
        "stage_and_physical_to_pixel": {
            "pixel_pos_x": row(mat_phys_to_pixel[0], phys2px_keys),
            "pixel_pos_y": row(mat_phys_to_pixel[1], phys2px_keys),
        },
        "fit_metrics": fit_metrics,
    }


# -----------------------------------------------------------------------------
# Diagnostic plots
# -----------------------------------------------------------------------------


def save_diagnostic_plots(
    df_all: pd.DataFrame,
    df_used: pd.DataFrame,
    fit: dict,
    out_dir: Path,
) -> None:
    """
    Write a single diagnostic PNG covering: predicted vs actual physical
    positions, residual histograms, and spatial residual maps.
    """
    X = df_used[["stage_x_mm", "stage_y_mm", "pixel_x_px", "pixel_y_px"]].to_numpy()
    y_x = df_used["physical_x_mm"].to_numpy()
    y_y = df_used["physical_y_mm"].to_numpy()
    inlier = fit["inlier_mask"]
    pred_x = fit["lr_x"].predict(X)
    pred_y = fit["lr_y"].predict(X)

    fig, axes = plt.subplots(2, 3, figsize=(15, 9))

    # Predicted vs actual
    for ax, y, pred, label in [
        (axes[0, 0], y_x, pred_x, "physical_x"),
        (axes[0, 1], y_y, pred_y, "physical_y"),
    ]:
        ax.scatter(y[inlier], pred[inlier], c="C0", s=12, label="inlier")
        ax.scatter(y[~inlier], pred[~inlier], c="C3", s=12, label="outlier")
        lim = [min(y.min(), pred.min()), max(y.max(), pred.max())]
        ax.plot(lim, lim, "k--", lw=0.8)
        ax.set_xlabel(f"actual {label} (mm)")
        ax.set_ylabel(f"predicted {label} (mm)")
        ax.set_title(f"{label} prediction")
        ax.grid(alpha=0.3)
        ax.legend(loc="upper left", fontsize=8)

    # Detection count per tag (raw vs kept)
    ax = axes[0, 2]
    raw_count = df_all.groupby("apriltag_id").size()
    used_count = df_used.groupby("apriltag_id").size()
    tag_ids = sorted(set(raw_count.index) | set(used_count.index))
    x = np.arange(len(tag_ids))
    ax.bar(x - 0.2, [raw_count.get(t, 0) for t in tag_ids], 0.4, label="detected")
    ax.bar(x + 0.2, [used_count.get(t, 0) for t in tag_ids], 0.4, label="after MAD")
    ax.set_xticks(x)
    ax.set_xticklabels(tag_ids)
    ax.set_xlabel("AprilTag id")
    ax.set_ylabel("rows")
    ax.set_title("samples per tag")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)

    # Residual histograms
    res_x = y_x - pred_x
    res_y = y_y - pred_y
    ax = axes[1, 0]
    ax.hist(res_x[inlier], bins=40, alpha=0.7, label="inlier x")
    ax.hist(res_y[inlier], bins=40, alpha=0.7, label="inlier y")
    ax.axvline(0, color="k", ls="--", lw=0.8)
    ax.set_xlabel("residual (mm)")
    ax.set_title("inlier residuals")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)

    # Spatial map of inliers / outliers in physical space
    ax = axes[1, 1]
    ax.scatter(y_x[inlier], y_y[inlier], c="C0", s=14, alpha=0.6, label="inlier")
    ax.scatter(
        y_x[~inlier],
        y_y[~inlier],
        c="C3",
        marker="x",
        s=40,
        label="outlier",
    )
    ax.set_xlabel("physical_x (mm)")
    ax.set_ylabel("physical_y (mm)")
    ax.set_title("inlier/outlier (arena space)")
    ax.invert_yaxis()
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)

    # Spatial map of residual magnitude
    ax = axes[1, 2]
    mag = np.sqrt(res_x**2 + res_y**2)
    sc = ax.scatter(y_x, y_y, c=mag, cmap="inferno", s=14)
    plt.colorbar(sc, ax=ax, label="|residual| (mm)")
    ax.set_xlabel("physical_x (mm)")
    ax.set_ylabel("physical_y (mm)")
    ax.set_title("residual magnitude")
    ax.invert_yaxis()
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(alpha=0.3)

    fig.tight_layout()
    fig.savefig(out_dir / "diagnostics.png", dpi=120)
    plt.close(fig)


# -----------------------------------------------------------------------------
# Top-level entry point
# -----------------------------------------------------------------------------


def fit_arena_registration(
    arena_dir: str | Path,
    family: str = APRILTAG_FAMILY,
    min_decision_margin: float = 20.0,
    quad_decimate: float = 4.0,
    mad_threshold: float = 3.0,
    ransac_residual_threshold: float = 0.5,
    ransac_max_trials: int = 1000,
    visualize_detections: bool = False,
) -> dict:
    """
    End-to-end: load metadata + CSV, detect tags, drop outliers, fit
    model, save results to `<arena_dir>/model/`.

    Args:
        arena_dir: directory passed to `run-arena-registration-scan` (the
            one containing `metadata.yaml` and `mapping_scan/`).
        family: AprilTag family used on the registration board.
        min_decision_margin: drop AprilTag detections with decision
            margin below this value (suppresses noise-driven false
            positives from `tag16h5`).
        quad_decimate: decimation factor for the AprilTag quad detector.
            Lower values (e.g. 1.0) improve detection at small tag sizes
            at the cost of speed; higher values (e.g. 4.0) are faster.
        mad_threshold: per-tag burst-level rejection threshold.
        ransac_residual_threshold: RANSAC inlier threshold (mm).
        ransac_max_trials: RANSAC iterations.
        visualize_detections: also save per-image detection overlays
            under `<arena_dir>/model/detections/`.
    """
    arena_dir = Path(arena_dir).expanduser()
    out_dir = arena_dir / "model"
    out_dir.mkdir(parents=True, exist_ok=True)
    viz_dir = out_dir / "detections" if visualize_detections else None
    if viz_dir is not None:
        viz_dir.mkdir(parents=True, exist_ok=True)

    df_raw = gather_apriltag_points(
        arena_dir,
        family=family,
        min_decision_margin=min_decision_margin,
        quad_decimate=quad_decimate,
        viz_dir=viz_dir,
    )
    if len(df_raw) == 0:
        raise RuntimeError(
            f"No AprilTag corners detected in {arena_dir / 'mapping_scan'}."
        )
    print(f"Detected {len(df_raw)} corner points across all images.")

    df = filter_per_apriltag_outliers(df_raw, mad_threshold=mad_threshold)
    print(f"After per-tag MAD filtering:    {len(df)} corner points.")

    df.to_csv(out_dir / "calibration_points.csv", index=False)

    fit = fit_ransac_model(
        df,
        residual_threshold=ransac_residual_threshold,
        max_trials=ransac_max_trials,
    )
    m = fit["metrics"]
    print(
        f"RANSAC: {m['n_inliers']}/{m['n_total']} inliers "
        f"({m['n_outliers']} outliers); "
        f"RMSE x={m['rmse_x_mm']:.4f} mm, y={m['rmse_y_mm']:.4f} mm, "
        f"euclidean={m['rmse_euclidean_mm']:.4f} mm; "
        f"max residual={m['max_residual_mm']:.4f} mm; "
        f"R^2 x={m['r2_x']:.4f}, y={m['r2_y']:.4f}"
    )

    mat_pixel_to_phys = np.array(
        [
            [*fit["lr_x"].coef_, fit["lr_x"].intercept_],
            [*fit["lr_y"].coef_, fit["lr_y"].intercept_],
        ]
    )
    mat_phys_to_pixel = _invert_pixel_physical(mat_pixel_to_phys)

    result = _to_yaml_blocks(mat_pixel_to_phys, mat_phys_to_pixel, m)
    with open(out_dir / "calibration_result.yaml", "w") as f:
        yaml.safe_dump(result, f, sort_keys=False)

    save_diagnostic_plots(df_raw, df, fit, out_dir)

    print(f"Wrote results to {out_dir}/")
    return {"result": result, "metrics": m, "df_used": df, "df_raw": df_raw, "fit": fit}
