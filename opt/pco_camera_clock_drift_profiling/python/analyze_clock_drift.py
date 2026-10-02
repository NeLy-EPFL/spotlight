# /// script
# requires-python = ">=3.11"
# dependencies = ["numpy>=2.0", "matplotlib>=3.10"]
# ///
"""Analyzes a recording of cpp/pco_camera_clock_drift_profiling (see
../README.md): prints a summary and saves clock_drift.png in the recording
directory.

Usage: uv run analyze_clock_drift.py <recording_dir> [--min-pulse-us 10]
"""

import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

CAMERAS = ["calcium", "fiducial"]
# Plot colors (categorical slots 1-2, recessive ink and grid)
CAMERA_COLORS = {"calcium": "#2a78d6", "fiducial": "#eb6834"}
TEXT_PRIMARY = "#0b0b0b"
TEXT_SECONDARY = "#52514e"
GRID_COLOR = "#e4e3df"
SURFACE_COLOR = "#fcfcfb"
# Data points shown per violin (randomly sampled if there are more)
MAX_POINTS_PER_VIOLIN = 300


def load_samples(bin_path: Path, n_samples: int) -> dict[str, np.ndarray]:
    """Unpacks the samples (4 per byte; sample i in bits 2 * (i % 4) (calcium)
    and 2 * (i % 4) + 1 (fiducial) of byte i / 4)."""
    bits = np.unpackbits(np.fromfile(bin_path, np.uint8), bitorder="little")
    bits = bits[: 2 * n_samples]
    return {"calcium": bits[0::2], "fiducial": bits[1::2]}


def deglitch(signal: np.ndarray, min_run_samples: int) -> tuple[np.ndarray, int]:
    """Removes HIGH pulses, then LOW gaps, shorter than min_run_samples (runs
    touching the start or end of the recording are kept). Returns the cleaned
    signal and the number of removed runs."""
    clean = signal.copy()
    n_removed = 0
    for level in (1, 0):
        starts = np.concatenate([[0], np.flatnonzero(np.diff(clean)) + 1])
        lengths = np.diff(np.concatenate([starts, [len(clean)]]))
        is_short = (clean[starts] == level) & (lengths < min_run_samples)
        is_short[[0, -1]] = False
        for start, length in zip(starts[is_short], lengths[is_short]):
            clean[start : start + length] = 1 - level
        n_removed += is_short.sum()
    return clean, n_removed


def find_pulses(signal: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Returns the sample indices of the rising and falling edges of all
    complete pulses."""
    changes = np.flatnonzero(np.diff(signal)) + 1
    rises = changes[signal[changes] == 1]
    falls = changes[signal[changes] == 0]
    falls = falls[falls > rises[0]]
    return rises[: len(falls)], falls


def style_axes(ax: plt.Axes, title: str, xlabel: str, ylabel: str) -> None:
    ax.set_title(title, loc="left", color=TEXT_PRIMARY, fontsize=11)
    ax.set_xlabel(xlabel, color=TEXT_SECONDARY)
    ax.set_ylabel(ylabel, color=TEXT_SECONDARY)
    ax.set_facecolor(SURFACE_COLOR)
    ax.grid(color=GRID_COLOR, linewidth=0.8)
    ax.set_axisbelow(True)
    ax.tick_params(colors=TEXT_SECONDARY, labelsize=9)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(GRID_COLOR)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("recording_dir", type=Path)
    parser.add_argument(
        "--min-pulse-us",
        type=float,
        default=10,
        help="HIGH pulses and LOW gaps shorter than this are removed as glitches",
    )
    args = parser.parse_args()

    metadata = json.loads((args.recording_dir / "metadata.json").read_text())
    period_us = metadata["sample_period_us"]
    signals = load_samples(
        args.recording_dir / "samples.bin", metadata["n_samples"]
    )
    print(
        f"{metadata['n_samples']} samples every {period_us} us "
        f"({metadata['n_samples'] * period_us / 1e6:.2f} s)"
    )

    # Common-time starts (rising edges) and durations (us), and their
    # theoretical values: frame interval = applied shutter-open time + readout
    # time, common time = applied shutter-open time - rolling time
    on_us, duration_us, interval_us, common_time_us = {}, {}, {}, {}
    for name in CAMERAS:
        clean, n_glitches = deglitch(
            signals[name], round(args.min_pulse_us / period_us)
        )
        rises, falls = find_pulses(clean)
        on_us[name] = rises * period_us
        duration_us[name] = (falls - rises) * period_us
        shutter_open_time_us = metadata[f"{name}_shutter_open_time_us"]
        interval_us[name] = shutter_open_time_us + metadata["readout_time_us"]
        common_time_us[name] = (
            shutter_open_time_us - metadata["roi_size"] * metadata["line_time_us"]
        )
        intervals = np.diff(on_us[name])
        print(
            f"{name}: {len(rises)} frames, {n_glitches} glitches removed, "
            f"theoretical interval {interval_us[name]:.3f} us, theoretical "
            f"common time {common_time_us[name]:.3f} us"
        )
        print(
            f"  interval:    mean {intervals.mean():.3f} us, "
            f"std {intervals.std():.3f} us, "
            f"range [{intervals.min():.0f}, {intervals.max():.0f}] us"
        )
        print(
            f"  common time: mean {duration_us[name].mean():.3f} us, "
            f"std {duration_us[name].std():.3f} us, "
            f"range [{duration_us[name].min():.0f}, "
            f"{duration_us[name].max():.0f}] us"
        )

    # Deviation from the theoretical starts t0 + k * interval, with t0 the
    # calcium camera's first start for both cameras. Frames are paired by index.
    n_frames = min(len(on_us[name]) for name in CAMERAS)
    if any(len(on_us[name]) != n_frames for name in CAMERAS):
        print(f"WARNING: frame counts differ; using the first {n_frames} frames")
    t0_us = on_us["calcium"][0]
    frame_index = np.arange(n_frames)
    deviation_us = {
        name: on_us[name][:n_frames] - (t0_us + frame_index * interval_us[name])
        for name in CAMERAS
    }
    time_s = on_us["calcium"][:n_frames] / 1e6
    gap_us = deviation_us["fiducial"] - deviation_us["calcium"]
    # Linear fit of the gap; the slope in us per s is the drift in ppm
    drift_ppm, intercept_us = np.polyfit(time_s, gap_us, 1)
    residuals_us = gap_us - (drift_ppm * time_s + intercept_us)
    rmse_us = np.sqrt(np.mean(residuals_us**2))
    r2 = 1 - np.sum(residuals_us**2) / np.sum((gap_us - gap_us.mean()) ** 2)
    print(
        f"fiducial - calcium: first {gap_us[0]:+.0f} us, last {gap_us[-1]:+.0f} "
        f"us, linear fit: drift {drift_ppm:+.2f} ppm, intercept "
        f"{intercept_us:+.2f} us, RMSE {rmse_us:.2f} us, R2 {r2:.4f}"
    )

    fig, axes = plt.subplots(2, 2, figsize=(13, 8.5), facecolor=SURFACE_COLOR)
    ax_deviation, ax_gap, ax_interval, ax_duration = axes.flat
    for name in CAMERAS:
        ax_deviation.plot(
            on_us[name][:n_frames] / 1e6,
            deviation_us[name],
            color=CAMERA_COLORS[name],
            lw=2,
            label=name,
        )
    style_axes(
        ax_deviation,
        "(a) Common-time start - theoretical start",
        "time since acquire enable (s)",
        "deviation (us)",
    )
    ax_deviation.legend(frameon=False, fontsize=9, labelcolor=TEXT_SECONDARY)

    ax_gap.plot(time_s, gap_us, color=CAMERA_COLORS["calcium"], lw=1.5, label="data")
    ax_gap.plot(
        time_s[[0, -1]],
        drift_ppm * time_s[[0, -1]] + intercept_us,
        color=TEXT_PRIMARY,
        lw=2,
        ls="--",
        label=f"linear fit: {drift_ppm:+.2f} ppm, RMSE {rmse_us:.2f} us, "
        f"R$^2$ {r2:.3f}",
    )
    style_axes(
        ax_gap,
        "(b) Fiducial - calcium deviation",
        "time since acquire enable (s)",
        "difference (us)",
    )
    ax_gap.legend(frameon=False, fontsize=9, labelcolor=TEXT_SECONDARY)
    ax_gap.text(
        0.98,
        0.04,
        f"difference = {drift_ppm:.2f} us/s * t {intercept_us:+.2f} us",
        transform=ax_gap.transAxes,
        ha="right",
        va="bottom",
        fontsize=10,
        color=TEXT_PRIMARY,
    )

    # Horizontal violin plots (kernel density estimates) with the data points
    # (jittered vertically only), and the theoretical values as dashed lines.
    # The values are quantized to the sample period, so the kernel width is one
    # sample period (narrower kernels show one peak per quantization step).
    rng = np.random.default_rng(0)
    for ax, values, theoretical, title, xlabel in (
        (
            ax_interval,
            {name: np.diff(on_us[name]) for name in CAMERAS},
            interval_us,
            "(c) Intervals between consecutive common-time starts",
            "interval (us)",
        ),
        (
            ax_duration,
            duration_us,
            common_time_us,
            "(d) Common-time durations",
            "duration (us)",
        ),
    ):
        violins = ax.violinplot(
            [values[name] for name in CAMERAS],
            orientation="horizontal",
            showmedians=True,
            bw_method=lambda kde: period_us / kde.dataset.std(),
        )
        for position, (body, name) in enumerate(
            zip(violins["bodies"], CAMERAS), start=1
        ):
            body.set_facecolor(CAMERA_COLORS[name])
            body.set_edgecolor(CAMERA_COLORS[name])
            body.set_alpha(0.3)
            points = values[name]
            if len(points) > MAX_POINTS_PER_VIOLIN:
                points = rng.choice(points, MAX_POINTS_PER_VIOLIN, replace=False)
            ax.scatter(
                points,
                position + rng.uniform(-0.15, 0.15, len(points)),
                s=8,
                color=CAMERA_COLORS[name],
                alpha=0.6,
                lw=0,
            )
        for part in ("cbars", "cmins", "cmaxes", "cmedians"):
            violins[part].set_color(TEXT_SECONDARY)
        for i, value in enumerate(sorted(set(theoretical.values()))):
            ax.axvline(
                value,
                color=TEXT_SECONDARY,
                lw=1.5,
                ls="--",
                label="theoretical" if i == 0 else None,
            )
        ax.set_yticks(range(1, len(CAMERAS) + 1), CAMERAS)
        style_axes(ax, title, xlabel, "")
        ax.legend(frameon=False, fontsize=9, labelcolor=TEXT_SECONDARY)

    fig.tight_layout()
    output_path = args.recording_dir / "clock_drift.png"
    fig.savefig(output_path, dpi=150, facecolor=SURFACE_COLOR)
    print(f"Saved {output_path}")


if __name__ == "__main__":
    main()
