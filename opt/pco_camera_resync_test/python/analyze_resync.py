# /// script
# requires-python = ">=3.11"
# dependencies = ["numpy>=2.0", "matplotlib>=3.10"]
# ///
"""Analyzes a recording of cpp/pco_camera_resync_test (see ../README.md):
prints a summary and saves resync_test.png in the recording directory.

Usage: uv run analyze_resync.py <recording_dir> [--min-pulse-us 10]
"""

import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

CAMERAS = ["calcium", "fiducial"]
# Plot colors (categorical slots 1-2, recessive ink and grid), same as
# pco_camera_clock_drift_profiling/python/analyze_clock_drift.py
CAMERA_COLORS = {"calcium": "#2a78d6", "fiducial": "#eb6834"}
TEXT_PRIMARY = "#0b0b0b"
TEXT_SECONDARY = "#52514e"
GRID_COLOR = "#e4e3df"
SURFACE_COLOR = "#fcfcfb"
ACQUIRE_LOW_COLOR = "#e4e3df"
# Time from sending the command to the start of sampling: the firmware holds
# acquire enable LOW for 1 s first (USB latency is neglected)
SAMPLING_START_HOST_S = 1.0


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


def find_rises(signal: np.ndarray) -> np.ndarray:
    """Returns the sample indices of the rising edges."""
    changes = np.flatnonzero(np.diff(signal)) + 1
    return changes[signal[changes] == 1]


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
    n_samples = metadata["n_samples"]
    duration_us = n_samples * period_us
    signals = load_samples(args.recording_dir / "samples.bin", n_samples)
    interval_us = (
        metadata["calcium_shutter_open_time_us"] + metadata["readout_time_us"]
    )
    # Acquire-enable LOW windows (re-syncs), in us since the start of sampling
    resync_us = np.arange(
        metadata["resync_interval_us"], duration_us, metadata["resync_interval_us"]
    )
    release_us = resync_us + metadata["acquire_low_us"]
    print(
        f"{n_samples} samples every {period_us} us ({duration_us / 1e6:.2f} s), "
        f"{len(resync_us)} re-syncs (acquire enable LOW for "
        f"{metadata['acquire_low_us']} us), theoretical interval "
        f"{interval_us:.3f} us"
    )

    # Common-time starts (us since the start of sampling). The common time
    # (SMA #4 HIGH) starts one rolling time after the exposure of the first
    # line, so a frame whose common time starts while acquire is LOW was
    # started before.
    rolling_time_us = metadata["roi_size"] * metadata["line_time_us"]
    on_us = {}
    for name in CAMERAS:
        clean, n_glitches = deglitch(
            signals[name], round(args.min_pulse_us / period_us)
        )
        on_us[name] = find_rises(clean) * period_us
        exposure_start_us = on_us[name] - rolling_time_us
        print(f"{name}: {len(on_us[name])} exposures, {n_glitches} glitches removed")
        n_started_while_low = sum(
            np.count_nonzero(
                (exposure_start_us > low) & (exposure_start_us < release)
            )
            for low, release in zip(resync_us, release_us)
        )
        if n_started_while_low:
            print(
                f"  WARNING: {n_started_while_low} exposures started while "
                "acquire was LOW"
            )
        for low, release in zip(resync_us, release_us):
            before = exposure_start_us[exposure_start_us < low][-1]
            after = exposure_start_us[exposure_start_us > release][0]
            print(
                f"  re-sync at {low / 1e6:.0f} s: last exposure started "
                f"{low - before:.0f} us before LOW, first exposure "
                f"{after - release:.0f} us after release, gap "
                f"{after - before:.0f} us ({(after - before) / interval_us:.2f} "
                "frame intervals)"
            )

    # Fiducial - calcium offset, pairing frames by index within each segment
    segment_bounds_us = np.concatenate([[0], release_us, [duration_us]])
    offset_time_s, offset_us = [], []
    for start, end in zip(segment_bounds_us[:-1], segment_bounds_us[1:]):
        starts = {
            name: on_us[name][(on_us[name] > start) & (on_us[name] < end)]
            for name in CAMERAS
        }
        n_frames = min(len(starts[name]) for name in CAMERAS)
        if any(len(starts[name]) != n_frames for name in CAMERAS):
            print(
                f"WARNING: frame counts differ in the segment starting at "
                f"{start / 1e6:.2f} s ({len(starts['calcium'])} calcium, "
                f"{len(starts['fiducial'])} fiducial)"
            )
        offset = starts["fiducial"][:n_frames] - starts["calcium"][:n_frames]
        print(
            f"segment {start / 1e6:5.2f}-{end / 1e6:5.2f} s: {n_frames} frames, "
            f"fiducial - calcium offset {offset[0]:+.0f} us at start, "
            f"{offset[-1]:+.0f} us at end"
        )
        offset_time_s.append(starts["calcium"][:n_frames] / 1e6)
        offset_us.append(offset)

    # Grabbed frames (host time since the command was sent, shifted to the
    # Arduino's time axis)
    grabbed = {}
    for name in CAMERAS:
        frames = np.loadtxt(
            args.recording_dir / f"frames_{name}.csv",
            delimiter=",",
            skiprows=1,
            ndmin=2,
        )
        grabbed[name] = frames
        image_numbers = frames[:, 1]
        n_skipped = int(np.sum(np.diff(image_numbers) - 1))
        print(
            f"{name}: {len(frames)} frames grabbed (recorder image numbers "
            f"{image_numbers[0]:.0f}-{image_numbers[-1]:.0f}, {n_skipped} skipped "
            f"by the grab loop), {len(on_us[name])} exposures"
        )
    errors = (args.recording_dir / "grab_errors.csv").read_text().splitlines()[1:]
    print(f"{len(errors)} grab errors")
    for line in errors:
        print(f"  {line}")

    fig, (ax_offset, ax_grab) = plt.subplots(
        2, 1, figsize=(12, 8), facecolor=SURFACE_COLOR, sharex=True
    )
    for ax in (ax_offset, ax_grab):
        for low, release in zip(resync_us, release_us):
            ax.axvspan(low / 1e6, release / 1e6, color=ACQUIRE_LOW_COLOR, lw=0)
            ax.axvline(low / 1e6, color=TEXT_SECONDARY, lw=1, ls="--")
    for i, (time_s, offset) in enumerate(zip(offset_time_s, offset_us)):
        ax_offset.plot(
            time_s,
            offset,
            color=TEXT_PRIMARY,
            lw=2,
            label="fiducial - calcium" if i == 0 else None,
        )
    style_axes(
        ax_offset,
        "(a) Fiducial - calcium common-time start offset (dashed: re-sync)",
        "",
        "offset (us)",
    )
    ax_offset.legend(frameon=False, fontsize=9, labelcolor=TEXT_SECONDARY)

    for name in CAMERAS:
        host_time_s = grabbed[name][:, 0] / 1e6 - SAMPLING_START_HOST_S
        ax_grab.plot(
            host_time_s[1:],
            np.diff(grabbed[name][:, 0]) / 1e3,
            ".",
            ms=4,
            color=CAMERA_COLORS[name],
            label=name,
        )
    ax_grab.axhline(interval_us / 1e3, color=TEXT_SECONDARY, lw=1.5, ls=":")
    style_axes(
        ax_grab,
        "(b) Host time between grabbed frames (dotted: theoretical frame interval)",
        "time since sampling start (s; host times aligned approximately)",
        "interval (ms)",
    )
    ax_grab.legend(frameon=False, fontsize=9, labelcolor=TEXT_SECONDARY)

    fig.tight_layout()
    output_path = args.recording_dir / "resync_test.png"
    fig.savefig(output_path, dpi=150, facecolor=SURFACE_COLOR)
    print(f"Saved {output_path}")


if __name__ == "__main__":
    main()
