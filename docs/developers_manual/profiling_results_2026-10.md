# Recorder profiling results (October 2026)

Profiling of `run-spotlight` and `run-spotlight-headless` done on the rig on
2026-10-11, on the `headless-recorder-profiling` branch (per-stage timing via
`stage_timing.h`; `STAGE_TIMING` lines are logged at shutdown).

## Setup

- Profile `sibo_261005`, arena `arena146x146`, OpenCV 4.8.0 (self-built in
  `/usr/local`, libjpeg-turbo 2.1.5, libtiff 4.3.0), Qt 6.8.2, Ubuntu 26.04.
- Workload: 20 s streaming (muscle off), 20 s streaming (muscle on), 65 s open
  recording at 396 fps behavior / 66 fps muscle (sync ratio 6), 1 ms exposure
  and light-on, tracking on (no fly), saving to `/mnt/spotlight-data`.
- CPU per thread from `pidstat`, system from `mpstat`. Processes ran at nice 5;
  the machine shows a constant ~25% iowait even when idle (not real disk load).
- "On disk" means written to the OS page cache (no fsync).

## Headline results

- The `Recorder` refactor does not change performance (GUI before/after equal
  within run-to-run noise, identical frame counts).
- No frames are dropped. The only gaps are the 6 muscle re-sync pauses per 65 s
  (one muscle frame period every `resync_interval_s` = 10 s), by design.
- **Debug vs Release: no difference** (all stages within ~1%). The hot code is
  in prebuilt libraries (OpenCV, libjpeg-turbo, libtiff, Qt); the recorder's own
  `-O0` code is not on the hot path.
- The GUI costs ~3.5 cores in total: ~1.2 directly (GUI thread 95%, Qt pool
  threads) and ~2 indirectly, because the savers run ~15% slower while it is
  active (memory bandwidth / cache contention).

## Where time goes (GUI open, Release; Debug is the same)

Per item: median / p99 (ms). Cores = per-item time x rate.

| Stage | Per item | Cores | Headless |
|---|---|---|---|
| Behavior save, per 3-frame group (132/s) | 24.3 / 34 | ~3.2 | 19.4 / 29 |
| - JPEG encode, quality 100 | 16.5 / 23 | 2.2 | 14.0 / 19 |
| - pack 3 frames into pseudo-BGR | 6.6 / 13 | 0.9 | 4.6 / 13 |
| - write JPEG + CSV | 0.7 | 0.1 | 0.5 |
| Muscle save, per pair (66/s) | 56.4 / 67 | ~3.7 (4 threads) | 49.3 / 58 |
| - TIFF LZW encode, per image | 26.1 / 33 | 3.4 | 23.3 / 29 |
| - rotate + write TIFF + CSV, per image | ~2 | 0.3 | ~1.4 |
| GUI behavior preview, per tick | 31.9 / 42 | 0.6 | - |
| - OpenCV (rotate, color, red tint, marker) | 21.0 | | |
| - cv::Mat -> QImage, fromImage, scaled | 10.7 | | |
| GUI muscle preview, per tick | 13.6 / 20 | 0.3 | - |
| - 2x histogram + 16->8 bit normalize | 8.7 | | |
| Tracking loop (30 Hz) | 7.5 / 16 | 0.2 | 5.9 / 10 |
| Stage position logger (75 Hz) | 2.9 / 7.5 | 0.2 | 2.6 / 5.7 |
| Copy frame out of grabber buffer (396/s) | 0.44 / 1.3 | 0.2 | 0.33 / 1.1 |

The two preview ticks need ~46 ms of work per 40 ms period, so they actually run
at ~19 Hz instead of the configured 25 Hz, with the GUI thread saturated.

## Latency (median / p99, ms)

| Path | GUI open | Headless |
|---|---|---|
| Behavior: grabber buffer timestamp -> recorder has frame | 2.35 / 3.9 | 2.33 / 2.8 |
| Behavior: frame received -> JPEG on disk (newest / oldest in group) | 24.8 / 35 - 29.8 / 40 | 19.8 / 29 - 24.8 / 34 |
| Behavior: wait in saver queue | 0.5 / 1.9 | 0.35 / 1.2 |
| Muscle: PCO server fetched -> recorder has pair | 0.6 / 2.3 | 0.4 / 0.9 |
| Muscle: wait in saver queue | 0.55 / 13.5 | 0.23 / 1.3 |
| Muscle: server fetched -> both TIFFs on disk | 58.3 / 75 | 50.1 / 59 |
| GUI: frame received -> behavior preview handed to Qt | 33.8 / 44 | - |
| GUI: frame received -> muscle preview handed to Qt | 21.4 / 34 | - |
| Tracking: frame age when the tracking loop reads it | 2.0 / 38.6 | 1.6 / 3.0 |

- The grabber timestamp is presumably taken at trigger / start of exposure (2.3 ms
  matches 1 ms exposure + ~1.2 ms CoaXPress transfer of 3 MP); not confirmed in
  the Euresys docs. Exposure -> JPEG on disk is therefore ~28-33 ms typical,
  ~44 ms p99 with the GUI open.
- On-screen preview latency adds up to one preview period (~50 ms at 19 Hz) and
  the display refresh, i.e. roughly 35-90 ms.
- Muscle camera exposure -> PCO server fetch is not measured.

## Preview micro-benchmark (idle machine, Debug)

Behavior tick 12.8 ms (red tint 4.2, BGR->RGB copy 3.3, `QPixmap::fromImage`
2.7), muscle tick 9.6 ms (histograms 7.2). Downscaling the behavior frame to the
label size first, then doing everything on the small image: 2.6 ms (~5x faster).

## Recommendations (from these results)

1. TIFF LZW is the single largest cost (~3.4 cores); the 4 muscle savers are
   ~93% busy with the GUI open. Evaluate faster lossless TIFF compression
   (Deflate via libdeflate, ZSTD, with horizontal predictor) or no compression.
2. JPEG quality 100 costs ~2.2 cores. OpenCV already uses libjpeg-turbo.
3. Packing 3 frames into the pseudo-BGR image takes 6.6 ms; worth a look.
4. Downscale before drawing in the behavior preview; this also reduces the
   savers' slowdown and the tracking frame-age spikes caused by the GUI.
5. A Release build brings no speedup.

## Follow-up: OpenCV 4.10, preview downscaling, TIFF codecs, OpenCV threads

All Debug, same workload, GUI open.

### OpenCV 4.10 (Ubuntu) vs self-built 4.8

The self-built 4.8 had Intel IPP and the pthreads backend; Ubuntu's 4.10 has no
IPP and uses TBB. Some OpenCV calls got slower (packing 3 behavior frames 6.6 ->
9.1 ms, muscle preview 13.5 -> 17.6 ms per tick); JPEG encoding got faster
(16.5 -> 13.8 ms, less GUI contention). libjpeg-turbo is used by both.

### Downscale-first behavior preview

Behavior preview tick 32.0 -> 5.8 ms (p50); frame age when handed to Qt 33.8 ->
7.5 ms; GUI thread 95% -> 63% while recording. Old and new pipelines produce the
same image (mean |diff| < 1 gray level with the real arena mask/calibration).

### OpenCV worker threads (`cv::setNumThreads(1)`)

OpenCV's internal thread pool spins between the many small parallel calls
(previews, tracking): ~12 pool threads at 13-20% CPU each even while only
streaming, with both 4.8 and 4.10. `OPENCV_FOR_THREADS_NUM=1` had no effect;
`cv::setNumThreads(1)` did:

| Process CPU | 4.8, old preview | 4.10, new preview | + setNumThreads(1) |
|---|---|---|---|
| Streaming, muscle off | 215% | 265% | 75% |
| Streaming, muscle on | 293% | 375% | 123% |
| Recording | 1027% | 980% | 811% |

Savers got slightly faster (muscle pair 54.3 -> 52.0 ms, behavior received ->
on disk 24.3 -> 22.7 ms). Single calls get slower: preview downscale 3.9 -> 15
ms under load (INTER_AREA at a non-integer factor; idle single-thread 3.6 ms,
1.3 ms when halving first with INTER_AREA), tracking loop 8.6 -> 12.1 ms (budget
33 ms).

### TIFF compression for muscle frames (200 real frames, single thread, idle)

Through `cv::imwrite` (default levels):

| Option | Encode | MB / image | Ratio |
|---|---|---|---|
| None | 1.4 ms | 2.51 | 1.00 |
| LZW (current) | 15.0 ms | 1.57 | 1.59 |
| Deflate | 52.0 ms | 1.43 | 1.76 |
| ZSTD | 29.6 ms | 1.35 | 1.85 |

The horizontal predictor does not help on these frames (slightly larger files).
With libtiff directly (levels not settable via OpenCV): Deflate level 1 25.2 ms /
1.36 MB, **ZSTD level 1 2.6 ms / 1.53 MB**, ZSTD level 3 13.9 ms / 1.54 MB.

Readers: opencv-python 4.14 (used by `tools/`) cannot read ZSTD TIFFs ("ZSTD
compression support is not configured"); tifffile can. LZW, Deflate, and
uncompressed TIFFs are read by both.
