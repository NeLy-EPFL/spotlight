# `spotlight_ik` tools and caller scripts

Migrated from poseforge2's `spotlight_ik` (library + tools + caller
scripts), namespaced under `spotlight_tools` instead of
`poseforge.motion_prior`. The "Tools" section below documents the reusable
CLI tools in this directory; "Caller scripts" documents the
hardcoded-parameter orchestration scripts and the experiment history
behind them, exactly as originally written in poseforge2 (see the note at
the end of that section for what changed in the move).

## Tools

Reusable, general-purpose CLI tools for fitting inverse kinematics (via
[QuickIK](../../../../quickik)) to this project's 2D pose predictions (see
`scripts/spotlight_pose2d/`), and for rendering QA videos of the fit. Each
takes `--input-path`/`--output-path` (or `--output-dir`) and other CLI args
(run `--help` on either of them for the full list). Library code
(`calibration.py`, `periods.py`, `neuromechfly.py`, `io_utils.py`) lives in
`src/spotlight_postprocessing/spotlight_ik/`.

### One-time setup: the body-plan asset

`solve_ik.py` fits the NeuroMechFly body plan; its JSON export
(`src/spotlight_tools/assets/neuromechfly_ypr_legs.json`) is generated once
via `../flygym/scripts/export_model_for_quickik.py` (writes to the current
directory):

```sh
cd tools/src/spotlight_tools/assets && uv run python ../../../../../flygym/scripts/export_model_for_quickik.py
```

### Single-trial pipeline

1. `solve_ik.py`: takes one trial's dense pose `.h5` (see
   `spotlight_pose2d.io_utils.save_pose_h5`), finds contiguous
   high-confidence periods, fits IK/FK to each via QuickIK, and saves a
   periods+IK/FK `.h5` (see `spotlight_ik.io_utils.save_ikfk_h5`).
2. `make_videos.py`: takes one `solve_ik.py` output `.h5` and renders a
   short QA clip per sampled period - the raw prediction skeleton (white),
   and, with `--with-ik`, the IK/FK fit on top in the same per-leg colors as
   `visualize_predictions.py` (see `spotlight_pose2d.viz`), plus a second
   panel with a synthetic 3D view of the IK reconstruction.

## Caller scripts

Hardcoded-parameter orchestration scripts for this project's specific
dataset - no CLI args, chaining the reusable tools above over
`run_inference_final.sh`'s (see `scripts/spotlight_pose2d/`) 29-trial
output.

`run_solve_ik.sh` fits IK/FK to every trial into
`inverse_kinematics/ikfk/<trial>_ikfk.h5`; `run_make_videos.sh` then renders
one QA clip per trial (with the IK/FK overlay and 3D panel) into
`inverse_kinematics/qa_videos/`. Both skip trials they've already produced
output for, so re-running is safe. Everything lives under
`bulk_data/motion_prior/2dpose_model/inverse_kinematics/`, separate from the
2D-pose model's own `labels/`.

**Note on the history below**: this section (commands, paths, dates) is
preserved exactly as written when this code lived in poseforge2 -- the
`bulk_data/motion_prior/2dpose_model/...` paths and
`cd /home/sibwang/Projects/poseforge2` below are poseforge2's own layout at
the time, not spotlight-tools'. It's kept as-is for the experiment record;
none of the tuned values it describes (`neutral_weight=0.2`,
`--min-joint-excursion-deg 10.0`, etc.) changed in the move to
spotlight-tools -- they're still `solve_ik.py`'s defaults here.

### Commands run

One-time body-plan asset generation (see "Tools" above), then both caller
scripts over the full 29-trial dataset:

```sh
cd src/poseforge/assets && uv run python ../../../../flygym/scripts/export_model_for_quickik.py
cd /home/sibwang/Projects/poseforge2

bash scripts/spotlight_ik/run_solve_ik.sh
bash scripts/spotlight_ik/run_make_videos.sh
```

QuickIK was later rebuilt from its `solver-api-redesign` branch (`mapper=
XYView()` -> `projection=Projection.new_ortho_xy()`); `solve_ik.py` was
updated to match (`uv sync --reinstall-package quickik` to pick up the
rebuild for this project's own Python 3.13 venv, since it had initially
only been rebuilt for 3.12/3.14).

Per-joint observation weights (`spotlight_ik.neuromechfly.
base_observation_weight`) and `neutral_weight` were then tuned (see
`inverse_kinematics/qa_videos/neutral_weight_sweep/README.md` for the
`neutral_weight` sweep specifically -- settled on 0.2 for production).
Both caller scripts were re-run in full afterward, so every trial's output
reflects these final settings uniformly (all previous, since-superseded
output was deleted first, not left mixed in):

```sh
rm -rf bulk_data/motion_prior/2dpose_model/inverse_kinematics/ikfk
bash scripts/spotlight_ik/run_solve_ik.sh
bash scripts/spotlight_ik/run_make_videos.sh
```

29/29 trials fit successfully; fk-to-pred mismatch averaged ~0.077mm (mean)
and ~0.192mm (mean of each frame's worst leg keypoint) across all trials,
well under the `--max-mismatch` 0.3mm acceptance threshold (used for every
production run -- only ever disabled, via `--max-mismatch inf`, for the
`neutral_weight` sweep's own diagnostic comparisons, never for real
output). QA clips (`inverse_kinematics/qa_videos/*.mp4`) show the IK/FK fit
(per-leg colors, matching `visualize_predictions.py`) tracking the raw
prediction (white) closely on all six legs, both in the 2D overlay and the
synthetic 3D panel.

A third refinement stage (`spotlight_ik.periods.compute_joint_excursion_deg`,
`solve_ik.py`'s `--min-joint-excursion-deg`/`--movement-window-ms`) was
added next, rejecting frames where the fly is holding still (in joint-angle
space, not body displacement -- grooming/poking in place still counts as
"moving"), so periods only cover genuine leg activity. Both caller scripts
were re-run in full again, once `spotlight_orient`'s own v13 training (co-
resident on the same machine) finished, to avoid resource contention:

```sh
rm -rf bulk_data/motion_prior/2dpose_model/inverse_kinematics/ikfk
bash scripts/spotlight_ik/run_solve_ik.sh
bash scripts/spotlight_ik/run_make_videos.sh
```

29/29 trials fit successfully; across all trials, 1183 confidence-based
periods -> 1432 mismatch-refined -> 1032 movement-refined (241,516 frames
retained overall), mean mismatch ~0.075mm. The movement filter's effect
varies a lot by trial -- negligible on some, substantial on others (e.g.
one trial: 76 mismatch-refined periods down to 39) -- consistent with how
much genuine stillness each trial's fly happened to have within its
already-quality-gated periods.

**Bug found and fixed**: the movement re-segmentation's own `find_periods`
call used `movement_window_frames` (100ms, ~40 frames) as its closing
size. `scipy.ndimage.binary_closing`'s erosion step treats out-of-bounds
neighbors as `False`, so closing with a window longer than a period erases
the *entire* period regardless of its actual movement content -- silently
discarding many legitimately-active short (30-45 frame) periods. Fixed by
using `--filtering-mask-frames` (the same small closing size the mismatch
stage already uses) instead, decoupled from the excursion-smoothing
window. Verified with a direct before/after comparison on one trial (44
vs. 25 periods retained at the same threshold, before vs. after the fix).

With the fix in place, an empirical look at the excursion distribution
across all trials (with the movement filter disabled, so as not to have
already cut what we're trying to measure) showed a fairly continuous
climb, not a sharp gap: `[0,3)deg` 8.4% of frames, `[3,5)deg` 13.9%,
`[5,8)deg` 6.8%, `[8,10)deg` 1.9%, `[10,15)deg` 4.7%, `[15,20)deg` 5.6%,
`[20,30)deg` 17.5%, `[30,inf)deg` 41.0%. `--min-joint-excursion-deg` was
raised from 5.0 to **10.0** (clearing the still-fairly-subtle 5-10 degree
band too) and both caller scripts re-run in full again:

```sh
rm -rf bulk_data/motion_prior/2dpose_model/inverse_kinematics/ikfk
bash scripts/spotlight_ik/run_solve_ik.sh
bash scripts/spotlight_ik/run_make_videos.sh
```

29/29 trials fit successfully; 1183 confidence-based -> 1432
mismatch-refined -> **1662 movement-refined** periods, 251,499 frames
retained, mean mismatch ~0.076mm. Period count and total frames both went
up relative to the pre-fix 5deg run (1032 periods / 241,516 frames)
despite the higher threshold -- the erasure-bug fix recovered more
previously-discarded valid short periods than the higher threshold newly
excludes.
