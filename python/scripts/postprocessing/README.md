# Inverse kinematics

`src/spotlight/postprocessing/invkin/` fits [QuickIK](../../../../quickik)
to this project's 2D pose predictions (see
`scripts/postprocessing/model_training/pose2d/`) and renders QA clips of
the fit:

- `solve_ik.solve_ik`: takes one trial's dense pose `.h5` (see
  `pose2d.io_utils.save_pose_h5`), fits IK/FK per frame via QuickIK
  (skipping frames the localization model flagged as flipped or with low
  2D-pose confidence), and saves a dense per-frame `inverse_kinematics.h5`
  (see `invkin.io_utils.save_kinematics_h5`).
- `qa_clips.render_qa_clips`: takes a `solve_ik` output and the aligned
  behavior video it came from, and renders a QA clip showing the raw
  prediction skeleton alongside the IK/FK fit, plus a synthetic 3D panel of
  the reconstruction.

Both are plain functions, called directly from `postprocess-recording`'s
own pipeline (`--inverse-kinematics.enabled`); there's no separate CLI for
either. Library code (`mapping.py`, `neuromechfly.py`, `io_utils.py`) lives
alongside them in `invkin/`.

## One-time setup: the body-plan asset

`solve_ik` fits the NeuroMechFly body plan; its JSON export
(`src/spotlight/assets/inverse_kinematics/neuromechfly_ypr_legs.json`) is generated once via
`../flygym/scripts/export_model_for_quickik.py` (writes to the current
directory):

```sh
cd python/src/spotlight/assets && uv run python ../../../../../flygym/scripts/export_model_for_quickik.py
```

## History

This project used to run IK fitting and QA-video rendering as two separate
CLI scripts (`solve_ik.py`, `make_videos.py`) driven by hardcoded-parameter
caller scripts (`run_solve_ik.sh`, `run_make_videos.sh`) over a fixed
29-trial dataset, with a "periods" concept (only fitting IK over
confidence-gated, mismatch-gated, movement-gated windows rather than every
frame). Both the CLI scripts and periods are gone: IK fitting is now dense
(every frame) and called as a plain function from `postprocess-recording`.
What follows is the original tuning log, kept for the record; none of the
commands below still work as written.

### Commands run

One-time body-plan asset generation (see above), then both caller scripts
over the full 29-trial dataset:

```sh
cd src/poseforge/assets && uv run python ../../../../flygym/scripts/export_model_for_quickik.py
cd /home/sibwang/Projects/poseforge2

bash scripts/postprocessing/run_solve_ik.sh
bash scripts/postprocessing/run_make_videos.sh
```

QuickIK was later rebuilt from its `solver-api-redesign` branch
(`mapper=XYView()` -> `projection=Projection.new_ortho_xy()`);
`solve_ik.py` was updated to match (`uv sync --reinstall-package quickik`
to pick up the rebuild for this project's own Python 3.13 venv, since it
had initially only been rebuilt for 3.12/3.14).

Per-joint observation weights (`invkin.neuromechfly.
base_observation_weight`) and `neutral_weight` were then tuned (see
`inverse_kinematics/qa_videos/neutral_weight_sweep/README.md` for the
`neutral_weight` sweep specifically; settled on 0.2 for production). Both
caller scripts were re-run in full afterward, so every trial's output
reflects these final settings uniformly (all previous, since-superseded
output was deleted first, not left mixed in):

```sh
rm -rf bulk_data/motion_prior/2dpose_model/inverse_kinematics/ikfk
bash scripts/postprocessing/run_solve_ik.sh
bash scripts/postprocessing/run_make_videos.sh
```

29/29 trials fit successfully; fk-to-pred mismatch averaged ~0.077mm
(mean) and ~0.192mm (mean of each frame's worst leg keypoint) across all
trials, well under the `--max-mismatch` 0.3mm acceptance threshold (used
for every production run; only ever disabled, via `--max-mismatch inf`,
for the `neutral_weight` sweep's own diagnostic comparisons, never for
real output). QA clips (`inverse_kinematics/qa_videos/*.mp4`) show the
IK/FK fit (per-leg colors, matching `visualize_predictions.py`) tracking
the raw prediction (white) closely on all six legs, both in the 2D overlay
and the synthetic 3D panel.

A third refinement stage (`spotlight_ik.periods.compute_joint_excursion_deg`,
`solve_ik.py`'s `--min-joint-excursion-deg`/`--movement-window-ms`) was
added next, rejecting frames where the fly is holding still (in
joint-angle space, not body displacement: grooming/poking in place still
counts as "moving"), so periods only cover genuine leg activity. Both
caller scripts were re-run in full again, once `spotlight_orient`'s own
v13 training (co-resident on the same machine) finished, to avoid
resource contention:

```sh
rm -rf bulk_data/motion_prior/2dpose_model/inverse_kinematics/ikfk
bash scripts/postprocessing/run_solve_ik.sh
bash scripts/postprocessing/run_make_videos.sh
```

29/29 trials fit successfully; across all trials, 1183 confidence-based
periods -> 1432 mismatch-refined -> 1032 movement-refined (241,516 frames
retained overall), mean mismatch ~0.075mm. The movement filter's effect
varied a lot by trial (negligible on some, substantial on others, e.g. one
trial: 76 mismatch-refined periods down to 39), consistent with how much
genuine stillness each trial's fly happened to have within its
already-quality-gated periods.

**Bug found and fixed**: the movement re-segmentation's own `find_periods`
call used `movement_window_frames` (100ms, ~40 frames) as its closing
size. `scipy.ndimage.binary_closing`'s erosion step treats out-of-bounds
neighbors as `False`, so closing with a window longer than a period erases
the entire period regardless of its actual movement content, silently
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
bash scripts/postprocessing/run_solve_ik.sh
bash scripts/postprocessing/run_make_videos.sh
```

29/29 trials fit successfully; 1183 confidence-based -> 1432
mismatch-refined -> **1662 movement-refined** periods, 251,499 frames
retained, mean mismatch ~0.076mm. Period count and total frames both went
up relative to the pre-fix 5deg run (1032 periods / 241,516 frames)
despite the higher threshold: the erasure-bug fix recovered more
previously-discarded valid short periods than the higher threshold newly
excludes.
