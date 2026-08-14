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

## SLURM scripts

`slurm/` contains SLURM script(s) for runing various tasks on the SCITAS
cluster.

## History

Earlier versions of this pipeline ran IK fitting and QA-video rendering as
two separate CLI scripts (`solve_ik.py`, `make_videos.py`), driven by
hardcoded-parameter caller scripts, over a fixed 29-trial dataset with a
"periods" concept (only fitting IK over confidence-gated, mismatch-gated,
movement-gated windows rather than every frame). All of that is gone: IK
fitting is now dense (every frame) and called as a plain function from
`postprocess-recording`.

Tuning notes kept for the record:

- Per-joint observation weights (`invkin.neuromechfly.
  base_observation_weight`) and `neutral_weight` were tuned empirically;
  `neutral_weight` settled on 0.2 for production.
- A movement-gating stage (rejecting frames where the fly holds still, in
  joint-angle space) had a bug: its `find_periods` closing size
  (`movement_window_frames`, ~40 frames) was longer than many valid
  periods, and `scipy.ndimage.binary_closing`'s erosion treats
  out-of-bounds neighbors as `False`, silently erasing entire periods
  regardless of content. Fixed by using the same small closing size the
  mismatch stage uses, decoupled from the excursion-smoothing window
  (confirmed via before/after comparison: 44 vs. 25 periods retained on
  one trial at the same threshold).
- With the fix in place, an empirical look at the excursion distribution
  showed a continuous climb with no sharp gap, so
  `--min-joint-excursion-deg` was raised from 5.0 to 10.0 to clear the
  still-subtle 5-10 degree band.
- Final full-dataset run (29/29 trials fit): 1183 confidence-based -> 1432
  mismatch-refined -> 1662 movement-refined periods, 251,499 frames
  retained, mean mismatch ~0.076mm (well under the 0.3mm acceptance
  threshold).
