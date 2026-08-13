# `spotlight_pose2d` tools and caller scripts

Migrated from poseforge2's `spotlight_pose2d` (library + tools + caller
scripts), namespaced under `spotlight_tools` instead of
`poseforge.motion_prior`.

## Tools

Reusable, general-purpose CLI tools for the Spotlight 2D pose pipeline: from
raw SLEAP predictions, through a human-in-the-loop correction/retraining
loop, to a trained `RepVGGPoseModel` checkpoint. Each takes
`--input-path`/`--output-path` and other CLI args (run `--help` on any of
them for the full list) and aborts if its output already exists unless
`--override` is passed. Library code (`model.py`, `dataset.py`,
`augment.py`, `io_utils.py`, `geometry.py`) lives in
`src/spotlight_postprocessing/spotlight_pose2d/`.

### Legacy SLEAP env

`slp_convert_legacy.py --current-to-legacy` and `slp_export_onnx.py` need
an isolated `sleap==1.4.1` conda env (the main `.venv` uses `sleap-nn`
instead), created once via:

```sh
conda create -y -n sleap -c conda-forge -c nvidia -c sleap/label/dev -c sleap -c anaconda sleap=1.4.1
```

### Single-trial pipeline

Each tool takes one trial in, one trial out:

1. `slp_run_inference.py`: runs `sleap-track` (one model dir for
   single-instance, two -- centroid then centered-instance -- for top-down).
2. `slp_apply_alignment.py --align`/`--unalign`: maps points between the
   raw fullsize video's domain and the aligned (cropped) video's, via the
   trial's own per-frame affine transforms.
3. `slp_promote_by_confidence.py`: promotes confident, non-redundant
   predicted instances to user labels.
4. `slp_convert_h5.py --slp-to-h5`/`--h5-to-slp`: converts between a `.slp`
   and this project's dense, one-row-per-frame `.h5` schema. `--h5-to-slp
   --append-to existing.slp --priority mine|theirs` incrementally combines
   trials into one multi-video `.slp`.

Once a trial has a dense pose `.h5`, the `RepVGGPoseModel` tools take over:
`cache_video_frames.py` caches its aligned video's frames as JPEG (sequential
decode is ~80x faster than the per-frame random seeking training would
otherwise do), then `train.py` trains on it, and `infer.py`/
`predict_unlabeled_frames.py`/`sample_predictions_after_training.py` (the
last runs the second over every trial under a directory, merging the
results) run a trained checkpoint back over it. `visualize_predictions.py`
renders an annotated QA video for one trial: the model's own per-frame
heatmap output (all keypoints pooled into one channel) plus the predicted
skeleton, drawn with cv2 and encoded via `pvio`.

### Other tools

`slp_convert_legacy.py`, `slp_export_onnx.py`, `slp_merge.py` (union
several single-video `.slp` files with disjoint videos into one),
`slp_split_by_video.py` (the inverse: split a merged `.slp` back into one
`.slp` per trial), `slp_check_quality.py` (read-only QA over a merged
`.slp`'s labeled frames: keypoint visibility, per-edge segment-length
outliers, left/right separation), `extract_metadata_from_initial_slp.py`
(the real skeleton plus which frames are genuinely hand-labeled, into one
JSON other tools take as `--skeleton-json-path`/`--metadata-json-path`),
and `split_train_val.py` (seeded train/val split that always keeps a
well-hand-labeled trial in train).

Exporting a checkpoint (RepVGG, to ONNX -- on hold until real-time inference
moves to C++ -- and TorchScript, for loading in a different codebase
without it; both at fp32 and fp16) is a plain importable function now, not
a CLI tool: `pose2d.export.export_checkpoint`, in
`src/spotlight_postprocessing/pose2d/`. `train.py` calls it automatically
whenever training stops -- there's no separate standalone invocation for
it anymore (run it ad hoc via `tyro.cli` if a checkpoint ever needs
re-exporting outside of training).

## Caller scripts

Hardcoded-parameter orchestration scripts for this project's specific
dataset -- no CLI args, chaining the reusable tools above over the actual
data under `bulk_data/motion_prior/2dpose_model/`. When a round's config
changes, write a new file rather than editing an old one, so every past
run's exact invocation stays readable straight from git history.

**Note on paths below**: preserved as originally written in poseforge2 --
`bulk_data/motion_prior/2dpose_model/...` is poseforge2's own layout, not
spotlight-tools'. `checkpoints/iter2b/best.pt` is the production checkpoint
that was actually migrated here (see `run_inference_final.sh`, which
predicts every frame of every trial using it) -- it's what
`postprocess_recording`'s pose2d stage uses.

### Bootstrapping the dataset (one-time, already done)

`run_test_round.sh` validated the pipeline end to end on the original
hand-labeled batch (`labels/first_round/`) and produced `labels/
metadata.json`. Then, for the 29-trial LM-ported dataset:

`copy_lm_predictions_locally.py` copies each trial's existing LM prediction
from the NAS locally, `port_lm_pred_to_aligned.sh` runs each through
legacy-to-current conversion, alignment, and confidence promotion (writing
throwaway intermediates to `labels/pipeline_intermediates/`, and the final
`*_slp16_aligned_promoted.slp` to `labels/ported_lm_predictions/`),
`port_preexisting_handlabels.py` substitutes the original hand labels back
in wherever they exist, `convert_promoted_to_h5.sh` converts each trial's
final `.slp` to `*_pose.h5`, and `cache_all_trial_frames.py` caches every
trial's video frames as JPEG. `labels/train_val_split_29trials.json` is
that dataset's train/val split.

`sync_spotlight_recordings.sh` mirrors the raw Spotlight recordings
themselves (not this pipeline's own derived data) from the NAS into
`bulk_data/motion_prior/spotlight_recordings/`, excluding the bulkiest,
least pose-relevant per-trial data (behavior/muscle images).

### Training loop

Each round has its own `run_training_iter<round>.sh` (hardcoded
hyperparameters as shell variables) and `run_inference_iter<round>.sh`
(spot-checks the resulting checkpoint over a random per-trial sample,
merging the predictions into one `.slp` for GUI review). Checkpoints land
in `checkpoints/<round>/`; each round's own data and sample predictions
land in `labels/rounds/<round>/`.

- **iter0c**: first stable checkpoint, trained from ImageNet weights on
  the full LM-ported + confidence-promoted dataset.
- **iter1a**, **iter2a**: successive rounds fine-tuned on hand-corrected
  labels only (no pseudo-labels), warm-started from the previous round's
  checkpoint. Each rebuilds its own training data from that round's own
  corrected `.slp` (still under GUI review between rounds), and each
  inference script chains in every prior round's `--corrected-slp-paths`
  so a new checkpoint never silently overwrites an earlier correction.
- **iter2b**: a sibling of iter2a, not a continuation from it -- same
  warm start (`iter1a/best.pt`), same training data, but
  `--heatmap-sigma 1.0` instead of iter2a's 2.0 (see `dataset.HEATMAP_SIGMA`),
  to isolate that one hyperparameter's effect. This is the checkpoint
  `run_inference_final.sh` actually uses (and the one migrated here), not
  iter2a.

`run_training_iter2a.sh`/`run_training_iter2b.sh` finish with a QA pass:
exhaustive (every frame) inference on one small trial with the resulting
`best.pt`, then a `visualize_predictions.py` summary video, landing
alongside that round's own training data in `labels/rounds/<round>/` as
`<trial>_qa_full.{h5,slp}`/`<trial>_qa_full_viz.mp4` -- a fast visual
sanity check, and a same-trial comparison point across rounds.

`run_inference_final.sh` is the exhaustive, all-trial, all-frame
production pass (as opposed to every other inference script's per-round
2000-frame spot check), using the latest checkpoint (`iter2b/best.pt`).
Output goes to `labels/final_predictions/`.

`iter0` and `iter0b` were quick hyperparameter-testing rounds (tuning batch
size, learning rate, and the backbone/head LR split) and are not part of
this migration; `iter0c` is the first checkpoint trained with settings
that stuck.

IK/FlyGym replay (`quickik_solve.py`, `flygym_replay.py`) are not part of
this migration either -- they're being rewritten separately (this refers
to poseforge2's own earlier internal migration note, predating and
unrelated to `spotlight_ik`'s later move into spotlight-tools).
