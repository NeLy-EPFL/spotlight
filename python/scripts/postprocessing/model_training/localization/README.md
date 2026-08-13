# Localization model tools and caller scripts

## Tools

Reusable, general-purpose CLI tools for `TinyLocalizationModel`: a small
CNN that predicts coarse neck/thorax/abdomen position plus a binary "is
the fly upside down or sideways" flag directly on the raw (unaligned,
uncropped) camera frame at 0.25x resolution. Distinct from
`spotlight.postprocessing.pose2d`'s RepVGG-A0 model, which needs the fly
already identified, rotated, and cropped, this one runs earlier in the
pipeline, so it has to be small and cheap enough to run on the full frame
every frame. Library code (`model.py`, `dataset.py`, `augment.py`,
`flip_label.py`, `box.py`, `io_utils.py`, `constants.py`) lives in
`src/spotlight/postprocessing/localization/`.

Both targets are pseudo-labels derived from an existing, already-trained
RepVGG-A0 model's own exhaustive per-frame predictions (pose2d's
`final_predictions.h5`), not hand labels: keypoints from its predicted
`N`/`Th`/`A` nodes, and "flipped" from `flip_label.weighted_confidence` (a
proxy built from that same model's own per-keypoint confidence; see
`flip_label.py`'s docstring).

Each tool takes `--input-path`/`--output-path`-style CLI args (run
`--help` on any of them for the full list) and aborts if its output
already exists unless `--override` is passed:

- `train.py`: trains a checkpoint. `--use-global-context` (default True)
  selects `model.GlobalContextBlock`'s architecture; `--init-checkpoint`
  warm-starts from a prior checkpoint instead of random init;
  `--freeze-backbone` freezes every keypoint-relevant submodule (weights
  and BatchNorm running stats) so a new loss term (e.g. flip detection)
  can't disturb it.
- `infer.py`: runs a checkpoint over every cached frame of one trial
  (`cache_fullsize_frames.py` must have cached that trial first), saving
  dense per-frame predictions.
- `visualize_predictions.py`: renders an annotated QA video for one trial:
  the model's own combined per-keypoint heatmap (re-run live, since
  `infer.py` only saves the extracted points), the predicted keypoints,
  and the RepVGG-A0 pipeline's own 900x900 aligned-domain box
  reconstructed on the raw frame (see `box.py`), colored by predicted
  flip state.

Exporting a checkpoint to ONNX and TorchScript (fp32 and fp16 each), for
loading outside this project's own `TinyLocalizationModel` class or
outside PyTorch entirely, is a plain importable function:
`localization.export.export_checkpoint`. Mirrors `pose2d.export`'s role;
`train.py` calls it automatically whenever training stops.

## Caller scripts

Hardcoded-parameter orchestration scripts for this project's specific
dataset (no CLI args), chaining the reusable tools above over the same 29
trials pose2d's own `final_predictions.h5` covers. When a round's config
changes, write a new file rather than editing an old one, so every past
run's exact invocation stays readable straight from git history.

`cache_fullsize_frames.py` caches every trial's raw fullsize video frames
(resized to `TinyLocalizationModel`'s own 0.25x input resolution) as
JPEG, sequential decode once, rather than `TinyLocalizationDataset`
reading via `pvio`'s much slower random-access seeking on every training
run. A one-time step, needed before any `run_training_*.sh`/
`run_inference_*.sh` below.

`flip_proxy_analysis.py`/`flip_proxy_analysis_weighted.py` validate the
two candidate "is the fly flipped" proxy metrics
(`flip_label.proximal_leg_confidence`/`flip_label.weighted_confidence`)
against the real dataset: pooled/per-trial confidence histograms plus
1000 randomly sampled frames per side of the threshold, for visual
inspection.

### Training rounds

Each round has its own `run_training_v<n>.sh`; checkpoints land in
`checkpoints/v<n>/`. Keypoint-only rounds (v1-v8) progressively fixed
architecture/loss bugs found by inspecting predictions against real
frames:

- **v1**: first round with the fixed heatmap + soft-argmax keypoint head
  (an earlier GAP+FC regression head never learned anything, since global
  average pooling destroys spatial information before the head sees it).
- **v2**: flip detection first added, at `FLIP_LOSS_WEIGHT=0.1`, revisited
  much later (v9) once it turned out this value was only ever correct for
  v2's own, much larger keypoint loss scale.
- **v3**: `model.heatmap_variance` regularizer, fixing soft-argmax's
  centroid-shrinkage bias (a diffuse heatmap pulls predictions toward its
  own centroid instead of the true point).
- **v4**: replaces that with `train.gaussian_heatmap_loss`, the properly
  targeted (not just unboundedly minimized) version of the same fix.
- **v5**: cosine LR schedule, higher initial LR, wider target sigma.
- **v6**: `model.GlobalContextBlock` (`--use-global-context`), fixes a
  receptive field measured (via backprop from one heatmap pixel) at only
  ~92x92px, under a fifth of the fly's own ~460px body span. The single
  biggest accuracy jump of the whole series, 23.7px to 19.3px; v1-v5's
  checkpoints need `--no-use-global-context` to load.
- **v7**: best-checkpoint/early-stopping criterion switched from
  `val_loss` to `val_pixel_error` directly (the former didn't track it
  well); confirms 19.3px is a genuine plateau, not an early-stopping
  artifact.
- **v8**: target sigma back to 15px now that global context lets the
  model actually commit to sharper heatmaps. ~19.4px, judged good enough
  to stop keypoint-only tuning.

Flip detection then came back, this time with keypoint accuracy already
solved:

- **v9**: flip detection from a random init, `FLIP_LOSS_WEIGHT=0.1`
  (unchanged since v2), regressed keypoint error to 40.6px, since flip's
  raw BCE loss (~1.2-1.7 nats) vastly outweighs keypoint MSE
  (~0.0003-0.0006 at convergence) at that weight, and dominates the
  shared trunk's gradient.
- **v10**: warm-started from v8, `FLIP_LOSS_WEIGHT` recalibrated to
  0.0002 (matching keypoint loss's own converged scale), fixes v9's
  regression, but shrinking every parameter's gradient equally also
  starves the freshly-initialized flip head's own learning; precision
  stuck ~0.15-0.18.
- **v11**: `--freeze-backbone` (new `train.py` flag) structurally
  protects the keypoint-relevant trunk instead, letting
  `FLIP_LOSS_WEIGHT` go back to a normal 1.0, precision improved to
  ~0.42 with pixel error unchanged at 19.4px.
- **v12**: same recipe as v11, after switching `dataset.py`'s flip label
  from `proximal_leg_confidence` to `weighted_confidence` (weighs in
  distal-leg/other-node confidence too, reducing "just near the arena
  wall" false positives) and removing keypoint-set customization
  entirely (neck/thorax/abdomen is the only supported target from here
  on). Matched v11's numbers.
- **v13**: a single from-scratch run (no warm start, no frozen backbone)
  instead of v8->v11/v12's staged pipeline, for reproducibility. Reuses
  v10's small `FLIP_LOSS_WEIGHT` derivation, but at full learning rate
  and 150 epochs (early stopping disabled, so the run length is fixed) to
  give the flip head enough total gradient steps to converge without the
  staged approach. Matched v11/v12's numbers (16.6px best pixel error,
  ~0.49 precision). This is the checkpoint (`checkpoints/v13/best.pt`)
  used by `postprocess-recording`'s localization stage.

### Inference

Each training round has a matching `run_inference_v<n>.sh`: runs that
round's checkpoint over one or more full trials and renders an annotated
QA video (see `visualize_predictions.py`). v11-v13 all check the same 4
trials, for direct comparison across rounds.
