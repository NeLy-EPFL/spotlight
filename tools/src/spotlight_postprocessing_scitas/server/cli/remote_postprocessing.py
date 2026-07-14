import json
import logging
from os import environ
from pathlib import Path

import tyro

import spotlight_postprocessing_scitas.common.config as config
from spotlight_postprocessing_scitas.common.db import JobsDatabase, TrialStatus
from spotlight_postprocessing_scitas.common.zip import unzip_dir
from spotlight_tools.cli.postprocess_recording import (
    PostprocessingParams,
    postprocess_recording_data,
)

logger = logging.getLogger(__name__)


def _find_recording_dir(trial_dir: Path) -> Path:
    """After extraction, `trial_dir` contains exactly one subdirectory: the recording
    itself (named after the original trial directory)."""
    candidates = [path for path in trial_dir.iterdir() if path.is_dir()]
    if len(candidates) != 1:
        raise RuntimeError(
            f"Expected exactly one extracted recording directory under {trial_dir}, "
            f"found {len(candidates)}."
        )
    return candidates[0]


def remote_postprocess_recording(from_json: str) -> None:
    """Postprocess a single trial's recording on SCITAS, as dispatched by the job
    dispatcher (`spotlight_job_dispatcher`).

    Marks the trial `PROCESSING` in Firestore, then unzips the trial's compressed
    input data and runs the standard postprocessing pipeline
    (`postprocess_recording_data`) on it in place, so the client can copy the
    "metadata"/"stage_position"/"processed" subdirectories straight out of
    `<trial_dir>/<recording_name>/` to the persistent NAS server. Reports the
    resulting trial status (`OUTPUT_READY` or `FAILED`) back to Firestore.

    Args:
        from_json: Path to a JSON task manifest (written by `spotlight_job_dispatcher`)
            describing the trial to process.
    """
    logging.basicConfig(
        level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
    )

    task = json.loads(Path(from_json).read_text())
    job_id = task["job_id"]
    trial_id = task["trial_id"]
    trial_dir = Path(task["trial_dir"])
    postprocessing_params = PostprocessingParams(**task["postprocessing_params"])

    # Override num_workers to the number of cores allocated to this SLURM job
    n_cores_per_task = environ.get("SLURM_CPUS_PER_TASK")
    if n_cores_per_task is not None:
        postprocessing_params.num_workers = int(n_cores_per_task)

    db = JobsDatabase(config.FIRESTORE_SERVICE_ACCOUNT_KEY_PATH)
    db.update_trial_status(job_id, trial_id, TrialStatus.PROCESSING)

    try:
        logger.info(f"Extracting input data for trial '{trial_id}'...")
        unzip_dir(trial_dir / "input.tar.gz", trial_dir)
        recording_dir = _find_recording_dir(trial_dir)

        logger.info(f"Postprocessing trial '{trial_id}'...")
        postprocess_recording_data(recording_dir, postprocessing_params)
    except Exception as e:
        logger.error(f"Failed to postprocess trial '{trial_id}': {e}")
        db.update_trial_status(job_id, trial_id, TrialStatus.FAILED, error=str(e))
        return

    db.update_trial_status(job_id, trial_id, TrialStatus.OUTPUT_READY)
    logger.info(f"Trial '{trial_id}' postprocessing complete; output ready.")


def main() -> None:
    tyro.cli(remote_postprocess_recording)


if __name__ == "__main__":
    main()
