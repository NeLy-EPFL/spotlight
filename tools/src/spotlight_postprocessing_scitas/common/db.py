from pathlib import Path
from enum import Enum
from datetime import datetime

from google.cloud import firestore

import spotlight_postprocessing_scitas.common.config as config


class TrialStatus(Enum):
    WAITING_ON_INPUT_COPY = "waiting_on_input_copy"
    INPUT_COPIED = "input_copied"
    PROCESSING = "processing"
    OUTPUT_READY = "output_ready"
    OUTPUT_COPYING = "output_copying"
    COMPLETE = "complete"
    FAILED = "failed"


class DispatcherStatus(Enum):
    SUBMITTED = "submitted"
    RUNNING = "running"
    COMPLETE = "complete"
    FAILED = "failed"


class JobsDatabase:
    def __init__(self, key_path: Path):
        self.db = firestore.Client.from_service_account_json(key_path)
        self.collection = self.db.collection(config.FIREBASE_COLLECTION_ID)

    def add_job(
        self,
        job_id: str,
        trials: list[tuple[str, str]],
        postprocessing_params: dict,
        scitas_params: dict,
    ):
        """Add a new postprocessing job to Firestore.

        Args:
            job_id: Unique identifier for the job.
            trials: List of `(trial_id, display_name)` tuples for the trials that are
                part of this job.
            postprocessing_params: Postprocessing parameters, as a dict.
            scitas_params: SCITAS job submission parameters, as a dict.
        """
        data = {
            "trials": {
                trial_id: {
                    "display_name": display_name,
                    "status": TrialStatus.WAITING_ON_INPUT_COPY.value,
                    "error": None,
                }
                for trial_id, display_name in trials
            },
            "postprocessing_params": postprocessing_params,
            "scitas_params": scitas_params,
            "submission_time": datetime.now(),
            "completion_time": None,
            "dispatcher": {
                "slurm_job_id": None,
                "status": None,
                "error": None,
            },
        }
        doc_ref = self.collection.document(job_id)
        doc_ref.set(data)

    def set_dispatcher_slurm_job_id(self, job_id: str, slurm_job_id: str) -> None:
        """Record the dispatcher's own SLURM job ID once it has been submitted."""
        doc_ref = self.collection.document(job_id)
        doc_ref.update(
            {
                "dispatcher.slurm_job_id": slurm_job_id,
                "dispatcher.status": DispatcherStatus.SUBMITTED.value,
            }
        )

    def update_dispatcher_status(
        self, job_id: str, status: DispatcherStatus, error: str = ""
    ) -> None:
        doc_ref = self.collection.document(job_id)
        doc_ref.update(
            {
                "dispatcher.status": status.value,
                "dispatcher.error": error,
            }
        )

    def update_trial_status(
        self, job_id: str, trial_id: str, status: TrialStatus, error: str = ""
    ):
        doc_ref = self.collection.document(job_id)
        doc_ref.update(
            {
                f"trials.{trial_id}.status": status.value,
                f"trials.{trial_id}.error": error,
            }
        )

    def get_trial_status(self, job_id: str, trial_id: str) -> dict:
        doc_ref = self.collection.document(job_id)
        doc = doc_ref.get()
        if not doc.exists:
            raise ValueError(f"Job {job_id} does not exist in Firestore.")
        data = doc.to_dict().get("trials", {}).get(trial_id)
        if not data:
            raise ValueError(
                f"Trial {trial_id} does not exist in job {job_id} in Firestore."
            )
        return {"status": TrialStatus(data["status"]), "error": data["error"]}

    def get_job(self, job_id: str) -> dict:
        """Fetch the full stored document for a job (trials, params, timestamps)."""
        doc_ref = self.collection.document(job_id)
        doc = doc_ref.get()
        if not doc.exists:
            raise ValueError(f"Job {job_id} does not exist in Firestore.")
        return doc.to_dict()

    def list_jobs(self) -> dict[str, dict]:
        """Fetch every job's full stored document, keyed by job ID."""
        return {doc.id: doc.to_dict() for doc in self.collection.stream()}

    def get_job_status(self, job_id: str) -> dict:
        doc_ref = self.collection.document(job_id)
        doc = doc_ref.get()
        if not doc.exists:
            raise ValueError(f"Job {job_id} does not exist in Firestore.")
        data = doc.to_dict()
        trials_data = data.get("trials", {})
        trials_data = {
            k: {"status": TrialStatus(v["status"]), "error": v["error"]}
            for k, v in trials_data.items()
            if isinstance(v, dict)
        }
        completion_time = data.get("completion_time")
        return {"trials_status": trials_data, "completion_time": completion_time}

    def mark_job_complete(self, job_id: str):
        doc_ref = self.collection.document(job_id)
        doc_ref.update({"completion_time": datetime.now()})
