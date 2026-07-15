from pathlib import Path
from enum import Enum
from datetime import datetime, timezone

from google.cloud import firestore

import spotlight_postprocessing_scitas.common.config as config


class TrialStatus(Enum):
    WAITING_ON_INPUT_COPY = "waiting_on_input_copy"
    INPUT_COPIED = "input_copied"
    PROCESSING_QUEUED = "processing_queued"
    PROCESSING = "processing"
    OUTPUT_READY = "output_ready"
    OUTPUT_COPYING = "output_copying"
    COMPLETE = "complete"
    FAILED = "failed"


# Trial statuses the client-side submission program considers final: once every
# trial is in one of these, the job as a whole is done.
TERMINAL_TRIAL_STATUSES = frozenset({TrialStatus.COMPLETE, TrialStatus.FAILED})

# Trials the dispatcher still has work to do for: waiting on input, submitting the
# SLURM job, or watching its state.
DISPATCHER_ACTIVE_TRIAL_STATUSES = frozenset(
    {
        TrialStatus.WAITING_ON_INPUT_COPY,
        TrialStatus.INPUT_COPIED,
        TrialStatus.PROCESSING_QUEUED,
        TrialStatus.PROCESSING,
    }
)


class DispatcherStatus(Enum):
    SUBMITTED = "submitted"
    RUNNING = "running"
    COMPLETE = "complete"
    FAILED = "failed"


class JobStatus(Enum):
    """Overall outcome of a job, derived from its trials' final statuses."""

    RUNNING = "running"
    COMPLETE = "complete"
    PARTIAL_FAIL = "partial_fail"
    ALL_FAIL = "all_fail"


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
                    "slurm_job_id": None,
                }
                for trial_id, display_name in trials
            },
            "postprocessing_params": postprocessing_params,
            "scitas_params": scitas_params,
            "submission_time": datetime.now(),
            "completion_time": None,
            "job_status": JobStatus.RUNNING.value,
            # Refreshed periodically by the client-side submission program for as
            # long as it is alive; the dispatcher aborts the job if this goes stale.
            "client_heartbeat": datetime.now(timezone.utc),
            "dispatcher": {
                "slurm_job_id": None,
                "status": None,
                "error": None,
            },
        }
        doc_ref = self.collection.document(job_id)
        doc_ref.set(data)

    def set_trial_slurm_job_id(
        self, job_id: str, trial_id: str, slurm_job_id: str
    ) -> None:
        """Record a trial's own SLURM job ID once its postprocessing job has been
        submitted, so its state can later be cross-checked directly against
        `sacct`."""
        doc_ref = self.collection.document(job_id)
        doc_ref.update({f"trials.{trial_id}.slurm_job_id": slurm_job_id})

    def update_client_heartbeat(self, job_id: str) -> None:
        """Refresh the client-side submission program's heartbeat timestamp for a
        job. The dispatcher treats a stale heartbeat as the client having died."""
        doc_ref = self.collection.document(job_id)
        doc_ref.update({"client_heartbeat": datetime.now(timezone.utc)})

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

    def get_job(self, job_id: str) -> dict:
        """Fetch the full stored document for a job, as-is (trial/job/dispatcher
        status fields are still the raw strings they're stored as; use `parse_job` to
        get them back as their enums)."""
        doc_ref = self.collection.document(job_id)
        doc = doc_ref.get()
        if not doc.exists:
            raise ValueError(f"Job {job_id} does not exist in Firestore.")
        return doc.to_dict()

    def get_latest_job(self) -> tuple[str, dict] | None:
        """Fetch the ID and raw document of the most recently submitted job, or None
        if there are no jobs. Cheap: reads only that single document, rather than
        every job in Firestore, by querying for the highest `submission_time`."""
        docs = list(
            self.collection.order_by(
                "submission_time", direction=firestore.Query.DESCENDING
            )
            .limit(1)
            .stream()
        )
        if not docs:
            return None
        return docs[0].id, docs[0].to_dict()

    def get_job_status(self, job_id: str) -> dict:
        """Fetch a job and parse it into a typed status view (see `parse_job`)."""
        return parse_job(self.get_job(job_id))

    def mark_job_complete(self, job_id: str, status: JobStatus) -> None:
        """Record the job's final outcome once every trial has reached a terminal
        status, or the job was aborted."""
        doc_ref = self.collection.document(job_id)
        doc_ref.update({"completion_time": datetime.now(), "job_status": status.value})

    def fail_incomplete_trials(self, job_id: str, error: str) -> None:
        """Mark every trial not yet in a terminal status as `FAILED`, e.g. because the
        job is being aborted due to an unexpected error elsewhere in the pipeline."""
        job = self.get_job(job_id)
        for trial_id, trial in job.get("trials", {}).items():
            if TrialStatus(trial["status"]) not in TERMINAL_TRIAL_STATUSES:
                self.update_trial_status(job_id, trial_id, TrialStatus.FAILED, error)


def parse_job(job: dict) -> dict:
    """Parse a job document (as returned by `JobsDatabase.get_job`/`get_latest_job`)
    into a typed view, with trial/job/dispatcher status fields as their enums instead
    of the raw strings they're stored as."""
    trials = {
        trial_id: {
            "display_name": trial["display_name"],
            "status": TrialStatus(trial["status"]),
            "error": trial.get("error"),
            "slurm_job_id": trial.get("slurm_job_id"),
        }
        for trial_id, trial in job.get("trials", {}).items()
        if isinstance(trial, dict)
    }
    dispatcher = job.get("dispatcher") or {}
    dispatcher_status = dispatcher.get("status")
    job_status = job.get("job_status")
    return {
        "trials": trials,
        "submission_time": job.get("submission_time"),
        "completion_time": job.get("completion_time"),
        "client_heartbeat": job.get("client_heartbeat"),
        "job_status": JobStatus(job_status) if job_status else None,
        "dispatcher": {
            "slurm_job_id": dispatcher.get("slurm_job_id"),
            "status": (
                DispatcherStatus(dispatcher_status) if dispatcher_status else None
            ),
            "error": dispatcher.get("error"),
        },
    }
