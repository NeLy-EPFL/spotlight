import tyro

import spotlight_postprocessing_scitas.common.config as config
from spotlight_postprocessing_scitas.common.db import (
    DispatcherStatus,
    JobsDatabase,
    TrialStatus,
)


def _format_trial(trial_id: str, trial: dict) -> str:
    status = TrialStatus(trial["status"])
    line = f"    {trial_id}  {trial['display_name']:<40} {status.name}"
    if status == TrialStatus.FAILED and trial.get("error"):
        line += f"  ({trial['error']})"
    return line


def _format_dispatcher(job: dict) -> str | None:
    dispatcher = job.get("dispatcher")
    if not dispatcher or not dispatcher.get("slurm_job_id"):
        return None
    status_name = (
        DispatcherStatus(dispatcher["status"]).name if dispatcher.get("status") else "UNKNOWN"
    )
    line = f"    dispatcher: SLURM job {dispatcher['slurm_job_id']}  {status_name}"
    if dispatcher.get("status") == DispatcherStatus.FAILED.value and dispatcher.get("error"):
        line += f"  ({dispatcher['error']})"
    return line


def _format_job(job_id: str, job: dict) -> str:
    submission_time = job["submission_time"]
    completion_time = job.get("completion_time")
    status = (
        f"completed {completion_time:%Y-%m-%d %H:%M:%S}"
        if completion_time
        else "incomplete"
    )
    header = f"Job {job_id}  (submitted {submission_time:%Y-%m-%d %H:%M:%S}, {status})"

    dispatcher_line = _format_dispatcher(job)
    trials = job.get("trials", {})
    trial_lines = [_format_trial(trial_id, trials[trial_id]) for trial_id in sorted(trials)]
    lines = [header]
    if dispatcher_line:
        lines.append(dispatcher_line)
    lines.extend(trial_lines)
    return "\n".join(lines)


def view_job_status(job_id: str | None = None) -> None:
    """Print the status of postprocessing job(s) tracked in Firestore.

    Jobs and trials are always printed in a stable, sorted order (by ID, which sorts
    chronologically), so repeatedly re-running this (e.g. via `watch`) only highlights
    what actually changed.

    Args:
        job_id: If given, print only this job's status. Otherwise, print the status of
            every job that has not yet completed.
    """
    db = JobsDatabase(config.FIRESTORE_SERVICE_ACCOUNT_KEY_PATH)

    if job_id is not None:
        jobs = {job_id: db.get_job(job_id)}
    else:
        jobs = {
            jid: job
            for jid, job in db.list_jobs().items()
            if job.get("completion_time") is None
        }

    if not jobs:
        print("No incomplete jobs.")
        return

    for jid in sorted(jobs):
        print(_format_job(jid, jobs[jid]))
        print()


def main() -> None:
    tyro.cli(view_job_status)


if __name__ == "__main__":
    main()
