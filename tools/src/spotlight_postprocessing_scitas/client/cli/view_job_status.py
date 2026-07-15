import tyro
from tabulate import tabulate

import spotlight_postprocessing_scitas.common.config as config
from spotlight_postprocessing_scitas.common.db import (
    DispatcherStatus,
    JobsDatabase,
    TrialStatus,
    parse_job,
)

_TRIAL_TABLE_HEADERS = ["Trial", "Display Name", "Status", "Error"]


def _trial_row(trial_id: str, trial: dict) -> list[str]:
    error = trial["error"] if trial["status"] == TrialStatus.FAILED else None
    return [trial_id, trial["display_name"], trial["status"].name, error or ""]


def _format_dispatcher(dispatcher: dict) -> str | None:
    if not dispatcher["slurm_job_id"]:
        return None
    status_name = dispatcher["status"].name if dispatcher["status"] else "UNKNOWN"
    line = f"    dispatcher: SLURM job {dispatcher['slurm_job_id']}  {status_name}"
    if dispatcher["status"] == DispatcherStatus.FAILED and dispatcher["error"]:
        line += f"  ({dispatcher['error']})"
    return line


def _format_job(job_id: str, job: dict) -> str:
    submission_time = job["submission_time"]
    submitted_str = (
        f"{submission_time:%Y-%m-%d %H:%M:%S}" if submission_time else "unknown time"
    )
    job_status = job["job_status"].name if job["job_status"] else "UNKNOWN"
    completion_time = job["completion_time"]
    if completion_time:
        job_status += f" at {completion_time:%Y-%m-%d %H:%M:%S}"
    header = f"Job {job_id}  (submitted {submitted_str}, {job_status})"

    dispatcher_line = _format_dispatcher(job["dispatcher"])
    trials = job["trials"]
    rows = [_trial_row(trial_id, trials[trial_id]) for trial_id in sorted(trials)]

    lines = [header]
    if dispatcher_line:
        lines.append(dispatcher_line)
    if rows:
        table = tabulate(rows, headers=_TRIAL_TABLE_HEADERS)
        lines.append("\n".join(f"    {line}" for line in table.splitlines()))
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
        print(_format_job(jid, parse_job(jobs[jid])))
        print()


def main() -> None:
    tyro.cli(view_job_status)


if __name__ == "__main__":
    main()
