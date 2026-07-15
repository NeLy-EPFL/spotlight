import re
from pathlib import Path
from subprocess import CalledProcessError, run

from loguru import logger

_SBATCH_JOB_ID_RE = re.compile(r"Submitted batch job (\d+)")

# SLURM job states that mean the job is still pending or running. Anything else
# (COMPLETED, FAILED, CANCELLED, TIMEOUT, OUT_OF_MEMORY, NODE_FAIL, ...) means the job
# is done.
ACTIVE_STATES = {"PENDING", "RUNNING", "COMPLETING", "REQUEUED", "SUSPENDED"}


def submit_job(batch_script_path: Path) -> str:
    """Submit a SLURM batch script via `sbatch` and return its job ID."""
    result = run(
        ["sbatch", str(batch_script_path)], check=True, capture_output=True, text=True
    )
    match = _SBATCH_JOB_ID_RE.search(result.stdout)
    if not match:
        raise ValueError(
            f"Could not parse SLURM job ID from `sbatch` output: {result.stdout!r}"
        )
    return match.group(1)


def get_job_state(slurm_job_id: str) -> str | None:
    """Query a SLURM job's current state via `sacct`.

    Returns None if the state can't be determined right now (e.g. `sacct` hasn't
    picked up the job yet, or the query itself failed) -- callers should just try
    again on the next poll.
    """
    try:
        result = run(
            [
                "sacct",
                "-j",
                str(slurm_job_id),
                "--format=State",
                "--noheader",
                "--parsable2",
                "-X",
            ],
            check=True,
            capture_output=True,
            text=True,
        )
    except (CalledProcessError, FileNotFoundError) as e:
        logger.warning(f"Failed to query SLURM state for job {slurm_job_id}: {e}")
        return None
    lines = [line.strip() for line in result.stdout.splitlines() if line.strip()]
    if not lines:
        return None
    # State can be e.g. "CANCELLED by 12345"; only the first word matters.
    return lines[0].split()[0]


def cancel_job(slurm_job_id: str) -> None:
    """Cancel a SLURM job. No-op (does not raise) if the job no longer exists."""
    run(["scancel", str(slurm_job_id)], check=False)
