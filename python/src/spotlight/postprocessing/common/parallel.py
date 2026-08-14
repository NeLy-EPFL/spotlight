"""Shared CPU worker-count resolution for CLI knobs."""

from os import cpu_count, environ
from typing import Literal


def resolve_num_workers(num_workers: int | Literal["auto"]) -> int:
    """Resolve a `--*.num-workers`-style CLI value.

    If `num_workers` is a positive integer, it is returned as-is.
    If `num_workers` is -1, all available CPU cores are used; if -2, all but
    one, etc., matching joblib's convention. 0 is invalid and raises.
    If `num_workers` is "auto", `$SLURM_CPUS_PER_TASK` is used if set (i.e.
    when running as a Slurm job on a cluster), else -1 (all available cores).
    """
    if num_workers == 0:
        raise ValueError("'num_workers == 0' is invalid")

    if num_workers == "auto":
        if slurm_tasks := environ.get("SLURM_CPUS_PER_TASK"):
            return int(slurm_tasks)
        else:
            num_workers = -1

    if num_workers < 0:
        num_workers = max(cpu_count() + 1 + num_workers, 1)

    return num_workers
