"""Shared CPU worker-count resolution for CLI knobs whose default is
`"auto"`.
"""

import os
from typing import Literal


def resolve_num_workers(num_workers: int | Literal["auto"]) -> int:
    """Resolve a `--*.num-workers`-style CLI value.

    `"auto"` uses `$SLURM_CPUS_PER_TASK` if set (a Slurm job's own share of
    a node, which can be smaller than the node's full core count), else -1
    (joblib's own "all cores" auto-detection). Any other value is used as
    given, matching joblib's `n_jobs` convention directly.
    """
    if num_workers != "auto":
        return int(num_workers)
    slurm_cpus = os.environ.get("SLURM_CPUS_PER_TASK")
    return int(slurm_cpus) if slurm_cpus else -1
