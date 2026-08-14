"""Unit tests for `common.parallel`."""

import pytest

from spotlight.postprocessing.common import resolve_num_workers
from spotlight.postprocessing.common.parallel import cpu_count


def test_positive_int_passes_through():
    assert resolve_num_workers(4) == 4


def test_negative_one_uses_all_cores():
    assert resolve_num_workers(-1) == cpu_count()


def test_negative_two_leaves_one_core_free():
    assert resolve_num_workers(-2) == max(cpu_count() - 1, 1)


def test_zero_is_invalid():
    with pytest.raises(ValueError):
        resolve_num_workers(0)


def test_auto_without_slurm_uses_all_cores(monkeypatch):
    monkeypatch.delenv("SLURM_CPUS_PER_TASK", raising=False)
    assert resolve_num_workers("auto") == cpu_count()


def test_auto_with_slurm_uses_slurm_share(monkeypatch):
    monkeypatch.setenv("SLURM_CPUS_PER_TASK", "3")
    assert resolve_num_workers("auto") == 3
