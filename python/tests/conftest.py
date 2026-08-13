"""Shared pytest fixtures."""

import shutil
from pathlib import Path

import pytest

DATA_DIR = Path(__file__).parent / "data"


@pytest.fixture
def mini_recording_dir(tmp_path: Path) -> Path:
    """A copy of `tests/data/mini_recording/` under `tmp_path`.

    A real recording directory trimmed to 10 raw behavior files (30 real
    frames, frames 0-27) plus its metadata and stage position log: enough
    for the postprocessing pipeline to run end to end, small enough to be
    version-tracked. Copied per-test so a test's own `postprocessed/`
    output never touches the checked-in fixture.
    """
    dest = tmp_path / "mini_recording"
    shutil.copytree(DATA_DIR / "mini_recording", dest)
    return dest
