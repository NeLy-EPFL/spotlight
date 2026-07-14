import logging
import os
import random
import shutil
import string
import subprocess
import sys
from dataclasses import asdict, dataclass
from datetime import datetime, timedelta
from pathlib import Path
from tempfile import mkstemp
from time import sleep

import tyro
from tyro.conf import OmitArgPrefixes

import spotlight_postprocessing_scitas.common.config as config
from spotlight_postprocessing_scitas.common.db import JobsDatabase, TrialStatus
from spotlight_postprocessing_scitas.common.zip import zip_dir
from spotlight_tools.cli.postprocess_recording import PostprocessingParams

logger = logging.getLogger(__name__)

# Subdirectories of a postprocessed trial to copy to the persistent NAS server.
# "behavior_images" and "muscle_images" are deliberately excluded.
_TRIAL_OUTPUT_SUBDIRS = ("metadata", "stage_position", "processed")


def _generate_id(prefix: str):
    timestamp = datetime.now().strftime("%Y%m%d%H%M%S")
    suffix = "".join(random.choices(string.ascii_lowercase + string.digits, k=6))
    return f"{prefix}{timestamp}_{suffix}"


def _is_trial_finished(trial_status: TrialStatus) -> bool:
    return trial_status in (TrialStatus.COMPLETE, TrialStatus.FAILED)


def _parse_trial_pair(entry: str) -> tuple[Path, Path]:
    """Parse one `--trials` entry of the form `<local_dir>:<persistent_nas_dir>`."""
    if ":" not in entry:
        raise ValueError(
            f"Invalid --trials entry {entry!r}: expected "
            "`<local_dir>:<persistent_nas_dir>`."
        )
    local_str, nas_str = entry.split(":", 1)
    return Path(local_str).expanduser(), Path(nas_str).expanduser()


@dataclass(frozen=True)
class ScitasConfig:
    """Configuration for running postprocessing jobs on SCITAS."""

    username: str
    """Username for SSH access to SCITAS."""

    domainname: str = config.SCITAS_DOMAIN
    """Domain name for SSH access to SCITAS."""

    n_cores_per_job: int = 16
    """Number of CPU cores to request per job."""

    mem_per_job: str = "96G"
    """Number of GB of RAM to request per job."""

    walltime: str = "2:00:00"
    """Walltime to request per job, in HH:MM:SS format."""

    partition: str = "standard"
    """Partition to submit the job to on SCITAS."""


@dataclass(frozen=True)
class TrialSpec:
    """A single trial that is part of a job submission."""

    trial_id: str
    """Unique identifier generated for this trial."""

    display_name: str
    """Human-readable name for the trial."""

    trial_localpath: Path
    """Trial directory on the local machine."""

    trial_persistent_nas_path: Path
    """Destination directory for this trial's postprocessed output on the persistent
    NAS server."""


class ScitasJobSubmission:
    """Prepares, submits, and tracks a single postprocessing job on SCITAS.

    Usage:
        submission = ScitasJobSubmission()
        submission.add_trial(...)  # and/or submission.add_trials_by_diff(...)
        job_id = submission.add_job_to_db(scitas_params, postprocess_params)
        submission.create_trial_directories(job_id)
        submission.start_dispatcher_on_scitas(job_id, scitas_params)
        submission.copy_input_to_scitas_export(job_id)
        submission.wait_to_copy_output(job_id)
    """

    def __init__(self) -> None:
        self.localpath_to_trialspec: dict[Path, TrialSpec] = {}
        self.trialid_to_trialspec: dict[str, TrialSpec] = {}
        self.db = JobsDatabase(config.FIRESTORE_SERVICE_ACCOUNT_KEY_PATH)
        self.local_scitas_export_workdir = (
            config.SCITAS_EXPORT_MOUNTPOINT_LOCAL
            / config.SCITAS_EXPORT_RELATIVE_WORKDIR
        )

    def add_trial(
        self,
        display_name: str,
        trial_localpath: Path,
        trial_persistent_nas_path: Path,
    ) -> None:
        """Add a trial to the job submission.

        Args:
            display_name: Display name of the trial.
            trial_localpath: Trial directory on the local machine.
            trial_persistent_nas_path: Destination directory for this trial's
                postprocessed output on the persistent NAS server.
        """
        if not trial_localpath.exists():
            raise FileNotFoundError(f"Trial path {trial_localpath} does not exist.")
        trial_localpath = trial_localpath.resolve()
        if trial_localpath in self.localpath_to_trialspec:
            raise ValueError(f"Trial path {trial_localpath} has already been added.")
        trial_spec = TrialSpec(
            trial_id=_generate_id("trial"),
            display_name=display_name,
            trial_localpath=trial_localpath,
            trial_persistent_nas_path=trial_persistent_nas_path,
        )
        self.localpath_to_trialspec[trial_localpath] = trial_spec
        self.trialid_to_trialspec[trial_spec.trial_id] = trial_spec
        logger.info(
            f"Added trial '{display_name}' ({trial_localpath} -> "
            f"{trial_persistent_nas_path})."
        )

    def add_trials_by_diff(
        self, local_basedir: Path, remote_basedir: Path, skip_preview: bool = False
    ) -> None:
        """Add trials that exist under the local base directory but not under the remote
        base directory. The local and remote basedir are assumed to have the same
        trial hierarchy underneath them. Each new trial's persistent NAS output
        directory is set by mirroring its path relative to `local_basedir` under
        `remote_basedir`.

        Args:
            local_basedir: Base directory on the local machine.
            remote_basedir: Base directory on the persistent NAS server, mirroring
                `local_basedir`'s trial hierarchy. Also used as the destination base
                directory for postprocessed output.
            skip_preview: Whether to skip the interactive confirmation preview.
        """
        if not local_basedir.is_dir():
            raise FileNotFoundError(
                f"Local base directory {local_basedir} does not exist."
            )
        if not remote_basedir.is_dir():
            raise FileNotFoundError(
                f"Remote base directory {remote_basedir} does not exist."
            )

        local_trials = set(
            path.parent.parent.relative_to(local_basedir)
            for path in local_basedir.rglob("metadata/experiment_parameters.yaml")
        )
        remote_trials = set(
            path.parent.parent.relative_to(remote_basedir)
            for path in remote_basedir.rglob("metadata/experiment_parameters.yaml")
        )

        new_trials = []
        for trial_relpath in sorted(local_trials - remote_trials):
            params_path = (
                local_basedir / trial_relpath / "metadata/experiment_parameters.yaml"
            )
            mtime = params_path.stat().st_mtime
            new_trials.append((trial_relpath, mtime))
        new_trials.sort(key=lambda x: x[1])

        logger.info(f"Found {len(new_trials)} new trial(s) to submit for processing.")
        for trial_relpath, mtime in new_trials:
            print(f"    {trial_relpath}\t{datetime.fromtimestamp(mtime)}")

        if not skip_preview:
            fd, tempfile_str = mkstemp(suffix=".txt")
            os.close(fd)
            tempfile = Path(tempfile_str)
            with open(tempfile, "w") as f:
                for trial_relpath, mtime in new_trials:
                    f.write(f"{trial_relpath}\t{datetime.fromtimestamp(mtime)}\n")
            print(
                "\nTo accept and submit these trials, type [y] and press Enter.\n"
                f"To modify the list of trials, edit the file {tempfile} and save it. "
                "Once satisfied, type [y] and press Enter."
            )
            response = input("Do you want to continue? [y/N]: ")
            if response.lower() != "y":
                tempfile.unlink(missing_ok=True)
                logger.info("Aborting job submission.")
                sys.exit(1)
            with open(tempfile, "r") as f:
                new_trials_relpaths = set(
                    line.split("\t")[0]
                    for line in f.read().splitlines()
                    if line.strip()
                )
            tempfile.unlink(missing_ok=True)
            new_trials = [
                (path, mtime)
                for path, mtime in new_trials
                if path.as_posix() in new_trials_relpaths
            ]
            logger.info(f"Submitting {len(new_trials)} trial(s) for processing.")

        for trial_relpath, _ in new_trials:
            self.add_trial(
                trial_relpath.as_posix(),
                local_basedir / trial_relpath,
                remote_basedir / trial_relpath,
            )

    def add_job_to_db(
        self, scitas_params: ScitasConfig, postprocess_params: PostprocessingParams
    ) -> str:
        """Push the prepared job and its trials to the Firestore jobs database.

        Args:
            scitas_params: Configuration for running the job on SCITAS.
            postprocess_params: Postprocessing parameters to apply to every trial.

        Returns:
            The generated job ID.
        """
        if not self.localpath_to_trialspec:
            raise ValueError("No trials have been added to this job submission.")

        job_id = _generate_id("job")
        trials = [
            (trial_spec.trial_id, trial_spec.display_name)
            for trial_spec in self.localpath_to_trialspec.values()
        ]
        self.db.add_job(
            job_id=job_id,
            trials=trials,
            postprocessing_params=asdict(postprocess_params),
            scitas_params=asdict(scitas_params),
        )
        logger.info(
            f"Submitted job '{job_id}' with {len(trials)} trial(s) to Firestore."
        )
        return job_id

    def create_trial_directories(self, job_id: str) -> None:
        """Create the job's directory and each trial's subdirectory under the SCITAS
        export working directory.

        Must be called before `start_dispatcher_on_scitas`: `launch_dispatcher` (run
        remotely on SCITAS) also creates these directories (to write each trial's task
        manifest and SLURM script), so creating them here first avoids both sides
        racing to create the same directories over the network share.
        """
        job_dir = self.local_scitas_export_workdir / job_id
        job_dir.mkdir(parents=True, exist_ok=True)
        for trial_spec in self.localpath_to_trialspec.values():
            (job_dir / trial_spec.trial_id).mkdir(parents=True, exist_ok=True)

    def start_dispatcher_on_scitas(
        self, job_id: str, scitas_params: ScitasConfig
    ) -> None:
        """Launch dispatcher SLURM job via SSH.

        Args:
            job_id: The job ID to dispatch.
            scitas_params: Used to connect to SCITAS and to locate the SCITAS-side
                Spotlight installation.
        """
        ssh_target = f"{scitas_params.username}@{scitas_params.domainname}"
        remote_command = (
            f"source {config.SCITAS_SPOTLIGHT_REPO_DIR}/tools/.venv/bin/activate && "
            f"launch-spotlight-job-dispatcher --job-id {job_id}"
        )
        logger.info(f"Launching job dispatcher on SCITAS for job '{job_id}'...")
        print("If prompted, enter SCITAS password for SSH access below:")
        subprocess.run(["ssh", ssh_target, remote_command], check=True)

    def copy_input_to_scitas_export(self, job_id: str, n_workers: int = -1) -> None:
        """Compress each trial's local data and copy it to the SCITAS export working
        directory, updating each trial's status in Firestore as it completes.

        A trial that fails to copy is marked `TrialStatus.FAILED` in Firestore (with
        the error message attached) and does not block the remaining trials.

        Args:
            job_id: The job ID whose trials should be copied.
            n_workers: Number of parallel workers to use for compression (see
                `zip_dir`). -1 uses all available cores, -2 all but one, etc.
        """
        # `create_trial_directories` (called before this, and before
        # `start_dispatcher_on_scitas`) has already created the job and trial
        # directories.
        job_dir = self.local_scitas_export_workdir / job_id
        for trial_spec in self.localpath_to_trialspec.values():
            trial_dir = job_dir / trial_spec.trial_id
            zipped_path = trial_dir / "input.tar.gz"
            logger.info(
                f"Compressing and copying trial '{trial_spec.display_name}' to "
                f"{zipped_path}..."
            )
            try:
                zip_dir(trial_spec.trial_localpath, zipped_path, n_workers)
            except Exception as e:
                logger.error(
                    f"Failed to copy trial '{trial_spec.display_name}' to the "
                    f"SCITAS export share: {e}"
                )
                self.db.update_trial_status(
                    job_id, trial_spec.trial_id, TrialStatus.FAILED, error=str(e)
                )
                continue
            self.db.update_trial_status(
                job_id, trial_spec.trial_id, TrialStatus.INPUT_COPIED
            )
            logger.info(f"Trial '{trial_spec.display_name}' input copied.")

    def wait_to_copy_output(self, job_id: str, check_interval: int = 60) -> None:
        """Poll Firestore until every trial in the job has finished processing,
        copying each trial's output to its persistent NAS destination as soon as it
        becomes ready.

        Args:
            job_id: The job ID to wait on.
            check_interval: How often, in seconds, to poll Firestore for status
                updates.
        """
        print(
            "Waiting for trials to finish processing and copy output to persistent "
            "NAS. Do not close this terminal until the job is complete."
        )

        next_check_time = datetime.now()
        already_logged_failures: set[str] = set()

        while True:
            next_check_time += timedelta(seconds=check_interval)

            job_status = self.db.get_job_status(job_id)
            trials_status = job_status["trials_status"]

            if all(
                _is_trial_finished(trial_status["status"])
                for trial_status in trials_status.values()
            ):
                logger.info(f"All trials for job '{job_id}' are finished.")
                self.db.mark_job_complete(job_id)
                break

            for trial_id, trial_status in trials_status.items():
                trial_spec = self.trialid_to_trialspec.get(trial_id)
                display_name = trial_spec.display_name if trial_spec else trial_id

                if trial_status["status"] == TrialStatus.OUTPUT_READY:
                    logger.info(f"Trial '{display_name}' output is ready for copying.")
                    self.db.update_trial_status(
                        job_id, trial_id, TrialStatus.OUTPUT_COPYING
                    )
                    trial_dir = self.local_scitas_export_workdir / job_id / trial_id
                    recording_dir = trial_dir / trial_spec.trial_localpath.name
                    trial_persistent_nas_dir = trial_spec.trial_persistent_nas_path
                    try:
                        trial_persistent_nas_dir.mkdir(parents=True, exist_ok=True)
                        for name in _TRIAL_OUTPUT_SUBDIRS:
                            src = recording_dir / name
                            if src.exists():
                                shutil.copytree(
                                    src,
                                    trial_persistent_nas_dir / name,
                                    dirs_exist_ok=True,
                                )
                    except Exception as e:
                        logger.error(
                            f"Failed to copy output for trial '{display_name}' to "
                            f"{trial_persistent_nas_dir}: {e}"
                        )
                        self.db.update_trial_status(
                            job_id, trial_id, TrialStatus.FAILED, error=str(e)
                        )
                        continue
                    self.db.update_trial_status(job_id, trial_id, TrialStatus.COMPLETE)
                    logger.info(
                        f"Trial '{display_name}' output copied to "
                        f"{trial_persistent_nas_dir}."
                    )
                    try:
                        shutil.rmtree(trial_dir)
                        logger.info(
                            f"Removed temporary SCITAS export data for trial "
                            f"'{display_name}' ({trial_dir})."
                        )
                    except Exception as e:
                        logger.warning(
                            f"Failed to remove temporary SCITAS export data for "
                            f"trial '{display_name}' ({trial_dir}): {e}"
                        )
                elif (
                    trial_status["status"] == TrialStatus.FAILED
                    and trial_id not in already_logged_failures
                ):
                    already_logged_failures.add(trial_id)
                    logger.warning(
                        f"Trial '{display_name}' failed: "
                        f"{trial_status['error'] or 'unknown error'}"
                    )

            time_to_sleep = max(0, (next_check_time - datetime.now()).total_seconds())
            sleep(time_to_sleep)

        logger.info(f"Job '{job_id}' is complete.")


def submit_remote_postprocessing_job(
    scitas_params: ScitasConfig,
    trials: list[str] | None = None,
    local_basedir: Path | None = None,
    remote_basedir: Path | None = None,
    skip_preview: bool = False,
    zip_workers: int = -1,
    postprocess_params: OmitArgPrefixes[PostprocessingParams] = PostprocessingParams(),
) -> None:
    """Submit a postprocessing job for one or more trials to run remotely on SCITAS,
    then block until it completes, copying each trial's output to the persistent NAS
    server as it becomes ready.

    Trials to submit can be specified in one of two ways:
    1. Explicitly, via `trials`.
    2. By auto-diffing `local_basedir` against `remote_basedir`: every trial found
       under `local_basedir` but not yet under `remote_basedir` is submitted, with its
       persistent NAS destination set by mirroring its path (relative to
       `local_basedir`) under `remote_basedir`.

    Args:
        scitas_params: Configuration for running the job on SCITAS.
        trials: Trials to submit explicitly, each given as a single
            `<local_dir>:<persistent_nas_dir>` pair (repeat the flag once per trial,
            e.g. `--trials a:b c:d`). Neither path may itself contain a literal `:`.
            Must not be provided alongside `local_basedir`/`remote_basedir`.
        local_basedir: Base directory on the local machine to auto-diff for new
            trials. Must be provided together with `remote_basedir`, and not
            alongside `trials`.
        remote_basedir: Base directory on the persistent NAS server to auto-diff
            against and to mirror new trials' output under (see `local_basedir`).
        skip_preview: Whether to skip the interactive confirmation preview when
            auto-diffing trials.
        zip_workers: Number of parallel workers to use for compressing trial data. -1
            uses all available cores, -2 all but one, etc.
        postprocess_params: Postprocessing parameters to apply to every trial.
    """
    logging.basicConfig(
        level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
    )

    if local_basedir is not None:
        local_basedir = local_basedir.expanduser()
    if remote_basedir is not None:
        remote_basedir = remote_basedir.expanduser()

    submission = ScitasJobSubmission()

    # Add trials to submission
    explicit_trials_given = trials is not None
    autodiff_roots_given = local_basedir is not None and remote_basedir is not None
    if explicit_trials_given and not autodiff_roots_given:
        for entry in trials:
            trial_dir, trial_nas_dir = _parse_trial_pair(entry)
            submission.add_trial(trial_dir.name, trial_dir, trial_nas_dir)
    elif autodiff_roots_given and not explicit_trials_given:
        submission.add_trials_by_diff(local_basedir, remote_basedir, skip_preview)
    else:
        raise ValueError(
            "Either `trials` or the combination of `local_basedir` and "
            "`remote_basedir` must be provided, but not both."
        )

    # Add job to Firestore
    job_id = submission.add_job_to_db(scitas_params, postprocess_params)

    # Create the job's and each trial's directory on the SCITAS export share before
    # launching the dispatcher, so the two never race to create them.
    submission.create_trial_directories(job_id)

    # Launch dispatcher job on SCITAS to monitor the job and dispatch postprocessing
    # jobs
    submission.start_dispatcher_on_scitas(job_id, scitas_params)

    # Copy input data to SCITAS export workdir
    submission.copy_input_to_scitas_export(job_id, n_workers=zip_workers)

    # Wait for output to be ready and copy it to persistent NAS
    submission.wait_to_copy_output(job_id)


def main() -> None:
    tyro.cli(submit_remote_postprocessing_job)


if __name__ == "__main__":
    main()
