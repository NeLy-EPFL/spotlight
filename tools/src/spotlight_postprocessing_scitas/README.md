# Run postprocessing workflow remotely on SCITAS

Postprocessing a Spotlight recording compute-heavy and slow to run on the acquisition computer. This module offloads it to SCITAS instead: it packages up one or more recordings, ships them to SCITAS, submits a SLURM job per trial, and copies the postprocessed output back to the persistent NAS server, without requiring waiting for data transfer to complete, manually submitting SCITAS jobs, or babysiting `squeue`.

The overall workflow is:
- The user submits a _job_, which includes multiple _trials_, from the Spotlight computer. The submission script does the following:
    - Write job specifications to a database.
    - Instruct a SCITAS-side program to initiate a dispatcher via SSH.
    - Copy raw recording data to a shared storage server (this might take some time).
    - Wait for postprocessing tasks to complete on SCITAS (the Spotlight computer is mostly idle).
    - When a postprocessing task is complete, copy its output to an output location (this might take some time).
- When instructed to initiate a dispatcher, a script on SCITAS (login node) does the following:
    - Generate Slurm batch scripts to perform each postprocessing task.
    - Submit a dispatcher Slurm job that runs on a compute node (this job might be queued for some time, but it's very small is it shouldn't take too long).
- The dispatcher then does the following from a compute node (avoids running long jobs on the shared login nodes), and stays running for the entire job rather than quitting once every trial has been dispatched:
    - Wait for copying of input data into the shared storage server to complete. When input data become ready for a trial, submit a new Slurm job to run its postprocessing.
    - Cross-check every dispatched trial's Slurm job state, so jobs that get killed outright (out-of-memory, walltime exceeded, node failure, etc.) is marked failed.
    - If the submission script on the Spotlight computer stops, cancel all Slurm jobs and mark the whole job failed.
    - Quit once every trial has reached a terminal status (`output_ready` or `failed`).

Some more implementation details:
- We use _Firestore_ to track job status between the Spotlight computer and SCITAS.
- The shared storage server used to transmit data between the Spotlight computer and SCITAS is the `/export` share on SCITAS. It can be mounted on any machine on EPFL Intranet through Samba. See [SCITAS docs](https://scitas-doc.epfl.ch/user-guide/data-management/mount-scitas-smb/) for more information.
- The output location is typically a _different_ NAS meant for persistent storage (e.g., EPFL RCP NAS1, a.k.a. "the lab server").
- The formal lifecycle of a trial is: `waiting_on_input_copy` → `input_copied` → `processing_queued` → `processing` → `output_ready` → `output_copying` → `complete` (or `failed` at any point). A trial enters `processing_queued` as soon as the dispatcher submits its SLURM job, and `processing` once that SLURM job actually starts running.
- The job as a whole also has a top-level status, derived from its trials' final statuses once every trial is done (or the job is aborted): `running` → `complete` (every trial succeeded), `partial_fail` (some trials failed), or `all_fail` (every trial failed, or the job was aborted because the client-side submission program died or crashed).
- The submission script on the Spotlight computer periodically sends a heartbeat signal to Firestore. The SCITAS-side dispatcher monitors this signal to determine whether the submitting client has died.

## Setup

### Create firestore database
The Firestore database was set up following the procedure below. This procedure does not need to be repeated and is documented here only for the record.

We start by creating a Google Firebase account, a project, and a "collection". In our case, the Firestore project ID is `spotlight-processing`, and the collection ID is `firestore-processing-jobs`. Then, we create an IAM service account with read-write access to the Firestore database, and generate an access key:

```sh
PROJECT_ID="spotlight-processing"
SERVICE_ACCOUNT="spotlight-processing-firestore"

# Install the package using uv as usual
cd spotlight/tools/
uv sync
source .venv/bin/activate

# Log into Google Cloud and select project
gcloud auth login --no-launch-browser  # follow instructions in terminal
gcloud config set project $PROJECT_ID

# Create service account
gcloud iam service-accounts create $SERVICE_ACCOUNT \
    --display-name="Spotlight Processing Firestore read-write"

# Grant Firestore read+write permission to service account
gcloud projects add-iam-policy-binding $PROJECT_ID \
    --member="serviceAccount:$SERVICE_ACCOUNT@$PROJECT_ID.iam.gserviceaccount.com" \
    --role="roles/datastore.user"

# Create & download access key for service account
# KEY_PATH must be consistent with src/spotlight_postprocessing_scitas/common/config.py
KEY_PATH="$HOME/.config/spotlight-remote-processing/firestore-key.json"
mkdir -p "$(dirname $KEY_PATH)"
gcloud iam service-accounts keys create $KEY_PATH \
    --iam-account=$SERVICE_ACCOUNT@$PROJECT_ID.iam.gserviceaccount.com
chmod 600 $KEY_PATH
```

### Setup on Spotlight computer
1. Copy the Firestore key at `KEY_PATH` generated above to the same location on the Spotlight computer.
2. Verify that constants in `spotlight/tools/src/spotlight_postprocessing_scitas/common/config.py` are up-to-date.
3. Activate `uv` environment: `cd` into `spotlight/tools/` and run `source .venv/bin/activate`.
4. Mount the `/export` share on SCITAS to the local machine by running `mount-scitas-export-share --username <your Gaspar username>`. Note: this command will first require the password for running `sudo`. Then, it will require the GASPAR password. See also [SCITAS docs](https://scitas-doc.epfl.ch/user-guide/data-management/mount-scitas-smb/) for details.
5. Mount the normal NAS server as usual: see [NeLy knowledge base](https://github.com/NeLy-EPFL/knowledge-base/wiki/Computation%3A-Mounting-Network%E2%80%90Attached-Storage-%28NAS%29-servers).

### Setup on SCITAS
1. Ensure that the `spotlight/` repo is cloned at `SCITAS_SPOTLIGHT_REPO_DIR` specified in `spotlight/tools/src/spotlight_postprocessing_scitas/common/config.py`, and that its version is consistent with the copy on the Spotlight computer.
2. `cd` into `spotlight/tools/` and run `uv sync` to set up the `uv` environment there (the dispatcher activates it via `SCITAS_SPOTLIGHT_REPO_DIR` when it runs).
3. Copy the Firestore key at `KEY_PATH` generated above to the same location on SCITAS.

## Submit postprocessing jobs to SCITAS
On the local machine, run `submit-remote-postprocessing-job` with the same command-line arguments as `postprocess-recording`, plus additional arguments outlined in the help message (see output of `submit-remote-postprocessing-job --help`). In particular,

- Use `--trials` to specify an explicit list of trials to process, each as a single `<local_dir>:<persistent_nas_dir>` pair (repeat the flag once per trial). Alternatively, specify `--local-basedir` and `--remote-basedir` to automatically select recordings that exist under `local-basedir` only, and follow in-terminal instructions to confirm. 
- `--scitas-params.username` must be specified (the SCITAS jobs will be submitted from this user). Follow in-terminal instruction for authentication.

For example, to auto-diff and submit every new recording under a local directory, with muscle processing enabled:
```sh
submit-remote-postprocessing-job \
    --scitas-params.username "sibwang" \
    --local-basedir "/mnt/spotlight_data/SWC/" \
    --remote-basedir "/mnt/upramdya_data/SW/spotlight_data/" \
    --with-muscle \
    --make-visualizations
```

Or, to submit specific trials explicitly:
```sh
submit-remote-postprocessing-job \
    --scitas-params.username "sibwang" \
    --trials "G213xOGL16_260709/fly004_trial001:/mnt/upramdya_data/SW/spotlight_data/G213xOGL16_260709/fly004_trial001" \
    --with-muscle \
    --make-visualizations
```

`submit-remote-postprocessing-job` blocks in the foreground for the entire lifetime of the job: after registering it, it stays running to compress and copy each trial's input, then to wait for and copy back each trial's output once ready. Keep the terminal open (e.g. under `tmux`/`screen`) until it reports the job complete.

## Monitor job status
Run `view-job-status` (in a new terminal) to view the status of the most recent job, or `view-job-status --job-id <job-id>` to inspect a different job.

## CLIs
- **`submit-remote-postprocessing-job`** (Spotlight computer): Initiates job and tells SCITAS to start dispatcher.
- **`launch-spotlight-job-dispatcher`** (SCITAS login node, called by `submit-remote-postprocessing-job` via SSH): Creates Slurm batch scripts for individual trials based on job specs. Starts dispatcher job.
- **`spotlight-job-dispatcher`** (SCITAS compute node, started by `launch-spotlight-job-dispatcher`): Waits for input data to be ready and submits postprocessing jobs; stays running for the whole job, cross-checking dispatched trials' Slurm state and the client's heartbeat, until every trial is ready or failed.
- **`remote-postprocess-recording`** (SCITAS compute node, started by `spotlight-job-dispatcher`): Wrapper around `postprocess-recording`.
- **`view-job-status`** (anywhere): Monitor job status.