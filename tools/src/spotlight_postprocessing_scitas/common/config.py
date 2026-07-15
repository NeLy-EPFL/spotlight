from pathlib import Path

# SCITAS
SCITAS_DOMAIN = "jed.hpc.epfl.ch"
SCITAS_SPOTLIGHT_REPO_DIR = "$HOME/spotlight"
# Resrouce requirement for the dipatcher job
SCITAS_DISPATCHER_WALLTIME = "48:00:00"
SCITAS_DISPATCHER_MEMORY = "1G"
# The /export/ACCOUNT_NAME share on SCITAS can be mounted via Samba on machines on the
# EPFL intranet. We will use this to copy data back and forth.
SCITAS_EXPORT_MOUNTPOINT_SCITAS = Path("/export/upramdya")
SCITAS_EXPORT_MOUNTPOINT_LOCAL = Path("/mnt/scitas_export_upramdya")
SCITAS_EXPORT_RELATIVE_WORKDIR = "spotlight_remote_processing"
SCITAS_EXPORT_SAMBA_ADDR = "//samba.hpc.epfl.ch/upramdya"

# How often the client-side submission program refreshes its "still alive" heartbeat,
# and how long the dispatcher waits without a heartbeat before concluding the client
# has died and aborting the whole job.
CLIENT_HEARTBEAT_INTERVAL = 30
CLIENT_HEARTBEAT_TIMEOUT = 120

# Firestore
FIRESTORE_SERVICE_ACCOUNT_KEY_PATH = (
    Path.home() / ".config/spotlight-remote-processing/firestore-key.json"
)
FIREBASE_COLLECTION_ID = "firestore-processing-jobs"
