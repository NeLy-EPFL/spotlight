#!/usr/bin/env bash
#
# Copy postprocessed folders (if they exist, i.e. if postprocessing was
# successful) back to the lab NAS server.

# Mount the lab server NAS first and run this script on a *login node*:
# 
#     # Explicitly use a specific login node for easier monitoring (otherwise
#     # the cluster might put you on either kuma1 or kuma2 for load balancing)
#     ssh <username>@kuma1.hpc.epfl.ch
#
#     # Start tmux session
#     tmux new -s copy-postprocessing-output
#
#     # While in tmux session, mount lab server
#     # See https://scitas-doc.epfl.ch/user-guide/data-management/mount-nas/
#     # Prerequisite the following line is set in ~/.bashrc:
#     # export UPRAMDYA_DATA="smb://intranet;sibwang@sv-nas1.rcp.epfl.ch/upramdya/data"
#     dbus-run-session -- bash
#     gio mount $UPRAMDYA_DATA
#
#     # Start copying
#     cd path/to/folder/containing/this/script/and/manifest
#     bash copy_postprocessed_folder_to_nas.sh
#
#     # Detach from tmux session
#     # Press Ctrl+b, then d (not Ctrl+d, just d)
#     # To verify that the tmux session is still running:
#     tmux ls
#     # To attach back to this session to inspect logs:
#     tmux attach -d -t copy-postprocessing-output

set -euo pipefail

# Constants
MANIFEST_FILE="manifest.txt"
LABSERVER_MOUNTDIR="/run/user/$UID/gvfs/smb-share:domain=intranet,server=sv-nas1.rcp.epfl.ch,share=upramdya,user=$USER/"
LABSERVER_BASEDIR="data/VAS/poseforge_paper_data"
SCITAS_BASEDIR="/work/upramdya/sibo/spotlight_data"

# Helper to get path of a file/folder relative to another absolute path.
# Exit 1 if path is not strictly under anchor_path.
relative_path_to() {
    local path anchor_path
    path=$(realpath -m -- "$1") || return 1
    anchor_path=$(realpath -m -- "$2") || return 1
    if [[ "$anchor_path" == "/" ]]; then
        if [[ "$path" == "/" ]]; then return 1; fi
        printf '%s\n' "${path#/}"
    elif [[ "$path" == "$anchor_path"/* ]]; then
        printf '%s\n' "${path#"$anchor_path"/}"
    else
        return 1
    fi
}

if [[ ! -d "$LABSERVER_MOUNTDIR" ]]; then
    echo "ERROR: NAS mount not found at $LABSERVER_MOUNTDIR (is the share mounted?)" >&2
    exit 1
fi

if [[ ! -f "$MANIFEST_FILE" ]]; then
    echo "ERROR: manifest file $MANIFEST_FILE not found" >&2
    exit 1
fi

# First pass: check which folders to copy
ok_trialdirs_rel=()
has_error=0
total_trials=0
while IFS= read -r trialdir || [[ -n "$trialdir" ]]; do
    [[ -z "$trialdir" ]] && continue   # skip blank lines
    total_trials=$((total_trials + 1))
    if trialdir_rel=$(relative_path_to "$trialdir" "$SCITAS_BASEDIR"); then
        if [[ -d "$trialdir/postprocessed" ]]; then
            ok_trialdirs_rel+=("$trialdir_rel")
        else
            echo "WARNING: postprocessed folder not found for $trialdir, skipping"
        fi
    else
        echo "ERROR: $trialdir is not under SCITAS_BASEDIR $SCITAS_BASEDIR, quitting"
        has_error=1
    fi
done < "$MANIFEST_FILE"

if [[ "$has_error" -eq 1 ]]; then
    exit 1
fi

n_ok=${#ok_trialdirs_rel[@]}
echo "[$(date)] $n_ok/$total_trials trials have a valid postprocessed folder to copy"

# Copy in a loop
i=0
for trialdir_rel in "${ok_trialdirs_rel[@]}"; do
    i=$((i + 1))
    src="$SCITAS_BASEDIR/$trialdir_rel/postprocessed"
    tgt_dir="$LABSERVER_MOUNTDIR/$LABSERVER_BASEDIR/$trialdir_rel"
    mkdir -p "$tgt_dir"
    echo "[$(date)] Copying $trialdir_rel ($i/$n_ok)"
    cp -r "$src" "$tgt_dir"
done

echo "[$(date)] All trials copied."
