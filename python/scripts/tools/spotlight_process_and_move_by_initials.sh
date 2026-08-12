#!/usr/bin/env bash
set -euo pipefail

########################################
# Fixed base paths
########################################
INPUT_BASE="/mnt/spotlight-data"   # recordings live under $INPUT_BASE/<input_initials>/
DEST_BASE="/mnt/upramdya_data"     # results go to $DEST_BASE/<dest_initials>/<recording>/

########################################
# Usage
########################################
usage() {
    cat <<'EOF'
Usage: spotlight_process_and_move_by_initials.sh <input_initials> <dest_initials> [-- <postprocess-recording args...>]

Processes every recording directory under /mnt/spotlight-data/<input_initials>/ and moves
the results to /mnt/upramdya_data/<dest_initials>/<recording_name>/. Local data is removed
ONLY after the transfer is checksum-verified; otherwise it is kept.

Arguments:
  input_initials   Subfolder under /mnt/spotlight-data holding the recordings, e.g. "VS".
  dest_initials    Destination subfolder under /mnt/upramdya_data, e.g. "XY".
  -- ...           Everything after `--` is passed verbatim to `postprocess-recording`
                   (in addition to the always-set recording directory argument). If
                   omitted, no extra flags are passed.

Examples:
  spotlight_process_and_move_by_initials.sh VS XY -- --overwrite --muscle
  spotlight_process_and_move_by_initials.sh VS XY -- --overwrite --no-muscle
EOF
}

if [[ $# -lt 2 ]]; then
    usage
    exit 1
fi

INPUT_INITIALS="$1"
DEST_INITIALS="$2"
shift 2

# Collect pass-through preprocessing arguments (everything after an optional `--`).
PREPROC_ARGS=()
if [[ $# -gt 0 ]]; then
    [[ "$1" == "--" ]] && shift
    PREPROC_ARGS=("$@")
fi

INPUT_DIR="$INPUT_BASE/$INPUT_INITIALS"
if [[ ! -d "$INPUT_DIR" ]]; then
    echo "ERROR: input folder not found: $INPUT_DIR"
    exit 1
fi

########################################
# Verification helper: returns 0 iff dest is a byte-exact copy of src
########################################
verify_copy() {
    # $1 = source (append a trailing slash for directory *contents*), $2 = destination.
    # -c forces a checksum comparison, -n is a dry run; the grep keeps only itemized
    # lines where a FILE would still need transferring/updating. Empty -> identical.
    local out
    out="$(rsync -rcn --itemize-changes "$1" "$2" 2>/dev/null | grep -E '^[<>c]f' || true)"
    [[ -z "$out" ]]
}

########################################
# Discover recording directories
########################################
# A recording is any directory that contains a `behavior_images/` subfolder. Recordings
# may sit directly under $INPUT_DIR or be nested under group folders (e.g.
# <initials>/<group>/<fly_trial>/), and the depth can vary between recordings. We search
# down to a maximum depth of 4 (so behavior_images itself is at most 4 levels below
# $INPUT_DIR); this covers the nesting used in practice and avoids descending into the
# large image directories.
MAX_DEPTH=4
RECORDING_DIRS=()
while IFS= read -r -d '' BEHAVIOR_DIR; do
    RECORDING_DIRS+=("$(dirname "$BEHAVIOR_DIR")")
done < <(find "$INPUT_DIR" -maxdepth "$MAX_DEPTH" -type d -name behavior_images -print0 | sort -z)

if [[ ${#RECORDING_DIRS[@]} -eq 0 ]]; then
    echo "No recording directories (containing behavior_images/) found under $INPUT_DIR"
    exit 0
fi

echo "Found ${#RECORDING_DIRS[@]} recording(s) under $INPUT_DIR"

FAILED=()

for DIR in "${RECORDING_DIRS[@]}"; do
    [[ -d "$DIR" ]] || continue
    # Path of the recording relative to the input initials folder, e.g.
    # "Z-2419xCI55/fly000_trial000" or "beam". This is mirrored into the destination
    # so the original folder structure is preserved.
    REL_PATH="${DIR#"$INPUT_DIR"/}"

    echo "======================================"
    echo "Processing directory: $DIR"
    echo "======================================"

    ERROR_FILE="$DIR/error.txt"
    rm -f "$ERROR_FILE"

    ####################################
    # Run preprocessing (error tolerant; keep data on failure)
    ####################################
    if ! postprocess-recording \
        "$DIR" \
        "${PREPROC_ARGS[@]+"${PREPROC_ARGS[@]}"}" \
        &> "$ERROR_FILE"; then

        echo "WARNING: preprocessing failed for $DIR"
        echo "         Error saved to $ERROR_FILE; keeping local data."
        FAILED+=("$DIR (preprocess)")
        continue
    fi

    POSTPROCESSED_DIR="$DIR/postprocessed"
    if [[ ! -d "$POSTPROCESSED_DIR" ]]; then
        echo "ERROR: postprocessed folder not found in $DIR; keeping local data."
        FAILED+=("$DIR (no postprocessed)")
        continue
    fi

    ####################################
    # Prepare server destination (mirrors the input folder structure)
    ####################################
    SERVER_DIR="$DEST_BASE/$DEST_INITIALS/$REL_PATH"
    mkdir -p "$SERVER_DIR"

    TRANSFER_OK=true

    ####################################
    # Transfer + verify the postprocessed/ folder
    ####################################
    echo "Transferring postprocessed/ ..."
    if ! rsync -a --progress "$POSTPROCESSED_DIR/" "$SERVER_DIR/postprocessed/"; then
        echo "ERROR: rsync of postprocessed/ failed."
        TRANSFER_OK=false
    elif ! verify_copy "$POSTPROCESSED_DIR/" "$SERVER_DIR/postprocessed/"; then
        echo "ERROR: verification of postprocessed/ failed."
        TRANSFER_OK=false
    fi

    ####################################
    # Zip + transfer + verify the remaining subfolders
    ####################################
    if $TRANSFER_OK; then
        for SUBDIR in "$DIR"/*/; do
            SUBNAME="$(basename "$SUBDIR")"
            [[ "$SUBNAME" == "postprocessed" ]] && continue

            ZIPFILE="$SUBNAME.zip"

            echo "Zipping $SUBNAME ..."
            ( cd "$DIR" && rm -f "$ZIPFILE" && zip -r -q "$ZIPFILE" "$SUBNAME" )

            echo "Transferring $ZIPFILE ..."
            if ! rsync -a --progress "$DIR/$ZIPFILE" "$SERVER_DIR/$ZIPFILE"; then
                echo "ERROR: rsync of $ZIPFILE failed."
                TRANSFER_OK=false
            elif ! verify_copy "$DIR/$ZIPFILE" "$SERVER_DIR/$ZIPFILE"; then
                echo "ERROR: verification of $ZIPFILE failed."
                TRANSFER_OK=false
            fi

            rm -f "$DIR/$ZIPFILE"
            $TRANSFER_OK || break
        done
    fi

    ####################################
    # Delete local data ONLY if everything verified
    ####################################
    if $TRANSFER_OK; then
        echo "All transfers verified; removing local directory $DIR"
        rm -rf "$DIR"
        # Clean up now-empty group folders between the recording and $INPUT_DIR,
        # but never remove $INPUT_DIR itself. --ignore-fail-on-non-empty leaves any
        # group that still holds unmoved recordings untouched.
        PARENT="$(dirname "$DIR")"
        while [[ "$PARENT" != "$INPUT_DIR" && "$PARENT" == "$INPUT_DIR"/* ]]; do
            rmdir --ignore-fail-on-non-empty "$PARENT" 2>/dev/null || break
            [[ -d "$PARENT" ]] && break   # stop if it wasn't actually removed
            PARENT="$(dirname "$PARENT")"
        done
        echo "Completed $DIR"
    else
        echo "WARNING: transfer/verification problems for $DIR; KEEPING local data."
        FAILED+=("$DIR (transfer)")
    fi
done

########################################
# Summary
########################################
echo "======================================"
if [[ ${#FAILED[@]} -eq 0 ]]; then
    echo "All done successfully."
else
    echo "Done with issues. Kept local data for:"
    printf '  - %s\n' "${FAILED[@]}"
    exit 1
fi
