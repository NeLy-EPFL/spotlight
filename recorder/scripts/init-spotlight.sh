#!/usr/bin/env zsh
# NOTE: This shebang only matters if someone runs this file directly
# (which they shouldn't — see below). It is otherwise IGNORED when this
# file is sourced; the current shell interprets it then.
#
# Fun setup script to configure a shell for Spotlight experiments:
#
# It does f̶i̶v̶e̶ three things:
#   1. Greet the user with cowsay using a preset animal. This is a simple way to
#      ensure you're running the correct executable (i.e., a stable version
#      instead of a development version that might have bugs or different
#      behaviors). Tell you intern "If it's not a Stegosaurus that greets you,
#      you're using the wrong version.
#   2. Prompt the user to select a profile directory from the list of available
#      profiles, or enter a custom path.
#   3. Prompt the user to select an arena directory from the list of available
#      arenas, or enter a custom path.
#
# This script then sets up aliases so that when the user runs `align-cameras`,
# `run-arena-registration-scan`, and `run-spotlight`, the commands automatically
# invoke `align-cameras -p $profile_dir -a $arena_dir`, etc.
#
# For this to work, the developer should do the following manually:
#   1. Give this script executable permission and add it to PATH
#   2. Change the hardcoded RECORDER_BIN_PATH, PYTHON_VENV_PATH, and COWFILE
#      variables as needed (the COWFILE defines the animal).
#
# IMPORTANT: This script must be SOURCED, not executed, e.g.:
#     source init-spotlight.sh
# Sourcing is required so the venv activation and the aliases it defines
# stick around in *your* shell.
#
# REQUIRES: zsh.

# ========== CHANGE HARDCODED VARIABLES BELOW ==========
RECORDER_BIN_PATH="$HOME/project/spotlight-dev/spotlight-control/recorder/bin"
PYTHON_VENV_ACTIVATE_PATH="$HOME/project/spotlight-dev/spotlight-tools/.venv/bin/activate"
COWFILE="default"
# ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

# --- Hardcoded variables that are unlikely to change ---
PROFILES_BASEDIR="$HOME/Spotlight/profiles"
DEFAULT_PROFILE="$PROFILES_BASEDIR/default"
ARENAS_BASEDIR="$HOME/Spotlight/arenas"
DEFAULT_ARENA="$ARENAS_BASEDIR/arena146"

# Binaries that the aliases below depend on, expected directly under
# RECORDER_BIN_PATH. "fit-arena-registration" is checked separately below,
# since it's a separately-installed Python tool expected to be on PATH
# (e.g. installed into the venv) rather than in RECORDER_BIN_PATH.
REQUIRED_BINARIES=(align-cameras run-arena-registration-scan run-spotlight reset-camera)

# --- This script requires zsh ---
if [[ -z ${ZSH_VERSION:-} ]]; then
    echo "Error: this script requires zsh." >&2
    return 1 2>/dev/null || exit 1
fi

# --- Refuse to run unless sourced ---
# When sourced, ZSH_EVAL_CONTEXT ends in ":file"; when run as its own
# process, it doesn't.
case ${ZSH_EVAL_CONTEXT:-} in
    *:file) ;;
    *)
        cat >&2 << 'EOF'
This script sets up a Python venv and shell aliases for your *current*
shell, so it must be sourced rather than executed directly:

    source init-spotlight.sh
EOF
        return 1 2>/dev/null || exit 1
        ;;
esac

# --- Helper: list subdirectories of $1, with $2 always shown first as "0) default",
#     then let the user pick a number or type a custom path. Prompts/menus go
#     to stderr; the chosen path is printed to stdout for the caller to capture.
select_dir() {
    local basedir=$1 default_dir=$2 label=$3
    local -a options
    local choice entry i find_output

    if [[ ! -d $default_dir ]]; then
        print -u2 "Error: default $label directory not found at $default_dir."
        return 1
    fi

    print -u2 "Available ${label}s:"
    print -u2 "  0) default (path: $default_dir)"

    find_output=$(find "$basedir" -mindepth 1 -maxdepth 1 -type d ! -path "$default_dir" | sort)
    if [[ -n $find_output ]]; then
        options=("${(@f)find_output}")
    else
        options=()
    fi

    i=1
    for entry in "${options[@]}"; do
        print -u2 "  $i) $(basename "$entry") (path: $entry)"
        (( i++ ))
    done

    print -n -u2 -- "Select a $label by number, or enter a custom path [default: 0]: "
    read -r choice
    choice=${choice:-0}

    if [[ $choice == 0 ]]; then
        print -r -- "$default_dir"
    elif [[ $choice =~ ^[0-9]+$ ]] && (( choice >= 1 && choice <= ${#options} )); then
        print -r -- "${options[choice]}"
    else
        if [[ ! -d $choice ]]; then
            print -u2 "Error: custom $label directory not found at '$choice'."
            return 1
        fi
        print -r -- "$choice"
    fi
}

# Everything else lives in this function so that `nounset`/`pipefail`
# (set via `setopt local_options`) automatically revert when it returns,
# and so a failed step can just `return 1` without tearing down the
# user's sourcing shell.
_spotlight_main() {
    setopt local_options nounset pipefail
    local profile_dir arena_dir color bin

    cat << EOF
Using executables under $RECORDER_BIN_PATH
and activating Python venv at $PYTHON_VENV_ACTIVATE_PATH

EOF

    # --- Sanity-check the recorder binaries ---
    if [[ ! -d $RECORDER_BIN_PATH ]]; then
        print -u2 "Error: recorder bin directory not found at $RECORDER_BIN_PATH."
        return 1
    fi
    for bin in "${REQUIRED_BINARIES[@]}"; do
        if [[ ! -x "$RECORDER_BIN_PATH/$bin" ]]; then
            print -u2 "Error: required executable '$bin' not found (or not executable) at $RECORDER_BIN_PATH/$bin."
            return 1
        fi
    done

    # Activate Python virtual environment
    if [[ -f $PYTHON_VENV_ACTIVATE_PATH ]]; then
        source "$PYTHON_VENV_ACTIVATE_PATH"
    else
        print -u2 "Error: Python venv activation script not found at $PYTHON_VENV_ACTIVATE_PATH."
        return 1
    fi

    # fit-arena-registration is a separately-installed Python tool, expected
    # to be on PATH once the venv above is active.
    if ! command -v fit-arena-registration >/dev/null 2>&1; then
        print -u2 "Warning: 'fit-arena-registration' not found in PATH (expected to be installed separately, e.g. into the Python venv)."
    fi

    cowsay -f "$COWFILE" "Who would use Spotlight must answer me these questions three, ere the other side they see."

    print
    print "WHAT IS YOUR NAME?"
    profile_dir=$(select_dir "$PROFILES_BASEDIR" "$DEFAULT_PROFILE" "profile") || return 1
    print "Using profile: $profile_dir"

    print
    print "WHAT IS YOUR QUEST?"
    arena_dir=$(select_dir "$ARENAS_BASEDIR" "$DEFAULT_ARENA" "arena") || return 1
    print "Using arena: $arena_dir"

    # Monty Python joke
    print
    read -r "color?What is your favorite color? [default: blue] "
    [[ ${color:-blue} == yellow ]] && {
        print -u2 "Error: wrong color, ahhhhhh"
        return 1
    }
    
    # Set up aliases for this shell session
    alias align-cameras="$RECORDER_BIN_PATH/align-cameras -p $profile_dir -a $arena_dir"
    alias run-arena-registration-scan="$RECORDER_BIN_PATH/run-arena-registration-scan -p $profile_dir -a $arena_dir"
    alias run-spotlight="$RECORDER_BIN_PATH/run-spotlight -p $profile_dir -a $arena_dir"
    alias reset-camera="$RECORDER_BIN_PATH/reset-camera"
    alias fit-arena-registration="fit-arena-registration -a $arena_dir"
    
    cowsay -W80 -f "$COWFILE" << EOT
The following command-line programs are now available to you,
with no additional arguments required:

align-cameras

run-arena-registration-scan

fit-arena-registration

run-spotlight

reset-camera
EOT
}

_spotlight_main
