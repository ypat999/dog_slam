#!/bin/bash

JSZR_BASE_DIR=/home/jszr/jszr_workspace

[ -f "/opt/ros/humble/setup.bash" ] || {
  echo "ROS setup.bash not found"
  exit 1
}
[ -f "${JSZR_BASE_DIR}/install/setup.bash" ] || {
  echo "Workspace setup.bash not found"
  exit 1
}

source /opt/ros/humble/setup.bash
source "${JSZR_BASE_DIR}/install/setup.bash"
export ROS_DOMAIN_ID=24
export ROS_DISCOVERY_SERVER=192.168.133.1:20000

if [ "$#" -lt 1 ]; then
  echo "Usage: $0 <bag directory or container directory> [--ros2-bag extra arguments]"
  exit 1
fi

INPUT="$1"
shift

# If the provided argument is a directory
if [ -d "$INPUT" ]; then
  # If the directory contains metadata.yaml, treat it as a valid ROS2 bag directory
  if [ -f "$INPUT/metadata.yaml" ]; then
    BAG_DIR="$INPUT"
    echo "The provided argument is a valid bag directory: $BAG_DIR"
  else
    echo "The provided argument is a directory. Searching for the latest valid bag directory within..."
    # Search for all subdirectories (at the current level) that contain metadata.yaml,
    # sort them by modification time, and select the latest one.
    BAG_DIR=$(find "$INPUT" -mindepth 1 -maxdepth 1 -type d -exec test -f {}/metadata.yaml \; -printf "%T@ %p\n" |
      sort -n | tail -1 | cut -d' ' -f2-)
    if [ -z "$BAG_DIR" ]; then
      echo "No valid bag directory (missing metadata.yaml) found in '$INPUT'"
      exit 1
    fi
    echo "Found the latest valid bag directory: $BAG_DIR"
  fi
else
  # If the provided argument is not a directory, assume it is a valid bag directory
  BAG_DIR="$INPUT"
fi

# Check if tmux is installed
command -v tmux >/dev/null 2>&1 || {
  echo "tmux is not installed. Please install it first."
  exit 1
}

discovery="fastdds discovery -i 0 -l 192.168.133.1 -p 20000"
replay="bag play ${BAG_DIR} $@"

SESSION_NAME="jszr"
WINDOW_NAME="main"

# Arrays to hold pane titles and their corresponding commands
declare -a pane_titles
declare -a pane_commands

# Fixed module: discovery replay
pane_titles+=("discovery" "replay")
pane_commands+=("$discovery" "$replay")

# Safe split function for normal sessions.
function safe_split() {
  local cmd="$1"
  local title="$2"
  tmux split-window -t "$SESSION_NAME:$WINDOW_NAME" "bash -c '$cmd; exec bash'" 2>/dev/null
  if [ $? -ne 0 ]; then
    echo "Warning: No space for new pane, adjusting layout..."
    tmux select-layout -t "$SESSION_NAME:$WINDOW_NAME" even-horizontal
    tmux split-window -t "$SESSION_NAME:$WINDOW_NAME" "bash -c '$cmd; exec bash'"
  fi
  tmux select-pane -T "$title"
}

# Safe split function for inner sessions using a separate socket.
function safe_split_inner() {
  local cmd="$1"
  local title="$2"
  tmux -L inner_socket split-window -t "$SESSION_NAME:$WINDOW_NAME" "bash -c '$cmd; exec bash'" 2>/dev/null
  if [ $? -ne 0 ]; then
    echo "Warning (inner): No space for new pane, adjusting layout..."
    tmux -L inner_socket select-layout -t "$SESSION_NAME:$WINDOW_NAME" even-horizontal
    tmux -L inner_socket split-window -t "$SESSION_NAME:$WINDOW_NAME" "bash -c '$cmd; exec bash'"
  fi
  tmux -L inner_socket select-pane -T "$title"
}

function start() {
  # If running inside an outer tmux session, use a separate socket to launch an inner session.
  if [ -n "$TMUX" ]; then
    echo "Detected outer tmux session. Launching inner session on a separate socket."
    tmux -L inner_socket new-session -d -s "$SESSION_NAME" -n "$WINDOW_NAME" "bash -c '${pane_commands[0]}; exec bash'"
    tmux -L inner_socket select-pane -t "$SESSION_NAME:$WINDOW_NAME.0" -T "${pane_titles[0]}"
    for ((i = 1; i < ${#pane_commands[@]}; i++)); do
      safe_split_inner "${pane_commands[i]}" "${pane_titles[i]}"
    done
    tmux -L inner_socket select-layout -t "$SESSION_NAME:$WINDOW_NAME" tiled
    tmux -L inner_socket set-option -g mouse on
    tmux -L inner_socket set-option -w history-limit 100000
    tmux -L inner_socket set-option -g pane-border-status top
    tmux -L inner_socket set-option -g pane-border-format "#T"
    tmux -L inner_socket attach-session -t "$SESSION_NAME"
  else
    # Start a normal tmux session if not running inside another tmux.
    tmux new-session -d -s "$SESSION_NAME" -n "$WINDOW_NAME" "bash -c '${pane_commands[0]}; exec bash'"
    tmux select-pane -t "$SESSION_NAME:$WINDOW_NAME.0" -T "${pane_titles[0]}"
    for ((i = 1; i < ${#pane_commands[@]}; i++)); do
      safe_split "${pane_commands[i]}" "${pane_titles[i]}"
    done
    tmux select-layout -t "$SESSION_NAME:$WINDOW_NAME" tiled
    tmux set-option -g mouse on
    tmux set-option -w history-limit 100000
    tmux set-option -g pane-border-status top
    tmux set-option -g pane-border-format "#T"
    tmux attach-session -t "$SESSION_NAME"
  fi
}

function stop() {
  if [ -n "$TMUX" ]; then
    tmux -L inner_socket kill-session -t "$SESSION_NAME"
  else
    tmux kill-session -t "$SESSION_NAME"
  fi
}

# Check if a session with the same name already exists.
if [ -n "$TMUX" ]; then
  tmux -L inner_socket has-session -t "$SESSION_NAME" 2>/dev/null
else
  tmux has-session -t "$SESSION_NAME" 2>/dev/null
fi

if [ $? != 0 ]; then
  start
else
  echo "A tmux session named '$SESSION_NAME' is already running."
  echo "Restarting..."
  stop && start
fi
