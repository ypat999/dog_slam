#!/bin/bash

# ARC Test Tool Launcher Script
# Launches the tool in a terminal with black background

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Check if we're already running in a terminal, if not, launch one
if [ -t 0 ]; then
    # Already in a terminal, run directly
    exec python3 "$SCRIPT_DIR/arc_test_tool.py" "$@"
else
    # Not in a terminal, launch terminal with black background
    if command -v xterm >/dev/null 2>&1; then
        # xterm with black background and white foreground
        exec xterm -bg black -fg white -fa 'DejaVu Sans Mono' -fs 12 -e bash -c "cd '$SCRIPT_DIR' && python3 arc_test_tool.py $*; echo 'Press any key to exit...'; read -n 1"
    elif command -v gnome-terminal >/dev/null 2>&1; then
        # gnome-terminal with custom dark theme using ANSI escape sequences
        exec gnome-terminal -- bash -c "
            # Set terminal to dark theme using escape sequences
            printf '\033]11;#000000\007'  # Set background to black
            printf '\033]10;#ffffff\007'  # Set foreground to white
            cd '$SCRIPT_DIR' && python3 arc_test_tool.py $*
            echo 'Press any key to exit...'
            read -n 1
        "
    elif command -v konsole >/dev/null 2>&1; then
        # KDE Konsole with dark background
        exec konsole --background-color black --foreground-color white -e bash -c "cd '$SCRIPT_DIR' && python3 arc_test_tool.py $*; echo 'Press any key to exit...'; read -n 1"
    elif command -v terminator >/dev/null 2>&1; then
        # Terminator with dark profile
        exec terminator --profile=dark -e "bash -c 'cd \"$SCRIPT_DIR\" && python3 arc_test_tool.py $*; echo \"Press any key to exit...\"; read -n 1'"
    else
        echo "❌ No suitable terminal emulator found"
        echo "   Please install one of: xterm, gnome-terminal, konsole, or terminator"
        exit 1
    fi
fi