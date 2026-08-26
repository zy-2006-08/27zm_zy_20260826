#!/bin/bash
# Auto-start script for the robot onboard computer.
# Registered as a system autostart entry, so it runs on every boot
# and launches the vision program inside a detached screen session.

set -e

# Give the hardware time to finish enumerating after power-on
# (industrial camera over USB, serial board / CDC device node).
# Without this wait the program starts before the devices exist and dies.
sleep 5

# Enter the build directory that sits next to THIS script.
# Derived from the script's own location, so the repository can be
# cloned anywhere. The program must run from build/ because every
# config and asset path in the code is relative to it (../configs, ../assets).
cd "$(dirname "$(readlink -f "$0")")/build" || exit 1

# The screen log directory is not tracked by git, so create it here.
# screen refuses to start if the -Logfile directory is missing.
mkdir -p logs

# -L + -Logfile : record all terminal output to a timestamped log file
# -d -m         : start detached, keeps running with no terminal attached
screen \
    -L \
    -Logfile logs/$(date "+%Y-%m-%d_%H-%M-%S").screenlog \
    -d \
    -m \
    bash -c "./rb_auto_aim_debug"
