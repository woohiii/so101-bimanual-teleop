#!/usr/bin/env bash
# Same bimanual teleop as run_bimanual_teleop.sh, but wrist cams + Astra depth
# are folded into the same Rerun window - see run_bimanual_teleop_with_cameras.py.
#
# Prereq: ~/ROBOTICS_PROJECT/calibration/run_astra_depth_watchdog.sh must already
# be running in its own terminal (owns the Astra S device).
set -euo pipefail
cd /home/youngchan/lerobot

# A previous run left stopped/orphaned (Ctrl+Z'd, or terminal-closed) instead of
# Ctrl+C'd keeps holding the wrist cameras + serial ports, and the new run then
# fails with "Failed to open OpenCVCamera(4)" or "Could not connect on port" -
# not a bug in this script, just a leftover process. Resume+interrupt any such
# instance first so it exits through its own cleanup (torque disable) instead
# of leaving the arms/cameras stuck.
pkill -CONT -f "run_bimanual_teleop_with_cameras.py" 2>/dev/null || true
pkill -INT -f "run_bimanual_teleop_with_cameras.py" 2>/dev/null || true
sleep 1

env -u PYTHONPATH PATH="/home/youngchan/lerobot/.venv/bin:$PATH" \
  .venv/bin/python /home/youngchan/so101-bimanual-teleop/run_bimanual_teleop_with_cameras.py "$@"
