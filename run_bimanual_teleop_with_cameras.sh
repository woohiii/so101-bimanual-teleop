#!/usr/bin/env bash
# Same bimanual teleop as run_bimanual_teleop.sh, but wrist cams + Astra depth
# are folded into the same Rerun window - see run_bimanual_teleop_with_cameras.py.
#
# Prereq: ~/ROBOTICS_PROJECT/calibration/run_astra_depth_watchdog.sh must already
# be running in its own terminal (owns the Astra S device).
set -euo pipefail
cd /home/youngchan/lerobot

env -u PYTHONPATH PATH="/home/youngchan/lerobot/.venv/bin:$PATH" \
  .venv/bin/python /home/youngchan/so101-bimanual-teleop/run_bimanual_teleop_with_cameras.py "$@"
