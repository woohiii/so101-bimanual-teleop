#!/usr/bin/env bash
# Bimanual SO-101 teleoperation: 2 leader + 2 follower arms, one lerobot-teleoperate
# process. Cameras (depth + 2 wrist) are intentionally NOT attached here - see README
# "Why 3 separate processes". Run this after starting the camera processes.
#
# Ports below match /home/youngchan/ROBOTICS_PROJECT/calibration/arms.json as of the
# last calibration. USB re-enumeration can change /dev/ttyACM* on replug/reboot -
# always check `ls -l /dev/ttyACM*` against arms.json before running.
#
# Per-arm configs have no `.id` field - BiSOFollower/BiSOLeader derive each arm's
# calibration id as `<top-level id>_left` / `<top-level id>_right`. Our existing
# calibration files are follower_left/follower_right/leader_left/leader_right, so
# the top-level ids below MUST be exactly "follower" and "leader" to match them.
set -euo pipefail
cd /home/youngchan/lerobot

# --display_data=true spawns the Rerun viewer via `rr.spawn()`, which shells out
# to find a `rerun` binary on PATH (it does NOT look next to the running Python).
# The venv does have one (.venv/bin/rerun) - it's just not on PATH when this
# script calls .venv/bin/lerobot-teleoperate directly instead of activating the
# venv, so PATH needs it prepended here.
env -u PYTHONPATH PATH="/home/youngchan/lerobot/.venv/bin:$PATH" .venv/bin/lerobot-teleoperate \
  --robot.type=bi_so_follower \
  --robot.id=follower \
  --robot.left_arm_config.port=/dev/ttyACM4  --robot.left_arm_config.max_relative_target=5.0 \
  --robot.right_arm_config.port=/dev/ttyACM3 --robot.right_arm_config.max_relative_target=5.0 \
  --teleop.type=bi_so_leader \
  --teleop.id=leader \
  --teleop.left_arm_config.port=/dev/ttyACM2 \
  --teleop.right_arm_config.port=/dev/ttyACM1 \
  --display_data=true \
  "$@"
