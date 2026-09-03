#!/usr/bin/env bash
# Bimanual SO-101 teleoperation: 2 leader + 2 follower arms, one lerobot-teleoperate
# process. Cameras (depth + 2 wrist) are intentionally NOT attached here - see README
# "Why 3 separate processes". Run this after starting the camera processes.
#
# Ports below match /home/youngchan/ROBOTICS_PROJECT/calibration/arms.json as of the
# last calibration. USB re-enumeration can change /dev/ttyACM* on replug/reboot -
# always check `ls -l /dev/ttyACM*` against arms.json before running.
set -euo pipefail
cd /home/youngchan/lerobot

env -u PYTHONPATH .venv/bin/lerobot-teleoperate \
  --robot.type=bi_so_follower \
  --robot.id=bimanual_follower \
  --robot.left_arm_config.port=/dev/ttyACM4  --robot.left_arm_config.id=follower_left  --robot.left_arm_config.max_relative_target=5.0 \
  --robot.right_arm_config.port=/dev/ttyACM3 --robot.right_arm_config.id=follower_right --robot.right_arm_config.max_relative_target=5.0 \
  --teleop.type=bi_so_leader \
  --teleop.id=bimanual_leader \
  --teleop.left_arm_config.port=/dev/ttyACM2  --teleop.left_arm_config.id=leader_left \
  --teleop.right_arm_config.port=/dev/ttyACM1 --teleop.right_arm_config.id=leader_right \
  --display_data=true \
  "$@"
