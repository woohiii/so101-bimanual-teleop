#!/usr/bin/env python
"""Bimanual SO-101 teleop with wrist cameras AND the Astra S depth view folded
into the same Rerun window as the arm telemetry.

Wrist cameras use the normal `robot.cameras` path (plain `OpenCVCameraConfig`
per arm) - that's the same mechanism `lerobot-teleoperate` already supports via
CLI flags, nothing custom needed there.

The Astra S depth can't take that path on this rig: attaching it as a
`robot.cameras` entry would open it through OpenNI2 *in this same process*, and
OpenNI2 + OpenCV VideoCapture in one process is confirmed (see
camera_preview.py's docstring, py-spy-verified) to starve OpenNI2's USB event
thread and hang native reads forever. So this script never touches OpenNI2 -
it reads the depth array that the already-running `run_astra_depth_watchdog.sh`
publishes to a file, exactly like `camera_preview.py` does for its own depth
panel, and logs that into Rerun alongside everything else.

This is a thin copy of lerobot's own `lerobot_teleoperate.teleop_loop` (same
processors, same CycleTimer, same connect/disconnect) with one addition: the
published depth array is merged into the observation dict before it's logged,
so Rerun's own image-key-to-view logic (any HxWx1 array becomes a DepthImage
panel) picks it up automatically.

Prereqs:
  - ~/ROBOTICS_PROJECT/calibration/run_astra_depth_watchdog.sh already running
    in its own terminal (owns the Astra S; see README for why that script isn't
    vendored into this repo).
  - arms.json / cameras.json (in this repo) ports and camera names up to date.

Usage (from the lerobot venv):
    cd ~/lerobot && PATH="$PWD/.venv/bin:$PATH" .venv/bin/python \\
        ~/so101-bimanual-teleop/run_bimanual_teleop_with_cameras.py [--teleop_time_s=15]
"""

import argparse
import json
import time
from pathlib import Path

from camera_utils import ASTRA_DEPTH_MM_PATH, PublishedDepthSource, find_camera_index

from lerobot.cameras.opencv import OpenCVCameraConfig
from lerobot.processor import make_default_processors
from lerobot.robots.bi_so_follower import BiSOFollower, BiSOFollowerConfig
from lerobot.robots.so_follower import SOFollowerConfig
from lerobot.teleoperators.bi_so_leader import BiSOLeader, BiSOLeaderConfig
from lerobot.teleoperators.so_leader import SOLeaderConfig
from lerobot.utils.cycle_timer import CycleTimer
from lerobot.utils.utils import init_logging, move_cursor_up
from lerobot.utils.visualization_utils import init_visualization, log_visualization_data, shutdown_visualization

CALIB_DIR = Path(__file__).parent  # arms.json / cameras.json vendored alongside this script
MAX_RELATIVE_TARGET = 5.0  # degrees per step at FPS - see README "Motor safety"
FPS = 60


def wrist_camera_config(usb_name: str) -> OpenCVCameraConfig:
    idx = find_camera_index(usb_name)
    if idx is None:
        raise RuntimeError(f"Wrist camera '{usb_name}' not found - check it's plugged in (cameras.json).")
    return OpenCVCameraConfig(index_or_path=idx, width=640, height=480, fps=30)


def build_devices() -> tuple[BiSOFollower, BiSOLeader]:
    ports = {a["id"]: a["port"] for a in json.loads((CALIB_DIR / "arms.json").read_text())}
    cams = json.loads((CALIB_DIR / "cameras.json").read_text())

    robot = BiSOFollower(
        BiSOFollowerConfig(
            id="follower",
            left_arm_config=SOFollowerConfig(
                port=ports["follower_left"],
                max_relative_target=MAX_RELATIVE_TARGET,
                cameras={"wrist": wrist_camera_config(cams["wrist_1_name"])},
            ),
            right_arm_config=SOFollowerConfig(
                port=ports["follower_right"],
                max_relative_target=MAX_RELATIVE_TARGET,
                cameras={"wrist": wrist_camera_config(cams["wrist_2_name"])},
            ),
        )
    )
    teleop = BiSOLeader(
        BiSOLeaderConfig(
            id="leader",
            left_arm_config=SOLeaderConfig(port=ports["leader_left"]),
            right_arm_config=SOLeaderConfig(port=ports["leader_right"]),
        )
    )
    return robot, teleop


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--teleop_time_s", type=float, default=None)
    args = parser.parse_args()

    init_logging()
    robot, teleop = build_devices()
    teleop_action_processor, robot_action_processor, robot_observation_processor = make_default_processors()
    depth_source = PublishedDepthSource(ASTRA_DEPTH_MM_PATH)

    init_visualization("rerun", session_name="bimanual_teleop_with_depth")
    teleop.connect()
    robot.connect()

    display_len = max(len(key) for key in robot.action_features)
    timer = CycleTimer(FPS, records_data=False)
    start = time.perf_counter()
    try:
        while True:
            timer.tick()
            with timer.section("observe"):
                obs = robot.get_observation()
            with timer.section("teleop"):
                raw_action = teleop.get_action()
                teleop_action = teleop_action_processor((raw_action, obs))
                robot_action_to_send = robot_action_processor((teleop_action, obs))
            with timer.section("send"):
                robot.send_action(robot_action_to_send)
            with timer.section("telemetry"):
                obs_transition = robot_observation_processor(obs)
                depth_mm = depth_source.read()
                if depth_mm is not None:
                    obs_transition["astra_depth"] = depth_mm[..., None]
                log_visualization_data("rerun", observation=obs_transition, action=teleop_action)

                print("\n" + "-" * (display_len + 10))
                print(f"{'NAME':<{display_len}} | {'NORM':>7}")
                for motor, value in robot_action_to_send.items():
                    print(f"{motor:<{display_len}} | {value:>7.2f}")
                move_cursor_up(len(robot_action_to_send) + 3)

            timer.wait()
            if args.teleop_time_s is not None and time.perf_counter() - start >= args.teleop_time_s:
                return
    except KeyboardInterrupt:
        pass
    finally:
        timer.log_run_summary()
        shutdown_visualization("rerun")
        teleop.disconnect()
        robot.disconnect()


if __name__ == "__main__":
    main()
