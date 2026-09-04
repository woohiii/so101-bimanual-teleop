#!/usr/bin/env python
"""Bimanual SO-101 dataset recording (2 wrist cams + Astra S depth, all saved as
dataset features) — for the towel-fold IL data.

Reuses lerobot's own `record_loop` (episode looping, keyboard control, video
encoding) unchanged. The only addition: `BiSOFollower.get_observation()` is
wrapped at the instance level to merge in the Astra depth array (read from the
file `run_astra_depth_watchdog.sh` publishes — same mechanism as
`run_bimanual_teleop_with_cameras.py`, for the same OpenNI2/OpenCV
process-isolation reason, see that script's docstring), and
`observation_features` gets an extra `"astra_depth": (H, W, 1)` entry so
`aggregate_pipeline_dataset_features` picks it up. A (H, W, 1) shape is
LeRobotDataset's own convention for "this is a depth map, not RGB" (see
`hw_to_dataset_features` in lerobot/utils/feature_utils.py) - no other
plumbing needed, it already has a dedicated depth video codec path
(`DatasetRecordConfig.depth_encoder`).

Prereqs:
  - ~/ROBOTICS_PROJECT/calibration/run_astra_depth_watchdog.sh already running
    in its own terminal (owns the Astra S).
  - arms.json / cameras.json (in this repo) ports and camera names up to date.

Usage (from the lerobot venv):
    cd ~/lerobot && uv run python ~/so101-bimanual-teleop/record_bimanual_with_depth.py \\
        --repo_id local/towel_half_fold_bimanual \\
        --single_task "Fold the towel in half" \\
        --num_episodes 20
"""

import argparse
import json
from pathlib import Path

from camera_utils import ASTRA_DEPTH_MM_PATH, PublishedDepthSource, find_camera_index

from lerobot.cameras.opencv import OpenCVCameraConfig
from lerobot.configs.dataset import DatasetRecordConfig
from lerobot.datasets import (
    LeRobotDataset,
    VideoEncodingManager,
    aggregate_pipeline_dataset_features,
    create_initial_features,
)
from lerobot.processor import make_default_processors
from lerobot.robots.bi_so_follower import BiSOFollower, BiSOFollowerConfig
from lerobot.robots.so_follower import SOFollowerConfig
from lerobot.scripts.lerobot_record import record_loop
from lerobot.teleoperators.bi_so_leader import BiSOLeader, BiSOLeaderConfig
from lerobot.teleoperators.so_leader import SOLeaderConfig
from lerobot.utils.cycle_timer import CycleTimer
from lerobot.utils.feature_utils import combine_feature_dicts
from lerobot.utils.keyboard_input import init_keyboard_listener
from lerobot.utils.utils import init_logging, log_say
from lerobot.utils.visualization_utils import init_visualization, shutdown_visualization

CALIB_DIR = Path(__file__).parent  # arms.json / cameras.json vendored alongside this script
MAX_RELATIVE_TARGET = 10.0  # degrees per step at FPS - see README "Motor safety"
FPS = 30

# Astra S depth (registered to its RGB's pixel grid) is published at this native
# resolution - see astra_s_live.py's docstring on why it's lower than the RGB frame.
DEPTH_SHAPE = (240, 320, 1)


def wrist_camera_config(usb_name: str) -> OpenCVCameraConfig:
    idx = find_camera_index(usb_name)
    if idx is None:
        raise RuntimeError(f"Wrist camera '{usb_name}' not found - check it's plugged in (cameras.json).")
    return OpenCVCameraConfig(index_or_path=idx, width=640, height=480, fps=FPS)


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


def add_depth_to_robot(robot: BiSOFollower) -> None:
    """Instance-level patch: merge the published Astra depth array into every
    `get_observation()` call, and declare it in `observation_features` so the
    dataset schema (built from that property) includes it. Raises if the depth
    watchdog isn't publishing - recording a frame silently missing this feature
    would corrupt the dataset schema, so fail loud instead."""
    depth_source = PublishedDepthSource(ASTRA_DEPTH_MM_PATH)
    original_get_observation = robot.get_observation

    def get_observation_with_depth():
        obs = original_get_observation()
        depth_mm = depth_source.read()
        if depth_mm is None:
            raise RuntimeError(
                "No fresh Astra depth data - is run_astra_depth_watchdog.sh running "
                f"and publishing to {ASTRA_DEPTH_MM_PATH}?"
            )
        obs["astra_depth"] = depth_mm[..., None]
        return obs

    robot.get_observation = get_observation_with_depth
    # observation_features is a @cached_property; assigning here shadows it at the
    # instance level (cached_property stores its value in __dict__ under the same
    # name, so a direct instance attribute takes precedence the same way).
    robot.observation_features = {**robot.observation_features, "astra_depth": DEPTH_SHAPE}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--repo_id", default="local/towel_half_fold_bimanual")
    parser.add_argument("--single_task", required=True, help='e.g. "Fold the towel in half"')
    parser.add_argument("--num_episodes", type=int, default=20)
    parser.add_argument("--episode_time_s", type=float, default=20)
    parser.add_argument("--reset_time_s", type=float, default=10)
    parser.add_argument("--push_to_hub", action="store_true")
    args = parser.parse_args()

    dataset_cfg = DatasetRecordConfig(
        repo_id=args.repo_id,
        single_task=args.single_task,
        fps=FPS,
        episode_time_s=args.episode_time_s,
        reset_time_s=args.reset_time_s,
        num_episodes=args.num_episodes,
        push_to_hub=args.push_to_hub,
    )

    init_logging()
    init_visualization("rerun", session_name="bimanual_record_with_depth")

    robot, teleop = build_devices()
    add_depth_to_robot(robot)
    teleop_action_processor, robot_action_processor, robot_observation_processor = make_default_processors()

    dataset_features = combine_feature_dicts(
        aggregate_pipeline_dataset_features(
            pipeline=teleop_action_processor,
            initial_features=create_initial_features(action=robot.action_features),
            use_videos=dataset_cfg.video,
        ),
        aggregate_pipeline_dataset_features(
            pipeline=robot_observation_processor,
            initial_features=create_initial_features(observation=robot.observation_features),
            use_videos=dataset_cfg.video,
        ),
    )

    dataset_cfg.stamp_repo_id()
    dataset = LeRobotDataset.create(
        dataset_cfg.repo_id,
        dataset_cfg.fps,
        root=dataset_cfg.root,
        robot_type=robot.name,
        features=dataset_features,
        use_videos=dataset_cfg.video,
        image_writer_processes=dataset_cfg.num_image_writer_processes,
        image_writer_threads=dataset_cfg.num_image_writer_threads_per_camera * len(robot.cameras),
        batch_encoding_size=dataset_cfg.video_encoding_batch_size,
        rgb_encoder=dataset_cfg.rgb_encoder,
        depth_encoder=dataset_cfg.depth_encoder,
        encoder_threads=dataset_cfg.encoder_threads,
        streaming_encoding=dataset_cfg.streaming_encoding,
        encoder_queue_maxsize=dataset_cfg.encoder_queue_maxsize,
    )

    teleop.connect()
    robot.connect()
    listener, events = init_keyboard_listener()
    timer = CycleTimer(dataset_cfg.fps)

    try:
        with VideoEncodingManager(dataset):
            recorded_episodes = 0
            while recorded_episodes < dataset_cfg.num_episodes and not events["stop_recording"]:
                episode_index = dataset.num_episodes
                log_say(f"Recording episode {episode_index}", True)
                record_loop(
                    robot=robot,
                    events=events,
                    fps=dataset_cfg.fps,
                    teleop_action_processor=teleop_action_processor,
                    robot_action_processor=robot_action_processor,
                    robot_observation_processor=robot_observation_processor,
                    teleop=teleop,
                    dataset=dataset,
                    control_time_s=dataset_cfg.episode_time_s,
                    single_task=dataset_cfg.single_task,
                    display_data=True,
                    timer=timer,
                )

                if not events["stop_recording"] and (
                    (recorded_episodes < dataset_cfg.num_episodes - 1) or events["rerecord_episode"]
                ):
                    log_say("Reset the environment", True)
                    record_loop(
                        robot=robot,
                        events=events,
                        fps=dataset_cfg.fps,
                        teleop_action_processor=teleop_action_processor,
                        robot_action_processor=robot_action_processor,
                        robot_observation_processor=robot_observation_processor,
                        teleop=teleop,
                        control_time_s=dataset_cfg.reset_time_s,
                        single_task=dataset_cfg.single_task,
                        display_data=True,
                    )

                if events["rerecord_episode"]:
                    log_say("Re-record episode", True)
                    events["rerecord_episode"] = False
                    events["exit_early"] = False
                    dataset.clear_episode_buffer()
                    timer.log_episode_summary("discarded episode")
                    timer.restart()
                    continue

                dataset.save_episode()
                recorded_episodes += 1
                timer.log_episode_summary(f"episode {episode_index}")
                timer.restart()
    finally:
        timer.log_run_summary()
        log_say("Stop recording", True, blocking=True)
        dataset.finalize()
        if robot.is_connected:
            robot.disconnect()
        if teleop.is_connected:
            teleop.disconnect()
        listener.stop()
        shutdown_visualization("rerun")
        if dataset_cfg.push_to_hub and dataset.num_episodes > 0:
            dataset.push_to_hub(tags=dataset_cfg.tags, private=dataset_cfg.private)
        log_say("Exiting", True)


if __name__ == "__main__":
    main()
