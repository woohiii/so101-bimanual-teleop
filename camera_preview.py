#!/usr/bin/env python
"""3-window live camera preview: Astra S depth camera (depth mode) + 2 USB wrist cameras.

IMPORTANT - run with the OTHER venv, NOT the lerobot uv venv:
    ~/lerobot_song_venv/bin/python camera_preview.py

This script needs a GUI-enabled OpenCV build (cv2.imshow). That's not available
in /home/youngchan/lerobot's uv-managed venv (headless opencv there, cv2.imshow
raises an error) - only in ~/lerobot_song_venv. Do NOT run this via
`uv run --project /home/youngchan/lerobot ...`.

IMPORTANT - Astra S must be run as its OWN process, in a separate terminal,
BEFORE this script (confirmed via py-spy: running OpenNI2 and OpenCV
VideoCapture in the same process starves OpenNI2's USB events thread and
hangs native reads forever). Also confirmed this Astra S unit's OpenNI2
streams can wedge on their own after anywhere from seconds to minutes even
running standalone - use the watchdog, which auto-recovers via a USB reset,
rather than the plain script:

    ~/ROBOTICS_PROJECT/calibration/run_astra_depth_watchdog.sh

(That watchdog script + the OpenNI2 SDK it needs live in the ROBOTICS_PROJECT
repo, not here - not duplicated into this repo because of the large vendored
SDK binary tree it depends on. See this repo's README.)

That publishes the depth array to /tmp/vsp_astra_depth_mm.npy; this script
just reads that file instead of opening the Astra S device itself. Only ONE
process may hold the Astra S device open at a time - close any other running
astra_s_*.py script before starting the watchdog.
"""

import argparse
import json
import shutil
import sys
from pathlib import Path

import cv2
import numpy as np

from camera_utils import ASTRA_DEPTH_MM_PATH, PublishedDepthSource, find_camera_index

CAMERAS_JSON = Path(__file__).parent / "cameras.json"

# Same close-range convention as astra_s_live.py (350-800mm covers the table
# workspace on this rig without the far background washing out the range).
DEPTH_MIN_MM = 350
DEPTH_MAX_MM = 800


def load_cameras():
    with open(CAMERAS_JSON) as f:
        return json.load(f)


def self_test():
    checks = []
    try:
        cameras = load_cameras()
        checks.append(("cameras.json parses with wrist_1_name/wrist_2_name", {"wrist_1_name", "wrist_2_name"} <= cameras.keys()))
    except Exception as e:
        print(f"FAIL: cameras.json parses with wrist_1_name/wrist_2_name ({e})")
        cameras = {}

    for key in ("wrist_1_name", "wrist_2_name"):
        name = cameras.get(key)
        if name is None:
            continue
        idx = find_camera_index(name)
        if idx is not None:
            print(f"PASS: {key} ('{name}') resolved to /dev/video{idx}")
        else:
            print(f"FAIL: {key} ('{name}') not found (camera may not be plugged in)")

    checks.append(("v4l2-ctl available", shutil.which("v4l2-ctl") is not None))

    depth_source = PublishedDepthSource(ASTRA_DEPTH_MM_PATH)
    if depth_source.read() is not None:
        print(f"PASS: astra_s_depth_hub.py is publishing fresh depth frames to {ASTRA_DEPTH_MM_PATH}")
    else:
        print(
            f"INFO: no fresh depth frame at {ASTRA_DEPTH_MM_PATH} yet - "
            f"start run_astra_depth_watchdog.sh first (see this script's docstring), not a failure by itself"
        )

    ok = True
    for name, passed in checks:
        print(f"{'PASS' if passed else 'FAIL'}: {name}")
        ok = ok and passed

    return 0 if ok else 1


def depth_to_display(depth_mm):
    """Render raw mm depth as a color panel, same convention as astra_s_live.py."""
    clipped = np.clip(depth_mm, DEPTH_MIN_MM, DEPTH_MAX_MM).astype(np.float32)
    scaled = ((clipped - DEPTH_MIN_MM) * 255.0 / (DEPTH_MAX_MM - DEPTH_MIN_MM)).astype(np.uint8)
    image = cv2.applyColorMap(scaled, cv2.COLORMAP_JET)
    image[depth_mm == 0] = (0, 0, 0)
    return image


def open_wrist(name, label):
    idx = find_camera_index(name)
    if idx is None:
        print(f"ERROR: {label} ('{name}') not found - skipping this window")
        return None
    cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if not cap.isOpened():
        print(f"ERROR: {label} ('{name}') at /dev/video{idx} could not be opened - skipping this window")
        return None
    return cap


def run_preview(cameras):
    wrist1 = wrist2 = None
    depth_source = PublishedDepthSource(ASTRA_DEPTH_MM_PATH)
    try:
        wrist1 = open_wrist(cameras["wrist_1_name"], "Wrist 1")
        wrist2 = open_wrist(cameras["wrist_2_name"], "Wrist 2")

        while True:
            depth_mm = depth_source.read()
            if depth_mm is not None:
                cv2.imshow("Astra S Depth", depth_to_display(depth_mm))

            if wrist1 is not None:
                ok, img = wrist1.read()
                if ok:
                    cv2.imshow("Wrist 1", img)
            if wrist2 is not None:
                ok, img = wrist2.read()
                if ok:
                    cv2.imshow("Wrist 2", img)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        if wrist1 is not None:
            wrist1.release()
        if wrist2 is not None:
            wrist2.release()
        cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--self-test", action="store_true", help="Validate cameras.json + resolve indices (no hardware streams)")
    args = parser.parse_args()

    if args.self_test:
        sys.exit(self_test())

    run_preview(load_cameras())


if __name__ == "__main__":
    main()
