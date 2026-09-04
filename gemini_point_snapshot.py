#!/usr/bin/env python
"""One-shot snapshot: grab a frame from each of Astra RGB + 2 wrist cameras,
ask Gemini to point at objects in each, draw the results, save as PNG.

IMPORTANT - run with the OTHER venv, NOT the lerobot uv venv (same
constraint as camera_preview.py - see its docstring):
    ~/lerobot_song_venv/bin/python gemini_point_snapshot.py

Requires GEMINI_API_KEY set in the environment (read automatically by
google.genai.Client()).
"""

import argparse
import json
import os
import sys

import cv2

from camera_preview import load_cameras, open_wrist
from camera_utils import ASTRA_RGB_FRAME_PATH, PublishedFrameSource

PROMPT = (
    'Point to no more than 10 items in the image. The answer should follow the json format: '
    '[{"point": <point>, "label": <label1>}, ...]. The points are in [y, x] format normalized to 0-1000.'
)
MODEL = "gemini-robotics-er-2-preview"


def self_test():
    ok = True

    try:
        cameras = load_cameras()
        passed = {"wrist_1_name", "wrist_2_name"} <= cameras.keys()
        print(f"{'PASS' if passed else 'FAIL'}: cameras.json parses with wrist_1_name/wrist_2_name")
        ok = ok and passed
    except Exception as e:
        print(f"FAIL: cameras.json parses ({e})")
        ok = False

    has_key = bool(os.environ.get("GEMINI_API_KEY"))
    print(f"{'PASS' if has_key else 'FAIL'}: GEMINI_API_KEY is set")
    ok = ok and has_key

    try:
        import google.genai  # noqa: F401

        print("PASS: google.genai importable")
    except ImportError as e:
        print(f"FAIL: google.genai importable ({e})")
        ok = False

    return 0 if ok else 1


def capture_frames():
    """Returns {camera_key: frame_or_None}. One camera's capture failure
    doesn't prevent capturing the others."""
    frames = {}

    try:
        ok, frame = PublishedFrameSource(ASTRA_RGB_FRAME_PATH).read()
        frames["astra_rgb"] = frame if ok else None
        if not ok:
            print("WARNING: astra_rgb - no fresh published frame (is the Astra watchdog running?)")
    except Exception as e:
        print(f"WARNING: astra_rgb - capture failed ({e})")
        frames["astra_rgb"] = None

    cameras = load_cameras()
    for key, cam_key_name, label in (
        ("wrist1", "wrist_1_name", "Wrist 1"),
        ("wrist2", "wrist_2_name", "Wrist 2"),
    ):
        try:
            cap = open_wrist(cameras[cam_key_name], label)
            if cap is None:
                frames[key] = None
                continue
            ok, frame = cap.read()
            cap.release()
            frames[key] = frame if ok else None
            if not ok:
                print(f"WARNING: {key} - camera opened but read failed")
        except Exception as e:
            print(f"WARNING: {key} - capture failed ({e})")
            frames[key] = None

    return frames


def detect_points(frame) -> list[dict]:
    """Calls Gemini and parses its response into [{point, label}, ...].
    Raises on a network/API error; raises ValueError (with the raw response
    text attached) if the response isn't parseable JSON."""
    from google import genai
    from google.genai import types

    ok, png_bytes = cv2.imencode(".png", frame)
    if not ok:
        raise RuntimeError("cv2.imencode failed")

    client = genai.Client()
    response = client.models.generate_content(
        model=MODEL,
        contents=[types.Part.from_bytes(data=png_bytes.tobytes(), mime_type="image/png"), PROMPT],
    )
    text = response.text.strip()
    if text.startswith("```"):
        text = text.split("\n", 1)[1].rsplit("```", 1)[0]
    try:
        return json.loads(text)
    except json.JSONDecodeError as e:
        raise ValueError(f"could not parse Gemini response as JSON: {e}\nraw response: {response.text}") from e


def draw_points(frame, points):
    frame = frame.copy()
    h, w = frame.shape[:2]
    for item in points:
        y, x = item["point"]
        label = str(item.get("label", ""))
        px, py = int(x * w / 1000), int(y * h / 1000)

        cv2.circle(frame, (px, py), 8, (255, 255, 255), -1)
        cv2.circle(frame, (px, py), 8, (200, 0, 0), 2)

        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
        box_x, box_y = px + 12, py - th // 2 - 6
        cv2.rectangle(frame, (box_x, box_y), (box_x + tw + 8, box_y + th + 10), (200, 0, 0), -1)
        cv2.putText(frame, label, (box_x + 4, box_y + th + 3), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    return frame


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--self-test", action="store_true", help="Validate cameras.json + env + imports (no camera/network access)")
    args = parser.parse_args()

    if args.self_test:
        sys.exit(self_test())

    frames = capture_frames()

    for key, frame in frames.items():
        if frame is None:
            print(f"{key}: skipped (no frame captured)")
            continue
        try:
            points = detect_points(frame)
        except Exception as e:
            print(f"{key}: WARNING - Gemini call/parse failed ({e})")
            continue

        annotated = draw_points(frame, points)
        out_path = f"/tmp/gemini_points_{key}.png"
        cv2.imwrite(out_path, annotated)
        print(f"{key}: {len(points)} objects -> {out_path}")


if __name__ == "__main__":
    main()
