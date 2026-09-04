#!/usr/bin/env python
"""Live camera preview, same 2x2 grid as camera_preview.py (Astra RGB / Astra
Depth / Wrist 1 / Wrist 2). Only Astra RGB gets Gemini object-pointing
overlays - wrist panels and depth are untouched, same as camera_preview.py.

Gemini runs in a background thread against the live Astra RGB feed so the
displayed video is never blocked waiting on the API call - the overlay just
shows the latest available detection, updated as soon as each call returns.

IMPORTANT - run with the OTHER venv, NOT the lerobot uv venv (same
constraint as camera_preview.py - see its docstring):
    ~/lerobot_song_venv/bin/python camera_preview_gemini.py

Requires GEMINI_API_KEY set in the environment (read automatically by
google.genai.Client(), imported via gemini_point_snapshot.detect_points).
"""

import argparse
import sys
import threading
import time

import cv2

from camera_preview import _panel, depth_to_display, load_cameras, open_wrist, self_test
from camera_utils import ASTRA_DEPTH_MM_PATH, ASTRA_RGB_FRAME_PATH, PublishedDepthSource, PublishedFrameSource
from gemini_point_snapshot import detect_points, draw_points


class GeminiPointWorker:
    """Repeatedly calls Gemini on whatever the latest Astra RGB frame is, in
    its own thread, so the display loop never waits on the network. `latest`
    always holds the most recent detection (or [] before the first one)."""

    def __init__(self, rgb_source: PublishedFrameSource):
        self.rgb_source = rgb_source
        self.latest = []
        self._lock = threading.Lock()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thread.start()

    def get_latest(self):
        with self._lock:
            return self.latest

    def _run(self):
        while True:
            ok, frame = self.rgb_source.read()
            if not ok:
                time.sleep(0.5)  # no fresh frame yet - don't spin
                continue
            try:
                points = detect_points(frame)
                with self._lock:
                    self.latest = points
            except Exception as e:
                print(f"WARNING: astra_rgb - Gemini call/parse failed ({e})")
                # ponytail: no backoff delay - a real API/parse error is rare
                # enough that immediately retrying on the next frame is fine
                # for this MVP tool; add a sleep here if it turns out to spam.


def run_preview(cameras):
    wrist1 = wrist2 = None
    depth_source = PublishedDepthSource(ASTRA_DEPTH_MM_PATH)
    rgb_source = PublishedFrameSource(ASTRA_RGB_FRAME_PATH)
    gemini_worker = GeminiPointWorker(rgb_source)
    gemini_worker.start()

    try:
        wrist1 = open_wrist(cameras["wrist_1_name"], "Wrist 1")
        wrist2 = open_wrist(cameras["wrist_2_name"], "Wrist 2")

        while True:
            depth_mm = depth_source.read()
            depth_vis = depth_to_display(depth_mm) if depth_mm is not None else None

            ok, rgb = rgb_source.read()
            rgb = rgb if ok else None
            rgb_display = draw_points(rgb, gemini_worker.get_latest()) if rgb is not None else None

            wrist1_img = None
            if wrist1 is not None:
                ok, img = wrist1.read()
                wrist1_img = img if ok else None
            wrist2_img = None
            if wrist2 is not None:
                ok, img = wrist2.read()
                wrist2_img = img if ok else None

            grid = cv2.vconcat(
                [
                    cv2.hconcat([_panel(rgb_display, "Astra RGB (gemini)"), _panel(depth_vis, "Astra Depth")]),
                    cv2.hconcat([_panel(wrist1_img, "Wrist 1"), _panel(wrist2_img, "Wrist 2")]),
                ]
            )
            cv2.imshow("Camera Preview (Gemini)", grid)

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
