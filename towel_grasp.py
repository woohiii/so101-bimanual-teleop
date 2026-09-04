"""Suggests two grasp points on a light-pink towel: the two ends of its long
edge, for the left/right arm to each grab one end (two-arm pick + two-arm
fold task). Same HSV-threshold + contour approach as lerobot's
custom_scripts/vision_pick_place/cube_detector.py, adapted for endpoints
instead of a centroid.
"""

from dataclasses import dataclass

import cv2
import numpy as np

# Light pink, not sampled from real frames yet - if detection misses/false-
# positives on the actual towel, run this file directly (python towel_grasp.py)
# against a saved frame and widen/narrow against what's actually seen, same
# as cube_detector.py's LOWER_RED_1 tuning note.
LOWER_PINK = (140, 30, 120)
UPPER_PINK = (175, 150, 255)

MIN_CONTOUR_AREA = 500  # px^2 - filters small pink noise/specular highlights


@dataclass
class GraspPoints:
    left: tuple[int, int]  # smaller-x end of the towel's long axis
    right: tuple[int, int]  # larger-x end


def detect_towel_grasp_points(bgr_frame: np.ndarray) -> GraspPoints | None:
    hsv = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, LOWER_PINK, UPPER_PINK)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    largest = max(contours, key=cv2.contourArea)
    if cv2.contourArea(largest) < MIN_CONTOUR_AREA:
        return None

    # minAreaRect's 4 box points form 2 long edges + 2 short edges. The short
    # edges cap the two ends of the long axis - their midpoints are "both ends
    # of the long side" (vs. the 2 long-edge corners, which are the corners of
    # one end each).
    box = cv2.boxPoints(cv2.minAreaRect(largest))
    edges = [(box[i], box[(i + 1) % 4]) for i in range(4)]
    edge_lens = [np.linalg.norm(p2 - p1) for p1, p2 in edges]
    short_edge_idxs = np.argsort(edge_lens)[:2]
    ends = [tuple(((p1 + p2) / 2).astype(int)) for i in short_edge_idxs for p1, p2 in [edges[i]]]

    # ponytail: left/right assigned by on-screen x only, assumes the camera's
    # left/right roughly matches the arms' left/right layout - swap here if a
    # rig's camera is mounted rotated/mirrored relative to that assumption.
    ends.sort(key=lambda p: p[0])
    return GraspPoints(left=ends[0], right=ends[1])


def draw_grasp_points(bgr_frame: np.ndarray, points: GraspPoints | None) -> np.ndarray:
    out = bgr_frame.copy()
    if points is None:
        return out
    cv2.circle(out, points.left, 8, (0, 255, 0), -1)
    cv2.putText(out, "L", (points.left[0] + 10, points.left[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    cv2.circle(out, points.right, 8, (255, 0, 255), -1)
    cv2.putText(out, "R", (points.right[0] + 10, points.right[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)
    cv2.line(out, points.left, points.right, (255, 255, 255), 1)
    return out


def _demo() -> None:
    """Self-test: a synthetic pink rectangle on a gray background should
    yield two endpoints near its actual long-axis ends."""
    frame = np.full((480, 640, 3), (120, 120, 120), dtype=np.uint8)
    # BGR pink swatch inside the LOWER_PINK/UPPER_PINK HSV range.
    cv2.rectangle(frame, (150, 220), (490, 260), (180, 130, 220), -1)

    points = detect_towel_grasp_points(frame)
    assert points is not None, "expected a detection on the synthetic pink rectangle"
    assert points.left[0] < points.right[0], "left/right not sorted by x"
    assert abs(points.left[0] - 150) < 20, f"left end x off: {points.left}"
    assert abs(points.right[0] - 490) < 20, f"right end x off: {points.right}"
    for p in (points.left, points.right):
        assert abs(p[1] - 240) < 20, f"endpoint y off-center: {p}"
    print("PASS: detect_towel_grasp_points on synthetic pink rectangle ->", points)


if __name__ == "__main__":
    _demo()
