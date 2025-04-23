import cv2
import numpy as np
from scipy.spatial import distance
from typing import List, Tuple
from config import CIRCLE_PERSISTENCE_FRAMES, CIRCLE_MATCH_DIST
from utils import BallSmoother
from position_estimation import compute_circle_mask_score

from config import (
    CIRCLE_PERSISTENCE_FRAMES, CIRCLE_MATCH_DIST,
    CIRCLE_DRAW_SETTINGS        # <-- Add this
)

try:
    from rclpy.logging import get_logger
    log = get_logger("detection")
except ImportError:
    class DummyLogger:
        def debug(self, msg): print("[DEBUG]", msg)
        def info(self, msg): print("[INFO]", msg)
        def warning(self, msg): print("[WARN]", msg)
    log = DummyLogger()

prev_circles = {"Red Ball": [], "Blue Ball": []}
smooth_circles = {"Red Ball": [], "Blue Ball": []}
verified_flags = {"Red Ball": [], "Blue Ball": []}
LABEL_MAP = {
    "Red Ball": "red",
    "Blue Ball": "blue"
}
_persistent_tracks = {
    "Red Ball": [],
    "Blue Ball": []
}

def detect_colors(
    frame_bgr: np.ndarray,
    r1_h_min: int, r1_h_max: int,
    r2_h_min: int, r2_h_max: int,
    r_s_min: int, r_s_max: int,
    r_v_min: int, r_v_max: int,
    b_h_min: int, b_h_max: int,
    b_s_min: int, b_s_max: int,
    b_v_min: int, b_v_max: int,
    kernel_size: int, open_iter: int, close_iter: int
) -> Tuple[np.ndarray, np.ndarray]:
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    lower_red1 = (r1_h_min, r_s_min, r_v_min)
    upper_red1 = (r1_h_max, r_s_max, r_v_max)
    lower_red2 = (r2_h_min, r_s_min, r_v_min)
    upper_red2 = (r2_h_max, r_s_max, r_v_max)
    mask_red = cv2.inRange(hsv, lower_red1, upper_red1) + cv2.inRange(hsv, lower_red2, upper_red2)
    mask_blue = cv2.inRange(hsv, (b_h_min, b_s_min, b_v_min), (b_h_max, b_s_max, b_v_max))
    k = max(1, kernel_size)
    if k % 2 == 0:
        k += 1
    kernel = np.ones((k, k), np.uint8)
    mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel, iterations=open_iter)
    mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, kernel, iterations=close_iter)
    mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_OPEN, kernel, iterations=open_iter)
    mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_CLOSE, kernel, iterations=close_iter)
    return mask_red, mask_blue

def non_max_suppression_circles(
    circles: List[Tuple[int, int, int]], min_dist: int = 75
) -> List[Tuple[int, int, int]]:
    filtered = []
    for cx, cy, r in circles:
        if all(np.hypot(cx - fx, cy - fy) >= min_dist for fx, fy, _ in filtered):
            filtered.append((cx, cy, r))
    return filtered

def _persistent_circles(
    color_name: str, detected: List[Tuple[int, int, int]]
) -> List[Tuple[int, int, int]]:
    tracks = _persistent_tracks[color_name]
    for i in range(len(tracks)):
        tracks[i][3] -= 0.3
    tracks[:] = [t for t in tracks if t[3] > 0]
    for ncx, ncy, nr in detected:
        found = False
        for t in tracks:
            tcx, tcy, tr, count = t
            if np.hypot(ncx - tcx, ncy - tcy) < CIRCLE_MATCH_DIST:
                t[0], t[1], t[2], t[3] = ncx, ncy, nr, min(count+1, CIRCLE_PERSISTENCE_FRAMES+1)
                found = True
                break
        if not found:
            tracks.append([ncx, ncy, nr, 1])
    return [(cx, cy, r) for (cx, cy, r, count) in tracks if count >= CIRCLE_PERSISTENCE_FRAMES]

def detect_and_label_circles(
    mask: np.ndarray,
    color_name: str,
    frame: np.ndarray,
    dp: float,
    minDist: float,
    param1: float,
    param2: float,
    minRadius: int,
    maxRadius: int,
    smoothing_alpha: float
) -> List[Tuple[int, int, int]]:
    global prev_circles, smooth_circles, verified_flags
    dp_val = max(1, dp)
    blurred = cv2.GaussianBlur(mask, (9, 9), 2)
    minR = min(minRadius, maxRadius - 1)
    maxR = max(minRadius + 1, maxRadius)
    circles = cv2.HoughCircles(
        blurred,
        cv2.HOUGH_GRADIENT,
        dp=dp_val,
        minDist=minDist,
        param1=param1,
        param2=param2,
        minRadius=minR,
        maxRadius=maxR
    )
    temp_list = []
    if circles is not None:
        c_ = np.round(circles[0]).astype(int)
        img_h, img_w = mask.shape[:2]
        margin = CIRCLE_DRAW_SETTINGS["DRAW_MARGIN"]  # Amount inside edges you require

        for (cx, cy, r) in c_:
            # Only accept circle if entire circle fits in image (+ margin)
            if (margin <= cx - r < img_w - margin and
                margin <= cx + r < img_w - margin and
                margin <= cy - r < img_h - margin and
                margin <= cy + r < img_h - margin):
                if mask[cy, cx] > 0:
                    score = compute_circle_mask_score(mask, cx, cy, r)
                    if score > 0.65:
                        temp_list.append((cx, cy, r))
                    else:
                        cv2.circle(frame, (cx, cy), r, CIRCLE_DRAW_SETTINGS["COLOR_GRAY"], CIRCLE_DRAW_SETTINGS["DRAW_THICKNESS_MAIN"])
                        cv2.putText(frame, f"{score:.2f}", (cx, cy - 10),
                                    CIRCLE_DRAW_SETTINGS["FONT"], CIRCLE_DRAW_SETTINGS["FONT_SCALE_SCORE"], CIRCLE_DRAW_SETTINGS["COLOR_GRAY"], 1)
                        
    expected_r = (minRadius + maxRadius) // 2
    nms_dist = max(15, expected_r // 2)
    new_list = non_max_suppression_circles(temp_list, min_dist=nms_dist)
    last_smooth = smooth_circles[color_name]
    last_flags = verified_flags[color_name]
    smoothed = []
    new_flags = []
    used_indices = set()
    for idx, (lx, ly, lr) in enumerate(last_smooth):
        candidates = [(i, x, y, rr)
                      for i, (x, y, rr) in enumerate(new_list)
                      if i not in used_indices]
        if not candidates:
            continue
        i_best, nx, ny, nr = min(
            candidates,
            key=lambda c: distance.euclidean((lx, ly), (c[1], c[2]))
        )
        used_indices.add(i_best)
        sx = int(lx * (1 - smoothing_alpha) + nx * smoothing_alpha)
        sy = int(ly * (1 - smoothing_alpha) + ny * smoothing_alpha)
        sr = int(lr * (1 - smoothing_alpha) + nr * smoothing_alpha)
        smoothed.append((sx, sy, sr))
        new_flags.append(last_flags[idx])
    for i, (cx, cy, rr) in enumerate(new_list):
        if i not in used_indices:
            smoothed.append((cx, cy, rr))
            new_flags.append(False)
    smooth_circles[color_name] = smoothed
    verified_flags[color_name] = new_flags
    prev_circles[color_name] = smoothed
    stable = _persistent_circles(color_name, smoothed)
    for (cx, cy, r) in stable:
        cv2.circle(frame, (int(cx), int(cy)), int(r), CIRCLE_DRAW_SETTINGS["COLOR_CONFIRMED"], CIRCLE_DRAW_SETTINGS["DRAW_THICKNESS_MAIN"])
        score = compute_circle_mask_score(mask, int(cx), int(cy), int(r))
        cv2.putText(frame, f"{score:.2f}", (int(cx), int(cy)-10),
                    CIRCLE_DRAW_SETTINGS["FONT"], CIRCLE_DRAW_SETTINGS["FONT_SCALE_SCORE"], CIRCLE_DRAW_SETTINGS["COLOR_SCORE_AUX"], 1)
    return stable