import cv2
import numpy as np
from scipy.spatial import distance
from typing import List, Tuple
from config import CIRCLE_PERSISTENCE_FRAMES, CIRCLE_MATCH_DIST
from config import minDist
from utils import BallSmoother
from position_estimation import compute_circle_mask_score
from config import (
    CIRCLE_PERSISTENCE_FRAMES, CIRCLE_MATCH_DIST,
    CIRCLE_DRAW_SETTINGS,
    BALL_SMOOTHER_ALPHA,
)
from config import DEFAULT_SLIDER_VALUES  # Optional, for reference only!


try:
    from rclpy.logging import get_logger
    log = get_logger("detection")
except ImportError:
    class DummyLogger:
        def debug(self, msg): print("[DEBUG]", msg)
        def info(self, msg): print("[INFO]", msg)
        def warning(self, msg): print("[WARN]", msg)
    log = DummyLogger()

prev_balls = {"Red Ball": [], "Blue Ball": []}
smooth_balls = {"Red Ball": [], "Blue Ball": []}
verified_flags = {"Red Ball": [], "Blue Ball": []}

LABEL_MAP = {
    "Red Ball": "red",
    "Blue Ball": "blue"
}

_ball_smoothers = {
    "Red Ball": BallSmoother(alpha=BALL_SMOOTHER_ALPHA, max_lost=5, match_dist=CIRCLE_MATCH_DIST),
    "Blue Ball": BallSmoother(alpha=BALL_SMOOTHER_ALPHA, max_lost=5, match_dist=CIRCLE_MATCH_DIST),
}
_persistent_tracks = {
    "Red Ball": [],
    "Blue Ball": []
}

def detect_colors(
    frame_bgr: np.ndarray,
    # RED HSV 1
    R1_H_min: int, R1_H_max: int,
    # RED HSV 2
    R2_H_min: int, R2_H_max: int,
    # RED S/V
    R_S_min: int, R_S_max: int,
    R_V_min: int, R_V_max: int,
    # BLUE HSV
    B_H_min: int, B_H_max: int,
    B_S_min: int, B_S_max: int,
    B_V_min: int, B_V_max: int,
    # Morphology
    Kernel_Size: int, Open_Iter: int, Close_Iter: int
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Returns (red_mask, blue_mask) for the provided BGR frame.
    Parameters:
      R1_H_min, R1_H_max, R2_H_min, R2_H_max: int, red hue (low/high, second low/high)
      R_S_min, R_S_max, R_V_min, R_V_max: int, red saturation/value min/max
      B_H_min, B_H_max, B_S_min, B_S_max, B_V_min, B_V_max: int, blue min/max
      Kernel_Size: int, kernel size for morphological ops
      Open_Iter: int, open iterations
      Close_Iter: int, close iterations
    (parameter names and capitalization must match caller exactly)
    """
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    lower_red1 = (R1_H_min, R_S_min, R_V_min)
    upper_red1 = (R1_H_max, R_S_max, R_V_max)
    lower_red2 = (R2_H_min, R_S_min, R_V_min)
    upper_red2 = (R2_H_max, R_S_max, R_V_max)
    mask_red = cv2.inRange(hsv, lower_red1, upper_red1) + cv2.inRange(hsv, lower_red2, upper_red2)
    mask_blue = cv2.inRange(hsv, (B_H_min, B_S_min, B_V_min), (B_H_max, B_S_max, B_V_max))
    k = max(1, Kernel_Size)
    if k % 2 == 0:
        k += 1
    kernel = np.ones((k, k), np.uint8)
    mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel, iterations=Open_Iter)
    mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, kernel, iterations=Close_Iter)
    mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_OPEN, kernel, iterations=Open_Iter)
    mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_CLOSE, kernel, iterations=Close_Iter)
    return mask_red, mask_blue


def non_max_suppression_balls(
    balls: List[Tuple[int, int, int]], min_dist = int(minDist)
) -> List[Tuple[int, int, int]]:
    filtered = []
    for cx, cy, r in balls:
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
    smoothing_alpha: float,
    pi_mode: bool = False,
) -> List[Tuple[int, int, int]]:
    # --- Clamp all arguments to valid ranges for OpenCV HoughCircles ---
    if dp is None or dp <= 0:
        dp_val = 1.0
    else:
        dp_val = dp
    if minDist is None or minDist <= 0:
        minDist_val = 5.0
    else:
        minDist_val = minDist
    if param1 is None or param1 <= 0:
        param1_val = 50.0
    else:
        param1_val = param1
    if param2 is None or param2 <= 0:
        param2_val = 20.0
    else:
        param2_val = param2
    if minRadius is None or minRadius < 0:
        minR = 0
    else:
        minR = minRadius
    if maxRadius is None or maxRadius <= minR:
        maxR = minR + 1
    else:
        maxR = maxRadius

    blurred = cv2.GaussianBlur(mask, (9, 9), 2)

    circles = cv2.HoughCircles(
        blurred,
        cv2.HOUGH_GRADIENT,
        dp=dp_val,
        minDist=minDist_val,
        param1=param1_val,
        param2=param2_val,
        minRadius=minR,
        maxRadius=maxR
    )

    temp_list = []
    if circles is not None:
        c_ = np.round(circles[0]).astype(int)
        img_h, img_w = mask.shape[:2]
        margin = CIRCLE_DRAW_SETTINGS["DRAW_MARGIN"]
        for (cx, cy, r) in c_:
            if (margin <= cx - r < img_w - margin and
                margin <= cx + r < img_w - margin and
                margin <= cy - r < img_h - margin and
                margin <= cy + r < img_h - margin):
                if mask[cy, cx] > 0:
                    # Main threshold for acceptance
                    score = compute_circle_mask_score(mask, cx, cy, r)
                    if score > 0.65:
                        temp_list.append((cx, cy, r))

    expected_r = (minRadius + maxRadius) // 2
    nms_dist = max(15, expected_r // 2)
    detected = non_max_suppression_balls(temp_list, min_dist=nms_dist)

    _ball_smoothers[color_name].alpha = smoothing_alpha
    locked_balls = _ball_smoothers[color_name].smooth(color_name, detected)

    # Draw FAST colored circles only; never overlays/text.
    color = (0, 0, 255) if color_name == "Red Ball" else (255, 0, 0)
    for (cx, cy, r) in locked_balls:
        cv2.circle(frame, (int(cx), int(cy)), int(r), color, 2)
    return locked_balls