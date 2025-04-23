import cv2
import numpy as np
from utils import BallSmoother

from config import CIRCLE_DRAW_SETTINGS

try:
    from rclpy.logging import get_logger
    log = get_logger("module_name")
except ImportError:
    class DummyLogger:
        def debug(self, msg): print("[DEBUG]", msg)
        def info(self, msg): print("[INFO]", msg)
        def warning(self, msg): print("[WARN]", msg)
        def error(self, msg): print("[ERROR]", msg)
    log = DummyLogger()

def reproject_to_3d(u, v, depth_mm, fx, fy, cx, cy):
    X = -(u - cx) * depth_mm / fx
    Y = -(v - cy) * depth_mm / fy
    Z = depth_mm
    return X, Y, Z

def compute_circle_mask_score(mask, cx, cy, r):
    h, w = mask.shape
    Y, X = np.ogrid[:h, :w]
    dist = (X - cx) ** 2 + (Y - cy) ** 2
    circle_mask = dist <= r ** 2
    inside = mask[circle_mask]
    denom = np.count_nonzero(circle_mask)
    return float(np.sum(inside > 0)) / denom if denom > 0 else 0.0

def evaluate_ball(
    name, mask, frame, fx, fy, cx0, cy0, depth_frame, circles
):
    """
    Args:
        name: e.g. "Red Ball" or "Blue Ball"
        mask: binary mask for the ball color
        frame: BGR frame (annotated here)
        fx, fy, cx0, cy0: calibration params
        depth_frame: aligned depth image
        circles: list of (cx, cy, r) from BallSmoother for this frame
    Returns:
        list of (score, cx, cy, r, x_mm, y_mm, z_mm)
    """
    results = []
    inner_color = CIRCLE_DRAW_SETTINGS["COLOR_RED_BALL"] if name == "Red Ball" else CIRCLE_DRAW_SETTINGS["COLOR_BLUE_BALL"]
    for i, (cx, cy, r) in enumerate(circles):
        cxi = int(round(cx))
        cyi = int(round(cy))
        ri = int(round(r))
        score = compute_circle_mask_score(mask, cxi, cyi, ri)
        patch = depth_frame[max(0, cyi - 3):cyi + 4, max(0, cxi - 3):cxi + 4]
        valid = patch[patch > 0]
        depth = int(np.median(valid)) if len(valid) > 0 else -1
        x_mm, y_mm, z_mm = (0, 0, 0)
        if 0 < depth < 4000:
            x_mm, y_mm, z_mm = reproject_to_3d(cxi, cyi, depth, fx, fy, cx0, cy0)
        if score > CIRCLE_DRAW_SETTINGS["SCORE_THRESH_CONFIRMED"]:
            outer_color = CIRCLE_DRAW_SETTINGS["COLOR_CONFIRMED"]
        elif score > CIRCLE_DRAW_SETTINGS["SCORE_THRESH_CANDIDATE"]:
            outer_color = CIRCLE_DRAW_SETTINGS["COLOR_CANDIDATE"]
        else:
            outer_color = CIRCLE_DRAW_SETTINGS["COLOR_REJECTED"]

        cv2.circle(
            frame,
            (cxi, cyi),
            ri + CIRCLE_DRAW_SETTINGS["DRAW_THICKNESS_AUX"],
            outer_color,
            CIRCLE_DRAW_SETTINGS["DRAW_THICKNESS_AUX"]
        )
        cv2.circle(
            frame,
            (cxi, cyi),
            ri,
            inner_color,
            CIRCLE_DRAW_SETTINGS["DRAW_THICKNESS_MAIN"]
        )
        text = (
            f"Score={score:.2f} | X={int(x_mm)} Y={int(y_mm)} Z={int(z_mm)}"
            if z_mm > 0 else f"Score={score:.2f}"
        )
        org = (cxi - ri - 100, cyi - ri - 20)
        cv2.putText(
            frame,
            text,
            org,
            CIRCLE_DRAW_SETTINGS["FONT"],
            CIRCLE_DRAW_SETTINGS["FONT_SCALE"],
            (255, 255, 255),
            CIRCLE_DRAW_SETTINGS["FONT_THICKNESS"]
        )
        results.append((score, cxi, cyi, ri, x_mm, y_mm, z_mm))
    return results