# position_estimation.py

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
    name, mask, frame, fx, fy, cx0, cy0, depth_frame, balls, pi_mode=False
):
    """
    Args:
        name: e.g. "Red Ball" or "Blue Ball"
        mask: binary mask for the ball color
        frame: BGR frame (may be annotated)
        fx, fy, cx0, cy0: calibration params
        depth_frame: aligned depth image
        balls: list of (cx, cy, r) from BallSmoother for this frame
        pi_mode: if True, disables any overlay
    Returns:
        list of (score, cx, cy, r, x_mm, y_mm, z_mm)
    """
    results = []
    color = (0, 0, 255) if name == "Red Ball" else (255, 0, 0)

    for i, (cx, cy, r) in enumerate(balls):
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

        results.append((score, cxi, cyi, ri, x_mm, y_mm, z_mm))

        # Draw only a simple circle, in blue or red
        cv2.circle(frame, (cxi, cyi), ri, color, 2)

    return results