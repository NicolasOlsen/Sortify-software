"""
Module: position_estimation.py
Purpose: Convert 2-D detections + aligned depth map → 3-D camera-space coords
NOTE:   • No depth scaling raw millimetres straight from the device
        • Only DEPTH_OFFSET_MM is added to compensate for any fixed bias
"""

from typing import Tuple, Optional, Dict
import cv2
import numpy as np
from config import (
    MASK_OVERLAP_OK,
    DEPTH_MIN_MM,
    DEPTH_MAX_MM,
    DEPTH_OFFSET_MM,
)
from config import TILT_CORRECTION_DEGREES
import math


class PositionEstimator:

    @staticmethod
    def to_3d(
        u: float, v: float, z: float, fx: float, fy: float, cx: float, cy: float
    ) -> Tuple[float, float, float]:
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy

        angle_rad = math.radians(-TILT_CORRECTION_DEGREES)

        y_rot = y * math.cos(angle_rad) - z * math.sin(angle_rad)
        z_rot = y * math.sin(angle_rad) + z * math.cos(angle_rad)

        return float(x), float(y_rot), float(z_rot)


    @staticmethod
    def mask_overlap(mask: np.ndarray, cx: int, cy: int, r: int) -> float:
        if r <= 0:
            return 0.0
        h, w = mask.shape[:2]
        circ = np.zeros((h, w), dtype=np.uint8)
        cv2.circle(circ, (cx, cy), r, 255, -1)
        overlap = cv2.bitwise_and(mask, circ)
        area_circ = cv2.countNonZero(circ)
        area_inter = cv2.countNonZero(overlap)
        return (area_inter / area_circ) if area_circ else 0.0

    @staticmethod
    def estimate_position(
        det: Dict,
        depth_map: np.ndarray,
        fx: float,
        fy: float,
        cx0: float,
        cy0: float,
        mask: Optional[np.ndarray] = None,
    ) -> Dict:

        if not det or any(k not in det for k in ("cx", "cy", "r")):
            return PositionEstimator.empty_result()

        cx, cy, r = map(int, map(round, (det["cx"], det["cy"], det["r"])))
        h, w = depth_map.shape
        if not (0 <= cx < w and 0 <= cy < h) or r <= 3:
            return PositionEstimator.empty_result()

        roi = np.zeros_like(depth_map, dtype=np.uint8)
        cv2.circle(roi, (cx, cy), r, 255, -1)
        if mask is not None:
            roi = cv2.bitwise_and(roi, mask.astype(np.uint8))

        valid_depth = (depth_map >= DEPTH_MIN_MM) & (depth_map <= DEPTH_MAX_MM)
        depth_vals = depth_map[(roi > 0) & valid_depth]
        if depth_vals.size < 20:
            return PositionEstimator.empty_result()

        med = float(np.median(depth_vals))
        mad = float(np.median(np.abs(depth_vals - med))) or 1.0
        #inliers = np.abs(depth_vals - med) < DEPTH_OUTLIER_K * mad
        #depth_vals = depth_vals[inliers]
        if depth_vals.size < 15:
            return PositionEstimator.empty_result()

        z_mm = float(np.median(depth_vals)) + DEPTH_OFFSET_MM

        m = cv2.moments(roi, binaryImage=True)
        if m["m00"] == 0:
            return PositionEstimator.empty_result()
        u = m["m10"] / m["m00"]
        v = m["m01"] / m["m00"]

        x_mm, y_mm, z_mm = PositionEstimator.to_3d(u, v, z_mm, fx, fy, cx0, cy0)

        overlap = PositionEstimator.mask_overlap(mask if mask is not None else roi, cx, cy, r)

        return {
            "x_mm": x_mm,
            "y_mm": y_mm,
            "z_mm": z_mm,
            "depth_valid": DEPTH_MIN_MM <= z_mm <= DEPTH_MAX_MM,
            "depth_mask_valid": overlap >= MASK_OVERLAP_OK,
        }

    @staticmethod
    def empty_result() -> Dict:
        return {
            "x_mm": 0.0,
            "y_mm": 0.0,
            "z_mm": 0.0,
            "depth_valid": False,
            "depth_mask_valid": False,
        }


estimate_position = PositionEstimator.estimate_position
to_3d             = PositionEstimator.to_3d
mask_overlap      = PositionEstimator.mask_overlap
