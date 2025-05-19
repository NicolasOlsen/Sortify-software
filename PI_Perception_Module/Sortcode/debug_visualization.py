"""
Module: debug_visualization.py
Handles: on-screen drawing & debug view assembly
Owns:   * draw_detections() – per-object overlay
        * build_debug_view() – 2×2 composite debug window
Calls: cv2, numpy, config
Does not contain: detection, tracking, KPI logic
"""

from typing import List, Dict, Any
import cv2
import numpy as np
from config import COLOR_BGR


def draw_detections(img: np.ndarray, detections: List[Dict[str, Any]]) -> None:
    for det in detections:
        color = COLOR_BGR.get(det.get("color", "").lower(), (200, 200, 200))
        shape = det.get("shape", "")
        d = det.get("data", det)

        cx, cy = int(round(d.get("cx", 0))), int(round(d.get("cy", 0)))
        label = f"{shape} {d.get('track_id', '?')}"

        if shape == "circle":
            r = int(round(d.get("r", 0)))
            if r > 0:
                cv2.circle(img, (cx, cy), r, color, 2)
        elif shape == "square":
            if all(k in d for k in ("w", "h", "angle")):
                rect = ((cx, cy), (d["w"], d["h"]), d["angle"])
                box = cv2.boxPoints(rect).astype(int)
                cv2.drawContours(img, [box], -1, color, 2)

        cv2.putText(
            img, label, (cx + 8, cy - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2
        )

        if all(k in d for k in ("x_mm", "y_mm", "z_mm")):
            xyz = f"X:{d['x_mm']:.0f}  Y:{d['y_mm']:.0f}  Z:{d['z_mm']:.0f}"
            cv2.putText(
                img,
                xyz,
                (cx - 60, cy + 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                1,
            )


def build_debug_view(
    preproc: np.ndarray,           
    red_mask: np.ndarray,
    blue_mask: np.ndarray,
    overlay: np.ndarray,
) -> np.ndarray:

    h, w = preproc.shape[:2]

    def to_bgr(img: np.ndarray) -> np.ndarray:
        if img.ndim == 2:                            
            img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        return cv2.resize(img, (w, h))

    pane0   = to_bgr(preproc)
    red_bgr = to_bgr(red_mask)
    blue_bgr= to_bgr(blue_mask)
    overlay = to_bgr(overlay)

    top    = np.hstack((pane0,   red_bgr))
    bottom = np.hstack((blue_bgr, overlay))
    return np.vstack((top, bottom))
