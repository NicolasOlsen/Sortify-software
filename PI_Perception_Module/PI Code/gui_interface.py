"""
Module: gui_interface.py
Track-bar creation & live parameter read-back
(No per-frame slider reset → smoother UI)
"""

from __future__ import annotations
from typing import Dict, List, Any
import cv2

from config import SLIDER_CONFIG, TRACKBAR_WINDOW, ACTIVE_GROUPS, USE_GUI


def create_trackbars(cfg: Dict[str, Dict[str, Any]], groups: List[str] | None = None):
    cv2.namedWindow(TRACKBAR_WINDOW, cv2.WINDOW_NORMAL)

    def _dummy(_): ...

    for name, settings in cfg.items():
        if groups is None or settings.get("group") in groups:
            cv2.createTrackbar(
                name,
                TRACKBAR_WINDOW,
                int(settings.get("default", 0)),
                int(settings.get("max", 100)),
                _dummy,
            )


"""
Centralised trackbar reader.
Nothing else in the project is allowed to call cv2.getTrackbarPos().
"""
from typing import Dict, Any
import cv2
from config import SLIDER_CONFIG, TRACKBAR_WINDOW, USE_TRACKBARS

_DEFAULT = {name: cfg["default"] for name, cfg in SLIDER_CONFIG.items()}

def get_runtime_params() -> Dict[str, Any]:
    if not USE_TRACKBARS:
        return dict(_DEFAULT)

    out: Dict[str, Any] = {}
    for name, cfg in SLIDER_CONFIG.items():
        try:
            out[name] = cv2.getTrackbarPos(name, TRACKBAR_WINDOW)
        except cv2.error:
            out[name] = cfg["default"]
        return {
            k: cv2.getTrackbarPos(k, TRACKBAR_WINDOW) if USE_GUI else SLIDER_CONFIG[k]["default"]
            for k in SLIDER_CONFIG
            if SLIDER_CONFIG[k].get("group") in ACTIVE_GROUPS
        }

