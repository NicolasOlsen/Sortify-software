"""
Module: gui_interface.py
Trackbar creation & live parameter readback.

This module centralises all interaction with OpenCV trackbars so that the
rest of the pipeline can work headless when the GUI is disabled.
"""

from __future__ import annotations

from typing import Any, Dict, List

import cv2

from config import (
    SLIDER_CONFIG,
    TRACKBAR_WINDOW,
    ACTIVE_GROUPS,
    USE_GUI,
    USE_TRACKBARS,
)

__all__ = ["create_trackbars", "get_runtime_params"]

_GROUPS_FALLBACK = ACTIVE_GROUPS


def _noop(_: int) -> None:  # OpenCV callback placeholder
    ...


# --------------------------------------------------------------------------- #
# Helpers
# --------------------------------------------------------------------------- #
def create_trackbars(
    cfg: Dict[str, Dict[str, Any]] | None = None,
    groups: List[str] | None = None,
) -> None:

    if not (USE_GUI and USE_TRACKBARS):
        return 

    cfg = cfg or SLIDER_CONFIG
    groups = groups or _GROUPS_FALLBACK

    cv2.namedWindow(TRACKBAR_WINDOW, cv2.WINDOW_NORMAL)

    for name, settings in cfg.items():
        if settings.get("group") not in groups:
            continue
        cv2.createTrackbar(
            name,
            TRACKBAR_WINDOW,
            int(settings.get("default", 0)),
            int(settings.get("max", 100)),
            _noop,
        )


def get_runtime_params(groups: List[str] | None = None) -> Dict[str, Any]:
    groups = groups or _GROUPS_FALLBACK

    if not (USE_GUI and USE_TRACKBARS):
        return {
            name: cfg["default"]
            for name, cfg in SLIDER_CONFIG.items()
            if cfg.get("group") in groups
        }

    params: Dict[str, Any] = {}
    for name, cfg in SLIDER_CONFIG.items():
        if cfg.get("group") not in groups:
            continue
        try:
            params[name] = cv2.getTrackbarPos(name, TRACKBAR_WINDOW)
        except cv2.error:
            params[name] = cfg["default"]

    return params
