# gui_interface.py

import cv2
from typing import Dict, List, Tuple
from config import TRACKBAR_WINDOW, DEFAULT_SLIDER_VALUES, SLIDER_MAX_VALUES

def create_trackbars(sliders: List[Tuple[str, int]], maxes: List[int]) -> None:
    """
    Creates trackbars in a named OpenCV window for parameter tuning.
    Args:
        sliders: List of (name, default_value) pairs for trackbars.
        maxes: List of maximum values, same order as sliders.
    """
    def _dummy_callback(_x): pass
    cv2.namedWindow(TRACKBAR_WINDOW, cv2.WINDOW_NORMAL)
    for (name, value), max_val in zip(sliders, maxes):
        cv2.createTrackbar(name, TRACKBAR_WINDOW, value, max_val, _dummy_callback)

def read_trackbars(sliders: List[Tuple[str, int]]) -> Dict[str, int]:
    """
    Reads current trackbar positions and returns them in a dictionary.
    Args:
        sliders: List of (name, default_value) pairs for trackbars.
    Returns:
        Dictionary of parameter_name -> current_value.
    """
    results = {}
    for name, _ in sliders:
        results[name] = cv2.getTrackbarPos(name, TRACKBAR_WINDOW)
    # Optionally try to read nonstandard trackbars (ignore if not present)
    for extra in ["Oak ISP Mode", "CLAHE Clip", "Gaussian K", "Grayscale"]:
        try:
            results[extra] = cv2.getTrackbarPos(extra, TRACKBAR_WINDOW)
        except cv2.error:
            pass
    return results