import cv2
from typing import Dict, List, Tuple
from config import TRACKBAR_WINDOW, DEFAULT_SLIDER_VALUES, SLIDER_MAX_VALUES

WHITEBALANCE_MIN = 0  # Or whatever you want
WHITEBALANCE_NAME = "WhiteBalance"

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
        # Set minimum value for white balance if supported
        if name == WHITEBALANCE_NAME:
            try:
                cv2.setTrackbarMin(name, TRACKBAR_WINDOW, WHITEBALANCE_MIN)
            except AttributeError:
                pass  # Not available in some OpenCV versions

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
    # Only add Oak ISP Mode if it's an actual trackbar
    try:
        results["Oak ISP Mode"] = cv2.getTrackbarPos("Oak ISP Mode", TRACKBAR_WINDOW)
    except cv2.error:
        # Optional: Skip if that trackbar isn't present
        pass
    return results