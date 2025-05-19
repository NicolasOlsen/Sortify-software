"""
Module: config.py
Handles: Global parameters, HSV/color ranges, detection and tracking thresholds
Owns: TRACK_TARGETS list, per-shape settings, slider config, KPI settings
Calls: None (pure config)
Does not contain: logic, stateful code, processing functions
"""

from typing import Dict, List, Tuple, Any
import logging
import cv2

Z_CORRECTION_BY_ROW = {
    "top": 0.0,
    "middle": 0.0,
    "bottom": 0.0,
}
TILT_CORRECTION_DEGREES = 27.5


TRACK_TARGETS = [
    {"shape": "circle", "color": "red"},
    {"shape": "circle", "color": "blue"},
    {"shape": "circle", "color": "green"},
    #{"shape": "square", "color": "red"},
    #{"shape": "square", "color": "green"},
    # {"shape": "square", "color": "blue"},
]

COLOR_HSV_RANGES: Dict[str, List[Tuple[Tuple[int, int, int], Tuple[int, int, int]]]] = {
    "red": [
        ((0, 135, 105), (10, 255, 255)),
        ((160, 135, 105), (180, 255, 255)),
    ],
    "blue": [
        ((100, 110, 60), (140, 255, 255)),
    ],
    "green": [
        ((40, 100, 60), (80, 255, 255)),
    ],
    "yellow": [
        ((20, 100, 100), (35, 255, 255)),
    ],
}

COLOR_BGR: Dict[str, Tuple[int, int, int]] = {
    "red": (0, 0, 255),
    "blue": (255, 0, 0),
    "green": (0, 255, 0),
    "yellow": (0, 255, 255),
}

# ------------- global flags -------------------------------
USE_GUI:        bool = True    # False → skip all cv2.imshow / windows
USE_TRACKBARS:  bool = False    # False → do not create or read trackbars
ENABLE_YOLO:    bool = False   # False → yolo_verification.run_yolo_inference() off


for t in TRACK_TARGETS:
    s, c = t["shape"], t["color"]
    assert c in COLOR_HSV_RANGES, f"No HSV range for {c}"
    assert c in COLOR_BGR, f"No BGR color for {c}"


GROUND_TRUTH_POS: Dict[Tuple[str, str], Tuple[float, float, float]] = {
    ("circle", "red"): (-139.0, 34.5, 752.0),
    ("circle", "blue"): (140.2, 34, 751.0),
    # ("square", "red"):  (0.0, 0.0, 500.0),
    # ("square", "blue"): (0.0, 0.0, 500.0),
}

LOGGING_MAX_FRAMES_DEFAULT: int = 10000
LOGGING_TOGGLE_KEY: int = ord("l")

LOGGING_OUTPUT_DIR: str = "/Users/azi/Desktop/Sortify/PI Code/tests"


POS_ERROR_THRESHOLD_MM: float = 50.0
MAX_ERROR_SNAPSHOTS: int = 3


LOG = logging.getLogger("sortify")
if not LOG.handlers:
    LOG.addHandler(logging.StreamHandler())
LOG.setLevel(logging.INFO)


MASK_OVERLAP_OK: float = 0.65
DEPTH_MIN_MM: int = 300
DEPTH_MAX_MM: int = 5000

DEPTH_SCALE_K = 1.0
DEPTH_OFFSET_MM = 0


ACTIVE_GROUPS = [
    "circle",
    # "square",
    "hsv_red",
    "hsv_blue",
    # "hsv_green",
    "morphology",
    "tracking",
    "kalman",
    "preprocessing",
    "evaluation",
]

SLIDER_CONFIG: Dict[str, Dict[str, Any]] = {
    # Red HSV Range
    "R1 H min": {"default": 0, "max": 20, "group": "hsv_red"},
    "R1 H max": {"default": 4, "max": 180, "group": "hsv_red"},
    "R2 H min": {"default": 170, "max": 180, "group": "hsv_red"},
    "R2 H max": {"default": 180, "max": 180, "group": "hsv_red"},
    "R S min": {"default": 100, "max": 255, "group": "hsv_red"},
    "R S max": {"default": 255, "max": 255, "group": "hsv_red"},
    "R V min": {"default": 20, "max": 255, "group": "hsv_red"},
    "R V max": {"default": 255, "max": 255, "group": "hsv_red"},
    # Blue HSV Range
    "B H min": {"default": 90, "max": 180, "group": "hsv_blue"},
    "B H max": {"default": 145, "max": 180, "group": "hsv_blue"},
    "B S min": {"default": 105, "max": 255, "group": "hsv_blue"},
    "B S max": {"default": 255, "max": 255, "group": "hsv_blue"},
    "B V min": {"default": 30, "max": 205, "group": "hsv_blue"},
    "B V max": {"default": 255, "max": 255, "group": "hsv_blue"},
    # Green HSV Range
    "G H min": {"default": 40, "max": 80, "group": "hsv_green"},
    "G H max": {"default": 80, "max": 80, "group": "hsv_green"},
    "G S min": {"default": 100, "max": 255, "group": "hsv_green"},
    "G S max": {"default": 255, "max": 255, "group": "hsv_green"},
    "G V min": {"default": 35, "max": 255, "group": "hsv_green"},
    "G V max": {"default": 255, "max": 255, "group": "hsv_green"},
    # Yellow HSV Range
    "Y H min": {"default": 20, "max": 35, "group": "hsv_yellow"},
    "Y H max": {"default": 35, "max": 80, "group": "hsv_yellow"},
    "Y S min": {"default": 100, "max": 255, "group": "hsv_yellow"},
    "Y S max": {"default": 255, "max": 255, "group": "hsv_yellow"},
    "Y V min": {"default": 100, "max": 255, "group": "hsv_yellow"},
    "Y V max": {"default": 255, "max": 255, "group": "hsv_yellow"},
    # Morphology
    "Kernel Size": {"default": 0, "max": 31, "group": "morphology"},
    "Open Iter": {"default": 0, "max": 10, "group": "morphology"},
    "Close Iter": {"default": 0, "max": 20, "group": "morphology"},
    # Circle Detection
    "dp": {"default": 20, "max": 30, "group": "circle"},
    "minDist": {"default": 40, "max": 200, "group": "circle"},
    "param1": {"default": 200, "max": 255, "group": "circle"},
    "param2": {"default": 65, "max": 255, "group": "circle"},
    "minRadius": {"default": 40, "max": 200, "group": "circle"},
    "maxRadius": {"default": 90, "max": 200, "group": "circle"},
    "Min Circle Area PX": {"default": 2500, "max": 10000, "group": "circle"},
    "Circle Mask Score Thresh": {"default": 60, "max": 100, "group": "circle"},
    "Circle Min Circularity": {"default": 60, "max": 100, "group": "circle"},
    "Circle IoU Thresh": {"default": 60, "max": 100, "group": "circle"},
    # Square Detection
    "S Min Solidity": {"default": 90, "max": 100, "group": "square"},
    "S Min Extent": {"default": 80, "max": 100, "group": "square"},
    "S Min Area": {"default": 5000, "max": 50000, "group": "square"},
    "S Max Area": {"default": 80000, "max": 100000, "group": "square"},
    "S Epsilon": {"default": 120, "max": 500, "group": "square"},
    "S Aspect Min": {"default": 60, "max": 100, "group": "square"},
    "S Aspect Max": {"default": 110, "max": 150, "group": "square"},
    "S Max Circularity": {"default": 40, "max": 100, "group": "square"},
    "Square IoU Thresh": {"default": 60, "max": 100, "group": "square"},
    "Color Weight": {"default": 1, "max": 5, "group": "evaluation"},
    "Shape Weight": {"default": 1, "max": 5, "group": "evaluation"},
    "AI Weight": {"default": 1, "max": 5, "group": "evaluation"},
    "Depth Weight": {"default": 1, "max": 5, "group": "evaluation"},
    "Depth Mask Weight": {"default": 1, "max": 5, "group": "evaluation"},
    "Tracker Weight": {"default": 1, "max": 5, "group": "evaluation"},
    "Decision Accept Threshold": {"default": 3, "max": 10, "group": "evaluation"},
    # Focus and Tracking
    "Focus": {"default": 130, "max": 200, "group": "camera"},
    "TRAlpha": {"default": 5, "max": 100, "group": "tracking"},
    "MatchDist": {"default": 1500, "max": 5000, "group": "tracking"},
    "MaxLost": {"default": 30, "max": 30, "group": "tracking"},
    "SpawnPersist": {"default": 0, "max": 30, "group": "tracking"},
    "SpeedGain": {"default": 0, "max": 30, "group": "tracking"},
    "StableAge": {"default": 2, "max": 30, "group": "tracking"},
    "KF Q 2D": {"default": 2, "max": 50, "group": "kalman"},
    "KF R 2D": {"default": 5, "max": 50, "group": "kalman"},
    "KF Q 3D": {"default": 10, "max": 50, "group": "kalman"},
    "KF R 3D": {"default": 9, "max": 100, "group": "kalman"},
    # Preprocessing
    "CLAHE Clip": {"default": 0, "max": 31, "group": "preprocessing"},
    "Gaussian K": {"default": 5, "max": 31, "group": "preprocessing"},
    "Gaussian Sigma": {"default": 0, "max": 100, "group": "preprocessing"},
    "Grayscale": {"default": 1, "max": 1, "group": "preprocessing"},
}
SLIDER_MAX_VALUES: List[int] = [v["max"] for v in SLIDER_CONFIG.values()]


CAMERA_WIDTH: int = 1280
CAMERA_HEIGHT: int = 800
CAMERA_FPS: int = 60
CAMERA_PREVIEW_SIZE: Tuple[int, int] = (CAMERA_WIDTH, CAMERA_HEIGHT)
CAMERA_VIDEO_SIZE: Tuple[int, int] = (CAMERA_WIDTH, CAMERA_HEIGHT)
CAMERA_STILL_SIZE: Tuple[int, int] = (1920, 1080)
CAMERA_DEPTH_RESOLUTION: str = "THE_800_P"
CAMERA_CONFIDENCE_THRESHOLD: int = 200
CAMERA_MEDIAN_FILTER: str = "KERNEL_5x5"
CAMERA_STEREO_ALIGNMENT: str = "RGB"

TRACKBAR_WINDOW: str = "Trackbars"
TRACKBAR_MAX: int = 255
CIRCLE_DRAW_SETTINGS: Dict[str, Any] = {
    "COLOR_CONFIRMED": (250, 255, 231),
    "COLOR_CANDIDATE": (210, 255, 255),
    "COLOR_REJECTED": (88, 95, 115),
    "COLOR_RED_BALL": (188, 130, 224),
    "COLOR_BLUE_BALL": (255, 235, 110),
    "COLOR_SCORE_AUX": (220, 255, 250),
    "COLOR_GRAY": (58, 66, 78),
    "DRAW_THICKNESS_MAIN": 2,
    "DRAW_THICKNESS_AUX": 2,
    "FONT": cv2.FONT_HERSHEY_SIMPLEX,
    "FONT_SCALE": 0.54,
    "FONT_SCALE_SCORE": 0.44,
    "FONT_THICKNESS": 2,
    "DRAW_MARGIN": 3,
    "SCORE_THRESH_CONFIRMED": 0.80,
    "SCORE_THRESH_CANDIDATE": 0.45,
}

SHAPE_MATCH_DIST: int = 30

YOLO_CFG: str = "/Users/azi/Desktop/Sortify/darknet/cfg/yolov4-tiny-640x360.cfg"
YOLO_WEIGHTS: str = (
    "/Users/azi/Desktop/Sortify/darknet/backup/yolov4-tiny-640x360_last.weights"
)
YOLO_DATA: str = "/Users/azi/Desktop/Sortify/darknet/data/obj.data"
