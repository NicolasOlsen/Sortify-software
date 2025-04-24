# config.py

import cv2

# ---- GUI defaults and trackbar ranges ----
DEFAULT_SLIDER_VALUES = [
    ("R1 H min", 0),   ("R1 H max", 10),
    ("R2 H min", 160), ("R2 H max", 180),
    ("R S min", 135),  ("R S max", 255),
    ("R V min", 105),  ("R V max", 255),
    ("B H min", 100),  ("B H max", 140),
    ("B S min", 110),  ("B S max", 255),
    ("B V min", 60),   ("B V max", 255),
    ("Kernel Size", 5),("Open Iter", 5),
    ("Close Iter", 1), ("dp (x0.1)", 13),
    ("minDist", 35),   ("param1", 170),
    ("param2", 42),    ("minRadius", 35),
    ("maxRadius", 200),("Focus", 131),
    ("Alpha(x100)", 70),("Contrast", 0),
    ("Saturation", 2), ("Sharpness", 2),
    ("Brightness", 2), ("WhiteB", 2700),
    # New for preprocessing Pi Mode/trackbars:
    ("CLAHE Clip", 2), ("Gaussian K", 5),
    ("Grayscale", 0)
]
SLIDER_MAX_VALUES = [
    20,180, 180,180, 255,255, 255,255, 180,180,255,255,205,255,31,10,
    20,30, 200,255,100,120, 200,255,300,10,10,10,10,8500,
    10, 31, 1
]

minDist = next(v for k, v in DEFAULT_SLIDER_VALUES if k == "minDist")


TRACKBAR_WINDOW = "Trackbars"
TRACKBAR_MAX = 255

# -------- SYSTEM/PROCESSING ----------
USE_HARDWARE_PREPROCESSING = False
CIRCLE_PERSISTENCE_FRAMES = 5
CIRCLE_MATCH_DIST = 100
BALL_SMOOTHER_ALPHA = 0.25
PI_MODE = True  # Set to True for Pi performance, or use a script/env switch if you want.

# ---------- YOLO -------------
YOLO_CFG = "cfg/yolov4-tiny.cfg"
YOLO_WEIGHTS = "backup/yolov4-tiny_6obj_last.weights"
YOLO_DATA = "data/obj.data"
ENABLE_YOLO = True

HSV_SNAPSHOT_FOLDER = "./HSV_SNAPSHOTS"

# ------------ DRAWING/OVERLAY SETTINGS ----------
BALL_GLOW_COLOR_RED = (180, 100, 40)
BALL_GLOW_COLOR_BLUE = (255, 250, 230)
BALL_GLOW_THICKNESS = 4
BALL_GLOW_INTENSITY = 0.10
TRAIL_MAX_LENGTH = 10
TRAIL_START_WIDTH = 4
TRAIL_END_WIDTH = 1
TRAIL_COLOR_ALPHA = 0.11

STATUS_BAR_RECT_COLOR = (46, 54, 78)
STATUS_BAR_GLOW_COLOR = (255, 225, 120)
STATUS_BAR_ALPHA = 0.82
STATUS_BAR_GLOW_RADIUS = 2
STATUS_BAR_HEIGHT = 52
STATUS_BAR_TOP_MARGIN = 10
STATUS_BAR_SIDE_MARGIN = 16
STATUS_FONT = cv2.FONT_HERSHEY_TRIPLEX
STATUS_FONT_SCALE = 1.20
STATUS_FONT_THICKNESS = 2
STATUS_FONT_COLOR = (255, 255, 240)
STATUS_FONT_OUTLINE_COLOR = (104, 130, 160)
STATUS_FONT_OUTLINE_THICKNESS = 2
STATUS_MONOSPACE = True
STATUS_TEXT_X = 14
STATUS_TEXT_Y = 37

CIRCLE_DRAW_SETTINGS = {
    "COLOR_CONFIRMED": (250, 255, 231),  # white
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

# -------- Param manager for modular get_params() integrations --------
_param_cache = {s[0]: s[1] for s in DEFAULT_SLIDER_VALUES}
def get_params():
    # In a GUI context, reload from sliders; otherwise use `_param_cache` or reload-on-change-file
    # For advanced use, could read from JSON/YAML or cache, but here we assume GUI is source of truth
    # This function should be re-patched in production or unit-test for headless operation
    try:
        import gui_interface
        live = gui_interface.read_trackbars(DEFAULT_SLIDER_VALUES)
        _param_cache.update(live)
    except Exception:
        # In headless, just use last known (or DEFAULT)
        pass
    return _param_cache.copy()