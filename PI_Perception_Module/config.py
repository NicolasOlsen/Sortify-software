import cv2

DEFAULT_SLIDER_VALUES = [
 ("R1 H min", 0), ("R1 H max", 10),
 ("R2 H min", 160), ("R2 H max", 180),
 ("R S min", 135), ("R S max", 255),
 ("R V min", 105), ("R V max", 255),
 ("B H min", 100), ("B H max", 140),
 ("B S min", 110), ("B S max", 255),
 ("B V min", 60), ("B V max", 255),
 ("Kernel Size", 5), ("Open Iter", 5),
 ("Close Iter", 1), ("dp (x0.1)", 13),
 ("minDist", 75), ("param1", 170),
 ("param2", 42), ("minRadius", 35),
 ("maxRadius", 200), ("Focus", 131),
 ("Alpha(x100)", 70), ("Contrast", 0),
 ("Saturation", 2), ("Sharpness", 2),
 ("Brightness", 2), ("WhiteB", 2700)
]

# Set max values for each slider, same order as above!
SLIDER_MAX_VALUES = [
  20, 180,
  180, 180,
  255, 255,
  255, 255,
  180, 180,
  255, 255,
  205, 255,
  31, 10,
  20, 30,
  200, 255,
  100, 120,
  200, 255,
  300, 10,
  10, 10,
  10, 8500
]

TRACKBAR_WINDOW = "Trackbars"
TRACKBAR_MAX = 255
USE_HARDWARE_PREPROCESSING = False  # Set True to skip CLAHE + blur
CIRCLE_PERSISTENCE_FRAMES = 5    # Number of frames the circle needs to persist before becoming 'visible'
CIRCLE_MATCH_DIST = 100           # In pixels, how close two circlec must be to be considered "the same"
BALL_SMOOTHER_ALPHA = 0.25


# ==== ADVANCED ROBOTICS VISUALIZATION CONFIG (STRICT STRUCTURE) ====

# ---- START: BALL VISUALIZATION EFFECTS ----
BALL_GLOW_COLOR_RED = (180, 100, 40)      # Neo orange, clinical accent — high-tech glow (BGR)
BALL_GLOW_COLOR_BLUE = (255, 250, 230)    # Plasma ice blue — core robotics halo (BGR)
BALL_GLOW_THICKNESS = 4                   # Fine-edge, sharp highlight
BALL_GLOW_INTENSITY = 0.10                # Soft, digital emission
TRAIL_MAX_LENGTH = 10                     # Minimal after-trace
TRAIL_START_WIDTH = 4                     # Lean start
TRAIL_END_WIDTH = 1                       # Zeroed nano-fade
TRAIL_COLOR_ALPHA = 0.11                  # Light, ghostly effect
# ---- END: BALL VISUALIZATION EFFECTS ----

# ---- START: STATUS/FPS BAR VISUALIZATION ----
STATUS_BAR_RECT_COLOR = (46, 54, 78)          # Deep titanium — HUD foundation (BGR)
STATUS_BAR_GLOW_COLOR = (255, 225, 120)       # Neon cyan HUD edge (BGR)
STATUS_BAR_ALPHA = 0.82                       # Modern glass-panel opacity
STATUS_BAR_GLOW_RADIUS = 2                    # Fine cyber outline
STATUS_BAR_HEIGHT = 52                        # Slightly taller—capsule HUD look
STATUS_BAR_TOP_MARGIN = 10                    # Floating top margin
STATUS_BAR_SIDE_MARGIN = 16                   # Balanced horizontal padding
STATUS_FONT = cv2.FONT_HERSHEY_TRIPLEX
STATUS_FONT_SCALE = 1.20                      # High-tech legibility
STATUS_FONT_THICKNESS = 2                     # Razor sharp
STATUS_FONT_COLOR = (255, 255, 240)           # Polar white — pure digital text (BGR)
STATUS_FONT_OUTLINE_COLOR = (104, 130, 160)   # Frost shadow — subtle HUD depth (BGR)
STATUS_FONT_OUTLINE_THICKNESS = 2             # Clean edge
STATUS_MONOSPACE = True
STATUS_TEXT_X = 14                            # Minimal left margin
STATUS_TEXT_Y = 37                            # Centered in new bar height
# ---- END: STATUS/FPS BAR VISUALIZATION ----

# ---- START: CIRCLE DRAW SETTINGS (BALL OUTLINES, SCORES) ----
CIRCLE_DRAW_SETTINGS = {
    "COLOR_CONFIRMED": (250, 255, 231),       # Luminous white — precise confirmed lock-in (BGR)
    "COLOR_CANDIDATE": (210, 255, 255),       # Synth cyan — candidate, clinical clarity (BGR)
    "COLOR_REJECTED": (88, 95, 115),          # Graphite blue — unobtrusive, modern (BGR)
    "COLOR_RED_BALL": (188, 130, 224),        # Plasma violet — advanced accent (BGR)
    "COLOR_BLUE_BALL": (255, 235, 110),       # Cyber teal — signature robotics (BGR)
    "COLOR_SCORE_AUX": (220, 255, 250),       # Glass cyan — support overlay (BGR)
    "COLOR_GRAY": (58, 66, 78),               # Carbon neutral — HUD separator (BGR)
    "DRAW_THICKNESS_MAIN": 2,                 # Nano-precise edge
    "DRAW_THICKNESS_AUX": 2,                  # Match main, clean
    "FONT": cv2.FONT_HERSHEY_SIMPLEX,
    "FONT_SCALE": 0.54,                       # Sleek digital micro
    "FONT_SCALE_SCORE": 0.44,
    "FONT_THICKNESS": 2,                      # Technical clarity
    "DRAW_MARGIN": 3,
    "SCORE_THRESH_CONFIRMED": 0.80,
    "SCORE_THRESH_CANDIDATE": 0.45,
}
# ---- END: CIRCLE DRAW SETTINGS ----


HSV_SNAPSHOT_FOLDER = "/Users/azi/Desktop/Sortify/Modular code/HSV TESTING"
