# utils.py

import numpy as np
from typing import List, Dict, Tuple
from dataclasses import dataclass
import cv2
from config import (
    BALL_GLOW_COLOR_RED,
    BALL_GLOW_COLOR_BLUE,
    BALL_GLOW_THICKNESS,
    BALL_GLOW_INTENSITY,
    TRAIL_MAX_LENGTH,
    TRAIL_START_WIDTH,
    TRAIL_END_WIDTH,
    TRAIL_COLOR_ALPHA,
    
    STATUS_BAR_RECT_COLOR,
    STATUS_BAR_GLOW_COLOR,
    STATUS_BAR_ALPHA,
    STATUS_BAR_GLOW_RADIUS,
    STATUS_FONT,
    STATUS_FONT_SCALE,
    STATUS_FONT_THICKNESS,
    STATUS_FONT_COLOR,
    STATUS_FONT_OUTLINE_COLOR,
    STATUS_FONT_OUTLINE_THICKNESS,
)


def draw_status_bar(
    frame,
    text: str,
    pos: tuple = (10, -40),
    width: int = 300,
    height: int = 38
):
    """
    Draws a stylized FPS/Status bar with digital/monospace font and glowing background.
    Args:
        frame: destination BGR image.
        text: status string (e.g., "FPS: 23.4").
        pos: (x, y) tuple for bar, y<0 means from bottom (useful for bottom anchoring).
        width: bar width in pixels.
        height: bar height in pixels.
    """
    h, w = frame.shape[:2]
    x, y = pos

    if y < 0:
        y = h + y

    bar_rect = (x, y, min(width, w - 10), min(height, h - 10))
    overlay = frame.copy()

    # Draw glowing effect with several blurred rectangle layers
    for glow in range(STATUS_BAR_GLOW_RADIUS, 0, -3):
        glow_alpha = STATUS_BAR_ALPHA * 0.18 * (glow / STATUS_BAR_GLOW_RADIUS)
        cv2.rectangle(
            overlay,
            (bar_rect[0] - glow, bar_rect[1] - glow),
            (bar_rect[0] + bar_rect[2] + glow, bar_rect[1] + bar_rect[3] + glow),
            STATUS_BAR_GLOW_COLOR,
            -1,
        )
        cv2.addWeighted(overlay, glow_alpha, frame, 1 - glow_alpha, 0, frame)
        overlay = frame.copy()

    # Main status bar rectangle
    cv2.rectangle(
        frame,
        (bar_rect[0], bar_rect[1]),
        (bar_rect[0] + bar_rect[2], bar_rect[1] + bar_rect[3]),
        STATUS_BAR_RECT_COLOR,
        -1,
    )

    # Digital/monospace text: thick outline for glow, then main color
    (label_w, label_h), _ = cv2.getTextSize(
        text, STATUS_FONT, STATUS_FONT_SCALE, STATUS_FONT_THICKNESS
    )
    tx = bar_rect[0] + (bar_rect[2] - label_w) // 2
    ty = bar_rect[1] + (bar_rect[3] + label_h) // 2 - 3
    # Outline
    cv2.putText(
        frame,
        text,
        (tx, ty),
        STATUS_FONT,
        STATUS_FONT_SCALE,
        STATUS_FONT_OUTLINE_COLOR,
        STATUS_FONT_OUTLINE_THICKNESS,
        lineType=cv2.LINE_AA,
    )
    # Foreground digital/mono color
    cv2.putText(
        frame,
        text,
        (tx, ty),
        STATUS_FONT,
        STATUS_FONT_SCALE,
        STATUS_FONT_COLOR,
        STATUS_FONT_THICKNESS,
        lineType=cv2.LINE_AA,
    )

# ...BallTrack and BallSmoother unchanged...

def draw_ball_with_glow(frame, cx, cy, r, color):
    """
    Draw a glowing effect (radial blur/fake glow) around ball. Only uses the provided color.
    """
    overlay = frame.copy()
    for i in range(BALL_GLOW_THICKNESS, 0, -3):
        alpha = BALL_GLOW_INTENSITY * (i / BALL_GLOW_THICKNESS)
        cv2.circle(
            overlay,
            (int(cx), int(cy)),
            int(r) + i,
            color,
            -1
        )
        cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0, frame)
        overlay = frame.copy()

def draw_trajectories(frame, ball_smoother):
    """
    Enhanced: Draw futuristic trails for every tracked ball.
    Trails are color/width modulated, only using config colors.
    """
    colors = {"Red Ball": BALL_GLOW_COLOR_RED, "Blue Ball": BALL_GLOW_COLOR_BLUE}
    for color_name, color_val in colors.items():
        tc = ball_smoother.trajectories.get(color_name, {})
        for traj in tc.values():
            if len(traj) < 2:
                continue
            points = traj[-TRAIL_MAX_LENGTH:]
            for i in range(1, len(points)):
                pt1 = tuple(map(int, points[i - 1]))
                pt2 = tuple(map(int, points[i]))
                width = int(TRAIL_START_WIDTH -
                            (TRAIL_START_WIDTH - TRAIL_END_WIDTH) * (i / len(points)))
                # Color gets lighter for older segments
                color_fade = (
                    int(color_val[0] * (1 - i / len(points)) + 255 * (i / len(points))),
                    int(color_val[1] * (1 - i / len(points)) + 255 * (i / len(points))),
                    int(color_val[2] * (1 - i / len(points)) + 255 * (i / len(points))),
                )
                overlay = frame.copy()
                cv2.line(
                    overlay, pt1, pt2, color_fade, width
                )
                alpha = TRAIL_COLOR_ALPHA * (1.0 - (i / len(points)))
                cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0, frame)
                
                
def draw_detected_balls_with_effects(frame, detected, color_name):
    """
    For each detected ball, draws the glowing effect & an inner highlight.
    Uses only config-based colors—never hardcoded!
    """
    if color_name == "Red Ball":
        color = BALL_GLOW_COLOR_RED
    else:
        color = BALL_GLOW_COLOR_BLUE
    for (cx, cy, r) in detected:
        draw_ball_with_glow(frame, cx, cy, r, color)
        # Circle outline in config color (never yellow!):
        cv2.circle(frame, (int(cx), int(cy)), int(r), color, 2)
        # Always white core
        cv2.circle(frame, (int(cx), int(cy)), int(0.5 * r), (255, 255, 255), -1)


@dataclass
class BallTrack:
    x: float
    y: float
    r: float
    age: int = 1
    lost: int = 0
    id: int = -1

class BallSmoother:
    """
    Multi-object exponential smoother with position-based association for balls.
    Tracks up to N balls per color, with temporal consistency.
    Also tracks trajectory history for each ball.
    """
    _id_counter: int = 0

    def __init__(self, alpha: float = 0.3, max_lost: int = 3, match_dist: float = 70):
        """
        Args:
            alpha: smoothing factor
            max_lost: how many frames a ball can be missing before dropped
            match_dist: max distance to associate detections with tracks
        """
        self.alpha = alpha
        self.max_lost = max_lost
        self.match_dist = match_dist
        self.tracks: Dict[str, List[BallTrack]] = {}
        self.trajectories: Dict[str, Dict[int, List[Tuple[float, float]]]] = {}

    def reset(self):
        self.tracks.clear()
        self.trajectories.clear()

    def _next_id(self):
        BallSmoother._id_counter += 1
        return BallSmoother._id_counter

    def smooth(self, color: str, balls: List[Tuple[float, float, float]]) -> List[Tuple[float, float, float]]:
        """
        Args:
            color: e.g. "Red Ball"/"Blue Ball"
            balls: freshly detected (x, y, r) for this frame
        Returns:
            smoothed list of (x, y, r), same order as internal tracks
        """
        old_tracks = self.tracks.get(color, [])
        # --- Associate detected balls to existing tracks
        assigned = set()
        for track in old_tracks:
            # Find best match by lowest Euclidean distance
            best = None
            best_dist = self.match_dist + 1
            for i, (cx, cy, cr) in enumerate(balls):
                if i in assigned:
                    continue
                dist = np.hypot(track.x - cx, track.y - cy)
                if dist < best_dist:
                    best = i
                    best_dist = dist
            # Either associate, or (if no match) mark as lost
            if best is not None and best_dist <= self.match_dist:
                cx, cy, cr = balls[best]
                track.x = (1 - self.alpha) * track.x + self.alpha * cx
                track.y = (1 - self.alpha) * track.y + self.alpha * cy
                track.r = (1 - self.alpha) * track.r + self.alpha * cr
                track.lost = 0
                track.age += 1
                assigned.add(best)
            else:
                track.lost += 1
        # Start new tracks for unassigned detections
        for i, (cx, cy, cr) in enumerate(balls):
            if i not in assigned:
                old_tracks.append(
                    BallTrack(cx, cy, cr, age=1, lost=0, id=self._next_id())
                )
        # Remove tracks lost for too many frames
        tracks_out = [t for t in old_tracks if t.lost <= self.max_lost]
        self.tracks[color] = tracks_out

        # ========= Trajectory tracking =========
        if color not in self.trajectories:
            self.trajectories[color] = {}
        for track in tracks_out:
            traj = self.trajectories[color].setdefault(track.id, [])
            traj.append((track.x, track.y))
            if len(traj) > 30:   # Keep only last 30 pts
                traj.pop(0)
        # Remove dead trajectories
        active_ids = {t.id for t in tracks_out}
        self.trajectories[color] = {tid: tr for tid, tr in self.trajectories[color].items() if tid in active_ids}
        # =======================================

        # Return output as list of tuples (order: most stable first)
        return [(t.x, t.y, t.r) for t in tracks_out if t.lost == 0]
    
def draw_trajectories(frame, ball_smoother):
    """
    Draws path history ('tail') for every tracked ball, color-coded.
    Args:
        frame: Frame to overlay on.
        ball_smoother: BallSmoother instance.
    """
    colors = {"Red Ball": (0, 0, 255), "Blue Ball": (255, 0, 0)}
    for color_name, color_val in colors.items():
        tc = ball_smoother.trajectories.get(color_name, {})
        for traj in tc.values():
            for i in range(1, len(traj)):
                pt1 = tuple(map(int, traj[i-1]))
                pt2 = tuple(map(int, traj[i]))
                fade = int(255 * i / len(traj))
                line_color = (
                    min(color_val[0] + fade, 255),
                    min(color_val[1] + fade, 255),
                    min(color_val[2] + fade, 255)
                )
                cv2.line(frame, pt1, pt2, line_color, 2)