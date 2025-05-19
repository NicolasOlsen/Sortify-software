# utils.py

import numpy as np
from typing import List, Dict, Tuple
from dataclasses import dataclass
import cv2

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
    Multi-object exponential smoother with ID/trajectory for balls.
    """
    _id_counter: int = 0
    def __init__(self, alpha: float = 0.3, max_lost: int = 3, match_dist: float = 70):
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
        old_tracks = self.tracks.get(color, [])
        assigned = set()
        for track in old_tracks:
            best = None
            best_dist = self.match_dist + 1
            for i, (cx, cy, cr) in enumerate(balls):
                if i in assigned:
                    continue
                dist = np.hypot(track.x - cx, track.y - cy)
                if dist < best_dist:
                    best = i
                    best_dist = dist
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
        for i, (cx, cy, cr) in enumerate(balls):
            if i not in assigned:
                old_tracks.append(
                    BallTrack(cx, cy, cr, age=1, lost=0, id=self._next_id())
                )
        tracks_out = [t for t in old_tracks if t.lost <= self.max_lost]
        self.tracks[color] = tracks_out
        # trajectory/history
        if color not in self.trajectories:
            self.trajectories[color] = {}
        for track in tracks_out:
            traj = self.trajectories[color].setdefault(track.id, [])
            traj.append((track.x, track.y))
            if len(traj) > 30:
                traj.pop(0)
        # Remove old/dead trajectories
        active_ids = {t.id for t in tracks_out}
        self.trajectories[color] = {tid: tr for tid, tr in self.trajectories[color].items() if tid in active_ids}
        # Return all "current" balls as (x, y, r)
        return [(t.x, t.y, t.r) for t in tracks_out if t.lost == 0]