"""
Module: logging_handler.py
Handles: KPI logging, debug output, snapshot saving
Owns: CSV export, image capture, session metadata
Calls: config, cv2, os, time
Does not contain: detection, ROS, or vision logic
"""

from __future__ import annotations

import csv
import os
import time
from datetime import datetime
from typing import Any, Dict, List, Optional, Tuple, TextIO

import cv2
import logging

from config import (
    TRACK_TARGETS,
    POS_ERROR_THRESHOLD_MM,
    MAX_ERROR_SNAPSHOTS,
    LOGGING_OUTPUT_DIR,
    LOGGING_MAX_FRAMES_DEFAULT,
)


try:
    from config import GROUND_TRUTH_POS
except ImportError:
    GROUND_TRUTH_POS = {}


logger = logging.getLogger("kpi_logger")
logger.setLevel(logging.INFO)
if not logger.handlers:
    logger.addHandler(logging.StreamHandler())


def _objkey(shape: str, color: str) -> str:

    return f"{shape}_{color}"


_CSV_HEADER: List[str] = [
    "frame",
    "time_sec",
    "detected",
    "detected_color",
    "id",
    "pos_error_mm",
    "detection_rate_pct",
    "color_accuracy_pct",
    "id_consistency_pct",
    "fps",
]


class _ObjectStats:
    def __init__(self, shape, color, writer, fh, max_frames, snapshot_frames, run_dir, gt_xyz=None):
        self.shape = shape
        self.color = color
        self.writer = writer
        self.fh = fh  # <-- this line is crucial
        self.max_frames = max_frames
        self.snapshot_frames = set(snapshot_frames)
        self.run_dir = run_dir
        self.gt_xyz = gt_xyz
        self.frames_processed = 0
        self.frames_detected = 0
        self.frames_correct_color = 0
        self.frames_consistent_id = 0
        self._last_id = None
        self.error_snapshots_taken = 0
        self._detected_once = False
        self._missing_streak = False
        self._misclass_streak = False

    
    def finished(self): 
        return self.frames_processed >= self.max_frames

    def update_and_log(
        self,
        frame_rel: int,
        detection: Optional[Dict[str, Any]],
        misclassified: bool,
        fps_avg: float,
        t0: float,
        overlay_bgr,
    ) -> None:

        self.frames_processed += 1
        now_sec = time.time() - t0

        detected_str = "no"
        det_color = "none"
        det_id: str | int = "none"
        pos_error_val: str | float = "NA"

        if detection is not None:
            detected_str = "yes"
            det_color = detection.get("color", "none")
            det_id = detection.get("data", {}).get("track_id", "none")

            self.frames_detected += 1
            self.frames_correct_color += 1
            if det_id == self._last_id and det_id != "none":
                self.frames_consistent_id += 1
            self._last_id = det_id

            self._detected_once = True
            self._missing_streak = False
            self._misclass_streak = False

            if self.gt_xyz is not None:
                gx, gy, gz = self.gt_xyz
                d = detection.get("data", {})
                px, py, pz = d.get("x_mm", 0.0), d.get("y_mm", 0.0), d.get("z_mm", 0.0)
                pos_error_val = (
                    (gx - px) ** 2 + (gy - py) ** 2 + (gz - pz) ** 2
                ) ** 0.5

        dr = 100.0 * self.frames_detected / self.frames_processed
        ca = (
            100.0 * self.frames_correct_color / self.frames_detected
            if self.frames_detected
            else 0.0
        )
        ic = (
            100.0 * self.frames_consistent_id / self.frames_detected
            if self.frames_detected
            else 0.0
        )

        self.writer.writerow(
            [
                frame_rel,
                f"{now_sec:.3f}",
                detected_str,
                det_color,
                det_id,
                (
                    f"{pos_error_val:.2f}"
                    if isinstance(pos_error_val, float)
                    else pos_error_val
                ),
                f"{dr:.2f}",
                f"{ca:.2f}",
                f"{ic:.2f}",
                f"{fps_avg:.2f}",
            ]
        )

        if overlay_bgr is None:
            return

        if frame_rel in self.snapshot_frames:
            self._save_snapshot(
                overlay_bgr,
                f"snapshot_{self.color}_{frame_rel:03d}.png",
                error=False,
            )

        if self.error_snapshots_taken >= MAX_ERROR_SNAPSHOTS:
            return

        if self._detected_once and detection is None and not self._missing_streak:
            self._save_snapshot(
                overlay_bgr,
                f"fail_{self.color}_missing_{frame_rel:03d}.png",
                error=True,
            )
            self._missing_streak = True

        elif self._detected_once and misclassified and not self._misclass_streak:
            self._save_snapshot(
                overlay_bgr,
                f"fail_{self.color}_wrongcolor_{frame_rel:03d}.png",
                error=True,
            )
            self._misclass_streak = True

        elif (
            isinstance(pos_error_val, float) and pos_error_val > POS_ERROR_THRESHOLD_MM
        ):
            self._save_snapshot(
                overlay_bgr,
                f"fail_{self.color}_positionerror_{frame_rel:03d}.png",
                error=True,
            )

    def _save_snapshot(self, img, filename: str, *, error: bool) -> None:

        try:
            path = os.path.join(self.run_dir, filename)
            cv2.imwrite(path, img)
            if error:
                self.error_snapshots_taken += 1
            logger.info(f"Snapshot {filename} STORED")
        except Exception as exc:
            logger.warning(f"Snapshot could not SAVE {filename}: {exc}")


class KPIBatchLogger:

    def __init__(
        self,
        max_frames: int = LOGGING_MAX_FRAMES_DEFAULT,
        start_frame_global: int = 0,
    ):

        ts = datetime.now().strftime("run_%Y_%m_%d_%H_%M_%S")
        self.run_dir = os.path.join(LOGGING_OUTPUT_DIR, ts)
        os.makedirs(self.run_dir, exist_ok=True)

        self.start_frame_global = start_frame_global
        self.t0 = time.time()
        self._objects: Dict[str, _ObjectStats] = {}

        snapshot_frames = [
            0,
            max_frames // 3,
            (2 * max_frames) // 3,
            max_frames - 1,
        ]

        for t in TRACK_TARGETS:
            shape, color = t["shape"], t["color"]
            key = _objkey(shape, color)
            csv_path = os.path.join(self.run_dir, f"LOG_{color}_{shape}.csv")
            fh = open(csv_path, "w", newline="")
            writer = csv.writer(fh)
            writer.writerow(_CSV_HEADER)

            obj = _ObjectStats(
                shape=shape,
                color=color,
                writer=writer,
                fh=fh,  # <- new arg
                max_frames=max_frames,
                snapshot_frames=snapshot_frames,
                run_dir=self.run_dir,
                gt_xyz=GROUND_TRUTH_POS.get((shape, color)),
            )
            self._objects[key] = obj

            logger.info(f"CSV READY → {csv_path}")

    @property
    def active(self) -> bool:

        return any(not o.finished() for o in self._objects.values())

    def update(
        self,
        frame_global: int,
        detections: List[Dict[str, Any]],
        fps_avg: float,
        overlay_bgr,
    ):

        frame_rel = frame_global - self.start_frame_global

        det_map: Dict[str, Dict[str, Any]] = {
            _objkey(d["shape"], d["color"]): d for d in detections
        }
        shape_to_colors: Dict[str, List[str]] = {}
        for d in detections:
            shape_to_colors.setdefault(d["shape"], []).append(d["color"])

        for key, obj in self._objects.items():
            if obj.finished():
                continue
            detection = det_map.get(key)
            shape = obj.shape
            color = obj.color

            misclassified_flag = (
                detection is None
                and bool(shape_to_colors.get(shape))
                and color not in shape_to_colors.get(shape, [])
            )
            obj.update_and_log(
                frame_rel=frame_rel,
                detection=detection,
                misclassified=misclassified_flag,
                fps_avg=fps_avg,
                t0=self.t0,
                overlay_bgr=overlay_bgr,
            )

    def close(self):
        for obj in self._objects.values():
            try:
                obj.fh.close()
            except Exception:
                pass
        logger.info("All CSV files CLOSED")



__all__ = ["KPIBatchLogger"]
