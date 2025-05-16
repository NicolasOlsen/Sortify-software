"""
perception_controller.py

Module Purpose:
    Orchestrates end-to-end perception pipeline:
    - Frame acquisition ➔ Preprocessing ➔ Detection ➔ Scoring ➔ Tracking ➔ Publication

High-Level Flow:
    Video/depth frame --> preprocess --> detect --> filter/score --> track --> publish/visualize

Author: [Azi Sami], 2025
Version: 1.0.0
"""

from typing import List, Dict, Tuple, Optional
import time
import numpy as np
import cv2
import math

from kinematics import transform_camera_to_robot_frame
from hardware import DepthAICamera
from processing import preprocess
from detection import detect_objects, DetectColor
from position_estimation import estimate_position
from scoring_controller import DecisionResult, merge_detection_info
from shape_tracker import ShapeTracker
from debug_visualization import draw_detections, build_debug_view
from gui_interface import create_trackbars, get_runtime_params
from logging_handler import logger as console_logger, KPIBatchLogger
from ros_wrapper import ROSInterface, ros_shutdown

from config import (
    CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_FPS,
    SLIDER_CONFIG, TRACKBAR_WINDOW, TRACK_TARGETS, ACTIVE_GROUPS,
    LOGGING_TOGGLE_KEY, LOGGING_MAX_FRAMES_DEFAULT,
    DEPTH_MIN_MM, DEPTH_MAX_MM, GROUND_TRUTH_POS,
    USE_GUI, USE_TRACKBARS, LOG
)

# --- Types ---

DetectionData = Dict[str, float]
DetectionObj = Dict[str, object]
TrackedOutput = Dict[str, object]

class PerceptionController:


    def __init__(self, node_name: str = "perception_controller") -> None:

        self.ros: ROSInterface = ROSInterface(node_name)
        self.camera: DepthAICamera = DepthAICamera(
            width=CAMERA_WIDTH, height=CAMERA_HEIGHT, fps=CAMERA_FPS
        )
        self._fx: float = 0.0
        self._fy: float = 0.0
        self._cx0: float = 0.0
        self._cy0: float = 0.0

        self._prev_focus: int = -1
        self.trackers: Dict[Tuple[str, str], ShapeTracker] = {
            (t["shape"], t["color"]): ShapeTracker() for t in TRACK_TARGETS
        }
        self.global_frame_counter: int = 0
        self.kpi_logger = None 


    def initialize(self) -> None:

        if USE_GUI and USE_TRACKBARS:
            cv2.namedWindow(TRACKBAR_WINDOW, cv2.WINDOW_NORMAL)
            create_trackbars(SLIDER_CONFIG, groups=ACTIVE_GROUPS)

        for tr in self.trackers.values():
            tr.clear()  
        ShapeTracker.id_counter = 0

        self.camera.start()
        self._fx, self._fy, self._cx0, self._cy0 = self.camera.get_intrinsics()

    def process_frames(self) -> None:
        fps_timer = time.time()
        fps_count = 0
        fps_avg = 0.0

        while True:
            in_video, in_depth = self.camera.get_latest_frames()
            self.global_frame_counter += 1

            params: Dict[str, float] = get_runtime_params()


            color_frame = in_video.getCvFrame()
            depth_frame = in_depth.getFrame()

            if color_frame is None or depth_frame is None:
                raise RuntimeError("Camera returned None frame.")

            color_frame = cv2.flip(color_frame, -1)
            depth_frame = cv2.flip(depth_frame, -1)
            base_image = color_frame.copy()

            # -- Preprocessing --
            processed = preprocess(
                frame_bgr=color_frame,
                use_clahe=(params["CLAHE Clip"] > 0),
                clahe_clip=float(params["CLAHE Clip"]),
                blur_k=int(params["Gaussian K"]),
                use_gray=False,
            )

            # -- Detection --
            detections_raw: List[DetectionObj] = detect_objects(processed, params)
            detections_post: List[DetectionObj] = []

            # -- Post-processing: position, scoring, filtering --
            for obj in detections_raw:
                data2d = obj["data"]
                det_mask = obj.get("mask") or (data2d.get("mask") if isinstance(data2d, dict) else None)
                # 3D estimation
                pos3d = estimate_position(
                    data2d, depth_frame,
                    fx=self._fx, fy=self._fy, cx0=self._cx0, cy0=self._cy0,
                    mask=det_mask
                )
                if not pos3d["depth_mask_valid"]:
                    continue

                merged = merge_detection_info(obj, data2d, pos3d)
                decision = DecisionResult.get_decision(merged)
                if decision.accepted:
                    obj["data"] = merged 
                    detections_post.append(obj)
                    
                gt_key = (obj["shape"], obj["color"])
                if gt_key in GROUND_TRUTH_POS:
                    gx, gy, gz = GROUND_TRUTH_POS[gt_key]
                    dx = merged["x_mm"] - gx
                    dy = merged["y_mm"] - gy
                    dz = merged["z_mm"] - gz
                    merged["pos_err_mm"] = math.sqrt(dx**2 + dy**2 + dz**2)
                else:
                    merged["pos_err_mm"] = "NA"


            # -- Organize by (shape, color) for tracking --
            dets_by_type: Dict[Tuple[str, str], List[DetectionData]] = {}
            for obj in detections_post:
                key = (obj["shape"], obj["color"])
                dets_by_type.setdefault(key, []).append(obj["data"])

            # -- Run tracking for each category --
            tracked: List[TrackedOutput] = []
            for (shape, color), det_list in dets_by_type.items():
                tracker = self.trackers[(shape, color)]
                tracked_output: List[DetectionData] = tracker.track(shape, color, det_list, params)
                tracked.extend({
                    "shape": shape, "color": color, "data": t
                } for t in tracked_output)


            # -- Robot coordinate transforms, ROS publishing --
            img_h = depth_frame.shape[0]
            for det in tracked:
                d = det["data"]
                if not all(k in d for k in ["x_mm", "y_mm", "z_mm"]):
                    continue  # Defensive: skip malformed outputs

                # Robot-frame conversion
                x_r, y_r, z_r = transform_camera_to_robot_frame(
                    d["x_mm"], d["y_mm"], d["z_mm"]
                )

                # Publication (only if tracking confirmed)
                if d.get("tracker_valid", False): 
                    self.ros.publish(
                        det["shape"], det["color"], 
                        d.get("track_id", 255),
                        d["x_mm"], d["y_mm"], d["z_mm"]
                    )

            # -- Visualization --
            overlay = base_image.copy()
            draw_detections(overlay, tracked)

            # FPS meter
            fps_count += 1
            now = time.time()
            if now - fps_timer >= 1.0:
                fps_avg = fps_count / (now - fps_timer)
                fps_timer = now
                fps_count = 0
            cv2.putText(
                overlay, f"{fps_avg:.1f} FPS", (10, 32),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (100, 255, 200), 2
            )

            # Depth visualization
            depth_norm = np.clip(
                (depth_frame.astype(np.float32) - DEPTH_MIN_MM) *
                (255.0 / (DEPTH_MAX_MM - DEPTH_MIN_MM)),
                0, 255
            ).astype(np.uint8)
            depth_vis = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)

            # Color masks for debug view
            hsv = cv2.cvtColor(processed, cv2.COLOR_BGR2HSV)
            red_mask = DetectColor.red(hsv, params)
            blue_mask = DetectColor.blue(hsv, params)

            debug_img = build_debug_view(depth_vis, red_mask, blue_mask, overlay)


            if self.kpi_logger is None:
                from logging_handler import KPIBatchLogger
                self.kpi_logger = KPIBatchLogger(
                    test_type="NY test",  
                    max_frames=1000
                )

            flattened = []
            for det in tracked:
                merged = dict(det["data"])
                merged["shape"] = det["shape"]
                merged["color"] = det["color"]
                flattened.append(merged)

            self.kpi_logger.update(
                frame_rel=self.global_frame_counter,
                detections=flattened,
                fps_avg=fps_avg,
                overlay=overlay
            )


            if not self.kpi_logger.active():
                self.kpi_logger.close()
                self.kpi_logger = None

            # -- GUI / Logging --
            if USE_GUI:
                focus_val = params.get("Focus", 0) if USE_GUI else 0
                #self._prev_focus = int(params["Focus"])
                cv2.imshow("Debug View", debug_img)
                key = cv2.waitKey(1) & 0xFF
                if self._handle_key_press(key, base_image):
                    break


        cv2.destroyAllWindows()
        self.camera.shutdown()
        self.ros.destroy()
        ros_shutdown()

    def _handle_key_press(self, key: int, frame: np.ndarray) -> bool:

        # Immediate quit options
        if key in (ord("s"), ord("p"), ord("q")):
            return True

        elif key == ord("r"):
            # Reset all trackbars to default
            for name, val in SLIDER_CONFIG.items():
                if val.get("group") in ACTIVE_GROUPS:
                    cv2.setTrackbarPos(name, TRACKBAR_WINDOW, val["default"])
            LOG.info("Trackbars RESET")

        elif key == LOGGING_TOGGLE_KEY:
            if self.kpi_logger is None:
                self.kpi_logger = KPIBatchLogger(
                    max_frames=LOGGING_MAX_FRAMES_DEFAULT,
                    start_frame_global=self.global_frame_counter,
                )
                console_logger.info("Logging STARTED")
            else:
                self.kpi_logger.close()
                self.kpi_logger = None
                console_logger.info("Logging STOPPED")
        return False

def main() -> None:
    ctl = PerceptionController()
    ctl.initialize()
    ctl.process_frames()

if __name__ == "__main__":
    main()