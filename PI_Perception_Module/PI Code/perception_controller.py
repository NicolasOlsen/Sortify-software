"""
Module: perception_controller.py
Purpose: Orchestrates the full perception pipeline
Focus : frame acquisition → preprocess → detect → score → track → publish
"""

from kinematics import transform_camera_to_robot_frame, solve_ik

import time
from typing import List, Dict

import cv2

from hardware import DepthAICamera
from processing import preprocess
from detection import detect_objects, DetectColor
from position_estimation import estimate_position
from scoring_controller import DecisionResult, merge_detection_info
from shape_tracker import ShapeTracker
from debug_visualization import draw_detections, build_debug_view
import numpy as np

from config import (
    CAMERA_WIDTH,
    CAMERA_HEIGHT,
    CAMERA_FPS,
    SLIDER_CONFIG,
    TRACKBAR_WINDOW,
    TRACK_TARGETS,
    ACTIVE_GROUPS,
    LOGGING_TOGGLE_KEY,
    LOGGING_MAX_FRAMES_DEFAULT,
    Z_CORRECTION_BY_ROW,
    DEPTH_MIN_MM,                     
    DEPTH_MAX_MM,
    USE_GUI, USE_TRACKBARS                  
)
from gui_interface import create_trackbars, get_runtime_params
from logging_handler import logger as console_logger, KPIBatchLogger
from ros_wrapper import ROSInterface, ros_shutdown
from config import LOG


class PerceptionController:

    def __init__(self, node_name: str = "perception_controller") -> None:

        self.ros = ROSInterface(node_name)

        self.camera = DepthAICamera(
            width=CAMERA_WIDTH, height=CAMERA_HEIGHT, fps=CAMERA_FPS
        )
        self._fx = self._fy = self._cx0 = self._cy0 = 0.0 
        self._prev_focus = -1

        self.trackers = {
            (t["shape"], t["color"]): ShapeTracker() for t in TRACK_TARGETS
        }

        self.global_frame_counter = 0
        self.kpi_logger: KPIBatchLogger | None = None

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
        fps_timer, fps_count, fps_avg = time.time(), 0, 0.0

        while True:

            in_video, in_depth = self.camera.get_latest_frames()
            self.global_frame_counter += 1

            params = get_runtime_params()

            if params["Focus"] != self._prev_focus:
                self.camera.set_focus(params["Focus"])
                self._prev_focus = params["Focus"]

                
                
                
            frame = in_video.getCvFrame()
            depth = in_depth.getFrame()
            
            frame = cv2.flip(frame, -1)
            depth = cv2.flip(depth, -1)

            source = frame.copy()

            preproc = preprocess(
                frame_bgr=frame,
                use_clahe=(params["CLAHE Clip"] > 0),
                clahe_clip=float(params["CLAHE Clip"]),
                blur_k=int(params["Gaussian K"]),
                use_gray=False,
            )

            detections_raw = detect_objects(preproc, params)
            detections_post: List[Dict] = []

            for obj in detections_raw:
                data2d = obj["data"]

                det_mask = (
                    obj.get("mask")
                    or (data2d.get("mask") if isinstance(data2d, dict) else None)
                )

                pos3d = estimate_position(
                    data2d,
                    depth,
                    fx=self._fx,
                    fy=self._fy,
                    cx0=self._cx0,
                    cy0=self._cy0,
                    mask=det_mask,          
                )
                if not pos3d["depth_mask_valid"]:
                    continue

                merged = merge_detection_info(obj, data2d, pos3d)
                decision = DecisionResult.get_decision(merged)
                

                if decision.accepted:
                    obj["data"] = merged
                    detections_post.append(obj)

            dets_by_type: Dict[tuple, list] = {}
            for obj in detections_post:
                dets_by_type.setdefault((obj["shape"], obj["color"]), []).append(obj["data"])

            tracked: List[Dict] = []
            for (shape, color), lst in dets_by_type.items():
                tracker = self.trackers[(shape, color)]
                tracked.extend(
                    {"shape": shape, "color": color, "data": t}
                    for t in tracker.track(shape, color, lst, params)
                )

            img_h = depth.shape[0]
            for det in tracked:
                d = det["data"]

                # --- robot-frame conversion ---
                x_r_trans, y_r_trans, z_r_trans = transform_camera_to_robot_frame(
                    d["x_mm"], d["y_mm"], d["z_mm"]
                )
                print(f"[robot-frame] XYZ = ({x_r_trans:.1f}, {y_r_trans:.1f}, {z_r_trans:.1f})")

                # OPTIONAL:
                angles_trans = solve_ik(x_r_trans, y_r_trans, z_r_trans)
                print(f"[joint angles] {angles_trans}")

                cy = d.get("cy", 0)
                if cy < img_h // 3:
                    d["z_mm"] += Z_CORRECTION_BY_ROW["top"]
                elif cy > 2 * img_h // 3:
                    d["z_mm"] += Z_CORRECTION_BY_ROW["bottom"]
                else:
                    d["z_mm"] += Z_CORRECTION_BY_ROW["middle"]

                if d["tracker_valid"]:
                    self.ros.publish(
                        det["shape"],
                        det["color"],
                        d.get("track_id", 255),
                        d["x_mm"],
                        d["y_mm"],
                        d["z_mm"],
                    )

                        
                    
            overlay = source.copy()
            draw_detections(overlay, tracked)

            fps_count += 1
            now = time.time()
            if now - fps_timer >= 1.0:
                fps_avg = fps_count / (now - fps_timer)
                fps_timer = now
                fps_count = 0
            cv2.putText(
                overlay,
                f"{fps_avg:.1f} FPS",
                (10, 32),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (100, 255, 200),
                2,
            )
            
            depth_norm = np.clip(
                (depth.astype(np.float32) - DEPTH_MIN_MM) *
                (255.0 / (DEPTH_MAX_MM - DEPTH_MIN_MM)),
                0, 255,                      
            ).astype(np.uint8)

            depth_vis = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)


            hsv        = cv2.cvtColor(preproc, cv2.COLOR_BGR2HSV)
            red_mask   = DetectColor.red(hsv, params)
            blue_mask  = DetectColor.blue(hsv, params)

            debug_img = build_debug_view(depth_vis, red_mask, blue_mask, overlay)

            if USE_GUI:
                cv2.imshow("Debug View", debug_img)
                key = cv2.waitKey(1) & 0xFF
                if self._handle_key_press(key, source):
                    break

            if self.kpi_logger and self.kpi_logger.active:
                self.kpi_logger.update(
                    frame_global=self.global_frame_counter,
                    detections=tracked,
                    fps_avg=fps_avg,
                    overlay_bgr=overlay,
                )
            if self.kpi_logger and not self.kpi_logger.active:
                console_logger.info("Run COMPLETED")
                self.kpi_logger.close()
                self.kpi_logger = None



        cv2.destroyAllWindows()
        self.camera.shutdown()
        self.ros.destroy()
        ros_shutdown()

    def _handle_key_press(self, key: int, frame) -> bool:

        if key in (ord("s"), ord("p"), ord("q")):
            return True

        elif key == ord("r"):
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
