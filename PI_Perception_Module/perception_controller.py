import time
import os
import cv2
import depthai as dai
from typing import Any, Dict

from ros_wrapper import USE_ROS, Node, String
from detection import detect_colors, detect_and_label_circles
from position_estimation import evaluate_ball
from yolo_verification import trigger_yolo, yolo_in_progress
from preprocessing import apply_clahe, apply_gaussian_blur

from config import DEFAULT_SLIDER_VALUES, TRACKBAR_WINDOW, HSV_SNAPSHOT_FOLDER, SLIDER_MAX_VALUES
from gui_interface import create_trackbars, read_trackbars
from logging_handler import (
    create_csv_logger,
    logger as console_logger,
    save_frame_overlay,
    save_hsv_snapshot
)

from utils import BallSmoother, draw_trajectories, draw_detected_balls_with_effects, draw_status_bar

from config import (
    STATUS_BAR_RECT_COLOR,
    STATUS_BAR_ALPHA,
    STATUS_BAR_HEIGHT,
    STATUS_BAR_TOP_MARGIN,
    STATUS_BAR_SIDE_MARGIN,
    STATUS_TEXT_X,
    STATUS_TEXT_Y,
    STATUS_FONT,
    STATUS_FONT_SCALE,
    STATUS_FONT_COLOR,
    STATUS_FONT_OUTLINE_COLOR,
    STATUS_FONT_THICKNESS,
    STATUS_FONT_OUTLINE_THICKNESS,
)


class PerceptionController:
    """
    Manages camera pipeline, trackbars, image processing, and result publishing.
    """
    def __init__(self, node_name: str = "perception_controller"):
        if USE_ROS:
            self.ros_node = Node(node_name)
            self.publisher = self.ros_node.create_publisher(String, 'object_data', 10)
        else:
            self.ros_node = None
            self.publisher = None

        # ONLY ONE smoother, used app-wide:
        self.ball_smoother = BallSmoother(alpha=0.6, max_lost=3, match_dist=100)

        self.csv_logger, self.csv_file = create_csv_logger()
        self.spam_mode = False
        self._pipeline = self._build_pipeline()
        self._device = dai.Device(self._pipeline)
        self._last_isp_params = {"Contrast": -1, "Saturation": -1, "Sharpness": -1, "Brightness": -1}

        # ISP Settings...
        ctrl = dai.CameraControl()
        ctrl.setContrast(3)
        ctrl.setSaturation(3)
        ctrl.setSharpness(2)
        ctrl.setBrightness(2)  # <---- FIX: setBrightness exists and is needed!
        self.q_control = self._device.getInputQueue("control")
        self._fx = self._fy = self._cx0 = self._cy0 = 0.0
        
        
        
        
        
        
    def initialize(self) -> None:
        """
        Initializes windows, trackbars, camera calibration, and queues.
        """
        cv2.namedWindow(TRACKBAR_WINDOW, cv2.WINDOW_NORMAL)
        create_trackbars(DEFAULT_SLIDER_VALUES, SLIDER_MAX_VALUES)

        calib = self._device.readCalibration()
        intr = calib.getCameraIntrinsics(dai.CameraBoardSocket.RGB, 1280, 720)
        self._fx, self._fy = intr[0][0], intr[1][1]
        self._cx0, self._cy0 = intr[0][2], intr[1][2]

        self.q_video = self._device.getOutputQueue("video", 4, False)
        self.q_depth = self._device.getOutputQueue("depth", 4, False)
        
        
        
        
        
        
        
    def _set_focus(self, focus_val: int) -> None:
        """
        Sets auto or manual focus on the device.

        Args:
            focus_val: 0 for auto-focus, otherwise manual focus value.
        """
        ctrl = dai.CameraControl()
        if focus_val == 0:
            ctrl.setAutoFocusMode(dai.CameraControl.AutoFocusMode.AUTO)
        else:
            ctrl.setManualFocus(focus_val)
        self._device.getInputQueue("control").send(ctrl)
        
        
        
        
        
        

    def _publish_results(self, color: str, results: Any) -> None:
        """
        Publishes detection results to ROS or logs them if spam_mode is active.

        Args:
            color: Color label ("red" or "blue").
            results: List of (score, cx, cy, r, x, y, z, verified).
        """
        t = time.time()
        for (score, cx, cy, r, x, y, z) in results:
            msg_text = f"{color.upper()} | x={int(x)} y={int(y)} z={int(z)}"
            if USE_ROS and self.publisher:
                ros_msg = String()
                ros_msg.data = msg_text
                self.publisher.publish(ros_msg)
            if self.spam_mode:
                self.csv_logger.writerow([t, color, score, r, int(x), int(y), int(z)])
                
                
                
                
                
                
                
    # perception_controller.py / class PerceptionController / def _build_pipeline(self)
    def _build_pipeline(self) -> dai.Pipeline:
        pipeline = dai.Pipeline()
        cam_rgb = pipeline.createColorCamera()
        # ---- REDUCE RESOLUTION TO 640x360 FOR SPEED ---- #
        cam_rgb.setPreviewSize(640, 360)
        cam_rgb.setVideoSize(640, 360)
        cam_rgb.setStillSize(640, 360)
        cam_rgb.setInterleaved(False)
        cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        ctrl_in = pipeline.createXLinkIn()
        ctrl_in.setStreamName("control")
        ctrl_in.out.link(cam_rgb.inputControl)
        xout_rgb = pipeline.createXLinkOut()
        xout_rgb.setStreamName("video")
        cam_rgb.video.link(xout_rgb.input)
        mono_left = pipeline.createMonoCamera()
        mono_right = pipeline.createMonoCamera()
        mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_left.setBoardSocket(dai.CameraBoardSocket.LEFT)
        mono_right.setBoardSocket(dai.CameraBoardSocket.RIGHT)
        stereo = pipeline.createStereoDepth()
        stereo.initialConfig.setConfidenceThreshold(200)
        stereo.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_5x5)
        stereo.setLeftRightCheck(True)
        stereo.setSubpixel(True)
        stereo.setDepthAlign(dai.CameraBoardSocket.RGB)
        stereo.setOutputSize(640, 360)
        mono_left.out.link(stereo.left)
        mono_right.out.link(stereo.right)
        xout_depth = pipeline.createXLinkOut()
        xout_depth.setStreamName("depth")
        stereo.depth.link(xout_depth.input)
        return pipeline
    
    
    
    
    

    # perception_controller.py / class PerceptionController / def run_camera_loop(self)
    def run_camera_loop(self) -> None:
        frame_count = 0
        fps = 0.0
        last_time = time.time()
        while True:
            p = self._read_slider_params()
            self.ball_smoother.alpha = p['alpha']
            self._set_focus(int(p["Focus"]))
            new_vals = {
                "Contrast": p.get("Contrast", 3),
                "Saturation": p.get("Saturation", 3),
                "Sharpness": p.get("Sharpness", 2),
                "Brightness": p.get("Brightness", 2),
                "WhiteB": p.get("WhiteB", 4000)
            }
            if new_vals != self._last_isp_params:
                ctrl = dai.CameraControl()
                ctrl.setContrast(new_vals["Contrast"])
                ctrl.setSaturation(new_vals["Saturation"])
                ctrl.setSharpness(new_vals["Sharpness"])
                ctrl.setBrightness(new_vals["Brightness"])
                ctrl.setManualWhiteBalance(int(new_vals["WhiteB"]))
                self.q_control.send(ctrl)
                self._last_isp_params = new_vals.copy()

            in_video = self.q_video.get()
            in_depth = self.q_depth.get()
            if not in_video or not in_depth:
                continue
            frame = cv2.flip(in_video.getCvFrame(), 0)
            depth = cv2.flip(in_depth.getFrame(), 0)

            # ---- 1. FAST CLAHE/BLUR BYPASS ---- #
            from config import USE_HARDWARE_PREPROCESSING  # Re-import in class scope
            if not USE_HARDWARE_PREPROCESSING:
                from preprocessing import apply_clahe, apply_gaussian_blur
                frame = apply_clahe(frame)
                frame = apply_gaussian_blur(frame)

            # ---- 2. DETECTION (unchanged) ---- #
            mask_red, mask_blue = detect_colors(
                frame,
                p['r1_h_min'], p['r1_h_max'],
                p['r2_h_min'], p['r2_h_max'],
                p['r_s_min'], p['r_s_max'],
                p['r_v_min'], p['r_v_max'],
                p['b_h_min'], p['b_h_max'],
                p['b_s_min'], p['b_s_max'],
                p['b_v_min'], p['b_v_max'],
                p['kernel_size'], p['open_iter'], p['close_iter']
            )
            red_new = detect_and_label_circles(
                mask_red, "Red Ball", frame,
                p['dp'], p['minDist'],
                p['param1'], p['param2'],
                p['minRadius'], p['maxRadius'],
                p['alpha']
            )
            blue_new = detect_and_label_circles(
                mask_blue, "Blue Ball", frame,
                p['dp'], p['minDist'],
                p['param1'], p['param2'],
                p['minRadius'], p['maxRadius'],
                p['alpha']
            )
            red_tracked = self.ball_smoother.smooth("Red Ball", red_new)
            blue_tracked = self.ball_smoother.smooth("Blue Ball", blue_new)

            red_results = evaluate_ball("Red Ball", mask_red, frame,
                self._fx, self._fy, self._cx0, self._cy0, depth, red_tracked)
            blue_results = evaluate_ball("Blue Ball", mask_blue, frame,
                self._fx, self._fy, self._cx0, self._cy0, depth, blue_tracked)
            self._publish_results("red", red_results)
            self._publish_results("blue", blue_results)

            # ---- 3. SKIP ALL EXPENSIVE DRAWING ---- #
            # Comment out heavy visualization for speed!
            # draw_detected_balls_with_effects(frame, red_tracked, "Red Ball")
            # draw_detected_balls_with_effects(frame, blue_tracked, "Blue Ball")
            # draw_trajectories(frame, self.ball_smoother)
            # ...skipping status/fps bar, overlays, putText, rectangle...

            frame_count += 1
            now = time.time()
            if now - last_time >= 1.0:
                fps = frame_count / (now - last_time)
                frame_count, last_time = 0, now

            # ---- 4. Just Show Output Frame (optional) ---- #
            cv2.imshow("Color Frame", frame)
            key = cv2.waitKey(1) & 0xFF
            self._handle_key_press(key, frame, p)
            if key == ord('q'):
                break

        cv2.destroyAllWindows()
        self.csv_file.close()
        if USE_ROS and self.ros_node is not None:
            self.ros_node.destroy_node()
                
            
            
            
            
            

    def _read_slider_params(self) -> Dict[str, float]:
        raw = read_trackbars(DEFAULT_SLIDER_VALUES)
        return {
            'r1_h_min': raw["R1 H min"],
            'r1_h_max': raw["R1 H max"],
            'r2_h_min': raw["R2 H min"],
            'r2_h_max': raw["R2 H max"],
            'r_s_min': raw["R S min"],
            'r_s_max': raw["R S max"],
            'r_v_min': raw["R V min"],
            'r_v_max': raw["R V max"],
            'b_h_min': raw["B H min"],
            'b_h_max': raw["B H max"],
            'b_s_min': raw["B S min"],
            'b_s_max': raw["B S max"],
            'b_v_min': raw["B V min"],
            'b_v_max': raw["B V max"],
            'kernel_size': raw["Kernel Size"],
            'open_iter': raw["Open Iter"],
            'close_iter': raw["Close Iter"],
            'dp': raw["dp (x0.1)"] / 10.0,
            'minDist': raw["minDist"],
            'param1': raw["param1"],
            'param2': raw["param2"],
            'minRadius': raw["minRadius"],
            'maxRadius': raw["maxRadius"],
            'Focus': raw["Focus"],
            'alpha': raw["Alpha(x100)"] / 100.0,
            'Contrast': raw["Contrast"],
            'Saturation': raw["Saturation"],
            'Sharpness': raw["Sharpness"],
            'Brightness': raw["Brightness"],
            'WhiteB': raw.get("WhiteB", 4000)
        }
        
        
        
        
        
        

    def _handle_key_press(self, key: int, frame: Any, params: Dict[str, float]) -> None:
        if key == ord('s'):
            save_hsv_snapshot(frame, params, HSV_SNAPSHOT_FOLDER)
        elif key == ord('r'):
            for name, val in DEFAULT_SLIDER_VALUES:
                cv2.setTrackbarPos(name, TRACKBAR_WINDOW, val)
            console_logger.info("Trackbars reset to default.")
        elif key == ord('p'):
            save_frame_overlay(frame)
        elif key == ord('g'):
            self.spam_mode = not self.spam_mode
            console_logger.info(f"[SPAM MODE] {'ENABLED' if self.spam_mode else 'DISABLED'}")
            
            
            
            
            


def main():
    controller = PerceptionController()
    controller.initialize()
    controller.run_camera_loop()
    while True:
        time.sleep(1)


if __name__ == "__main__":
    main()