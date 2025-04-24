# perception_controller.py

import time
import cv2
import depthai as dai
from typing import Any, Dict
from ros_wrapper import USE_ROS, Node, String, rclpy
from detection import detect_colors, detect_and_label_circles
from position_estimation import evaluate_ball
from yolo_verification import trigger_yolo
from preprocessing import preprocess
from config import DEFAULT_SLIDER_VALUES, TRACKBAR_WINDOW, HSV_SNAPSHOT_FOLDER, SLIDER_MAX_VALUES
from gui_interface import create_trackbars, read_trackbars
from logging_handler import create_csv_logger, logger as console_logger, save_frame_overlay, save_hsv_snapshot
from utils import BallSmoother
from config import PI_MODE

# ========= Aggressive Pi Mode Settings for MAX FPS =========

PI_MODE_CONFIG = {
    'r1_h_min': 0,   'r1_h_max': 10,
    'r2_h_min': 160, 'r2_h_max': 180,
    'r_s_min': 135,  'r_s_max': 255,
    'r_v_min': 105,  'r_v_max': 255,
    'b_h_min': 100,  'b_h_max': 140,
    'b_s_min': 110,  'b_s_max': 255,
    'b_v_min': 60,   'b_v_max': 255,
    'kernel_size': 1,   # Minimal
    'open_iter': 1,     # Minimal
    'close_iter': 5,    # Minimal
    'dp': 1.3,          # A bit lower for speed
    'minDist': 35,      # Reasonable min distance
    'param1': 170,
    'param2': 35,
    'minRadius': 35,
    'maxRadius': 100,   # Lower, adjust for your object size!
    'Focus': 131,
    'alpha': 0.30,
    'Contrast': 0,
    'Saturation': 2,
    'Sharpness': 2,
    'Brightness': 2,
    'WhiteB': 2700,
    'CLAHE_Clip': 2,    # OFF
    'Gaussian_K': 10,    # OFF
    'Grayscale': 1
}

class PerceptionController:
    """
    Manages camera pipeline, trackbars, image processing, and result publishing.
    """

    def __init__(self, node_name: str = "perception_controller", pi_mode: bool = False):
        self.pi_mode = pi_mode
        if USE_ROS:
            self.ros_node = Node(node_name)
            self.publisher = self.ros_node.create_publisher(String, 'object_data', 10)
        else:
            self.ros_node = None
            self.publisher = None

        self.ball_smoother = BallSmoother(alpha=0.6, max_lost=3, match_dist=100)
        self.csv_logger, self.csv_file = create_csv_logger()
        self.spam_mode = False

        # Build pipeline with correct resolution for pi_mode
        self._pipeline = self._build_pipeline()
        self._device = dai.Device(self._pipeline)
        self.q_control = self._device.getInputQueue("control")
        self._last_isp_params = {"Contrast": -1, "Saturation": -1, "Sharpness": -1, "Brightness": -1}

        self._fx = self._fy = self._cx0 = self._cy0 = 0.0

    def initialize(self) -> None:
        if not self.pi_mode:
            cv2.namedWindow(TRACKBAR_WINDOW, cv2.WINDOW_NORMAL)
            create_trackbars(DEFAULT_SLIDER_VALUES, SLIDER_MAX_VALUES)

        calib = self._device.readCalibration()
        # Camera intrinsics at current image size (pi mode: 640x360, normal: 1280x720)
        if self.pi_mode:
            intr = calib.getCameraIntrinsics(dai.CameraBoardSocket.RGB, 640, 360)
            cv2.namedWindow("PiMode Output", cv2.WINDOW_NORMAL)

        else:
            intr = calib.getCameraIntrinsics(dai.CameraBoardSocket.RGB, 1280, 720)
        self._fx, self._fy = intr[0][0], intr[1][1]
        self._cx0, self._cy0 = intr[0][2], intr[1][2]
        # Setup queues for frames
        self.q_video = self._device.getOutputQueue("video", 1, False)
        self.q_depth = self._device.getOutputQueue("depth", 1, False)

    def _set_focus(self, focus_val: int) -> None:
        ctrl = dai.CameraControl()
        if focus_val == 0:
            ctrl.setAutoFocusMode(dai.CameraControl.AutoFocusMode.AUTO)
        else:
            ctrl.setManualFocus(focus_val)
        self.q_control.send(ctrl)

    def _publish_results(self, color: str, results: Any) -> None:
        t = time.time()
        if self.pi_mode:
            return  # Pi: no log, ROS, or CSV
        for (score, cx, cy, r, x, y, z) in results:
            msg_text = f"{color.upper()} | x={int(x)} y={int(y)} z={int(z)}"
            if USE_ROS and self.publisher:
                ros_msg = String()
                ros_msg.data = msg_text
                self.publisher.publish(ros_msg)
            if self.spam_mode:
                self.csv_logger.writerow([t, color, score, r, int(x), int(y), int(z)])

    def _build_pipeline(self) -> dai.Pipeline:
        pipeline = dai.Pipeline()
        cam_rgb = pipeline.createColorCamera()
        
        if self.pi_mode:
            cam_rgb.setPreviewSize(640, 360)
            cam_rgb.setVideoSize(640, 360)
        else:
            cam_rgb.setPreviewSize(1280, 720)
            cam_rgb.setVideoSize(1280, 720)
        cam_rgb.setStillSize(1920, 1080)
        cam_rgb.setInterleaved(False)
        cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        cam_rgb.setFps(40)

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
        mono_left.setFps(40)
        mono_right.setFps(40)
        stereo = pipeline.createStereoDepth()
        stereo.initialConfig.setConfidenceThreshold(200)
        stereo.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_5x5)
        stereo.setLeftRightCheck(True)
        stereo.setSubpixel(True)
        stereo.setDepthAlign(dai.CameraBoardSocket.RGB)
        # Output size
        if self.pi_mode:
            stereo.setOutputSize(640, 360)
        else:
            stereo.setOutputSize(1280, 720)
        mono_left.out.link(stereo.left)
        mono_right.out.link(stereo.right)
        xout_depth = pipeline.createXLinkOut()
        xout_depth.setStreamName("depth")
        stereo.depth.link(xout_depth.input)
        return pipeline

    def run_camera_loop(self) -> None:
        frame_count = 0
        fps = 0.0
        last_time = time.time()

        while True:
            if self.pi_mode:
                p = PI_MODE_CONFIG
            else:
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
            orig_frame = frame.copy()

            proc_frame = preprocess(
                frame_bgr=orig_frame,
                use_clahe=(p.get("CLAHE_Clip", 0) > 0),
                clahe_clip=float(p.get("CLAHE_Clip", 2)),
                blur_k=int(p.get("Gaussian_K", 5)),
                use_gray=False
            )

            mask_red, mask_blue = detect_colors(
                proc_frame,
                p['r1_h_min'], p['r1_h_max'],
                p['r2_h_min'], p['r2_h_max'],
                p['r_s_min'], p['r_s_max'],
                p['r_v_min'], p['r_v_max'],
                p['b_h_min'], p['b_h_max'],
                p['b_s_min'], p['b_s_max'],
                p['b_v_min'], p['b_v_max'],
                p['kernel_size'], p['open_iter'], p['close_iter']
            )

            red_new = detect_and_label_circles(mask_red, "Red Ball", orig_frame, p['dp'], p['minDist'],
                                            p['param1'], p['param2'], p['minRadius'], p['maxRadius'], p['alpha'],
                                            pi_mode=self.pi_mode)
            blue_new = detect_and_label_circles(mask_blue, "Blue Ball", orig_frame, p['dp'], p['minDist'],
                                                p['param1'], p['param2'], p['minRadius'], p['maxRadius'], p['alpha'],
                                                pi_mode=self.pi_mode)

            red_tracked = self.ball_smoother.smooth("Red Ball", red_new)
            blue_tracked = self.ball_smoother.smooth("Blue Ball", blue_new)

            frame_count += 1
            now = time.time()
            if now - last_time >= 1.0:
                fps = frame_count / (now - last_time)
                frame_count, last_time = 0, now

            # --- PI MODE: ONLY THESE LINES ---
            if self.pi_mode:
                display_frame = orig_frame.copy()
                for (cx, cy, r) in red_tracked:
                    cv2.circle(display_frame, (int(cx), int(cy)), int(r), (0, 0, 255), 2)
                for (cx, cy, r) in blue_tracked:
                    cv2.circle(display_frame, (int(cx), int(cy)), int(r), (255, 0, 0), 2)
                cv2.putText(display_frame, f"{fps:.1f} FPS", (10,32), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0,255,0), 2)
                cv2.imshow("PiMode Output", display_frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                continue

            # ======= NORMAL MODE: overlays, yolo, log, ros =======
            red_results = evaluate_ball("Red Ball", mask_red, orig_frame,
                                        self._fx, self._fy, self._cx0, self._cy0, depth, red_tracked, pi_mode=self.pi_mode)
            blue_results = evaluate_ball("Blue Ball", mask_blue, orig_frame,
                                        self._fx, self._fy, self._cx0, self._cy0, depth, blue_tracked, pi_mode=self.pi_mode)

            trigger_yolo(frame, pi_mode=self.pi_mode, spam_mode=self.spam_mode)

            self._publish_results("red", red_results)
            self._publish_results("blue", blue_results)

            cv2.imshow("Color Frame", orig_frame)
            key = cv2.waitKey(1) & 0xFF
            self._handle_key_press(key, orig_frame, p)
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
            'WhiteB': raw.get("WhiteB", 4000),
            'CLAHE_Clip': raw.get('CLAHE Clip', 2),
            'Gaussian_K': raw.get('Gaussian K', 5),
            'Grayscale': raw.get('Grayscale', 0)
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
    import sys
    controller = PerceptionController(pi_mode=PI_MODE)
    controller.initialize()
    controller.run_camera_loop()
    while True:
        time.sleep(1)

if __name__ == "__main__":
    main()