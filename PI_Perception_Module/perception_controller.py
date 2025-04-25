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
from config import DEFAULT_SLIDER_VALUES, TRACKBAR_WINDOW, HSV_SNAPSHOT_FOLDER, SLIDER_MAX_VALUES, get_params
from gui_interface import create_trackbars
from logging_handler import create_csv_logger, logger as console_logger, save_frame_overlay, save_hsv_snapshot
import sys

# --- ROS Node Initialization ---
if USE_ROS:
    rclpy.init(args=sys.argv)

USE_GUI_SLIDERS = False  # Only place to change for sliders or headless

class PerceptionController:
    """
    Modular camera & perception controller.
    Always gets config via get_params().
    """
    def __init__(self, node_name: str = "perception_controller"):
        if USE_ROS:
            self.ros_node = Node(node_name)
            self.publisher = self.ros_node.create_publisher(String, 'object_data', 10)
        else:
            self.ros_node = None
            self.publisher = None
        self.csv_logger, self.csv_file = create_csv_logger()
        self.spam_mode = False
        self._pipeline = self._build_pipeline()
        self._device = dai.Device(self._pipeline)
        self.q_control = self._device.getInputQueue("control")
        self._last_isp_params = {"Contrast": -1, "Saturation": -1, "Sharpness": -1, "Brightness": -1, "WhiteB": -1}
        self._fx = self._fy = self._cx0 = self._cy0 = 0.0

    def initialize(self) -> None:
        if USE_GUI_SLIDERS:
            cv2.namedWindow(TRACKBAR_WINDOW, cv2.WINDOW_NORMAL)
            create_trackbars(DEFAULT_SLIDER_VALUES, SLIDER_MAX_VALUES)
        else:
            # HEADLESS MODE -- build the param dict using default slider values
            # (You MUST store p for use during all calls that need parameters!)
            self.p = {k: v for (k, v) in DEFAULT_SLIDER_VALUES}
        calib = self._device.readCalibration()
        intr = calib.getCameraIntrinsics(dai.CameraBoardSocket.RGB, 1280, 720)
        self._fx, self._fy = intr[0][0], intr[1][1]
        self._cx0, self._cy0 = intr[0][2], intr[1][2]
        self.q_video = self._device.getOutputQueue("video", maxSize=4, blocking=False)
        self.q_depth = self._device.getOutputQueue("depth", maxSize=4, blocking=False)

    def _set_focus(self, focus_val: int) -> None:
        ctrl = dai.CameraControl()
        if focus_val == 0:
            ctrl.setAutoFocusMode(dai.CameraControl.AutoFocusMode.AUTO)
        else:
            ctrl.setManualFocus(focus_val)
        self.q_control.send(ctrl)

    def _publish_results(self, color: str, results: Any) -> None:
        t = time.time()
        shape = "circle"
        for (score, cx, cy, r, x, y, z) in results:
            msg_text = f"{shape}|{color}|{int(x)},{int(y)},{int(z)}"
            if USE_ROS and self.publisher:
                ros_msg = String()
                ros_msg.data = msg_text
                self.publisher.publish(ros_msg)
            if self.spam_mode:
                self.csv_logger.writerow([t, color, score, r, int(x), int(y), int(z)])

    def _build_pipeline(self) -> dai.Pipeline:
        pipeline = dai.Pipeline()
        cam_rgb = pipeline.createColorCamera()
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
        proc_w, proc_h = 640, 360

        while True:
            in_video = None
            in_depth = None
            while self.q_video.has():
                in_video = self.q_video.get()
            while self.q_depth.has():
                in_depth = self.q_depth.get()
            if in_video is None or in_depth is None:
                in_video = self.q_video.get()
                in_depth = self.q_depth.get()

            if USE_GUI_SLIDERS:
                p = get_params()
            else:
                p = self.p

            # FOCUS FIX HERE
            focus_val = int(p["Focus"])
            if focus_val < 0:
                focus_val = 0
            self._set_focus(focus_val)

            # ISP params, unchanged
            new_vals = {
                "Contrast": p["Contrast"],
                "Saturation": p["Saturation"],
                "Sharpness": p["Sharpness"],
                "Brightness": p["Brightness"],
                "WhiteB": p["WhiteB"]
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

            frame_full = cv2.flip(in_video.getCvFrame(), 0)
            depth_full = cv2.flip(in_depth.getFrame(), 0)
            orig_frame = frame_full.copy()
            proc_frame = cv2.resize(frame_full, (proc_w, proc_h), interpolation=cv2.INTER_AREA)
            proc_depth = cv2.resize(depth_full, (proc_w, proc_h), interpolation=cv2.INTER_NEAREST)

            preprocessed = preprocess(
                frame_bgr=proc_frame,
                use_clahe=(p["CLAHE Clip"] > 0),
                clahe_clip=float(p["CLAHE Clip"]),
                blur_k=int(p["Gaussian K"]),
                use_gray=False
            )

            mask_red, mask_blue = detect_colors(
                preprocessed,
                p["R1 H min"], p["R1 H max"],
                p["R2 H min"], p["R2 H max"],
                p["R S min"],  p["R S max"],
                p["R V min"],  p["R V max"],
                p["B H min"],  p["B H max"],
                p["B S min"],  p["B S max"],
                p["B V min"],  p["B V max"],
                p["Kernel Size"], p["Open Iter"], p["Close Iter"]
            )

            smoothing_alpha = p["Alpha(x100)"] / 100.0
            red_tracked = detect_and_label_circles(
                mask_red, "Red Ball", preprocessed, p["dp (x0.1)"] / 10.0, p["minDist"],
                p["param1"], p["param2"], p["minRadius"], p["maxRadius"],
                smoothing_alpha=smoothing_alpha, pi_mode=not USE_GUI_SLIDERS
            )
            blue_tracked = detect_and_label_circles(
                mask_blue, "Blue Ball", preprocessed, p["dp (x0.1)"] / 10.0, p["minDist"],
                p["param1"], p["param2"], p["minRadius"], p["maxRadius"],
                smoothing_alpha=smoothing_alpha, pi_mode=not USE_GUI_SLIDERS
            )

            frame_count += 1
            now = time.time()
            if now - last_time >= 1.0:
                fps = frame_count / (now - last_time)
                frame_count, last_time = 0, now

            scale_x = orig_frame.shape[1] / proc_w
            scale_y = orig_frame.shape[0] / proc_h

            def upscale_balls(tracked):
                for cx, cy, r in tracked:
                    radius_px = r * ((scale_x + scale_y) / 2)
                    yield (cx * scale_x, cy * scale_y, radius_px)

            red_tracked_full = list(upscale_balls(red_tracked))
            blue_tracked_full = list(upscale_balls(blue_tracked))

            red_results = evaluate_ball(
                "Red Ball", mask_red, orig_frame,
                self._fx, self._fy, self._cx0, self._cy0,
                proc_depth, red_tracked_full
            )
            blue_results = evaluate_ball(
                "Blue Ball", mask_blue, orig_frame,
                self._fx, self._fy, self._cx0, self._cy0,
                proc_depth, blue_tracked_full
            )
            trigger_yolo(frame_full, pi_mode=not USE_GUI_SLIDERS, spam_mode=self.spam_mode)
            self._publish_results("red", red_results)
            self._publish_results("blue", blue_results)

            overlay = orig_frame.copy()
            for (cx, cy, r) in red_tracked_full:
                cv2.circle(overlay, (int(cx), int(cy)), int(r), (0, 0, 255), 2)
            for (cx, cy, r) in blue_tracked_full:
                cv2.circle(overlay, (int(cx), int(cy)), int(r), (255, 0, 0), 2)
            cv2.putText(overlay, f"{fps:.1f} FPS", (10, 32),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (100, 255, 200), 2)
            cv2.imshow("Color Frame", overlay)
            key = cv2.waitKey(1) & 0xFF
            if self._handle_key_press(key, orig_frame, p):
                break

        cv2.destroyAllWindows()
        self.csv_file.close()
        if USE_ROS and self.ros_node is not None:
            self.ros_node.destroy_node()
        if USE_ROS:
            rclpy.shutdown()

    def _handle_key_press(self, key: int, frame: Any, params: Dict[str, float]) -> bool:
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
        elif key == ord('q'):
            return True
        return False

def main():
    controller = PerceptionController()
    controller.initialize()
    controller.run_camera_loop()

if __name__ == "__main__":
    main()