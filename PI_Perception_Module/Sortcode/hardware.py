"""
Module: hardware.py
Handles: DepthAI camera setup and access
Owns: Frame capture, resolution config, ISP controls, focus setting
Calls: DepthAI API
Does not contain: detection, tracking, or result publishing
"""

from config import (
    CAMERA_WIDTH,
    CAMERA_HEIGHT,
    CAMERA_FPS,
    CAMERA_STILL_SIZE,
    CAMERA_DEPTH_RESOLUTION,
    CAMERA_CONFIDENCE_THRESHOLD,
    CAMERA_MEDIAN_FILTER,
    CAMERA_STEREO_ALIGNMENT,
)
import depthai as dai


class DepthAICamera:
    def __init__(self, width=CAMERA_WIDTH, height=CAMERA_HEIGHT, fps=CAMERA_FPS):
        self.width = width
        self.height = height
        self.fps = fps
        self.device = None
        self.q_video = None
        self.q_depth = None
        self.q_control = None
        self._fx = self._fy = self._cx0 = self._cy0 = 0.0

    def start(self):
        self.device = dai.Device(self.create_pipeline())
        self.q_video = self.device.getOutputQueue("video", maxSize=4, blocking=False)
        self.q_depth = self.device.getOutputQueue("depth", maxSize=4, blocking=False)
        self.q_control = self.device.getInputQueue("control")
        self.load_calibration()

    def create_pipeline(self):
        pipeline = dai.Pipeline()

        cam_rgb = pipeline.createColorCamera()
        cam_rgb.setPreviewSize(self.width, self.height)
        cam_rgb.setVideoSize(self.width, self.height)
        cam_rgb.setStillSize(*CAMERA_STILL_SIZE)
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
        mono_left.setResolution(
            getattr(dai.MonoCameraProperties.SensorResolution, CAMERA_DEPTH_RESOLUTION)
        )
        mono_right.setResolution(
            getattr(dai.MonoCameraProperties.SensorResolution, CAMERA_DEPTH_RESOLUTION)
        )
        mono_left.setBoardSocket(dai.CameraBoardSocket.LEFT)
        mono_right.setBoardSocket(dai.CameraBoardSocket.RIGHT)
        mono_left.setFps(self.fps)
        mono_right.setFps(self.fps)

        stereo = pipeline.createStereoDepth()
        stereo.initialConfig.setConfidenceThreshold(CAMERA_CONFIDENCE_THRESHOLD)
        stereo.initialConfig.setMedianFilter(
            getattr(dai.MedianFilter, CAMERA_MEDIAN_FILTER)
        )
        stereo.setLeftRightCheck(True)
        stereo.setSubpixel(True)
        stereo.setOutputSize(self.width, self.height)
        mono_left.out.link(stereo.left)
        mono_right.out.link(stereo.right)
        
        align_socket = getattr(dai.CameraBoardSocket, CAMERA_STEREO_ALIGNMENT)
        stereo.setDepthAlign(align_socket)  

        xout_depth = pipeline.createXLinkOut()
        xout_depth.setStreamName("depth")
        stereo.depth.link(xout_depth.input)

        return pipeline

    def load_calibration(self):
        calib = self.device.readCalibration()
        socket = getattr(dai.CameraBoardSocket, CAMERA_STEREO_ALIGNMENT)
        intr   = calib.getCameraIntrinsics(socket, self.width, self.height)
        self._fx, self._fy = intr[0][0], intr[1][1]
        self._cx0, self._cy0 = intr[0][2], intr[1][2]


    def get_intrinsics(self):
        return self._fx, self._fy, self._cx0, self._cy0

    def get_latest_frames(self):
        in_video = in_depth = None
        while self.q_video.has():
            in_video = self.q_video.get()
        while self.q_depth.has():
            in_depth = self.q_depth.get()
        if in_video is None:
            in_video = self.q_video.get()
        if in_depth is None:
            in_depth = self.q_depth.get()
        return in_video, in_depth

    def set_focus(self, focus_val):
        ctrl = dai.CameraControl()
        if focus_val <= 0:
            ctrl.setAutoFocusMode(dai.CameraControl.AutoFocusMode.AUTO)
        else:
            ctrl.setManualFocus(int(focus_val))
        self.q_control.send(ctrl)

    def shutdown(self):
        if self.device:
            del self.device
