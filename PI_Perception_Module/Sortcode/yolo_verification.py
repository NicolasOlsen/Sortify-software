"""
Module: yolo_verification.py
Handles: External YOLO detection (optional)
Owns: YOLO process call, output parsing, structured verification result
"""

import cv2
import subprocess
import tempfile
import os
from typing import List, Dict
from config import YOLO_CFG, YOLO_WEIGHTS, YOLO_DATA, ENABLE_YOLO


try:
    from rclpy.logging import get_logger

    log = get_logger("yolo_verification")
except ImportError:

    class DummyLogger:
        def debug(self, msg):
            print("DEBUG ", msg)

        def info(self, msg):
            print("INFO ", msg)

        def warning(self, msg):
            print("WARN ", msg)

        def error(self, msg):
            print("ERROR ", msg)

    log = DummyLogger()

YOLO_LABELS = {"0": "red", "1": "blue"}




def parse_yolo_output(output_str: str) -> List[Dict]:
    results = []
    for line in output_str.splitlines():
        if "%" in line and "left_x" in line:
            try:
                parts = line.split()
                class_id = parts[0].rstrip(":")
                label = YOLO_LABELS.get(class_id, class_id)
                conf = float(parts[1].strip("%"))
                left = int(parts[3])
                top = int(parts[5])
                right = left + int(parts[7])
                bottom = top + int(parts[9].rstrip(")"))
                results.append(
                    {
                        "label": label,
                        "conf": conf,
                        "left": left,
                        "top": top,
                        "right": right,
                        "bottom": bottom,
                        "cx": (left + right) // 2,
                        "cy": (top + bottom) // 2,
                    }
                )
            except Exception as e:
                log.warning(f"Failed to PARSE LINE: '{line}' ({e})")
    return results


def run_yolo_inference(image_bgr) -> List[Dict]:
    if not ENABLE_YOLO:
        return []

    image_resized = cv2.resize(image_bgr, (416, 416))
    with tempfile.NamedTemporaryFile(suffix=".jpg", delete=False) as tmp:
        temp_filename = tmp.name
        cv2.imwrite(temp_filename, image_resized)

    cmd = [
        "/Users/azi/Desktop/Sortify/darknet/darknet",
        "detector",
        "test",
        YOLO_DATA,
        YOLO_CFG,
        YOLO_WEIGHTS,
        temp_filename,
        "-dont_show",
        "-ext_output",
        "-thresh",
        "0.10",
    ]

    try:
        result = subprocess.run(cmd, capture_output=True, timeout=10)
        stdout_str = result.stdout.decode("utf-8", errors="replace")
    except Exception as e:
        log.error(f"Subprocess FAILED: {e}")
        return []
    finally:
        os.remove(temp_filename)

    return parse_yolo_output(stdout_str)


def verify_with_yolo_crop(image_bgr) -> Dict[str, bool]:
    detections = run_yolo_inference(image_bgr)
    return {"ai_valid": len(detections) > 0}
