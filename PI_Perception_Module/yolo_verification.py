# yolo_verification.py

import cv2
import subprocess
import threading
from config import YOLO_CFG, YOLO_WEIGHTS, YOLO_DATA

try:
    from rclpy.logging import get_logger
    log = get_logger("module_name")
except ImportError:
    class DummyLogger:
        def debug(self, msg): print("[DEBUG]", msg)
        def info(self, msg): print("[INFO]", msg)
        def warning(self, msg): print("[WARN]", msg)
        def error(self, msg): print("[ERROR]", msg)
    log = DummyLogger()

YOLO_LABELS = {
    "0": "red",
    "1": "blue"
}
yolo_detections = []
yolo_in_progress = False
yolo_thread = None

# yolo_verification.py

def parse_yolo_output(output_str, spam_mode=False):
    detections = []
    for line in output_str.splitlines():
        if "%" in line and "left_x" in line:
            try:
                parts = line.split()
                class_id = parts[0].rstrip(':')
                label = YOLO_LABELS.get(class_id, class_id)
                conf = float(parts[1].strip('%'))
                left = int(parts[3])
                top = int(parts[5])
                right = left + int(parts[7])
                bottom = top + int(parts[9].rstrip(')'))
                detections.append({
                    "label": label,
                    "conf": conf,
                    "left": left,
                    "top": top,
                    "right": right,
                    "bottom": bottom,
                    "cx": (left + right) // 2,
                    "cy": (top + bottom) // 2
                })
                if spam_mode:
                    print(f"[YOLO][SPAM] Detected: label={label}, conf={conf}%, box=({left},{top},{right},{bottom})")
            except Exception as e:
                print(f"[YOLO][SPAM][ERROR] Could not parse line: '{line}' Exception: {e}")
    return detections

def yolo_inference(frame, pi_mode=False, spam_mode=False):
    global yolo_detections, yolo_in_progress
    if pi_mode:
        yolo_in_progress = False
        return
    _, img_encoded = cv2.imencode('.jpg', frame)
    img_bytes = img_encoded.tobytes()
    cmd = [
    "/Users/azi/Desktop/Sortify/darknet/darknet", "detector", "test",
    YOLO_DATA, YOLO_CFG, YOLO_WEIGHTS,
    "-", "-dont_show", "-ext_output", "-thresh", "0.25"
]
    try:
        result = subprocess.run(
            cmd,
            input=img_bytes,
            capture_output=True,
            text=False,
            timeout=10
        )
        stdout_text = result.stdout.decode('utf-8', errors='replace')
        if spam_mode:
            print("[YOLO][SPAM] YOLO subprocess completed.")
            print("[YOLO][SPAM] Raw output:")
            print(stdout_text)
    except Exception as e:
        print(f"[YOLO][SPAM][FAIL] YOLO subprocess failed: {e}")
        yolo_in_progress = False
        return
    detections = parse_yolo_output(stdout_text, spam_mode)
    if spam_mode:
        if detections:
            print(f"[YOLO][SPAM] Parsed {len(detections)} detections.")
        else:
            print("[YOLO][SPAM] No objects detected by YOLO.")
    yolo_detections.clear()
    yolo_detections.extend(detections)
    yolo_in_progress = False

def trigger_yolo(frame, pi_mode=False, spam_mode=False):
    global yolo_in_progress, yolo_thread
    if pi_mode:
        return
    if not yolo_in_progress:
        yolo_in_progress = True
        yolo_thread = threading.Thread(target=yolo_inference, args=(frame, pi_mode, spam_mode))
        yolo_thread.start()
        if spam_mode:
            print("[YOLO][SPAM] YOLO detection thread started.")