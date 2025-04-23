import cv2
import subprocess
import threading
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


# Label expectations
YOLO_LABELS = {
    "0": "red",
    "1": "blue"
}

# Shared state from detection
#from detection import prev_circles, verified_flags, LABEL_MAP

# YOLO tracking state
yolo_detections = []
yolo_in_progress = False
yolo_thread = None

def parse_yolo_output(output_str):
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
            except Exception as e:
                log.warn(f"Failed to parse YOLO line: {line} -> {e}")
    return detections

def is_verified_by_yolo(cx, cy, det_list, color_name, margin=15):
    expected = LABEL_MAP[color_name].lower()
    for det in det_list:
        if det["label"].lower() == expected:
            if (det["left"] - margin <= cx <= det["right"] + margin) and \
               (det["top"] - margin <= cy <= det["bottom"] + margin):
                return True
    return False

def yolo_inference(frame):
    global yolo_detections, yolo_in_progress

    _, img_encoded = cv2.imencode('.jpg', frame)
    img_bytes = img_encoded.tobytes()

    cmd = [
        "/absolute/path/to/darknet",  # CHANGE THIS
        "detector", "test",
        "data/obj.data",
        "cfg/yolov4-tiny-obj.cfg",
        "backup/yolov4-tiny_last.weights",
        "-",
        "-dont_show", "-ext_output", "-thresh", "0.01"
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
    except Exception as e:
        log.error(f"YOLO subprocess failed: {e}")
        yolo_in_progress = False
        return

    detections = parse_yolo_output(stdout_text)
    yolo_detections.clear()
    yolo_detections.extend(detections)

    for color_name in ["Red Ball", "Blue Ball"]:
        for i, (cx, cy, rr) in enumerate(prev_circles[color_name]):
            match = is_verified_by_yolo(cx, cy, yolo_detections, color_name, margin=30)
            if match:
                verified_flags[color_name][i] = True

    yolo_in_progress = False

def trigger_yolo(frame):
    global yolo_in_progress, yolo_thread
    if not yolo_in_progress:
        yolo_in_progress = True
        yolo_thread = threading.Thread(target=yolo_inference, args=(frame,))
        yolo_thread.start()
