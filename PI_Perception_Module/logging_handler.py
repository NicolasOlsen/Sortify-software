import csv
import os
import time
from datetime import datetime
import cv2
import logging
from typing import Tuple, TextIO

logger = logging.getLogger("perception_logger")
logger.setLevel(logging.INFO)
console_handler = logging.StreamHandler()
console_handler.setLevel(logging.INFO)
logger.addHandler(console_handler)

def create_csv_logger() -> Tuple[csv.writer, TextIO]:
    """
    Creates a CSV writer and file handle for logging detection data.

    Returns:
        (writer, file_handle): CSV writer and the opened file object.
    """
    filename = f"detection_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    f = open(filename, "w", newline="")
    writer = csv.writer(f)
    writer.writerow(["timestamp", "color", "score", "circle_count", "x_mm", "y_mm", "z_mm"])
    logger.info(f"CSV log initialized: {filename}")
    return writer, f

def save_frame_overlay(frame, prefix: str = "frame_overlay") -> None:
    """
    Saves the overlayed frame as an image.

    Args:
        frame: BGR image to save.
        prefix: String prefix to use for filename.
    """
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    filename = f"{prefix}_{timestamp}.png"
    cv2.imwrite(filename, frame)
    logger.info(f"Frame saved as {filename}")

def save_hsv_snapshot(frame, params: dict, folder: str, prefix: str = "HSV_TEST") -> None:
    """
    Saves the current frame (for HSV debugging) and a text file with parameters.

    Args:
        frame: The BGR image to save.
        params: Dictionary of HSV/tracking parameters to record.
        folder: Folder path to save into.
        prefix: File prefix for naming.
    """
    os.makedirs(folder, exist_ok=True)
    uid = time.strftime("%Y%m%d_%H%M%S")
    img_filename = f"{folder}/{prefix}_{uid}.png"
    param_filename = f"{folder}/{prefix}_PARAMETERS_{uid}.txt"
    cv2.imwrite(img_filename, frame)
    with open(param_filename, "w") as f:
        for k, v in params.items():
            f.write(f"{k}: {v}\n")

    logger.info(f"[PARAMETER SNAPSHOT] Saved: {img_filename}, {param_filename}")
