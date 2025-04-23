import cv2
import numpy as np

def apply_clahe(frame_bgr, clip_limit=2.0, tile_grid_size=(8, 8)):
    """
    Applies CLAHE to the L-channel of LAB color space.
    Returns contrast-enhanced BGR image.
    """
    lab = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    clahe = cv2.createCLAHE(clipLimit=clip_limit, tileGridSize=tile_grid_size)
    l_eq = clahe.apply(l)
    lab_eq = cv2.merge((l_eq, a, b))
    return cv2.cvtColor(lab_eq, cv2.COLOR_LAB2BGR)

def apply_gaussian_blur(frame_bgr, kernel_size=5):
    """
    Applies Gaussian blur to the BGR image.
    Kernel size must be odd.
    """
    k = kernel_size if kernel_size % 2 == 1 else kernel_size + 1
    return cv2.GaussianBlur(frame_bgr, (k, k), 0)

def to_grayscale(frame_bgr):
    """
    Converts BGR to grayscale.
    """
    return cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
