# preprocessing.py

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

def apply_gaussian_blur(frame, kernel_size=5):
    """
    Applies Gaussian blur to the image.
    Works for grayscale or BGR.
    Kernel size must be odd.
    """
    k = kernel_size if kernel_size % 2 == 1 else kernel_size + 1
    return cv2.GaussianBlur(frame, (k, k), 0)

def apply_grayscale(frame):
    """Robustly convert to grayscale only if input has 3 channels."""
    if len(frame.shape) == 2:  # Already grayscale/mask
        return frame
    elif frame.shape[2] == 3:
        return cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    else:
        raise ValueError("Invalid image shape for grayscale conversion")

def preprocess(frame_bgr, use_clahe=True, clahe_clip=2.0, blur_k=5, use_gray=False):
    """
    Full preprocessing pipeline for BGR frame.
    Each step can be toggled.
    Returns preprocessed frame (grayscale or BGR as needed).
    """
    out = frame_bgr.copy()
    if use_clahe:
        out = apply_clahe(out, clip_limit=clahe_clip)
    if blur_k > 0:
        out = apply_gaussian_blur(out, kernel_size=blur_k)
    if use_gray:
        out = apply_grayscale(out)
    return out