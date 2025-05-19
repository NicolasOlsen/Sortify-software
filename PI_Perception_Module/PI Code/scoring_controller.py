"""
Module: scoring_controller
Handles: Per-object scoring & accept/reject decision
"""

from config import SLIDER_CONFIG
from gui_interface import get_runtime_params



class DecisionResult:
    def __init__(self, accepted: bool, reason: str, label: str):
        self.accepted = accepted
        self.reason = reason
        self.label = label

    @staticmethod
    def weights(params=None) -> dict:
        if params is None:
            params = get_runtime_params()
        return {k: params.get(k, SLIDER_CONFIG[k]["default"])
                for k in ("Color Weight","Shape Weight","AI Weight",
                           "Depth Weight","Depth Mask Weight","Tracker Weight",
                           "Decision Accept Threshold")}

    @staticmethod
    def get_decision(raw):
        w = DecisionResult.weights()

        label = f"{raw['color']}_{raw['shape']}"
        score = 0.0
        score += (1 if raw.get("color_valid") else 0) * w["Color Weight"]
        score += (1 if raw.get("shape_valid") else 0) * w["Shape Weight"]
        score += (1 if raw.get("ai_valid") else 0) * w["AI Weight"]
        score += (1 if raw.get("depth_valid") else 0) * w["Depth Weight"]
        score += (1 if raw.get("depth_mask_valid") else 0) * w["Depth Mask Weight"]
        score += (1 if raw.get("tracker_valid") else 0) * w["Tracker Weight"]

        threshold = w["Decision Accept Threshold"]
        if score >= threshold:
            return DecisionResult(True, "ok", label)
        return DecisionResult(False, "low_score", label)


def merge_detection_info(obj, shape_data, pos_data):

    merged = dict(shape_data)
    merged.update(
        {
            "x_mm": pos_data["x_mm"],
            "y_mm": pos_data["y_mm"],
            "z_mm": pos_data["z_mm"],
            "depth_valid": pos_data["depth_valid"],
            "depth_mask_valid": pos_data["depth_mask_valid"],
            "color_valid": obj.get("color") is not None,
            "shape_valid": obj.get("shape") is not None,
            "color": obj.get("color"),
            "shape": obj.get("shape"),
            "ai_valid": shape_data.get("ai_valid", False),
            "tracker_valid": False,
        }
    )
    return merged
