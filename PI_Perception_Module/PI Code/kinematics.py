"""
kinematics.py
─────────────
Coordinate conversion + inverse kinematics for Fenris arm.

• transform_camera_to_robot_frame()
    – swaps Y/Z to match robot’s frame
    – applies the fixed camera offset
• solve_ik()              (optional)
    – returns joint angles (deg) if you want to drive servos directly
• All steps are trace‑logged with print() to stdout.
"""

import math
from typing import Tuple, Dict

# ────────── geometry constants ──────────
L1, L2, L3 = 210.0, 190.0, 220.0

CAMERA_LEFT_MM = 0
CAMERA_ABOVE_BASE_MM  = 220.0          # <- was 203 → 283 (+80 mm)

# ────────── quick‑and‑dirty trims (deg) ──────────
#  +  = push the servo forward (towards bigger numbers you read today)
#  –  = pull it back
SERVO_TRIM = {
    "base":     21,   # leave base alone
    "shoulder": 0, # drop ~10°  → 130 → 120
    "elbow":    0,  # you said elbow is fine now
    "wrist":   0,  # drop ~15°  → 160 → 145
}


# ────────── convert DepthAI 3D → robot base frame ──────────
def transform_camera_to_robot_frame(
    xc: float, yc: float, zc: float
) -> Tuple[float, float, float]:

    print(f"[cam input]   x={xc:.1f}, y={yc:.1f}, z={zc:.1f}")

    # swap axes
    x_r = xc - CAMERA_LEFT_MM        # not  + CAMERA_LEFT_MM
    y_r = zc                     # forward / back
    z_r = CAMERA_ABOVE_BASE_MM - yc  # not  yc - CAMERA_ABOVE_BASE_MM

    # translate: camera is 100 mm left of base centre
    #x_r += CAMERA_LEFT_MM

    print(f"[robot frame] x={x_r:.1f}, y={y_r:.1f}, z={z_r:.1f}")
    return x_r, y_r, z_r

# ------------- joint limits (deg) -----------------
LIMITS = {
    "base":     (0.0, 360.0),
    "shoulder": (70.0, 260.0),
    "elbow":    (70.0, 290.0),
    "wrist":    (70.0, 290.0),
}

def _inside(name: str, angle: float) -> bool:
    lo, hi = LIMITS[name]
    return lo <= angle <= hi


# --- IK (returns legal servo angles) -----------------------------
def solve_ik(x: float, y: float, z: float) -> Dict[str, float]:
    print(f"[IK input]    robot x={x:.2f}, y={y:.2f}, z={z:.2f}")

    r_total = math.hypot(x, y)
    r       = r_total - L3                     # horizontal reach in arm‑plane
    s       = z                                # vertical
    print(f"[IK planar]   r_total={r_total:.2f}, r={r:.2f}, s={s:.2f}")

    # ------------ base rotation (servo 0) --------------------------
    theta_base  = -math.atan2(x, y)             # rad
    servo_base  = 90.0 + math.degrees(theta_base)

    # ------------ elbow helper ------------------------------------
    c2 = (r*r + s*s - L1*L1 - L2*L2) / (2.0*L1*L2)
    if abs(c2) > 1.0:
        print("[FAIL] target outside workspace")
        return {j: 180.0 for j in LIMITS}      # dummy fail pose

    solutions = []
    for sign in (+1, -1):                      # elbow‑up / elbow‑down
        theta_elbow   = sign * math.acos(c2)
        servo_elbow   = 180.0 - math.degrees(theta_elbow)

        # correct shoulder equation  (note the **minus**)
        theta_shoulder = math.atan2(s, r) + -math.atan2(
            L2*math.sin(theta_elbow),
            L1 + L2*math.cos(theta_elbow)
        )
        servo_shoulder = 180.0 + math.degrees(theta_shoulder)

        # keep end‑effector vertical
        theta_wrist  = math.pi/2 + (theta_shoulder - theta_elbow)
        servo_wrist  = 180.0 + math.degrees(theta_wrist)

        angles = {
            "base":     round(servo_base,     2),
            "shoulder": round(servo_shoulder, 2),
            "elbow":    round(servo_elbow,    2),
            "wrist":    round(servo_wrist,    2),
        }
        solutions.append(angles)


    # pick the first solution inside limits
    for ang in solutions:
        if all(_inside(j, v) for j, v in ang.items()):
            # --- apply per‑joint trim here ------------------------
            for j, off in SERVO_TRIM.items():
                ang[j] = round(ang[j] + off, 2)
            print(f"[IK output] {ang}")
            return ang



    raise ValueError("Unreachable in servo limits")
