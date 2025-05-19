"""
kinematics.py
─────────────
Coordinate conversion + inverse kinematics for Fenris arm.

• transform_camera_to_robot_frame()
    swaps Y/Z to match robot frame
    applies the fixed camera offset
• solve_ik()              (optional)
    returns joint angles (deg) if you want to drive servos directly
• All steps are trace-logged with print() to stdout.
"""

# ──────────  constants ──────────

CAMERA_LEFT_MM = 228.0
CAMERA_ABOVE_BASE_MM  = 170.5          # <- was 203 → 283 (+80 mm)


# ────────── convert DepthAI 3D → robot base frame ──────────
def transform_camera_to_robot_frame(x_mm, y_mm, z_mm):
    x_r = x_mm - CAMERA_LEFT_MM
    y_r = y_mm
    z_r = CAMERA_ABOVE_BASE_MM - z_mm
    #print(f"[robot frame] x={x_r:.1f}, y={y_r:.1f}, z={z_r:.1f}")
    return x_r, y_r, z_r

