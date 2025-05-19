import math
import numpy as np

def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))

def rotation_matrix(axis, angle_rad):
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    c = math.cos(angle_rad)
    s = math.sin(angle_rad)
    C = 1 - c

    return np.array([
        [c + x**2*C, x*y*C - z*s, x*z*C + y*s],
        [y*x*C + z*s, c + y**2*C, y*z*C - x*s],
        [z*x*C - y*s, z*y*C + x*s, c + z**2*C]
    ])

def forward_kinematics_4dof(a_deg, b_deg, c_deg, d_deg, L, M, N):
    a = math.radians(a_deg)
    b = math.radians(b_deg)
    c = math.radians(c_deg)
    d = math.radians(d_deg)

    pos = np.array([0.0, 0.0, 0.0])
    R = np.eye(3)

    R = R @ rotation_matrix(np.array([0,0,1]), d)
    R = R @ rotation_matrix(np.array([0,1,0]), a)
    pos += R @ np.array([0, 0, L])

    R = R @ rotation_matrix(np.array([0,1,0]), b)
    pos += R @ np.array([0, 0, M])

    R = R @ rotation_matrix(np.array([0,1,0]), c)
    pos += R @ np.array([0, 0, N])

    return pos

def to_servo_angle(angle_deg):
    return (angle_deg + 90) % 360

def calculate_4dof_angles_elbow_up(x, y, z, L, M, N, limits=None):
    R_xy = math.hypot(x, y)
    s = R_xy - N
    Q = math.hypot(s, z)
    theta_base = math.atan2(y, x)

    if Q > (L + M):
        raise ValueError("Target out of reach")

    f = math.atan2(z, s)
    cos_theta3 = ((L**2 + M**2 - Q**2) / (2 * L * M))
    cos_theta3 = clamp(cos_theta3, -1, 1)
    theta3_options = [math.acos(cos_theta3), -math.acos(cos_theta3)]

    solutions = []
    for theta3 in theta3_options:
        cos_g = ((L**2 + Q**2 - M**2) / (2 * L * Q))
        cos_g = clamp(cos_g, -1, 1)
        g = math.acos(cos_g)
        theta_shoulder = f + g

        theta_wrist = - (theta_shoulder + theta3) + 2 * math.pi

        # Normalize to [0, 360)
        a_deg = math.degrees(theta_shoulder) % 360
        b_deg = math.degrees(theta3) % 360
        c_deg = math.degrees(theta_wrist) % 360
        d_deg = math.degrees(theta_base) % 360

        # Apply joint limits (robot frame!)
        if limits:
            a_deg = clamp(a_deg, *limits["a"])
            b_deg = clamp(b_deg, *limits["b"])
            c_deg = clamp(c_deg, *limits["c"])
            d_deg = clamp(d_deg, *limits["d"])

        # Elbow height test (robot frame)
        a_rad = math.radians(a_deg)
        d_rad = math.radians(d_deg)
        R_base = rotation_matrix(np.array([0,0,1]), d_rad)
        R_shoulder = rotation_matrix(np.array([0,1,0]), a_rad)
        elbow_pos = R_base @ R_shoulder @ np.array([0, 0, L])
        if elbow_pos[2] > 0:
            # FK error check
            fk_pos = forward_kinematics_4dof(a_deg, b_deg, c_deg, d_deg, L, M, N)
            target = np.array([x, y, z])
            error = np.linalg.norm(fk_pos - target)
            solutions.append((error, (a_deg, b_deg, c_deg, d_deg)))

    if not solutions:
        raise ValueError("No elbow-up solution found")

    solutions.sort(key=lambda s: s[0])
    best_robot_angles = solutions[0][1]

    # Final step: convert a,b,c to servo angles
    a_servo = to_servo_angle(best_robot_angles[0])
    b_servo = to_servo_angle(best_robot_angles[1])
    c_servo = to_servo_angle(best_robot_angles[2])
    d_servo = best_robot_angles[3]  # base remains raw

    return (a_servo, b_servo, c_servo, d_servo)

# -------------------
# Example test program
# -------------------
if __name__ == "__main__":
    L = 208
    M = 222
    N = 240

    # ✅ Original joint limits restored
    joint_limits = {
        "a": (70, 260),    # shoulder limits
        "b": (70, 290),    # elbow limits
        "c": (70, 290),    # wrist limits
        "d": (0, 360)      # base unlimited
    }

    x_target = 0
    y_target = 400
    z_target = 0

    servo_angles = calculate_4dof_angles_elbow_up(x_target, y_target, z_target, L, M, N, joint_limits)

    print(f"\nServo angles (final output only):")
    print(f"  Shoulder servo : {servo_angles[0]:.2f}°")
    print(f"  Elbow servo    : {servo_angles[1]:.2f}°")
    print(f"  Wrist servo    : {servo_angles[2]:.2f}°")
    print(f"  Base servo     : {servo_angles[3]:.2f}°")

    # Validate FK
    robot_angles = [(servo_angles[0] - 90) % 360, (servo_angles[1] - 90) % 360,
                    (servo_angles[2] - 90) % 360, servo_angles[3]]
    pos = forward_kinematics_4dof(*robot_angles, L, M, N)
    print(f"\nEnd effector position: {pos}")

    target = np.array([x_target, y_target, z_target])
    error = np.linalg.norm(pos - target)
    print(f"Validation error: {error:.4f} mm\n")
