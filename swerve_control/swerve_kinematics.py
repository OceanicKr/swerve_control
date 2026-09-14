#frame convention: x = forward, y = left
import math

L = 0.7777
W = 0.475

WHEEL_POSITIONS = {
    "LF": (L / 2, W / 2),
    "LR": (-L / 2, W / 2),
    "RF": (L / 2, -W / 2),
    "RR": (-L / 2, -W / 2),
}

WHEEL_ORDER = ["RF", "RR", "LF", "LR"]

def compute_wheel_commands(vx, vy, wz):
    results = []
    for name in WHEEL_ORDER:
        x_i, y_i = WHEEL_POSITIONS[name]
        vx_i = vx - wz * y_i      # rigid-body velocity at a point: v_i = (v_center + w * r_i) in 2D, treating wz as a vector along +z,
        vy_i = vy + wz * x_i      # the cross product; w x r_i = wz*z_cap x (x_i, y_i, 0) = (-wz*y_i, wz*x_i)
        speed = math.hypot(vx_i, vy_i)
        angle_deg = math.degrees(math.atan2(vy_i, vx_i))
        results.append((angle_deg, speed))
    return results

def normalize_angle(angle_deg):
    angle_deg = angle_deg % 360.0
    if angle_deg > 180.0:
        angle_deg -= 360.0
    return angle_deg

def ramp_value(current, target, max_delta):
    # to slowly ramp a value from current to target, with a maximum change of max_delta
    if max_delta <= 0.0: return target
    if target > current: return min(target, current + max_delta)
    else: return max(target, current - max_delta)

def optimize_wheel(desired_angle_deg, desired_speed, last_angle_deg, speed_deadband=1e-3, boundary_tol=1e-6):
    if desired_speed < speed_deadband:
        return last_angle_deg, 0.0
 
    angle = normalize_angle(desired_angle_deg)
    if angle > 90.0:
        angle -= 180.0
        desired_speed = -desired_speed
    elif angle < -90.0:
        angle += 180.0
        desired_speed = -desired_speed

    # crab fix
    if abs(abs(angle) - 90.0) < boundary_tol:
        alt_angle = -angle
        current_dist = abs(normalize_angle(angle - last_angle_deg))
        alt_dist = abs(normalize_angle(alt_angle - last_angle_deg))
        if alt_dist < current_dist:
            angle = alt_angle
            desired_speed = -desired_speed
 
    return angle, desired_speed
