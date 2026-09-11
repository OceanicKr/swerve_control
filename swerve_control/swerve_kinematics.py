#frame convention: x = forward, y = left
import math

L = 1.0
W = 0.6

WHEEL_POSITIONS = {
    "RF": (L / 2, -W / 2),
    "RR": (-L / 2, -W / 2),
    "LF": (L / 2, W / 2),
    "LR": (-L / 2, W / 2),
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

def optimize_wheel(desired_angle_deg, desired_speed, last_angle_deg, speed_deadband=1e-3):
    if desired_speed < speed_deadband:
        return last_angle_deg, 0.0

    angle = normalize_angle(desired_angle_deg)
    if angle > 90.0:
        angle -= 180.0
        desired_speed = -desired_speed
    elif angle < -90.0:
        angle += 180.0
        desired_speed = -desired_speed

    return angle, desired_speed

if __name__ == "__main__":
    cases = {
        "Straight forward (vx=1)": (1.0, 0.0, 0.0),
        "Straight backward (vx=-1)": (-1.0, 0.0, 0.0),
        "Pure strafe left (vy=1)": (0.0, 1.0, 0.0),
        "Pure pivot CCW (wz=1)": (0.0, 0.0, 1.0),
        "Diagonal (vx=1, vy=1)": (1.0, 1.0, 0.0),
    }
    for label, (vx, vy, wz) in cases.items():
        print(f"\n{label}: vx={vx}, vy={vy}, wz={wz}")
        for name, (angle, speed) in zip(WHEEL_ORDER, compute_wheel_commands(vx, vy, wz)):
            print(f"  {name}: angle={angle:7.2f} deg  speed={speed:.3f}")

    print("\noptimizer sequence test")
    last_angle = 0.0
    sequence = [
        ("forward", (1.0, 0.0, 0.0)),
        ("stop", (0.0, 0.0, 0.0)),
        ("backward", (-1.0, 0.0, 0.0)),
        ("stop again", (0.0, 0.0, 0.0)),
        ("strafe left", (0.0, 1.0, 0.0)),
    ]
    for label, (vx, vy, wz) in sequence:
        raw_angle, raw_speed = compute_wheel_commands(vx, vy, wz)[0]
        opt_angle, opt_speed = optimize_wheel(raw_angle, raw_speed, last_angle)
        print(f"{label:15s} raw=({raw_angle:7.2f}, {raw_speed:.3f})  "
              f"-> optimized=({opt_angle:7.2f}, {opt_speed:.3f})  "
              f"[was at {last_angle:.2f}]")
        last_angle = opt_angle
