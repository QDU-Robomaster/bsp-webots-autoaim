from controller import Supervisor
import math


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def raised_cosine_pulse(seconds, period_s, center_s, half_width_s):
    phase = seconds % period_s
    delta = abs(phase - center_s)
    delta = min(delta, period_s - delta)
    if delta >= half_width_s:
        return 0.0
    return 0.5 * (1.0 + math.cos(math.pi * delta / half_width_s))


def alternating_pulse(seconds, period_s, center_s, half_width_s):
    sign = 1.0 if int(math.floor(seconds / period_s)) % 2 == 0 else -1.0
    return sign * raised_cosine_pulse(seconds, period_s, center_s, half_width_s)


def smoothstep(value):
    value = clamp(value, 0.0, 1.0)
    return value * value * (3.0 - 2.0 * value)


def main():
    robot = Supervisor()
    step_ms = int(robot.getBasicTimeStep())
    dt = step_ms / 1000.0

    self_node = robot.getSelf()
    translation_field = self_node.getField("translation")
    if translation_field is None:
        raise RuntimeError("target translation field not found")

    spin_node = robot.getFromDef("TARGET_SPIN")
    if spin_node is None:
        raise RuntimeError("TARGET_SPIN DEF not found")
    spin_rotation_field = spin_node.getField("rotation")
    if spin_rotation_field is None:
        raise RuntimeError("TARGET_SPIN rotation field not found")

    origin = translation_field.getSFVec3f()
    origin_x = origin[0]
    origin_y = origin[1]
    origin_z = origin[2]

    spin_phase = 0.0
    next_log_s = 0.0
    prev_x = origin_x
    prev_y = origin_y

    def raw_x_offset(seconds):
        return (
            0.34 * math.sin(2.0 * math.pi * seconds / 3.2)
            + 0.11 * math.sin(2.0 * math.pi * seconds / 1.05 + 0.6)
            + 0.10 * alternating_pulse(seconds + 0.45, 3.4, 0.55, 0.32)
        )

    def raw_y_offset(seconds):
        return (
            0.23 * math.sin(2.0 * math.pi * seconds / 4.7 + 1.1)
            + 0.09 * math.sin(2.0 * math.pi * seconds / 1.7 + 2.4)
            - 0.12 * alternating_pulse(seconds + 1.35, 4.1, 0.70, 0.42)
        )

    while robot.step(step_ms) != -1:
        t = robot.getTime()
        startup = smoothstep(t / 1.0)
        x_offset = startup * raw_x_offset(t)
        y_offset = startup * raw_y_offset(t)
        x = origin_x + clamp(x_offset, -0.52, 0.52)
        y = origin_y + clamp(y_offset, -0.42, 0.42)

        spin_v = (
            3.4
            + 1.6 * math.sin(2.0 * math.pi * t / 3.9)
            + 0.9 * math.sin(2.0 * math.pi * t / 1.4 + 0.7)
            + 2.2 * alternating_pulse(t + 0.70, 5.6, 0.85, 0.50)
            - 1.4 * raised_cosine_pulse(t + 2.0, 6.8, 0.35, 0.65)
        )
        spin_v = clamp(spin_v, -1.2, 7.2)
        spin_phase += spin_v * dt

        translation_field.setSFVec3f([x, y, origin_z])
        spin_rotation_field.setSFRotation([0, 0, 1, spin_phase])

        if t >= next_log_s:
            speed_xy = math.hypot(x - prev_x, y - prev_y) / dt if dt > 0.0 else 0.0
            print(
                f"[target-maneuver] t={t:.2f}s x={x:.3f}m y={y:.3f}m "
                f"speed_xy={speed_xy:.3f}m/s omega_cmd={spin_v:.3f}rad/s "
                f"phase={spin_phase:.3f}rad"
            )
            next_log_s = t + 0.5
        prev_x = x
        prev_y = y


if __name__ == "__main__":
    main()

