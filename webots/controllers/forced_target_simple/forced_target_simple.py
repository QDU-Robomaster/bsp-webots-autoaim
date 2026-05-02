from controller import Supervisor
import math
import os


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
    origin_y = origin[1]
    origin_z = origin[2]

    travel = float(os.environ.get("XR_TARGET_TRAVEL", "0.30"))
    period_s = float(os.environ.get("XR_TARGET_PERIOD_S", "1.6"))
    spin_base = float(os.environ.get("XR_TARGET_SPIN_BASE", "4.0"))
    spin_amp = float(os.environ.get("XR_TARGET_SPIN_AMP", "1.0"))
    spin_period_s = float(os.environ.get("XR_TARGET_SPIN_PERIOD_S", "4.4"))
    spin_omega = 2.0 * math.pi / spin_period_s
    spin_phase = 0.0
    next_log_s = 0.0

    def triangle_wave(seconds):
        phase = (seconds / period_s) % 1.0
        if phase < 0.25:
            return 4.0 * phase
        if phase < 0.75:
            return 2.0 - 4.0 * phase
        return -4.0 + 4.0 * phase

    while robot.step(step_ms) != -1:
        t = robot.getTime()
        x = travel * triangle_wave(t)
        spin_v = spin_base + spin_amp * math.sin(spin_omega * t)
        spin_phase += spin_v * dt

        translation_field.setSFVec3f([x, origin_y, origin_z])
        spin_rotation_field.setSFRotation([0, 0, 1, spin_phase])

        if t >= next_log_s:
            print(
                f"[spin-supervisor] t={t:.2f}s omega_cmd={spin_v:.3f}rad/s "
                f"phase={spin_phase:.3f}rad x={x:.3f}m"
            )
            next_log_s = t + 0.5


if __name__ == "__main__":
    main()

