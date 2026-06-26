from __future__ import annotations

from dataclasses import dataclass
from time import sleep

from tcr_minibot.motion.differential_drive import WheelCommands, clamp

try:
    from fusion_hat.motor import Motor  # type: ignore
except Exception:  # pragma: no cover
    Motor = None


@dataclass
class MotorConfig:
    left_port: str = "M0"
    right_port: str = "M1"
    left_reversed: bool = False
    right_reversed: bool = True
    max_power_percent: float = 20.0


class DifferentialMotors:
    """
    Guarded Fusion HAT+ motor wrapper.

    This class refuses to drive unless armed=True. That keeps sensor-only scripts safe
    while your 7.2 V motor battery is unplugged or while the robot is not propped up.
    """

    def __init__(self, cfg: MotorConfig, *, armed: bool = False) -> None:
        if Motor is None:
            raise RuntimeError("fusion_hat is not installed/importable. Install SunFounder fusion-hat first.")
        self.cfg = cfg
        self.armed = armed
        self.left = Motor(cfg.left_port, is_reversed=cfg.left_reversed)
        self.right = Motor(cfg.right_port, is_reversed=cfg.right_reversed)
        self.stop()

    def set_armed(self, armed: bool) -> None:
        self.armed = armed
        if not armed:
            self.stop()

    def drive_power(self, cmd: WheelCommands) -> None:
        if not self.armed:
            self.stop()
            raise RuntimeError("Motors are not armed. This is intentional safety behavior.")
        max_p = abs(self.cfg.max_power_percent)
        left = clamp(cmd.left, -max_p, max_p)
        right = clamp(cmd.right, -max_p, max_p)
        self.left.power(left)
        self.right.power(right)

    def tiny_pulse(self, power_percent: float = 12.0, seconds: float = 0.20) -> None:
        if not self.armed:
            raise RuntimeError("Motors are not armed. Refusing tiny pulse.")
        p = clamp(power_percent, -self.cfg.max_power_percent, self.cfg.max_power_percent)
        try:
            self.left.power(p)
            self.right.power(p)
            sleep(seconds)
        finally:
            self.stop()

    def stop(self) -> None:
        # Some Fusion HAT versions expose stop(); power(0) is also supported by examples.
        for motor in (getattr(self, "left", None), getattr(self, "right", None)):
            if motor is None:
                continue
            try:
                motor.power(0)
            except Exception:
                pass
            try:
                motor.stop()
            except Exception:
                pass
