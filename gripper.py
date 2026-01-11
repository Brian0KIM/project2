"""
Gripper control for a Medium Motor.

All comments are intentionally in English (per user rule).
"""

from pybricks.parameters import Stop
from pybricks.tools import wait

import config


class Gripper:
    def __init__(self, gripper_motor):
        self._m = gripper_motor

        # Reset angle to make open/close deterministic.
        try:
            self._m.reset_angle(0)
        except Exception:
            pass

    def open(self):
        # Open to a fixed angle.
        self._m.run_target(config.GRIPPER_SPEED_DPS, config.GRIPPER_OPEN_ANGLE, then=Stop.HOLD, wait=True)
        wait(100)

    def close(self):
        # Close to a fixed angle.
        self._m.run_target(config.GRIPPER_SPEED_DPS, config.GRIPPER_CLOSE_ANGLE, then=Stop.HOLD, wait=True)
        wait(100)


