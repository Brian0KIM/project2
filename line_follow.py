"""
Line following + simple situation detection (intersection / lost line).

All comments are intentionally in English (per user rule).
"""

from pybricks.tools import StopWatch

import config
from utils import clamp


def _blackness_from_reflection(ref: int) -> float:
    """
    Convert reflection (0..100) into blackness (0..1),
    using REFLECTION_BLACK_MAX/REFLECTION_WHITE_MIN as calibration anchors.
    """
    black = float(config.REFLECTION_BLACK_MAX)
    white = float(config.REFLECTION_WHITE_MIN)
    if white <= black:
        # Avoid division by zero if misconfigured.
        return 0.0
    # Normalize reflection into [0..1] whiteness, then invert to blackness.
    whiteness = clamp((float(ref) - black) / (white - black), 0.0, 1.0)
    return 1.0 - whiteness


class LineFollower:
    def __init__(self):
        self._sw = StopWatch()
        self._intersection_ms = 0
        self._lost_ms = 0
        self._last_t = self._sw.time()

    def reset_flags(self) -> None:
        # Reset detection timers to avoid immediate re-trigger after a maneuver.
        self._intersection_ms = 0
        self._lost_ms = 0
        self._last_t = self._sw.time()

    def _dt_ms(self) -> int:
        t = self._sw.time()
        dt = int(t - self._last_t)
        self._last_t = t
        if dt < 0:
            return 0
        if dt > 200:
            # Prevent huge jumps if the loop was paused.
            return 200
        return dt

    def on_line(self, ref: int) -> bool:
        return int(ref) <= int(config.LINE_THRESHOLD)

    def compute_turn_rate(self, ref_l: int, ref_c: int, ref_r: int) -> float:
        # Weighted blackness position estimate: left=-1, center=0, right=+1.
        bl = _blackness_from_reflection(ref_l)
        bc = _blackness_from_reflection(ref_c)
        br = _blackness_from_reflection(ref_r)

        strength = bl + bc + br
        if strength < config.MIN_LINE_STRENGTH:
            # If line strength is too low, do not steer aggressively.
            return 0.0

        pos = (br - bl) / strength  # -1..+1 approximately
        return float(config.KP_TURN) * pos

    def update_flags(self, ref_l: int, ref_c: int, ref_r: int):
        """
        Update internal timers and return (is_intersection, is_lost).

        - Intersection: center is on line AND at least one side is also on line.
        - Lost: all sensors are off line.
        """
        dt = self._dt_ms()

        l_on = self.on_line(ref_l)
        c_on = self.on_line(ref_c)
        r_on = self.on_line(ref_r)

        is_intersection_now = c_on and (l_on or r_on)
        is_lost_now = (not l_on) and (not c_on) and (not r_on)

        if is_intersection_now:
            self._intersection_ms += dt
        else:
            self._intersection_ms = 0

        if is_lost_now:
            self._lost_ms += dt
        else:
            self._lost_ms = 0

        is_intersection = self._intersection_ms >= int(config.INTERSECTION_CONFIRM_MS)
        is_lost = self._lost_ms >= int(config.LOST_CONFIRM_MS)

        return is_intersection, is_lost


