"""
State-based line following + situation detection (intersection / lost line).

All comments are intentionally in English (per user rule).
"""

from pybricks.tools import StopWatch

import config


def _is_black_reflection(ref: int) -> bool:
    # Reflection thresholding for left/right sensors.
    return int(ref) <= int(config.LINE_THRESHOLD)


class LineFollower:
    def __init__(self):
        self._sw = StopWatch()
        self._state0_ms = 0
        self._state7_ms = 0
        self._last_t = self._sw.time()

    def reset_flags(self) -> None:
        # Reset detection timers to avoid immediate re-trigger after a maneuver.
        self._state0_ms = 0
        self._state7_ms = 0
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

    def compute_turn_rate(self, ref_l: int, ref_r: int) -> float:
        """
        Line following uses only left/right reflection for stability.
        Positive turn_rate means turning right.
        """
        # Normalize the error to roughly [-1, +1] using calibrated range.
        black = float(config.REFLECTION_BLACK_MAX)
        white = float(config.REFLECTION_WHITE_MIN)
        span = max(1.0, white - black)

        nl = (float(ref_l) - black) / span
        nr = (float(ref_r) - black) / span

        # If right is "whiter" than left, we are drifting left => steer right (positive).
        error = nr - nl
        return float(config.KP_TURN) * error

    def state_from_sensors(self, robot) -> int:
        """
        3-bit state from (left_black, center_black, right_black) => 0..7.
        Bits: [L, C, R] = [bit2, bit1, bit0]
        """
        ref_l = robot.left_color.reflection()
        ref_r = robot.right_color.reflection()
        l = 1 if _is_black_reflection(ref_l) else 0
        c = 1 if robot.center_is_black() else 0
        r = 1 if _is_black_reflection(ref_r) else 0
        return (l << 2) | (c << 1) | r

    def update_flags_from_state(self, state: int):
        """
        Update internal timers and return (is_intersection_candidate, is_lost).

        - Intersection candidate: state==7 (111) sustained.
        - Lost: state==0 (000) sustained.
        """
        dt = self._dt_ms()

        is_state7 = int(state) == 7
        is_state0 = int(state) == 0

        if is_state7:
            self._state7_ms += dt
        else:
            self._state7_ms = 0

        if is_state0:
            self._state0_ms += dt
        else:
            self._state0_ms = 0

        is_intersection = self._state7_ms >= int(config.INTERSECTION_CONFIRM_MS)
        is_lost = self._state0_ms >= int(config.LOST_CONFIRM_MS)

        return is_intersection, is_lost


