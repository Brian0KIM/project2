"""
Small utilities used across modules.

All comments are intentionally in English (per user rule).
"""

from pybricks.tools import StopWatch, wait


def clamp(x: float, lo: float, hi: float) -> float:
    # Clamp numeric value into [lo, hi].
    if x < lo:
        return lo
    if x > hi:
        return hi
    return x


class EdgeDebounce:
    """Time-based debounce helper using milliseconds."""

    def __init__(self, cooldown_ms: int):
        self._cooldown_ms = int(cooldown_ms)
        self._sw = StopWatch()
        self._last_ms = -10**9

    def ready(self) -> bool:
        return (self._sw.time() - self._last_ms) >= self._cooldown_ms

    def trigger(self) -> None:
        self._last_ms = self._sw.time()


def sleep_ms(ms: int) -> None:
    wait(int(ms))


def fmt_ms(ms: int) -> str:
    # Format milliseconds into s with 0.01s resolution.
    return f"{ms/1000:.2f}s"


