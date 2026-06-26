from __future__ import annotations

import time


class Rate:
    def __init__(self, hz: float) -> None:
        if hz <= 0:
            raise ValueError("hz must be positive")
        self.period_s = 1.0 / hz
        self.next_t = time.monotonic() + self.period_s

    def sleep(self) -> None:
        now = time.monotonic()
        delay = self.next_t - now
        if delay > 0:
            time.sleep(delay)
        self.next_t = max(self.next_t + self.period_s, time.monotonic())
