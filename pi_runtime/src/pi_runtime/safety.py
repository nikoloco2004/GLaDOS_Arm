"""Failsafe when brain link is lost."""

from __future__ import annotations

import time
from dataclasses import dataclass, field


@dataclass
class LinkWatchdog:
    """If no inbound message from brain for `failsafe_s`, trigger callback once."""

    failsafe_s: float = 8.0
    _last_brain_ts: float = field(default_factory=time.monotonic)
    _fired: bool = False

    def on_brain_message(self) -> None:
        self._last_brain_ts = time.monotonic()
        self._fired = False

    def check(self) -> bool:
        """Return True if failsafe should run now."""
        if self._fired:
            return False
        if time.monotonic() - self._last_brain_ts >= self.failsafe_s:
            self._fired = True
            return True
        return False

    def reset_after_failsafe(self) -> None:
        self._last_brain_ts = time.monotonic()
        self._fired = False
