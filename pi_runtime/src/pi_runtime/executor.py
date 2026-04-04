"""Map incoming commands to hardware. Stub: wire to glados_arm.RobotController later."""

from __future__ import annotations

import logging
from typing import Any

log = logging.getLogger(__name__)


def execute_command(name: str, args: dict[str, Any]) -> tuple[bool, str]:
    """
    Returns (ok, detail).

    Phase 1 stubs:
      - neutral: would call RobotController.neutral()
      - ping: no-op success
    """
    if name == "ping":
        return True, "pong"
    if name == "neutral":
        # from glados_arm.controller import RobotController  # wire when serial configured
        log.warning("neutral: hardware not wired; stub OK")
        return True, "stub neutral"
    if name == "echo":
        return True, repr(args)
    return False, f"unknown command: {name}"
