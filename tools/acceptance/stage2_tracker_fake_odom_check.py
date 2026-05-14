#!/usr/bin/env python3
"""Report the phase-2 fake-odom tracker check as not implemented."""

from __future__ import annotations

import sys


def main() -> int:
    print("FAIL stage2_tracker_fake_odom: Pure Pursuit tracker and fake odom runner are not implemented")
    print("Required inputs after implementation: fixtures/stage2_paths/*.json and fake /odom sequence")
    print("Required outputs after implementation: /ackermann_cmd and /path_tracking/diagnostics")
    return 2


if __name__ == "__main__":
    sys.exit(main())
