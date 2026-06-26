#!/usr/bin/env python3
from __future__ import annotations

import importlib.util


def main() -> None:
    spec = importlib.util.find_spec("fusion_hat")
    if spec is None:
        print("fusion_hat module not found yet.")
        print("Install using SunFounder docs before motor tests.")
        return
    print("fusion_hat module found.")
    try:
        from fusion_hat.motor import Motor  # noqa: F401
        print("fusion_hat.motor.Motor import OK.")
        print("No motors were moved by this script.")
    except Exception as e:
        print(f"fusion_hat.motor import failed: {e}")


if __name__ == "__main__":
    main()
