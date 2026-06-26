from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml


def repo_root() -> Path:
    return Path(__file__).resolve().parents[3]


def load_config(path: str | Path | None = None) -> dict[str, Any]:
    cfg_path = Path(path) if path else repo_root() / "config" / "robot.yaml"
    with cfg_path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def get_nested(cfg: dict[str, Any], dotted_key: str, default: Any = None) -> Any:
    current: Any = cfg
    for part in dotted_key.split("."):
        if not isinstance(current, dict) or part not in current:
            return default
        current = current[part]
    return current
