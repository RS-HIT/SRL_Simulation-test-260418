"""运行 SRL2 原模型 MuJoCo 回放示例。"""

from __future__ import annotations

import runpy
from pathlib import Path


if __name__ == "__main__":
    example_path = Path(__file__).resolve().parents[1] / "examples" / "mujoco_replay_demo.py"
    runpy.run_path(str(example_path), run_name="__main__")
