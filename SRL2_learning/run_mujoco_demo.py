"""学习版 MuJoCo 回放快捷入口。

适用场景：
- 如果你已经 `cd SRL2_learning`，可以直接运行：
  `python run_mujoco_demo.py`
- 如果你当前在 `codetest` 根目录，请运行：
  `python SRL2_learning/scripts/run_mujoco_demo.py`
"""

from __future__ import annotations

import runpy
from pathlib import Path


if __name__ == "__main__":
    script_path = Path(__file__).resolve().parent / "scripts" / "run_mujoco_demo.py"
    runpy.run_path(str(script_path), run_name="__main__")
