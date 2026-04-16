"""运行基于外部 URDF 的 MuJoCo 简化仿真示例。"""

from __future__ import annotations

import runpy
from pathlib import Path


if __name__ == "__main__":
    example_path = Path(__file__).resolve().parents[1] / "examples" / "urdf_mujoco_demo.py"
    runpy.run_path(str(example_path), run_name="__main__")
