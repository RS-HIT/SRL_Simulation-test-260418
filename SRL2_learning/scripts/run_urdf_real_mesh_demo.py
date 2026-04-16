"""运行基于真实 STL 网格的 URDF MuJoCo 仿真。"""

from __future__ import annotations

import runpy
from pathlib import Path


if __name__ == "__main__":
    example_path = Path(__file__).resolve().parents[1] / "examples" / "urdf_real_mesh_demo.py"
    runpy.run_path(str(example_path), run_name="__main__")
