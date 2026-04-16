"""运行最小离线示例。"""

from __future__ import annotations

import runpy
from pathlib import Path


if __name__ == "__main__":
    example_path = Path(__file__).resolve().parents[1] / "examples" / "minimal_joint_motion.py"
    runpy.run_path(str(example_path), run_name="__main__")

