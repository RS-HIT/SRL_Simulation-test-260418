"""执行当前学习版的低风险验证。"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path


def main() -> int:
    project_root = Path(__file__).resolve().parents[1]
    command = [sys.executable, "-m", "unittest", "discover", "-s", str(project_root / "tests"), "-v"]
    result = subprocess.run(command, cwd=project_root)
    return int(result.returncode)


if __name__ == "__main__":
    raise SystemExit(main())

