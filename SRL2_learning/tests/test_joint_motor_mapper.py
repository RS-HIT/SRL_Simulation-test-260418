from __future__ import annotations

import math
import unittest

from pathlib import Path
import sys

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = PROJECT_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from srl2_learning.core.constants import (
    JOINT_1_REDUCTION_RATIO,
    JOINT_2_REDUCTION_RATIO,
    JOINT_3_REDUCTION_RATIO,
)
from srl2_learning.core.joint_motor_mapper import (
    joint_to_motor_commands,
    motor_to_joint_state,
)


class JointMotorMapperTests(unittest.TestCase):
    def test_joint_to_motor_primary_axes_match_reference_formula(self) -> None:
        joint_positions = [0.1, -0.2, 0.3, 0.4, -0.5, 0.6, -0.7]
        joint_velocities = [0.05, -0.1, 0.15, 0.2, -0.3, 0.4, -0.5]
        joint_torques = [1.0, -2.0, 3.0, 4.0, -5.0, 6.0, -7.0]

        result = joint_to_motor_commands(joint_positions, joint_velocities, joint_torques)

        self.assertAlmostEqual(result.motor_positions[0], JOINT_1_REDUCTION_RATIO * 0.1)
        self.assertAlmostEqual(
            result.motor_positions[1],
            JOINT_2_REDUCTION_RATIO * (-0.2) + JOINT_3_REDUCTION_RATIO * 0.3,
        )
        self.assertAlmostEqual(
            result.motor_positions[2],
            JOINT_2_REDUCTION_RATIO * (-0.2) - JOINT_3_REDUCTION_RATIO * 0.3,
        )
        self.assertAlmostEqual(result.motor_positions[4], 0.5)
        self.assertAlmostEqual(result.motor_positions[5], -0.6)
        self.assertAlmostEqual(result.motor_positions[6], 0.7)

        self.assertAlmostEqual(result.motor_velocities[0], JOINT_1_REDUCTION_RATIO * 0.05)
        self.assertAlmostEqual(result.motor_velocities[3], 0.0)
        self.assertAlmostEqual(result.motor_torques[3], 0.0)

    def test_primary_motor_to_joint_roundtrip_is_consistent(self) -> None:
        joint_positions = [0.15, -0.25, 0.35, 0.0, 0.0, 0.0, 0.0]
        joint_velocities = [0.02, -0.04, 0.06, 0.0, 0.0, 0.0, 0.0]
        joint_torques = [1.5, -2.5, 3.5, 0.0, 0.0, 0.0, 0.0]

        motor_result = joint_to_motor_commands(joint_positions, joint_velocities, joint_torques)
        joint_result = motor_to_joint_state(
            motor_result.motor_positions,
            motor_result.motor_velocities,
            motor_result.motor_torques,
        )

        for index in range(3):
            self.assertAlmostEqual(joint_result.joint_positions[index], joint_positions[index])
            self.assertAlmostEqual(joint_result.joint_velocities[index], joint_velocities[index])
            self.assertAlmostEqual(joint_result.joint_torques[index], joint_torques[index])


if __name__ == "__main__":
    unittest.main()

