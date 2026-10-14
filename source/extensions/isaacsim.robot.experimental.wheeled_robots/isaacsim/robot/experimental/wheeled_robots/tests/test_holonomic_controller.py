# SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Tests for holonomic (mecanum) controller."""

import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
import omni.kit.test
from isaacsim.robot.experimental.wheeled_robots.controllers.holonomic_controller import HolonomicController
from isaacsim.robot.experimental.wheeled_robots.robots import HolonomicRobotUsdSetup
from pxr import UsdGeom


class TestHolonomicController(omni.kit.test.AsyncTestCase):
    """Tests for HolonomicController kinematics."""

    async def setUp(self) -> None:
        """Set up test fixtures."""

    # ----------------------------------------------------------------------
    async def tearDown(self) -> None:
        """Tear down test fixtures."""

    # ----------------------------------------------------------------------

    async def test_holonomic_drive(self) -> None:
        """Verify wheel velocities for a negative X/Y and positive yaw command."""
        wheel_radius = [0.04, 0.04, 0.04]
        wheel_orientations = [[0, 0, 0, 1], [0.866, 0, 0, -0.5], [0.866, 0, 0, 0.5]]
        wheel_positions = [
            [-0.0980432, 0.000636773, -0.050501],
            [0.0493475, -0.084525, -0.050501],
            [0.0495291, 0.0856937, -0.050501],
        ]
        mecanum_angles = [90, 90, 90]
        # Verify the expected wheel velocities for negative X/Y and positive yaw.
        velocity_command = [-1.0, -1.0, 0.1]

        controller = HolonomicController(
            wheel_radius=wheel_radius,
            wheel_positions=wheel_positions,
            wheel_orientations=wheel_orientations,
            mecanum_angles=mecanum_angles,
        )
        actions = controller.forward(velocity_command)
        self.assertEqual(len(actions), controller.num_wheels, "Output length should match number of wheels")
        self.assertTrue(np.all(np.isfinite(actions)), "All action values should be finite")
        self.assertAlmostEqual(float(actions[0]), -25.105, delta=0.01)
        self.assertAlmostEqual(float(actions[1]), 14.3182, delta=0.01)
        self.assertAlmostEqual(float(actions[2]), -14.5417, delta=0.01)

    async def test_forward_only_positive_x(self) -> None:
        """Test that a pure forward command produces positive net X velocity.

        Verifies the euler-angle convention is correct by checking that the
        wheel velocity vector, when projected through the wheel geometry,
        produces motion in the expected direction.
        """
        wheel_radius = [0.04, 0.04, 0.04]
        wheel_orientations = [[0, 0, 0, 1], [0.866, 0, 0, -0.5], [0.866, 0, 0, 0.5]]
        wheel_positions = [
            [-0.0980432, 0.000636773, -0.050501],
            [0.0493475, -0.084525, -0.050501],
            [0.0495291, 0.0856937, -0.050501],
        ]
        mecanum_angles = [90, 90, 90]

        controller = HolonomicController(
            wheel_radius=wheel_radius,
            wheel_positions=wheel_positions,
            wheel_orientations=wheel_orientations,
            mecanum_angles=mecanum_angles,
        )
        actions = controller.forward([1.0, 0.0, 0.0])
        net_x = np.dot(controller.base_dir[0, :], actions)
        self.assertGreater(net_x, 0, "Forward command should produce positive net X velocity")

    async def test_mecanum_angles_are_canonicalized_modulo_180(self) -> None:
        """Verify equivalent roller-axis angles produce identical geometry and commands."""
        wheel_kwargs = {
            "wheel_radius": [0.05] * 4,
            "wheel_positions": [
                [0.15, 0.10, 0.0],
                [0.15, -0.10, 0.0],
                [-0.15, 0.10, 0.0],
                [-0.15, -0.10, 0.0],
            ],
            "wheel_orientations": [[1.0, 0.0, 0.0, 0.0]] * 4,
        }
        canonical = HolonomicController(**wheel_kwargs, mecanum_angles=[45.0, 135.0, 135.0, 45.0])
        legacy = HolonomicController(**wheel_kwargs, mecanum_angles=[-135.0, -45.0, -45.0, -135.0])

        np.testing.assert_allclose(legacy.mecanum_angles, canonical.mecanum_angles)
        np.testing.assert_allclose(legacy.base_dir, canonical.base_dir, atol=1e-12)
        np.testing.assert_allclose(legacy.forward([0.4, -0.2, 0.3]), canonical.forward([0.4, -0.2, 0.3]), atol=1e-6)

    async def test_init_raises_on_missing_required_param(self) -> None:
        """Constructor must raise a descriptive ValueError when a required array is omitted."""
        base = {
            "wheel_radius": [0.04, 0.04, 0.04],
            "wheel_positions": [[0.0, 0.0, 0.0]] * 3,
            "wheel_orientations": [[0, 0, 0, 1]] * 3,
            "mecanum_angles": [90, 90, 90],
        }
        for missing in ("wheel_radius", "wheel_positions", "wheel_orientations", "mecanum_angles"):
            kwargs = dict(base)
            kwargs[missing] = None
            with self.assertRaises(ValueError):
                HolonomicController(**kwargs)

    async def test_init_raises_on_negative_or_non_finite_speed_limits(self) -> None:
        """Constructor must raise a ValueError when any speed limit is negative, NaN, or infinite."""
        base = {
            "wheel_radius": [0.04, 0.04, 0.04],
            "wheel_positions": [[0.0, 0.0, 0.0]] * 3,
            "wheel_orientations": [[0, 0, 0, 1]] * 3,
            "mecanum_angles": [90, 90, 90],
        }
        for limit_name in ("max_linear_speed", "max_angular_speed", "max_wheel_speed"):
            for bad_value in (-1.0, float("nan"), float("inf"), float("-inf")):
                kwargs = dict(base)
                kwargs[limit_name] = bad_value
                with self.assertRaises(ValueError):
                    HolonomicController(**kwargs)

    async def test_init_accepts_nonnegative_speed_limits(self) -> None:
        """Constructor must not raise when speed limits are zero, positive, or None."""
        controller = HolonomicController(
            wheel_radius=[0.04, 0.04, 0.04],
            wheel_positions=[[0.0, 0.0, 0.0]] * 3,
            wheel_orientations=[[0, 0, 0, 1]] * 3,
            mecanum_angles=[90, 90, 90],
            max_linear_speed=0.0,
            max_angular_speed=1.0,
            max_wheel_speed=None,
        )
        self.assertEqual(controller.max_linear_speed, 0.0)
        self.assertEqual(controller.max_angular_speed, 1.0)
        self.assertIsNone(controller.max_wheel_speed)

    async def test_forward_clamps_to_max_linear_speed(self) -> None:
        """A command exceeding max_linear_speed must be clamped to the same direction, scaled magnitude."""
        wheel_kwargs = {
            "wheel_radius": [0.04, 0.04, 0.04],
            "wheel_orientations": [[0, 0, 0, 1], [0.866, 0, 0, -0.5], [0.866, 0, 0, 0.5]],
            "wheel_positions": [
                [-0.0980432, 0.000636773, -0.050501],
                [0.0493475, -0.084525, -0.050501],
                [0.0495291, 0.0856937, -0.050501],
            ],
            "mecanum_angles": [90, 90, 90],
        }
        unclamped = HolonomicController(**wheel_kwargs)
        baseline = unclamped.forward([1.0, 0.0, 0.0])

        clamped = HolonomicController(**wheel_kwargs, max_linear_speed=1.0)
        actions = clamped.forward([10.0, 0.0, 0.0])
        np.testing.assert_allclose(actions, baseline, atol=1e-6)

    async def test_forward_clamps_to_max_angular_speed(self) -> None:
        """A command exceeding max_angular_speed must be clamped to the same sign, scaled magnitude."""
        wheel_kwargs = {
            "wheel_radius": [0.04, 0.04, 0.04],
            "wheel_orientations": [[0, 0, 0, 1], [0.866, 0, 0, -0.5], [0.866, 0, 0, 0.5]],
            "wheel_positions": [
                [-0.0980432, 0.000636773, -0.050501],
                [0.0493475, -0.084525, -0.050501],
                [0.0495291, 0.0856937, -0.050501],
            ],
            "mecanum_angles": [90, 90, 90],
        }
        unclamped = HolonomicController(**wheel_kwargs)
        baseline = unclamped.forward([0.0, 0.0, 1.0])

        clamped = HolonomicController(**wheel_kwargs, max_angular_speed=1.0)
        actions = clamped.forward([0.0, 0.0, 10.0])
        np.testing.assert_allclose(actions, baseline, atol=1e-6)

    async def test_forward_clamps_to_max_wheel_speed(self) -> None:
        """Wheel outputs exceeding max_wheel_speed must be uniformly rescaled to respect the limit."""
        wheel_kwargs = {
            "wheel_radius": [0.04, 0.04, 0.04],
            "wheel_orientations": [[0, 0, 0, 1], [0.866, 0, 0, -0.5], [0.866, 0, 0, 0.5]],
            "wheel_positions": [
                [-0.0980432, 0.000636773, -0.050501],
                [0.0493475, -0.084525, -0.050501],
                [0.0495291, 0.0856937, -0.050501],
            ],
            "mecanum_angles": [90, 90, 90],
        }
        velocity_command = [1.0, 1.0, 0.1]
        unclamped = HolonomicController(**wheel_kwargs)
        baseline = unclamped.forward(velocity_command)
        peak = float(np.max(np.abs(baseline)))

        max_wheel_speed = peak / 2.0
        clamped = HolonomicController(**wheel_kwargs, max_wheel_speed=max_wheel_speed)
        actions = clamped.forward(velocity_command)
        np.testing.assert_allclose(actions, baseline * 0.5, atol=1e-6)
        self.assertLessEqual(float(np.max(np.abs(actions))), max_wheel_speed + 1e-6)

    async def test_solve_is_independent_of_unhit_speed_limits(self) -> None:
        """Regression test: an unhit saturation limit must not change the solved output.

        The conditioning scale is derived from the command magnitude,
        so an unconstrained (`None`) limit, an enormous explicit limit, and a normally-scaled
        limit must all solve to the same wheel velocities for the same command.
        """
        wheel_radius = [0.04, 0.04, 0.04]
        wheel_orientations = [[0, 0, 0, 1], [0.866, 0, 0, -0.5], [0.866, 0, 0, 0.5]]
        wheel_positions = [
            [-0.0980432, 0.000636773, -0.050501],
            [0.0493475, -0.084525, -0.050501],
            [0.0495291, 0.0856937, -0.050501],
        ]
        mecanum_angles = [90, 90, 90]
        velocity_command = [0.5, 0.0, 0.0]

        reference_actions = None
        for max_linear_speed in (None, 1.0e20, 2.0):
            controller = HolonomicController(
                wheel_radius=wheel_radius,
                wheel_positions=wheel_positions,
                wheel_orientations=wheel_orientations,
                mecanum_angles=mecanum_angles,
                max_linear_speed=max_linear_speed,
                max_angular_speed=None if max_linear_speed is None else 1.0e20,
                max_wheel_speed=None if max_linear_speed is None else 1.0e20,
            )
            actions = controller.forward(velocity_command)
            self.assertTrue(np.all(np.isfinite(actions)), f"Non-finite output for max_linear_speed={max_linear_speed}")
            if reference_actions is None:
                reference_actions = actions
            else:
                np.testing.assert_allclose(
                    actions,
                    reference_actions,
                    atol=1e-6,
                    err_msg=f"Output changed for max_linear_speed={max_linear_speed} vs. unconstrained reference",
                )

    async def test_zero_gain_zeroes_output_without_dividing_by_zero(self) -> None:
        """A nonzero raw command with linear_gain=angular_gain=0.0 must not raise or return NaN/inf.

        np.allclose(command, 0.0) only catches an all-zero raw command; zero gains zero out
        v_speed and w_speed downstream instead, which is exactly the case the solver's 1e-9
        conditioning-scale floor exists to protect against.
        """
        controller = HolonomicController(
            wheel_radius=[0.04, 0.04, 0.04],
            wheel_positions=[
                [-0.0980432, 0.000636773, -0.050501],
                [0.0493475, -0.084525, -0.050501],
                [0.0495291, 0.0856937, -0.050501],
            ],
            wheel_orientations=[[0, 0, 0, 1], [0.866, 0, 0, -0.5], [0.866, 0, 0, 0.5]],
            mecanum_angles=[90, 90, 90],
            linear_gain=0.0,
            angular_gain=0.0,
        )
        actions = controller.forward([1.0, 1.0, 1.0])
        self.assertTrue(np.all(np.isfinite(actions)), "All action values should be finite")
        self.assertTrue(np.allclose(actions, 0.0), "Zero gains should zero out the wheel commands")


class TestHolonomicRobotUsdSetup(omni.kit.test.AsyncTestCase):
    """Tests for loading holonomic controller parameters from USD."""

    async def test_raises_when_robot_has_no_mecanum_joints(self) -> None:
        """Test that a robot without mecanum attributes raises a descriptive error."""
        await stage_utils.create_new_stage_async()
        stage = stage_utils.get_current_stage(backend="usd")
        UsdGeom.Xform.Define(stage, "/World/PlainRobot")

        with self.assertRaisesRegex(ValueError, "No mecanum wheel joints.*isaacmecanumwheel:angle"):
            HolonomicRobotUsdSetup(robot_prim_path="/World/PlainRobot", com_prim_path="")
