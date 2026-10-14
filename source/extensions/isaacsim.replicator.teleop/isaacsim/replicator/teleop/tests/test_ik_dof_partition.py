# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Tests for the articulation DOF partition that arm IK writes to.

The controlled DOFs are the movable joints on the root-to-EE kinematic chain,
matched to articulation DOFs by name.  A positional prefix of the DOF array is
not equivalent: on the shipped UR3e + xArm arms the chain of six end-effector
links interleaves with gripper joints that are not on it.  Every test here
contrasts the resolved partition against that prefix so it fails if the
partition ever reverts to one.

Jacobian-based IK also requires a fixed-base articulation, and only the
simulated Jacobian layout reveals the base type - a world-anchoring joint in USD
is not sufficient.
"""

from __future__ import annotations

from typing import Any
from unittest.mock import patch

import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
import omni.kit.test
import omni.replicator.core
import omni.timeline
import omni.usd
from isaacsim.core.experimental.prims import Articulation, RigidPrim
from isaacsim.replicator.teleop import (
    CoordinateSystem,
    IKSolverType,
    PositionBasedIKController,
    RobotIKController,
)
from isaacsim.replicator.teleop.controllers._utils import (
    FloatingBaseArticulationError,
    slice_ee_jacobian,
)
from isaacsim.storage.native import get_assets_root_path_async
from pxr import Gf, UsdGeom, UsdPhysics

SOLO_STAGE_URL = "/Isaac/Samples/Replicator/Teleop/teleop_scenario_solo_ur3_xarm.usd"
DUAL_STAGE_URL = "/Isaac/Samples/Replicator/Teleop/teleop_scenario_dual_ur3_xarm_dex3.usd"

SOLO_ARM_PATH = "/World/teleop_solo_ur3_xarm/solo_arm/ur3e"
DUAL_PARENT_PATH = "/World/teleop_dual_ur3_xarm_dex3/dual_arm"
DUAL_LEFT_ARM_PATH = f"{DUAL_PARENT_PATH}/left_arm_ur3e_xarm/ur3e"
DUAL_RIGHT_ARM_PATH = f"{DUAL_PARENT_PATH}/right_arm_ur3e_dex3/ur3e"

# Articulation DOF indices of the root-to-EE chains on the shipped UR3e + xArm
# arm: 0-5 are the arm joints, 6-11 the xArm gripper's.
ARM_ONLY_CHAIN_INDICES = [0, 1, 2, 3, 4, 5]
RIGHT_FINGER_CHAIN_INDICES = [0, 1, 2, 3, 4, 5, 9, 11]
# Gripper DOFs that no chain through `right_finger` passes through. They belong
# to the grasp controller and must never receive an IK target.
RIGHT_FINGER_OFF_CHAIN_INDICES = frozenset({6, 7, 8, 10})
# Chain through the Dex3 thumb, whose gripper joints interleave differently.
THUMB_CHAIN_INDICES = [0, 1, 2, 3, 4, 5, 8, 11, 12]

PROCEDURAL_ROBOT_PATH = "/World/Robot"
PROCEDURAL_BASE_PATH = f"{PROCEDURAL_ROBOT_PATH}/base_link"
PROCEDURAL_ANCHOR_PATH = f"{PROCEDURAL_ROBOT_PATH}/root_joint"
PROCEDURAL_EE_LINK = "link_3"


async def _open_teleop_stage(test: omni.kit.test.AsyncTestCase, stage_url: str) -> None:
    """Open a shipped teleop scenario stage and wait for it to finish loading."""
    assets_root = await get_assets_root_path_async()
    test.assertTrue(assets_root, "Isaac assets root is required for the arm IK partition tests")
    opened, stage = await stage_utils.open_stage_async(f"{assets_root}{stage_url}")
    test.assertTrue(opened and stage is not None, f"Could not open {stage_url}")
    for _ in range(600):
        if not stage_utils.is_stage_loading():
            break
        await app_utils.update_app_async()
    test.assertFalse(stage_utils.is_stage_loading(), f"Timed out loading {stage_url}")
    await app_utils.update_app_async(steps=5)


def _validate_arm(arm_path: str, ee_link_name: str, explicit_arm_dofs: int = 0) -> tuple[RobotIKController, object]:
    """Configure one side of a fresh controller and return it with its validation result."""
    controller = RobotIKController(CoordinateSystem.RAW)
    controller.set_articulation_path("right", arm_path)
    controller.set_ee_link_name("right", ee_link_name)
    if explicit_arm_dofs:
        controller.set_num_arm_dofs("right", explicit_arm_dofs)
    return controller, controller.validate("right")


def _build_procedural_arm(*, anchor_to_world: bool, articulation_root_on_anchor: bool) -> str:
    """Author a 3-DOF chain and return the prim path carrying the articulation root.

    Args:
        anchor_to_world: Whether to add a fixed joint tying the base to the world.
        articulation_root_on_anchor: Whether to apply the articulation root to
            that joint rather than to the base rigid body.

    Returns:
        Prim path of the articulation root.
    """
    stage = stage_utils.get_current_stage()
    UsdPhysics.Scene(stage_utils.define_prim("/World/PhysicsScene", "PhysicsScene")).CreateGravityMagnitudeAttr(0.0)
    stage_utils.define_prim(PROCEDURAL_ROBOT_PATH, "Xform")

    base = UsdGeom.Cube.Define(stage, PROCEDURAL_BASE_PATH)
    base.CreateSizeAttr(0.2)
    UsdPhysics.RigidBodyAPI.Apply(base.GetPrim())
    UsdPhysics.MassAPI.Apply(base.GetPrim()).CreateMassAttr(5.0)

    previous = base.GetPath()
    for index in (1, 2, 3):
        link = UsdGeom.Cube.Define(stage, f"{PROCEDURAL_ROBOT_PATH}/link_{index}")
        link.CreateSizeAttr(0.2)
        link.AddTranslateOp().Set(Gf.Vec3d(0.25 * index, 0.0, 0.0))
        UsdPhysics.RigidBodyAPI.Apply(link.GetPrim())
        UsdPhysics.MassAPI.Apply(link.GetPrim()).CreateMassAttr(1.0)

        joint = UsdPhysics.RevoluteJoint.Define(stage, f"{PROCEDURAL_ROBOT_PATH}/joint_{index}")
        joint.CreateBody0Rel().SetTargets([previous])
        joint.CreateBody1Rel().SetTargets([link.GetPath()])
        joint.CreateAxisAttr("Z")
        previous = link.GetPath()

    if not anchor_to_world:
        UsdPhysics.ArticulationRootAPI.Apply(base.GetPrim())
        return PROCEDURAL_BASE_PATH

    anchor = UsdPhysics.FixedJoint.Define(stage, PROCEDURAL_ANCHOR_PATH)
    anchor.CreateBody1Rel().SetTargets([base.GetPath()])
    if articulation_root_on_anchor:
        UsdPhysics.ArticulationRootAPI.Apply(anchor.GetPrim())
        return PROCEDURAL_ANCHOR_PATH
    UsdPhysics.ArticulationRootAPI.Apply(base.GetPrim())
    return PROCEDURAL_BASE_PATH


class TestIKArmDofPartitionSoloUR3Xarm(omni.kit.test.AsyncTestCase):
    """Characterize the resolved arm DOF partition on the shipped solo UR3e + xArm."""

    async def setUp(self) -> None:
        """Open the solo teleop scenario stage with capture-on-play disabled."""
        await app_utils.update_app_async()
        # The scenario stage carries cameras, and these tests play the timeline
        # only to bring physics up; a capture triggered by play would outlive the
        # stage and stall the next one.
        omni.replicator.core.orchestrator.set_capture_on_play(False)
        await _open_teleop_stage(self, SOLO_STAGE_URL)

    async def tearDown(self) -> None:
        """Stop the timeline, drain Replicator, and release the stage."""
        omni.timeline.get_timeline_interface().stop()
        await app_utils.update_app_async()
        await omni.replicator.core.orchestrator.wait_until_complete_async()
        if stage_utils.is_stage_set() or omni.usd.get_context().get_stage() is not None:
            stage_utils.close_stage()
            await app_utils.update_app_async()
        while stage_utils.is_stage_loading():
            await app_utils.update_app_async()

    async def test_non_prefix_ee_link_resolves_interleaved_chain_dofs(self) -> None:
        """The default EE link resolves an interleaved chain, not a DOF prefix."""
        controller, result = _validate_arm(SOLO_ARM_PATH, "right_finger")

        self.assertTrue(result.valid, result.message)
        self.assertEqual(result.arm_dof_indices, RIGHT_FINGER_CHAIN_INDICES)
        self.assertEqual(controller.get_arm_dof_indices("right"), RIGHT_FINGER_CHAIN_INDICES)

        prefix_partition = list(range(len(RIGHT_FINGER_CHAIN_INDICES)))
        self.assertNotEqual(
            result.arm_dof_indices,
            prefix_partition,
            "A DOF prefix must not be mistaken for the chain; the two agree only for prefix-safe EE links",
        )
        self.assertEqual(
            sorted(set(prefix_partition) - set(result.arm_dof_indices)),
            [6, 7],
            f"A prefix partition commands off-chain gripper DOFs. Chain: {result.arm_dof_names}",
        )
        self.assertEqual(
            sorted(set(result.arm_dof_indices) - set(prefix_partition)),
            [9, 11],
            f"A prefix partition misses on-chain DOFs. Chain: {result.arm_dof_names}",
        )
        self.assertEqual(
            result.arm_dof_names,
            [
                "shoulder_pan_joint",
                "shoulder_lift_joint",
                "elbow_joint",
                "wrist_1_joint",
                "wrist_2_joint",
                "wrist_3_joint",
                "right_outer_knuckle_joint",
                "right_finger_joint",
            ],
        )

    async def test_default_ee_selection_is_the_interleaved_chain(self) -> None:
        """The link the Teleop window preselects is one with an interleaved chain."""
        _, result = _validate_arm(SOLO_ARM_PATH, "right_finger")

        # `ik_panel._populate_ee_combo` preselects the last link name.
        self.assertEqual(result.link_names[-1], "right_finger")
        self.assertEqual(result.num_dofs, 12)

        interleaved = [
            name
            for name in result.link_names
            for indices in [_validate_arm(SOLO_ARM_PATH, name)[1].arm_dof_indices]
            if indices and indices != list(range(len(indices)))
        ]
        self.assertIn("right_finger", interleaved)
        self.assertEqual(
            interleaved,
            [
                "left_inner_knuckle",
                "right_inner_knuckle",
                "right_outer_knuckle",
                "left_finger",
                "right_finger",
            ],
        )

    async def test_prefix_safe_ee_link_resolves_arm_dofs_only(self) -> None:
        """A wrist EE link resolves the six arm DOFs and no gripper DOF."""
        _, result = _validate_arm(SOLO_ARM_PATH, "wrist_3_link")

        self.assertTrue(result.valid, result.message)
        self.assertEqual(result.arm_dof_indices, ARM_ONLY_CHAIN_INDICES)
        self.assertEqual(result.arm_dofs, 6)
        # A count of seven is what a 6-DOF arm gets from a default rather than
        # from its chain, and index 6 is the gripper's primary actuator.
        self.assertEqual(result.dof_names[6], "drive_joint")
        self.assertNotIn(6, result.arm_dof_indices)

    async def test_ik_never_commands_off_chain_gripper_dofs(self) -> None:
        """Only chain DOFs receive IK position targets, over a run of frames."""
        controller, result = _validate_arm(SOLO_ARM_PATH, "right_finger")
        self.assertTrue(result.valid, result.message)
        self.assertTrue(controller.configure("right"))

        omni.timeline.get_timeline_interface().play()
        await app_utils.update_app_async(steps=5)

        ee_link_path = str(
            Articulation(result.articulation_path).link_paths[0][result.link_names.index("right_finger")]
        )
        ee_position = np.asarray(RigidPrim(ee_link_path).get_world_poses()[0].numpy(), dtype=np.float64).reshape(-1, 3)[
            0
        ]
        target = tuple(float(value) for value in ee_position + np.array([0.02, 0.0, 0.02]))

        commanded: list[list[int]] = []
        set_targets = Articulation.set_dof_position_targets

        def record_targets(
            articulation: Articulation, positions: Any, *, indices: Any = None, dof_indices: Any = None
        ) -> None:
            commanded.append([int(index) for index in np.asarray(dof_indices).reshape(-1)])
            set_targets(articulation, positions, indices=indices, dof_indices=dof_indices)

        self.assertTrue(controller.enable("right"))
        try:
            Articulation.set_dof_position_targets = record_targets
            for _ in range(30):
                controller.update_targets(None, None, target, None)
                await app_utils.update_app_async()
        finally:
            Articulation.set_dof_position_targets = set_targets

        self.assertTrue(commanded, "IK applied no DOF position targets while the timeline was playing")
        commanded_indices = {index for call in commanded for index in call}
        self.assertEqual(
            sorted(commanded_indices),
            RIGHT_FINGER_CHAIN_INDICES,
            f"IK commanded DOFs outside the root-to-EE chain. Per-call indices: {commanded}",
        )
        self.assertEqual(
            commanded_indices & RIGHT_FINGER_OFF_CHAIN_INDICES,
            set(),
            "Gripper DOFs off the chain are owned by the grasp controller and must stay uncommanded",
        )
        # A prefix partition of the same width would have driven two of them.
        self.assertEqual(
            set(range(len(RIGHT_FINGER_CHAIN_INDICES))) & RIGHT_FINGER_OFF_CHAIN_INDICES,
            {6, 7},
        )

    async def test_jacobian_slice_selects_chain_columns_not_a_prefix(self) -> None:
        """The Jacobian block the solvers use holds the chain's columns."""
        _, result = _validate_arm(SOLO_ARM_PATH, "right_finger")
        self.assertTrue(result.valid, result.message)

        robot = Articulation(result.articulation_path)
        ee_link_index = int(robot.get_link_indices("right_finger").numpy().item())

        omni.timeline.get_timeline_interface().play()
        await app_utils.update_app_async(steps=5)

        jacobians = robot.get_jacobian_matrices().numpy()
        self.assertEqual(
            jacobians.shape[-1],
            robot.num_dofs,
            "The shipped arm must report a fixed-base Jacobian with one column per DOF",
        )
        ee_jacobian = jacobians[:, ee_link_index - 1]

        chain_block = slice_ee_jacobian(
            jacobians, robot.num_dofs, ee_link_index, np.asarray(result.arm_dof_indices, dtype=np.int64)
        )
        prefix_block = ee_jacobian[..., : len(RIGHT_FINGER_CHAIN_INDICES)]

        self.assertEqual(chain_block.shape, prefix_block.shape)
        for column, dof_index in enumerate(RIGHT_FINGER_CHAIN_INDICES):
            np.testing.assert_array_equal(chain_block[..., column], ee_jacobian[..., dof_index])
        self.assertFalse(
            np.allclose(chain_block, prefix_block),
            "The chain block and a same-width prefix block must differ for an interleaved chain",
        )

    async def test_wrong_explicit_arm_dof_count_is_refused(self) -> None:
        """A pinned count that disagrees with the chain fails and names both."""
        _, result = _validate_arm(SOLO_ARM_PATH, "wrist_3_link", explicit_arm_dofs=7)

        self.assertFalse(result.valid)
        self.assertEqual(result.arm_dof_indices, [])
        for expected in ("7", "6", SOLO_ARM_PATH, "wrist_3_link", "shoulder_pan_joint", "drive_joint"):
            self.assertIn(expected, result.message)

    async def test_articulation_root_ee_link_is_refused(self) -> None:
        """The articulation root as EE link yields no chain and is refused."""
        controller, result = _validate_arm(SOLO_ARM_PATH, "base_link")

        self.assertFalse(result.valid)
        self.assertEqual(result.arm_dof_indices, [])
        self.assertEqual(controller.get_arm_dof_indices("right"), [])
        self.assertFalse(controller.configure("right"))
        self.assertIn("base_link", result.message)


class TestIKArmDofPartitionDualUR3(omni.kit.test.AsyncTestCase):
    """Characterize per-arm binding on the shipped dual UR3e stage."""

    async def setUp(self) -> None:
        """Open the dual teleop scenario stage."""
        await app_utils.update_app_async()
        await _open_teleop_stage(self, DUAL_STAGE_URL)

    async def tearDown(self) -> None:
        """Release the stage."""
        if stage_utils.is_stage_set() or omni.usd.get_context().get_stage() is not None:
            stage_utils.close_stage()
            await app_utils.update_app_async()
        while stage_utils.is_stage_loading():
            await app_utils.update_app_async()

    async def test_each_arm_resolves_its_own_root_and_indices(self) -> None:
        """Both sides bind their own articulation root and their own DOFs."""
        controller = RobotIKController(CoordinateSystem.RAW)
        results = {}
        for side, arm_path in (("left", DUAL_LEFT_ARM_PATH), ("right", DUAL_RIGHT_ARM_PATH)):
            controller.set_articulation_path(side, arm_path)
            controller.set_ee_link_name(side, "wrist_3_link")
            results[side] = controller.validate(side)

        for side, result in results.items():
            self.assertTrue(result.valid, f"{side}: {result.message}")
            self.assertEqual(result.arm_dof_indices, ARM_ONLY_CHAIN_INDICES)
            self.assertEqual(result.articulation_root_paths, [result.articulation_path])

        self.assertNotEqual(results["left"].articulation_path, results["right"].articulation_path)
        self.assertIn("left_arm_ur3e_xarm", results["left"].articulation_path)
        self.assertIn("right_arm_ur3e_dex3", results["right"].articulation_path)
        # Each arm carries a different end effector, so their DOF names differ
        # past the six shared arm joints.
        self.assertEqual(results["left"].num_dofs, 12)
        self.assertEqual(results["right"].num_dofs, 13)

    async def test_dex3_thumb_chain_is_interleaved(self) -> None:
        """A Dex3 fingertip EE link resolves an interleaved chain of its own."""
        _, result = _validate_arm(DUAL_RIGHT_ARM_PATH, "right_hand_thumb_2_link")

        self.assertTrue(result.valid, result.message)
        self.assertEqual(result.arm_dof_indices, THUMB_CHAIN_INDICES)
        self.assertNotEqual(result.arm_dof_indices, list(range(len(THUMB_CHAIN_INDICES))))
        self.assertEqual(sorted(set(range(len(THUMB_CHAIN_INDICES))) - set(result.arm_dof_indices)), [6, 7])

    async def test_shared_parent_path_is_refused(self) -> None:
        """A parent prim holding both arm roots is refused, not half-bound."""
        controller, result = _validate_arm(DUAL_PARENT_PATH, "wrist_3_link")

        self.assertFalse(
            result.valid,
            f"A path spanning both arms bound '{result.articulation_path}' alone: {result.message}",
        )
        self.assertEqual(controller.get_arm_dof_indices("right"), [])
        self.assertEqual(len(result.articulation_root_paths), 2)
        self.assertIn("left_arm_ur3e_xarm", " ".join(result.articulation_root_paths))
        self.assertIn("right_arm_ur3e_dex3", " ".join(result.articulation_root_paths))
        for root_path in result.articulation_root_paths:
            self.assertIn(root_path, result.message)


class TestIKFixedBaseRequirement(omni.kit.test.AsyncTestCase):
    """Characterize the fixed-base requirement of Jacobian-based arm IK."""

    async def setUp(self) -> None:
        """Create an empty stage for each procedural articulation."""
        await app_utils.update_app_async()
        await stage_utils.create_new_stage_async()
        stage_utils.define_prim("/World", "Xform")
        await app_utils.update_app_async()

    async def tearDown(self) -> None:
        """Stop physics and release the procedural stage."""
        omni.timeline.get_timeline_interface().stop()
        await app_utils.update_app_async()
        if stage_utils.is_stage_set() or omni.usd.get_context().get_stage() is not None:
            stage_utils.close_stage()
            await app_utils.update_app_async()

    async def _play(self) -> None:
        """Start the timeline and let the physics tensor entity come up."""
        omni.timeline.get_timeline_interface().play()
        await app_utils.update_app_async(steps=10)

    async def test_fixed_base_articulation_is_accepted(self) -> None:
        """An articulation root on the world-anchoring joint validates and slices."""
        root_path = _build_procedural_arm(anchor_to_world=True, articulation_root_on_anchor=True)
        await app_utils.update_app_async(steps=3)
        await self._play()

        _, result = _validate_arm(root_path, PROCEDURAL_EE_LINK)
        self.assertTrue(result.valid, result.message)
        self.assertTrue(result.is_fixed_base)
        self.assertEqual(result.arm_dof_indices, [0, 1, 2])

        robot = Articulation(root_path)
        jacobians = robot.get_jacobian_matrices().numpy()
        self.assertEqual(jacobians.shape[-1], robot.num_dofs)
        block = slice_ee_jacobian(jacobians, robot.num_dofs, 3, np.asarray([0, 1, 2], dtype=np.int64))
        self.assertEqual(block.shape[-1], 3)

    async def test_floating_base_articulation_is_refused(self) -> None:
        """A floating base is refused rather than sliced as if it were fixed."""
        root_path = _build_procedural_arm(anchor_to_world=False, articulation_root_on_anchor=False)
        await app_utils.update_app_async(steps=3)
        await self._play()

        robot = Articulation(root_path)
        jacobians = robot.get_jacobian_matrices().numpy()
        self.assertEqual(
            jacobians.shape[-1],
            robot.num_dofs + 6,
            "A floating base prepends six root columns to the Jacobian",
        )
        with self.assertRaises(FloatingBaseArticulationError):
            slice_ee_jacobian(jacobians, robot.num_dofs, 3, np.asarray([0, 1, 2], dtype=np.int64))

        controller, result = _validate_arm(root_path, PROCEDURAL_EE_LINK)
        self.assertFalse(result.valid, result.message)
        self.assertFalse(result.is_fixed_base)
        self.assertIn("fixed-base articulation", result.message)
        self.assertFalse(controller.configure("right"))

    async def test_pink_accepts_floating_base_articulation(self) -> None:
        """PINK validates a moving root without using the simulation Jacobian."""
        root_path = _build_procedural_arm(anchor_to_world=False, articulation_root_on_anchor=False)
        await app_utils.update_app_async(steps=3)
        await self._play()

        controller = RobotIKController(CoordinateSystem.RAW)
        controller.set_articulation_path("right", root_path)
        controller.set_ee_link_name("right", PROCEDURAL_EE_LINK)
        with patch.object(controller, "get_solver_availability", return_value=(True, "")):
            switched, message = controller.set_solver_type("right", IKSolverType.PINK)
        self.assertTrue(switched, message)

        result = controller.validate("right")
        self.assertTrue(result.valid, result.message)
        self.assertFalse(result.is_fixed_base)
        self.assertEqual(result.arm_dof_indices, [0, 1, 2])

    async def test_world_anchoring_joint_alone_leaves_the_base_floating(self) -> None:
        """A world joint without the articulation root on it is still floating."""
        root_path = _build_procedural_arm(anchor_to_world=True, articulation_root_on_anchor=False)
        await app_utils.update_app_async(steps=3)
        await self._play()

        stage = stage_utils.get_current_stage()
        anchor = UsdPhysics.Joint(stage.GetPrimAtPath(PROCEDURAL_ANCHOR_PATH))
        self.assertTrue(anchor, "The world-anchoring fixed joint should exist")
        self.assertEqual(list(anchor.GetBody0Rel().GetTargets()), [])
        self.assertEqual([str(path) for path in anchor.GetBody1Rel().GetTargets()], [PROCEDURAL_BASE_PATH])

        robot = Articulation(root_path)
        self.assertEqual(
            robot.get_jacobian_matrices().numpy().shape[-1],
            robot.num_dofs + 6,
            "A world-anchoring joint does not fix the base unless the articulation root is applied to it",
        )

        _, result = _validate_arm(root_path, PROCEDURAL_EE_LINK)
        self.assertFalse(result.valid, result.message)
        self.assertFalse(result.is_fixed_base)

    async def test_solver_refuses_floating_base_on_its_first_step(self) -> None:
        """A directly constructed solver returns no solution for a floating base."""
        root_path = _build_procedural_arm(anchor_to_world=False, articulation_root_on_anchor=False)
        await app_utils.update_app_async(steps=3)
        await self._play()

        robot = Articulation(root_path)
        ee_link_index = int(robot.get_link_indices(PROCEDURAL_EE_LINK).numpy().item())
        solver = PositionBasedIKController(
            robot=robot,
            ee_link=RigidPrim(str(robot.link_paths[0][ee_link_index]), reset_xform_op_properties=False),
            ee_link_index=ee_link_index,
            num_arm_dofs=3,
            arm_dof_indices=[0, 1, 2],
        )
        before = robot.get_dof_position_targets().numpy().copy()

        solver.set_target((0.4, 0.1, 0.3), None)
        self.assertIsNone(solver.compute())
        self.assertIsNone(solver.compute())
        self.assertFalse(solver.reachable)

        np.testing.assert_array_equal(robot.get_dof_position_targets().numpy(), before)
