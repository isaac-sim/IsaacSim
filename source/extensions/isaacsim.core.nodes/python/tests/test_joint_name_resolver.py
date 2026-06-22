# SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Verifies the JointNameResolver node maps Isaac joint name overrides back to authored joint names. Covers pass-through names, mixed original and overridden names, target prim input, single-name resolution, robot path output, unmatched names, and multiple overrides."""

import asyncio
from typing import Any

import carb
import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import omni.graph.core as og
import omni.graph.core.tests as ogts
import omni.kit.test
from isaacsim.storage.native import get_assets_root_path_async
from pxr import Sdf, UsdGeom
from usdrt import Sdf as UsdrtSdf


class TestJointNameResolver(ogts.OmniGraphTestCase):
    """Verify joint-name override resolution on a loaded Franka articulation."""

    GRAPH_PATH = "/ActionGraph"
    NODE_NAME = "JointNameResolver"
    ROBOT_PATH = "/panda"

    async def setUp(self) -> None:
        """Load a Franka stage and prepare the articulation path used by resolver graphs."""
        await omni.usd.get_context().new_stage_async()
        assets_root_path = await get_assets_root_path_async()
        if assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return
        await stage_utils.open_stage_async(assets_root_path + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd")
        self._stage = omni.usd.get_context().get_stage()

        # Build a lookup from prim name -> prim path for all prims under the robot root
        self._prim_name_to_path = {}
        root_prim = self._stage.GetPrimAtPath(self.ROBOT_PATH)
        self.assertTrue(root_prim.IsValid(), f"Robot root prim at {self.ROBOT_PATH} should be valid")
        for prim in UsdGeom.Imageable(root_prim).GetPrim().GetStage().Traverse():
            prim_path = str(prim.GetPath())
            if prim_path.startswith(self.ROBOT_PATH):
                self._prim_name_to_path[prim.GetName()] = prim_path

    async def tearDown(self) -> None:
        """Wait for pending stage loading work before the next resolver test."""
        await omni.kit.app.get_app().next_update_async()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            await asyncio.sleep(1.0)
        await omni.kit.app.get_app().next_update_async()

    def _find_prim_path(self, prim_name: str) -> str:
        """Find the full USD path for a prim by its short name under the robot root.

        Args:
            prim_name: Short prim name to resolve.

        Returns:
            Full prim path under the robot root.
        """
        path = self._prim_name_to_path.get(prim_name)
        self.assertIsNotNone(
            path,
            f"Could not find prim named '{prim_name}' under {self.ROBOT_PATH}. "
            f"Available names: {sorted(self._prim_name_to_path.keys())}",
        )
        return path

    def _set_name_override(self, prim_name: Any, override_name: Any) -> None:
        """Set the isaac:nameOverride attribute on a prim found by its short name.

        Args:
            prim_name: Short prim name to update.
            override_name: Override name to author.
        """
        prim_path = self._find_prim_path(prim_name)
        prim = self._stage.GetPrimAtPath(prim_path)
        self.assertTrue(prim.IsValid(), f"Prim at {prim_path} should be valid")
        attr = prim.GetAttribute("isaac:nameOverride")
        if not attr:
            attr = prim.CreateAttribute("isaac:nameOverride", Sdf.ValueTypeNames.String)
        attr.Set(override_name)

    def _create_resolver_graph_with_robot_path(self, joint_names: Any) -> Any:
        """Create an action graph with the JointNameResolver using robotPath input.

        Args:
            joint_names: Joint names to feed into the resolver.

        Returns:
            Created graph and graph nodes.
        """
        create_nodes = [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("JointNameArray", "omni.graph.nodes.ConstructArray"),
            (self.NODE_NAME, "isaacsim.core.nodes.IsaacJointNameResolver"),
        ]
        set_values = [
            ("JointNameArray.inputs:arraySize", len(joint_names)),
            ("JointNameArray.inputs:arrayType", "token[]"),
            (f"{self.NODE_NAME}.inputs:robotPath", self.ROBOT_PATH),
        ]
        create_attributes = []
        for i, name in enumerate(joint_names):
            set_values.append((f"JointNameArray.inputs:input{i}", name))
            if i > 0:
                create_attributes.append((f"JointNameArray.inputs:input{i}", "token"))

        edit_args = {
            og.Controller.Keys.CREATE_NODES: create_nodes,
            og.Controller.Keys.SET_VALUES: set_values,
            og.Controller.Keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", f"{self.NODE_NAME}.inputs:execIn"),
                ("JointNameArray.outputs:array", f"{self.NODE_NAME}.inputs:jointNames"),
            ],
        }
        if create_attributes:
            edit_args[og.Controller.Keys.CREATE_ATTRIBUTES] = create_attributes

        test_graph, new_nodes, _, _ = og.Controller.edit(
            {"graph_path": self.GRAPH_PATH, "evaluator_name": "execution"},
            edit_args,
        )
        return test_graph, new_nodes

    def _create_resolver_graph_with_target_prim(self, joint_names: Any) -> Any:
        """Create an action graph with the JointNameResolver using targetPrim input.

        Args:
            joint_names: Joint names to feed into the resolver.

        Returns:
            Created graph and graph nodes.
        """
        create_nodes = [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("JointNameArray", "omni.graph.nodes.ConstructArray"),
            (self.NODE_NAME, "isaacsim.core.nodes.IsaacJointNameResolver"),
        ]
        set_values = [
            ("JointNameArray.inputs:arraySize", len(joint_names)),
            ("JointNameArray.inputs:arrayType", "token[]"),
            (f"{self.NODE_NAME}.inputs:targetPrim", [UsdrtSdf.Path(self.ROBOT_PATH)]),
        ]
        create_attributes = []
        for i, name in enumerate(joint_names):
            set_values.append((f"JointNameArray.inputs:input{i}", name))
            if i > 0:
                create_attributes.append((f"JointNameArray.inputs:input{i}", "token"))

        edit_args = {
            og.Controller.Keys.CREATE_NODES: create_nodes,
            og.Controller.Keys.SET_VALUES: set_values,
            og.Controller.Keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", f"{self.NODE_NAME}.inputs:execIn"),
                ("JointNameArray.outputs:array", f"{self.NODE_NAME}.inputs:jointNames"),
            ],
        }
        if create_attributes:
            edit_args[og.Controller.Keys.CREATE_ATTRIBUTES] = create_attributes

        test_graph, new_nodes, _, _ = og.Controller.edit(
            {"graph_path": self.GRAPH_PATH, "evaluator_name": "execution"},
            edit_args,
        )
        return test_graph, new_nodes

    def _get_resolver_outputs(self, resolver_node: Any) -> tuple[Any, Any]:
        joint_names = og.Controller.attribute("outputs:jointNames", resolver_node).get()
        robot_path = og.Controller.attribute("outputs:robotPath", resolver_node).get()
        return joint_names, robot_path

    # ----------------------------------------------------------------------
    async def test_names_without_overrides_pass_through(self) -> None:
        """Joint names that do not match any override should pass through unchanged."""
        input_names = ["panda_joint2", "panda_joint3"]
        test_graph, new_nodes = self._create_resolver_graph_with_robot_path(input_names)

        app_utils.play()
        await app_utils.update_app_async(steps=60)
        await og.Controller.evaluate(test_graph)

        resolver_node = new_nodes[-1]
        joint_names, robot_path = self._get_resolver_outputs(resolver_node)

        self.assertEqual(len(joint_names), 2)
        self.assertEqual(joint_names[0], "panda_joint2")
        self.assertEqual(joint_names[1], "panda_joint3")
        self.assertEqual(robot_path, self.ROBOT_PATH)

    # ----------------------------------------------------------------------
    async def test_overridden_names_resolve_to_original(self) -> None:
        """Joint names matching an isaac:nameOverride should resolve to the original prim name."""
        self._set_name_override("panda_joint2", "custom_joint_2")
        self._set_name_override("panda_joint3", "custom_joint_3")

        input_names = ["custom_joint_2", "custom_joint_3"]
        test_graph, new_nodes = self._create_resolver_graph_with_robot_path(input_names)

        app_utils.play()
        await app_utils.update_app_async(steps=60)
        await og.Controller.evaluate(test_graph)

        resolver_node = new_nodes[-1]
        joint_names, robot_path = self._get_resolver_outputs(resolver_node)

        self.assertEqual(len(joint_names), 2)
        self.assertEqual(joint_names[0], "panda_joint2")
        self.assertEqual(joint_names[1], "panda_joint3")
        self.assertEqual(robot_path, self.ROBOT_PATH)

    # ----------------------------------------------------------------------
    async def test_mixed_overridden_and_original_names(self) -> None:
        """A mix of overridden and non-overridden names should be resolved correctly."""
        self._set_name_override("panda_joint2", "renamed_joint_2")

        input_names = ["renamed_joint_2", "panda_joint3"]
        test_graph, new_nodes = self._create_resolver_graph_with_robot_path(input_names)

        app_utils.play()
        await app_utils.update_app_async(steps=60)
        await og.Controller.evaluate(test_graph)

        resolver_node = new_nodes[-1]
        joint_names, robot_path = self._get_resolver_outputs(resolver_node)

        self.assertEqual(len(joint_names), 2)
        self.assertEqual(joint_names[0], "panda_joint2")
        self.assertEqual(joint_names[1], "panda_joint3")

    # ----------------------------------------------------------------------
    async def test_target_prim_input(self) -> None:
        """The node should work with targetPrim input when robotPath is empty."""
        self._set_name_override("panda_joint2", "tp_joint_2")

        input_names = ["tp_joint_2", "panda_joint3"]
        test_graph, new_nodes = self._create_resolver_graph_with_target_prim(input_names)

        app_utils.play()
        await app_utils.update_app_async(steps=60)
        await og.Controller.evaluate(test_graph)

        resolver_node = new_nodes[-1]
        joint_names, robot_path = self._get_resolver_outputs(resolver_node)

        self.assertEqual(len(joint_names), 2)
        self.assertEqual(joint_names[0], "panda_joint2")
        self.assertEqual(joint_names[1], "panda_joint3")
        self.assertEqual(robot_path, self.ROBOT_PATH)

    # ----------------------------------------------------------------------
    async def test_single_joint_name_resolution(self) -> None:
        """Resolver should handle a single joint name correctly."""
        self._set_name_override("panda_joint4", "my_j4")

        input_names = ["my_j4"]
        test_graph, new_nodes = self._create_resolver_graph_with_robot_path(input_names)

        app_utils.play()
        await app_utils.update_app_async(steps=60)
        await og.Controller.evaluate(test_graph)

        resolver_node = new_nodes[-1]
        joint_names, robot_path = self._get_resolver_outputs(resolver_node)

        self.assertEqual(len(joint_names), 1)
        self.assertEqual(joint_names[0], "panda_joint4")

    # ----------------------------------------------------------------------
    async def test_robot_path_output(self) -> None:
        """Output robotPath should match the provided input robotPath."""
        input_names = ["panda_joint1"]
        test_graph, new_nodes = self._create_resolver_graph_with_robot_path(input_names)

        app_utils.play()
        await app_utils.update_app_async(steps=60)
        await og.Controller.evaluate(test_graph)

        resolver_node = new_nodes[-1]
        _, robot_path = self._get_resolver_outputs(resolver_node)

        self.assertEqual(robot_path, self.ROBOT_PATH)

    # ----------------------------------------------------------------------
    async def test_unmatched_override_name_passes_through(self) -> None:
        """A name that doesn't exist in the prim hierarchy at all should pass through unchanged."""
        input_names = ["nonexistent_joint"]
        test_graph, new_nodes = self._create_resolver_graph_with_robot_path(input_names)

        app_utils.play()
        await app_utils.update_app_async(steps=60)
        await og.Controller.evaluate(test_graph)

        resolver_node = new_nodes[-1]
        joint_names, _ = self._get_resolver_outputs(resolver_node)

        self.assertEqual(len(joint_names), 1)
        self.assertEqual(joint_names[0], "nonexistent_joint")

    # ----------------------------------------------------------------------
    async def test_multiple_overrides_on_different_joints(self) -> None:
        """Multiple joints with different overrides should all resolve correctly."""
        self._set_name_override("panda_joint1", "j1_override")
        self._set_name_override("panda_joint3", "j3_override")
        self._set_name_override("panda_joint5", "j5_override")

        input_names = ["j1_override", "panda_joint2", "j3_override", "panda_joint4", "j5_override"]
        test_graph, new_nodes = self._create_resolver_graph_with_robot_path(input_names)

        app_utils.play()
        await app_utils.update_app_async(steps=60)
        await og.Controller.evaluate(test_graph)

        resolver_node = new_nodes[-1]
        joint_names, _ = self._get_resolver_outputs(resolver_node)

        self.assertEqual(len(joint_names), 5)
        self.assertEqual(joint_names[0], "panda_joint1")
        self.assertEqual(joint_names[1], "panda_joint2")
        self.assertEqual(joint_names[2], "panda_joint3")
        self.assertEqual(joint_names[3], "panda_joint4")
        self.assertEqual(joint_names[4], "panda_joint5")
