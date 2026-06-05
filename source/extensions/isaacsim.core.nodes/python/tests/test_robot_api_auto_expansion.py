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

"""Regression tests for OmniGraph node behavior when the user targets an `IsaacRobotAPI` root prim
whose `UsdPhysicsArticulationRootAPI` lives on a deeper link.

Asset layout (mirrors the `tb3_burger_processed.usda` shape that surfaced this gap):

::

    /World/Robot                          IsaacRobotAPI, isaac:physics:robotLinks -> [base_link, wheel_left, wheel_right]
    /World/Robot/base_link                IsaacLinkAPI, UsdPhysicsRigidBodyAPI, UsdPhysicsArticulationRootAPI
    /World/Robot/wheel_left               IsaacLinkAPI, UsdPhysicsRigidBodyAPI
    /World/Robot/wheel_right              IsaacLinkAPI, UsdPhysicsRigidBodyAPI

`IsaacComputeTransformTree` and `IsaacComputeOdometry` must consult `IsaacRobotAPI.robotLinks` to
resolve targets when their `UsdPhysicsArticulationRootAPI`/`RigidBodyAPI` check on the supplied
prim itself comes up empty. Without that fallback, `IsaacComputeTransformTree` emits one frame
(the root) instead of one per link, and `IsaacComputeOdometry` raises a logError plus an unguarded
`physx-tensors` warning for the non-rigid-body root.
"""

import omni.graph.core as og
import omni.graph.core.tests as ogts
import omni.usd
from isaacsim.core.experimental.objects import Cube
from isaacsim.core.experimental.prims import GeomPrim, RigidPrim
from pxr import Gf, Sdf, UsdGeom, UsdPhysics
from usdrt import Sdf as RtSdf


def _add_rigid_link(stage, path, position):
    """Define a Cube + RigidPrim at @p path so PhysX recognizes it as a rigid body."""
    Cube(path, sizes=1.0)
    RigidPrim(
        path,
        positions=[position],
        masses=1.0,
        reset_xform_op_properties=True,
    )
    GeomPrim(path, apply_collision_apis=True)


async def _next_update():
    await omni.kit.app.get_app().next_update_async()


class TestRobotApiAutoExpansion(ogts.OmniGraphTestCase):
    """Validate that OG nodes fall back to `IsaacRobotAPI.robotLinks` when the supplied prim does
    not itself carry `UsdPhysicsArticulationRootAPI` / `UsdPhysicsRigidBodyAPI`."""

    GRAPH_PATH = "/ActionGraph"

    async def setUp(self):
        await omni.usd.get_context().new_stage_async()
        self._stage = omni.usd.get_context().get_stage()
        self._timeline = omni.timeline.get_timeline_interface()
        UsdPhysics.Scene.Define(self._stage, "/World/physicsScene")

    async def tearDown(self):
        self._timeline.stop()
        await _next_update()
        self._stage = None

    async def _build_robot(self, robot_path="/World/Robot", add_site=False):
        """Construct the asset hierarchy described in the module docstring and return link paths.

        When @p add_site is True, also author an `IsaacSiteAPI` Xform under `base_link`
        (mirroring how `imu_link` sits under `base_link` in `tb3_burger_processed.usda`).
        Sites are deliberately *not* added to `isaac:physics:robotLinks` because the schema
        treats them as a separate concept from links.
        """
        # Create the IsaacRobotAPI root as a plain Xform (no physics APIs).
        UsdGeom.Xform.Define(self._stage, robot_path)

        base_link_path = f"{robot_path}/base_link"
        wheel_left_path = f"{robot_path}/wheel_left"
        wheel_right_path = f"{robot_path}/wheel_right"
        link_paths = [base_link_path, wheel_left_path, wheel_right_path]

        _add_rigid_link(self._stage, base_link_path, [0.0, 0.0, 0.0])
        _add_rigid_link(self._stage, wheel_left_path, [0.0, 0.5, 0.0])
        _add_rigid_link(self._stage, wheel_right_path, [0.0, -0.5, 0.0])

        if add_site:
            site_path = f"{base_link_path}/imu_link"
            UsdGeom.Xform.Define(self._stage, site_path)
            self._stage.GetPrimAtPath(site_path).AddAppliedSchema("IsaacSiteAPI")

        # `UsdPhysicsArticulationRootAPI` belongs on the deeper `base_link`, not the root — this
        # is the layout that the unfixed code path mishandles.
        UsdPhysics.ArticulationRootAPI.Apply(self._stage.GetPrimAtPath(base_link_path))

        # Two revolute joints make this a valid PhysX articulation (a zero-joint articulation root
        # is degenerate; the odometry test below depends on the articulation view materializing).
        joint_paths = []
        for joint_path, body1_path, body1_offset in (
            (f"{robot_path}/joint_left", wheel_left_path, Gf.Vec3f(0.0, 0.5, 0.0)),
            (f"{robot_path}/joint_right", wheel_right_path, Gf.Vec3f(0.0, -0.5, 0.0)),
        ):
            joint = UsdPhysics.RevoluteJoint.Define(self._stage, joint_path)
            joint.CreateBody0Rel().SetTargets([Sdf.Path(base_link_path)])
            joint.CreateBody1Rel().SetTargets([Sdf.Path(body1_path)])
            joint.CreateAxisAttr("Z")
            joint.CreateLocalPos0Attr().Set(body1_offset)
            joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
            # IsaacJointAPI is required for the joint prim to be picked up by
            # `isaac:physics:robotJoints` when the C++ side walks the relationship.
            self._stage.GetPrimAtPath(joint_path).AddAppliedSchema("IsaacJointAPI")
            joint_paths.append(joint_path)

        for link_path in link_paths:
            self._stage.GetPrimAtPath(link_path).AddAppliedSchema("IsaacLinkAPI")

        robot_prim = self._stage.GetPrimAtPath(robot_path)
        robot_prim.AddAppliedSchema("IsaacRobotAPI")
        rel = robot_prim.CreateRelationship("isaac:physics:robotLinks", custom=False)
        for link_path in link_paths:
            rel.AddTarget(Sdf.Path(link_path))

        joints_rel = robot_prim.CreateRelationship("isaac:physics:robotJoints", custom=False)
        for joint_path in joint_paths:
            joints_rel.AddTarget(Sdf.Path(joint_path))

        await _next_update()
        return robot_path, link_paths

    def _create_transform_tree_graph(self, target_prim_paths, parent_prim_path=None):
        set_values = [
            ("ComputeTransformTree.inputs:targetPrims", [RtSdf.Path(p) for p in target_prim_paths]),
        ]
        if parent_prim_path is not None:
            set_values.append(("ComputeTransformTree.inputs:parentPrim", [RtSdf.Path(parent_prim_path)]))

        og.Controller.edit(
            {"graph_path": self.GRAPH_PATH, "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("ComputeTransformTree", "isaacsim.core.nodes.IsaacComputeTransformTree"),
                ],
                og.Controller.Keys.SET_VALUES: set_values,
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "ComputeTransformTree.inputs:execIn"),
                ],
            },
        )

    def _create_joint_name_resolver_graph(self, robot_target_path, joint_names):
        """Create an action graph with `IsaacJointNameResolver` targeting `robot_target_path`."""
        og.Controller.edit(
            {"graph_path": self.GRAPH_PATH, "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("JointNameResolver", "isaacsim.core.nodes.IsaacJointNameResolver"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("JointNameResolver.inputs:targetPrim", [RtSdf.Path(robot_target_path)]),
                    ("JointNameResolver.inputs:jointNames", joint_names),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "JointNameResolver.inputs:execIn"),
                ],
            },
        )

    def _create_odometry_graph(self, chassis_prim_path):
        og.Controller.edit(
            {"graph_path": self.GRAPH_PATH, "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("ComputeOdometry", "isaacsim.core.nodes.IsaacComputeOdometry"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("ComputeOdometry.inputs:chassisPrim", [RtSdf.Path(chassis_prim_path)]),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "ComputeOdometry.inputs:execIn"),
                ],
            },
        )

    async def _step(self, num_steps=1):
        for _ in range(num_steps):
            await _next_update()

    async def test_transform_tree_expands_robot_api_root_to_links(self):
        """Targeting an `IsaacRobotAPI` root prim must emit one transform pair per `robotLinks`
        entry, with parent frames reconstructed from `isaac:physics:robotJoints`. Pre-fix this
        returned a single frame for the root only; pre-topology-fix it returned every link
        parented to `world` instead of mirroring the joint graph."""
        robot_path, link_paths = await self._build_robot()

        self._create_transform_tree_graph([robot_path])
        await self._step()
        self._timeline.play()
        await self._step(5)
        await og.Controller.evaluate(self.GRAPH_PATH)

        child_frames = og.Controller.get(f"{self.GRAPH_PATH}/ComputeTransformTree.outputs:childFrames")
        parent_frames = og.Controller.get(f"{self.GRAPH_PATH}/ComputeTransformTree.outputs:parentFrames")
        self.assertEqual(
            len(child_frames),
            len(link_paths),
            f"Expected one childFrame per IsaacRobotAPI link, got {list(child_frames)}",
        )
        for expected in ("base_link", "wheel_left", "wheel_right"):
            self.assertIn(expected, child_frames, f"Expected '{expected}' in childFrames")

        # `base_link` is the articulation root (no joint references it as body1), so its parent
        # falls back to the node's default parent frame `world` (no `parentPrim` set above).
        # Wheels are body1 of `joint_left` / `joint_right` whose body0 is `base_link`, so their
        # parents must resolve to `base_link` rather than the flat-to-world behavior of the
        # pre-topology-fix code path.
        parent_by_child = {child: parent for child, parent in zip(child_frames, parent_frames)}
        self.assertEqual(
            parent_by_child.get("base_link"),
            "world",
            f"Expected base_link's parent to be 'world', got {parent_by_child}",
        )
        self.assertEqual(
            parent_by_child.get("wheel_left"),
            "base_link",
            f"Expected wheel_left's parent to be 'base_link', got {parent_by_child}",
        )
        self.assertEqual(
            parent_by_child.get("wheel_right"),
            "base_link",
            f"Expected wheel_right's parent to be 'base_link', got {parent_by_child}",
        )

    async def test_transform_tree_skips_self_loop_when_parent_in_robot_links(self):
        """When `parentPrim` is set to a link that also appears in `robotLinks`, the IsaacRobotAPI
        fallback must NOT emit that link as one of its own children. Pre-fix this produced a
        degenerate `base_link -> base_link` transform that ROS TF rejects with
        `TF_SELF_TRANSFORM: Ignoring transform from authority "default_authority" with frame_id
        and child_frame_id "base_link" because they are the same`."""
        robot_path, _ = await self._build_robot()
        base_link_path = f"{robot_path}/base_link"

        self._create_transform_tree_graph([robot_path], parent_prim_path=base_link_path)
        await self._step()
        self._timeline.play()
        await self._step(5)
        await og.Controller.evaluate(self.GRAPH_PATH)

        child_frames = list(og.Controller.get(f"{self.GRAPH_PATH}/ComputeTransformTree.outputs:childFrames"))
        parent_frames = list(og.Controller.get(f"{self.GRAPH_PATH}/ComputeTransformTree.outputs:parentFrames"))
        pairs = list(zip(parent_frames, child_frames))

        self.assertNotIn(
            ("base_link", "base_link"),
            pairs,
            f"IsaacRobotAPI fallback emitted a self-loop pair (base_link, base_link). Pairs: {pairs}",
        )
        # The two wheels should still be present, parented to base_link.
        self.assertIn(("base_link", "wheel_left"), pairs, f"wheel_left missing or wrongly parented: {pairs}")
        self.assertIn(("base_link", "wheel_right"), pairs, f"wheel_right missing or wrongly parented: {pairs}")

    async def test_transform_tree_publishes_isaac_site_descendants(self):
        """`IsaacSiteAPI` prims under a robotLink (e.g. an `imu_link` sensor mount under
        `base_link`) must be published as TF frames parented to the link they descend from.
        Sites are part of the robot schema but not authored into `isaac:physics:robotLinks`,
        so without this branch the user-visible TF tree from the IsaacRobotAPI shortcut is
        missing sensor frames that the manual-list workaround would publish."""
        robot_path, _ = await self._build_robot(add_site=True)
        base_link_path = f"{robot_path}/base_link"

        self._create_transform_tree_graph([robot_path], parent_prim_path=base_link_path)
        await self._step()
        self._timeline.play()
        await self._step(5)
        await og.Controller.evaluate(self.GRAPH_PATH)

        child_frames = list(og.Controller.get(f"{self.GRAPH_PATH}/ComputeTransformTree.outputs:childFrames"))
        parent_frames = list(og.Controller.get(f"{self.GRAPH_PATH}/ComputeTransformTree.outputs:parentFrames"))
        pairs = list(zip(parent_frames, child_frames))

        self.assertIn(
            ("base_link", "imu_link"),
            pairs,
            f"Expected `imu_link` (IsaacSiteAPI under base_link) to be published, got pairs={pairs}",
        )
        # Still emit the wheel rigid bodies — site discovery must not crowd them out.
        self.assertIn(("base_link", "wheel_left"), pairs, f"wheel_left missing: {pairs}")
        self.assertIn(("base_link", "wheel_right"), pairs, f"wheel_right missing: {pairs}")

    async def test_odometry_resolves_robot_api_root_to_articulation_link(self):
        """Pointing `chassisPrim` at an `IsaacRobotAPI` root must succeed by resolving through
        `robotLinks` to the link that carries `UsdPhysicsArticulationRootAPI` / `RigidBodyAPI`.

        Pre-fix this raised a `logError` (`'/World/Robot' is not a valid rigid body or articulation
        root'`) and triggered an `omni.physx.tensors` warning because the root prim has neither API
        directly. The `compute()` then short-circuited via `return false`, leaving outputs at their
        defaults (zero) for the rest of simulation.

        We assert the post-fix behavior by warming up physics for long enough that an actually-
        attached chassis would have measurably fallen under gravity. A pre-fix run sits at exactly
        zero forever, post-fix the articulation root pose follows physx."""
        robot_path, _ = await self._build_robot()

        self._create_odometry_graph(robot_path)

        # Step the graph enough that gravity has time to act. 60 frames at the default timestep is
        # enough for the chassis to fall well below its starting height.
        await self._step()
        self._timeline.play()
        await self._step(60)
        await og.Controller.evaluate(self.GRAPH_PATH)

        position = og.Controller.get(f"{self.GRAPH_PATH}/ComputeOdometry.outputs:position")
        global_linear_velocity = og.Controller.get(f"{self.GRAPH_PATH}/ComputeOdometry.outputs:globalLinearVelocity")

        # Body is in free fall under gravity (no ground plane); z velocity must be measurably
        # negative and z position must have dropped well below the starting (0, 0, 0). If the node
        # falls back to its logError branch, both stay at exactly zero forever.
        self.assertLess(
            float(position[2]),
            -0.1,
            f"Expected body to have fallen under gravity, got position={list(position)}. "
            "A zero result means odometry did not resolve through IsaacRobotAPI to the articulation link.",
        )
        self.assertLess(
            float(global_linear_velocity[2]),
            -0.5,
            f"Expected negative z linear velocity from gravity, got vz={float(global_linear_velocity[2])}. "
            "A zero result means odometry did not resolve through IsaacRobotAPI to the articulation link.",
        )

    async def test_joint_name_resolver_descends_to_articulation_root(self):
        """Pointing `IsaacJointNameResolver` at an `IsaacRobotAPI` root prim must succeed by
        descending into the prim hierarchy to find the `UsdPhysicsArticulationRootAPI` link, then
        building the `isaac:nameOverride` map from that subtree.

        Pre-fix this raised a `logError` (`'Articulation not found for prim /World/Robot'`) because
        the root prim lacks `UsdPhysicsArticulationRootAPI` directly. The `compute()` then
        short-circuited via `return false`, leaving `outputs:jointNames` empty and silently
        breaking downstream joint-name resolution.
        """
        robot_path, _ = await self._build_robot()

        joint_left_path = f"{robot_path}/joint_left"
        joint_right_path = f"{robot_path}/joint_right"

        for joint_path, override in (
            (joint_left_path, "wheel_left_renamed"),
            (joint_right_path, "wheel_right_renamed"),
        ):
            joint_prim = self._stage.GetPrimAtPath(joint_path)
            self.assertTrue(joint_prim.IsValid(), f"Joint prim at {joint_path} should be valid")
            joint_prim.CreateAttribute("isaac:nameOverride", Sdf.ValueTypeNames.String).Set(override)

        input_names = ["wheel_left_renamed", "wheel_right_renamed"]
        self._create_joint_name_resolver_graph(robot_path, input_names)

        await self._step()
        self._timeline.play()
        await self._step(5)
        await og.Controller.evaluate(self.GRAPH_PATH)

        out_names = og.Controller.get(f"{self.GRAPH_PATH}/JointNameResolver.outputs:jointNames")
        out_robot_path = og.Controller.get(f"{self.GRAPH_PATH}/JointNameResolver.outputs:robotPath")

        # The override map must have been built from descendants of the IsaacRobotAPI root, so the
        # input override names resolve to the real joint prim names.
        self.assertEqual(
            list(out_names),
            ["joint_left", "joint_right"],
            f"Expected overrides to resolve to real joint names, got {list(out_names)}. "
            "An unresolved/echoed input means the resolver did not descend through IsaacRobotAPI "
            "to find the ArticulationRootAPI prim, leaving the override map empty.",
        )
        self.assertEqual(out_robot_path, robot_path)
