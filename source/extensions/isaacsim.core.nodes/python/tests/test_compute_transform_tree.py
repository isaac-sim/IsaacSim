# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

import asyncio

import isaacsim.core.experimental.utils.prim as prim_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import omni.graph.core as og
import omni.graph.core.tests as ogts
import omni.usd
from isaacsim.core.experimental.objects import Camera, Cube
from isaacsim.core.experimental.prims import GeomPrim, RigidPrim, XformPrim
from pxr import UsdGeom
from usdrt import Sdf


async def _next_update():
    await omni.kit.app.get_app().next_update_async()


def _add_rigid_prim(_stage, path, positions, orientations=None, size=1.0):
    """Add a rigid body prim at path so IsaacComputeTransformTree's discovery does not log errors."""
    Cube(path, sizes=size)
    RigidPrim(
        path,
        positions=positions,
        orientations=orientations,
        masses=1.0,
        reset_xform_op_properties=True,
    )
    GeomPrim(path, apply_collision_apis=True)


async def _set_xform_translation(stage, path, positions):
    """Update the existing translate op on a plain Xform prim."""
    XformPrim(path).set_local_poses(translations=positions)
    await _next_update()


def _set_name_override(_stage, path, name):
    """Author `isaac:nameOverride` on the prim at @p path."""
    attr = prim_utils.create_prim_attribute(path, name="isaac:nameOverride", type_name="string")
    attr.Set(name)


class TestIsaacComputeTransformTree(ogts.OmniGraphTestCase):
    GRAPH_PATH = "/ActionGraph"
    NODE_NAME = "ComputeTransformTree"
    NODE_TYPE = "isaacsim.core.nodes.IsaacComputeTransformTree"

    async def setUp(self):
        await omni.usd.get_context().new_stage_async()
        self._stage = omni.usd.get_context().get_stage()
        self._timeline = omni.timeline.get_timeline_interface()

    async def tearDown(self):
        self._timeline.stop()
        await _next_update()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            await asyncio.sleep(0.5)
        await _next_update()
        # Drop our strong handle so Kit's UsdContext refcount returns to 1 when it closes the
        # stage — suppresses the `Unexpected reference count` diagnostic that would otherwise
        # fire once per test.
        self._stage = None

    def _create_graph(self, target_prim_paths, parent_prim_path=None):
        set_values = [
            (f"{self.NODE_NAME}.inputs:targetPrims", [Sdf.Path(p) for p in target_prim_paths]),
        ]
        if parent_prim_path:
            set_values.append((f"{self.NODE_NAME}.inputs:parentPrim", [Sdf.Path(parent_prim_path)]))

        og.Controller.edit(
            {"graph_path": self.GRAPH_PATH, "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    (self.NODE_NAME, self.NODE_TYPE),
                ],
                og.Controller.Keys.SET_VALUES: set_values,
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", f"{self.NODE_NAME}.inputs:execIn"),
                ],
            },
        )

    def _get_outputs(self):
        return {
            "parentFrames": og.Controller.get(f"{self.GRAPH_PATH}/{self.NODE_NAME}.outputs:parentFrames"),
            "childFrames": og.Controller.get(f"{self.GRAPH_PATH}/{self.NODE_NAME}.outputs:childFrames"),
            "translations": og.Controller.get(f"{self.GRAPH_PATH}/{self.NODE_NAME}.outputs:translations"),
            "orientations": og.Controller.get(f"{self.GRAPH_PATH}/{self.NODE_NAME}.outputs:orientations"),
        }

    async def _step(self, num_steps=1):
        for _ in range(num_steps):
            await _next_update()

    async def _play_and_evaluate(self, warmup_steps=5):
        """Common driver for tests: finalize graph authoring (1 tick), start the timeline,
        let the sim warm up, then force one explicit graph evaluation. Consolidates the 4-line
        boilerplate each test used to repeat."""
        await self._step()
        self._timeline.play()
        await self._step(warmup_steps)
        await og.Controller.evaluate(self.GRAPH_PATH)

    async def test_single_xform_prim(self):
        """Single rigid prim at known translation; verify parent frame is 'world' and translation matches."""
        cube_path = "/World/Cube"
        _add_rigid_prim(self._stage, cube_path, [2.0, 3.0, 4.0])
        await self._step(2)

        self._create_graph([cube_path])
        await self._play_and_evaluate()

        out = self._get_outputs()
        self.assertGreater(len(out["childFrames"]), 0, "Expected at least one transform pair")
        # The parent frame should be 'world' (no parentPrim specified)
        self.assertEqual(out["parentFrames"][0], "world", "Parent frame should be 'world'")
        # Child frame name should match the prim name
        self.assertEqual(out["childFrames"][0], "Cube", "Child frame should be 'Cube'")
        # Translation should be close to the set position
        trans = out["translations"][0]
        self.assertAlmostEqual(trans[0], 2.0, delta=0.1, msg="Translation x")
        self.assertAlmostEqual(trans[1], 3.0, delta=0.1, msg="Translation y")
        self.assertAlmostEqual(trans[2], 4.0, delta=0.1, msg="Translation z")
        # Orientation should be near identity (x,y,z,w) = (0,0,0,1)
        ori = out["orientations"][0]
        self.assertAlmostEqual(ori[3], 1.0, delta=0.01, msg="Orientation w should be ~1")

    async def test_relative_to_parent_prim(self):
        """Child prim at a known offset from parent; translation should be relative offset."""
        parent_path = "/World/Parent"
        child_path = "/World/Parent/Child"
        _add_rigid_prim(self._stage, parent_path, [10.0, 0.0, 0.0])
        _add_rigid_prim(self._stage, child_path, [15.0, 0.0, 0.0])
        await self._step(2)

        self._create_graph([child_path], parent_prim_path=parent_path)
        await self._play_and_evaluate()

        out = self._get_outputs()
        self.assertGreater(len(out["childFrames"]), 0, "Expected at least one transform pair")
        trans = out["translations"][0]
        # Child is 5 units from parent along X
        self.assertAlmostEqual(trans[0], 5.0, delta=0.2, msg="Relative translation x should be ~5")
        self.assertAlmostEqual(trans[1], 0.0, delta=0.1, msg="Relative translation y should be ~0")
        self.assertAlmostEqual(trans[2], 0.0, delta=0.1, msg="Relative translation z should be ~0")

    async def test_no_target_prims_produces_no_output(self):
        """With no targetPrims set, the node's compute guard short-circuits before writing any
        output arrays. Previously named `_logs_error` but we don't actually assert on log output."""
        og.Controller.edit(
            {"graph_path": self.GRAPH_PATH, "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    (self.NODE_NAME, self.NODE_TYPE),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", f"{self.NODE_NAME}.inputs:execIn"),
                ],
            },
        )
        await self._play_and_evaluate()

        out = self._get_outputs()
        # Without target prims, outputs should be empty
        self.assertEqual(len(out["childFrames"]), 0, "childFrames should be empty with no target prims")

    async def test_multiple_prims_produce_multiple_pairs(self):
        """Two separate rigid prims result in two output transform pairs."""
        paths = ["/World/PrimA", "/World/PrimB"]
        for i, p in enumerate(paths):
            _add_rigid_prim(self._stage, p, [float(i), 0.0, 0.0])
        await self._step(2)

        self._create_graph(paths)
        await self._play_and_evaluate()

        out = self._get_outputs()
        self.assertEqual(len(out["childFrames"]), 2, "Expected two transform pairs for two target prims")
        self.assertEqual(len(out["translations"]), 2, "Expected two translation entries")
        self.assertEqual(len(out["orientations"]), 2, "Expected two orientation entries")

    async def test_outputs_are_consistent_lengths(self):
        """All four output arrays must have the same length."""
        cube_path = "/World/Cube"
        _add_rigid_prim(self._stage, cube_path, [0.0, 0.0, 1.0])
        await self._step(2)

        self._create_graph([cube_path])
        await self._play_and_evaluate()

        out = self._get_outputs()
        n = len(out["parentFrames"])
        self.assertEqual(len(out["childFrames"]), n, "childFrames length mismatch")
        self.assertEqual(len(out["translations"]), n, "translations length mismatch")
        self.assertEqual(len(out["orientations"]), n, "orientations length mismatch")
        self.assertGreater(n, 0, "Expected at least one output pair")

    async def test_orientation_is_unit_quaternion(self):
        """Output quaternion (x,y,z,w) should be unit-length."""

        cube_path = "/World/Cube"
        # Apply a 45-degree rotation around Z (w, x, y, z)
        _add_rigid_prim(
            self._stage,
            cube_path,
            [0.0, 0.0, 0.0],
            orientations=[[0.9238795, 0.0, 0.0, 0.3826834]],
        )
        await self._step(2)

        self._create_graph([cube_path])
        await self._play_and_evaluate()

        out = self._get_outputs()
        if len(out["orientations"]) > 0:
            ori = out["orientations"][0]  # (x, y, z, w)
            norm_sq = sum(float(v) * float(v) for v in ori)
            self.assertAlmostEqual(norm_sq, 1.0, delta=0.01, msg="Quaternion should be unit length")

    async def test_rigid_body_with_fixed_sensor_child(self):
        """Non-physics child of a rigid body: world pose includes parent motion + local offset."""
        body_path = "/World/RigidBody"
        sensor_path = "/World/RigidBody/IMU"

        _add_rigid_prim(self._stage, body_path, [3.0, 0.0, 0.0])
        stage_utils.define_prim(sensor_path, "Xform")
        XformPrim(sensor_path, translations=[0.0, 1.0, 0.0], reset_xform_op_properties=True)
        await self._step(2)

        self._create_graph([body_path, sensor_path])
        await self._play_and_evaluate()

        out = self._get_outputs()
        self.assertEqual(len(out["childFrames"]), 2, "Expected two pairs (body + sensor)")

        # Find the sensor pair (child frame "IMU")
        for i, cf in enumerate(out["childFrames"]):
            if cf == "IMU":
                trans = out["translations"][i]
                # Sensor is a child of body; both relative to world, so sensor should
                # be at body_position + local_offset ≈ (3.0, 1.0, 0.0)
                self.assertAlmostEqual(trans[0], 3.0, delta=0.5, msg="Sensor x")
                self.assertAlmostEqual(trans[1], 1.0, delta=0.5, msg="Sensor y")
                self.assertAlmostEqual(trans[2], 0.0, delta=0.5, msg="Sensor z")
                break
        else:
            self.fail("IMU frame not found in output")

    async def test_rigid_body_with_non_physics_parent_prim(self):
        """parentPrim is a non-physics xform; transform should be relative to it."""
        parent_path = "/World/Frame"
        body_path = "/World/Frame/Body"

        stage_utils.define_prim(parent_path, "Xform")
        XformPrim(parent_path, translations=[5.0, 0.0, 0.0], reset_xform_op_properties=True)
        _add_rigid_prim(self._stage, body_path, [5.0, 3.0, 0.0])
        await self._step(2)

        self._create_graph([body_path], parent_prim_path=parent_path)
        await self._play_and_evaluate()

        out = self._get_outputs()
        self.assertGreater(len(out["translations"]), 0)
        trans = out["translations"][0]
        # Body is 3.0 units along Y relative to parent frame
        self.assertAlmostEqual(trans[0], 0.0, delta=0.5, msg="Relative x should be ~0")
        self.assertAlmostEqual(trans[1], 3.0, delta=0.5, msg="Relative y should be ~3")
        self.assertAlmostEqual(trans[2], 0.0, delta=0.5, msg="Relative z should be ~0")

    async def test_multi_level_non_physics_chain(self):
        """Non-physics prim two levels below a rigid body: mount -> sensor."""
        body_path = "/World/Body"
        mount_path = "/World/Body/Mount"
        sensor_path = "/World/Body/Mount/Sensor"

        _add_rigid_prim(self._stage, body_path, [0.0, 0.0, 0.0])
        stage_utils.define_prim(mount_path, "Xform")
        XformPrim(mount_path, translations=[1.0, 0.0, 0.0], reset_xform_op_properties=True)
        stage_utils.define_prim(sensor_path, "Xform")
        XformPrim(sensor_path, translations=[0.0, 0.5, 0.0], reset_xform_op_properties=True)
        await self._step(2)

        self._create_graph([body_path, sensor_path])
        await self._play_and_evaluate()

        out = self._get_outputs()
        for i, cf in enumerate(out["childFrames"]):
            if cf == "Sensor":
                trans = out["translations"][i]
                # Sensor world pos = body(0,0,0) + mount_local(1,0,0) + sensor_local(0,0.5,0)
                self.assertAlmostEqual(trans[0], 1.0, delta=0.5, msg="Sensor x ≈ 1.0")
                self.assertAlmostEqual(trans[1], 0.5, delta=0.5, msg="Sensor y ≈ 0.5")
                self.assertAlmostEqual(trans[2], 0.0, delta=0.5, msg="Sensor z ≈ 0.0")
                break
        else:
            self.fail("Sensor frame not found")

    async def test_camera_under_rigid_body(self):
        """Camera (non-physics) under rigid body still gets 180° x-axis rotation."""
        body_path = "/World/CamBody"
        cam_path = "/World/CamBody/Camera"

        _add_rigid_prim(self._stage, body_path, [0.0, 0.0, 0.0])
        Camera(cam_path, translations=[0.0, 0.0, 0.5], reset_xform_op_properties=True)
        await self._step(2)

        self._create_graph([body_path, cam_path])
        await self._play_and_evaluate()

        out = self._get_outputs()
        for i, cf in enumerate(out["childFrames"]):
            if cf == "Camera":
                ori = out["orientations"][i]  # (x, y, z, w)
                # Camera should have 180° rotation around X from the ROS convention.
                # A 180° x-axis rotation quaternion is (1, 0, 0, 0) in (x,y,z,w).
                self.assertAlmostEqual(abs(ori[0]), 1.0, delta=0.1, msg="Camera ori x ≈ ±1")
                self.assertAlmostEqual(ori[3], 0.0, delta=0.1, msg="Camera ori w ≈ 0")
                break
        else:
            self.fail("Camera frame not found")

    async def test_mixed_physics_and_non_physics_targets(self):
        """Mix of rigid bodies and non-physics xforms as targets."""
        rigid_path = "/World/Rigid"
        xform_path = "/World/Xform"

        _add_rigid_prim(self._stage, rigid_path, [1.0, 0.0, 0.0])
        stage_utils.define_prim(xform_path, "Xform")
        XformPrim(xform_path, translations=[0.0, 2.0, 0.0], reset_xform_op_properties=True)
        await self._step(2)

        self._create_graph([rigid_path, xform_path])
        await self._play_and_evaluate()

        out = self._get_outputs()
        self.assertEqual(len(out["childFrames"]), 2, "Expected two pairs")

        frames = {out["childFrames"][i]: out["translations"][i] for i in range(len(out["childFrames"]))}
        self.assertIn("Rigid", frames, "Rigid frame expected")
        self.assertIn("Xform", frames, "Xform frame expected")
        self.assertAlmostEqual(frames["Rigid"][0], 1.0, delta=0.5)
        self.assertAlmostEqual(frames["Xform"][1], 2.0, delta=0.5)

    async def test_sensor_relative_to_physics_parent(self):
        """Sensor with its physics parent as parentPrim — output is the local offset."""
        body_path = "/World/Bot"
        sensor_path = "/World/Bot/Lidar"

        _add_rigid_prim(self._stage, body_path, [10.0, 0.0, 0.0])
        stage_utils.define_prim(sensor_path, "Xform")
        XformPrim(sensor_path, translations=[0.0, 0.0, 0.5], reset_xform_op_properties=True)
        await self._step(2)

        self._create_graph([sensor_path], parent_prim_path=body_path)
        await self._play_and_evaluate()

        out = self._get_outputs()
        self.assertGreater(len(out["translations"]), 0)
        trans = out["translations"][0]
        # Sensor is 0.5 above body along Z — relative transform should be (0, 0, 0.5)
        self.assertAlmostEqual(trans[0], 0.0, delta=0.2, msg="Relative x ≈ 0")
        self.assertAlmostEqual(trans[1], 0.0, delta=0.2, msg="Relative y ≈ 0")
        self.assertAlmostEqual(trans[2], 0.5, delta=0.2, msg="Relative z ≈ 0.5")

    async def test_all_physics_regression(self):
        """Only physics prims — behavior must match pre-optimization baseline."""
        paths = ["/World/A", "/World/B"]
        positions = [[1.0, 0.0, 0.0], [0.0, 2.0, 0.0]]
        for p, pos in zip(paths, positions):
            _add_rigid_prim(self._stage, p, pos)
        await self._step(2)

        self._create_graph(paths)
        await self._play_and_evaluate()

        out = self._get_outputs()
        self.assertEqual(len(out["childFrames"]), 2)

        frames = {out["childFrames"][i]: out["translations"][i] for i in range(len(out["childFrames"]))}
        self.assertAlmostEqual(frames["A"][0], 1.0, delta=0.5)
        self.assertAlmostEqual(frames["B"][1], 2.0, delta=0.5)

        for ori in out["orientations"]:
            norm_sq = sum(float(v) * float(v) for v in ori)
            self.assertAlmostEqual(norm_sq, 1.0, delta=0.01, msg="Quaternion unit length")

    async def test_all_non_physics_targets(self):
        """All targets are non-physics xforms — no xformView is created, uses pure USD fallback."""
        path_a = "/World/XformA"
        path_b = "/World/XformB"

        stage_utils.define_prim(path_a, "Xform")
        XformPrim(path_a, translations=[1.0, 2.0, 0.0], reset_xform_op_properties=True)
        stage_utils.define_prim(path_b, "Xform")
        XformPrim(path_b, translations=[0.0, 0.0, 3.0], reset_xform_op_properties=True)
        await self._step(2)

        self._create_graph([path_a, path_b])
        await self._play_and_evaluate()

        out = self._get_outputs()
        self.assertEqual(len(out["childFrames"]), 2, "Expected two pairs for two non-physics targets")

        frames = {out["childFrames"][i]: out["translations"][i] for i in range(len(out["childFrames"]))}
        self.assertIn("XformA", frames)
        self.assertIn("XformB", frames)
        self.assertAlmostEqual(frames["XformA"][0], 1.0, delta=0.5, msg="XformA x")
        self.assertAlmostEqual(frames["XformA"][1], 2.0, delta=0.5, msg="XformA y")
        self.assertAlmostEqual(frames["XformB"][2], 3.0, delta=0.5, msg="XformB z")

    async def test_sensor_with_undiscovered_ancestor(self):
        """Sensor targeted alone — its physics ancestor is NOT in the target list but gets auto-discovered."""
        body_path = "/World/Robot"
        sensor_path = "/World/Robot/IMU"

        _add_rigid_prim(self._stage, body_path, [5.0, 0.0, 0.0])
        stage_utils.define_prim(sensor_path, "Xform")
        XformPrim(sensor_path, translations=[0.0, 0.0, 1.0], reset_xform_op_properties=True)
        await self._step(2)

        # Only target the sensor — the rigid body ancestor should be auto-discovered
        self._create_graph([sensor_path])
        await self._play_and_evaluate()

        out = self._get_outputs()
        self.assertGreater(len(out["childFrames"]), 0, "Expected at least one pair")

        trans = out["translations"][0]
        # Sensor world pose = body(5,0,0) + local(0,0,1) ≈ (5,0,1)
        self.assertAlmostEqual(trans[0], 5.0, delta=0.5, msg="Sensor x ≈ 5.0")
        self.assertAlmostEqual(trans[1], 0.0, delta=0.5, msg="Sensor y ≈ 0.0")
        self.assertAlmostEqual(trans[2], 1.0, delta=0.5, msg="Sensor z ≈ 1.0")

    async def test_non_physics_parent_with_physics_ancestor(self):
        """parentPrim is non-physics but has a physics ancestor in its hierarchy."""
        body_path = "/World/Chassis"
        mount_path = "/World/Chassis/SensorMount"
        sensor_path = "/World/Chassis/SensorMount/Lidar"

        _add_rigid_prim(self._stage, body_path, [2.0, 0.0, 0.0])
        stage_utils.define_prim(mount_path, "Xform")
        XformPrim(mount_path, translations=[0.0, 1.0, 0.0], reset_xform_op_properties=True)
        stage_utils.define_prim(sensor_path, "Xform")
        XformPrim(sensor_path, translations=[0.0, 0.0, 0.5], reset_xform_op_properties=True)
        await self._step(2)

        # Target the sensor, parent is the non-physics mount
        self._create_graph([sensor_path], parent_prim_path=mount_path)
        await self._play_and_evaluate()

        out = self._get_outputs()
        self.assertGreater(len(out["translations"]), 0)
        trans = out["translations"][0]
        # Sensor is 0.5 above mount along Z — relative transform should be (0, 0, 0.5)
        self.assertAlmostEqual(trans[0], 0.0, delta=0.5, msg="Relative x ≈ 0")
        self.assertAlmostEqual(trans[1], 0.0, delta=0.5, msg="Relative y ≈ 0")
        self.assertAlmostEqual(trans[2], 0.5, delta=0.5, msg="Relative z ≈ 0.5")

    async def test_all_non_physics_name_overrides(self):
        """In an all-non-physics graph, `isaac:nameOverride` on parent and child must be honored."""
        parent_path = "/World/ParentXform"
        child_path = "/World/ParentXform/ChildXform"

        stage_utils.define_prim(parent_path, "Xform")
        XformPrim(parent_path, translations=[0.0, 0.0, 0.0], reset_xform_op_properties=True)
        stage_utils.define_prim(child_path, "Xform")
        XformPrim(child_path, translations=[1.0, 0.0, 0.0], reset_xform_op_properties=True)
        _set_name_override(self._stage, parent_path, "base_frame")
        _set_name_override(self._stage, child_path, "sensor_frame")
        await self._step(2)

        self._create_graph([child_path], parent_prim_path=parent_path)
        await self._play_and_evaluate()

        out = self._get_outputs()
        self.assertGreater(len(out["childFrames"]), 0)
        self.assertEqual(out["parentFrames"][0], "base_frame", "parentFrame must honor nameOverride")
        self.assertEqual(out["childFrames"][0], "sensor_frame", "childFrame must honor nameOverride")

    async def test_all_non_physics_target_with_parent_prim_frame(self):
        """In an all-non-physics graph with a parentPrim input, parent frame must be the USD leaf name, not 'world'."""
        parent_path = "/World/Frame"
        child_path = "/World/Frame/Leaf"

        stage_utils.define_prim(parent_path, "Xform")
        XformPrim(parent_path, translations=[0.0, 0.0, 0.0], reset_xform_op_properties=True)
        stage_utils.define_prim(child_path, "Xform")
        XformPrim(child_path, translations=[0.0, 1.0, 0.0], reset_xform_op_properties=True)
        await self._step(2)

        self._create_graph([child_path], parent_prim_path=parent_path)
        await self._play_and_evaluate()

        out = self._get_outputs()
        self.assertGreater(len(out["childFrames"]), 0)
        self.assertEqual(out["parentFrames"][0], "Frame", "parentFrame must fall back to USD leaf name")
        self.assertEqual(out["childFrames"][0], "Leaf")

    async def test_non_physics_ancestor_scale_affects_child_offset(self):
        """A scaled non-physics ancestor must scale its child's world translation (2x scale -> 2x offset)."""
        mount_path = "/World/ScaledMount"
        sensor_path = "/World/ScaledMount/Sensor"

        stage_utils.define_prim(mount_path, "Xform")
        XformPrim(mount_path, translations=[0.0, 0.0, 0.0], scales=[2.0, 2.0, 2.0], reset_xform_op_properties=True)
        stage_utils.define_prim(sensor_path, "Xform")
        XformPrim(sensor_path, translations=[1.0, 0.0, 0.0], reset_xform_op_properties=True)
        await self._step(2)

        self._create_graph([sensor_path])
        await self._play_and_evaluate()

        out = self._get_outputs()
        self.assertGreater(len(out["translations"]), 0)
        trans = out["translations"][0]
        # Sensor local x=1 is scaled 2x by the mount, so world translation x should be ~2
        self.assertAlmostEqual(trans[0], 2.0, delta=0.1, msg="Scaled ancestor must scale child offset")

    async def test_non_physics_ancestor_rotation_affects_child_offset(self):
        """A rotated non-physics ancestor must rotate its child's world translation."""
        mount_path = "/World/RotatedMount"
        sensor_path = "/World/RotatedMount/Sensor"

        # 90 deg around Z: (w, x, y, z) = (sqrt(2)/2, 0, 0, sqrt(2)/2). Maps +x -> +y.
        stage_utils.define_prim(mount_path, "Xform")
        mount = XformPrim(mount_path, translations=[0.0, 0.0, 0.0], reset_xform_op_properties=True)
        mount.set_local_poses(orientations=[0.7071068, 0.0, 0.0, 0.7071068])
        stage_utils.define_prim(sensor_path, "Xform")
        XformPrim(sensor_path, translations=[1.0, 0.0, 0.0], reset_xform_op_properties=True)
        await self._step(2)

        self._create_graph([sensor_path])
        await self._play_and_evaluate()

        out = self._get_outputs()
        self.assertGreater(len(out["translations"]), 0)
        trans = out["translations"][0]
        # Sensor local (1, 0, 0) rotated 90 deg about Z -> (0, 1, 0).
        self.assertAlmostEqual(trans[0], 0.0, delta=0.1, msg="Rotated ancestor x")
        self.assertAlmostEqual(trans[1], 1.0, delta=0.1, msg="Rotated ancestor y")

    async def test_reset_xform_stack_ignores_ancestor_transforms(self):
        """A non-physics prim with `!resetXformStack!` must have its local matrix treated as the
        world matrix — ancestor translation/scale must be discarded, matching USD's
        `ComputeLocalToWorldTransform` semantics."""
        mount_path = "/World/Mount"
        sensor_path = "/World/Mount/Sensor"

        # Mount translates +10 on x and scales 2x; sensor sits at local (1, 0, 0). Without the
        # reset flag the sensor world x would be 10 + 2*1 = 12. With the reset flag the sensor
        # must ignore the mount entirely and land at (1, 0, 0).
        stage_utils.define_prim(mount_path, "Xform")
        XformPrim(mount_path, translations=[10.0, 0.0, 0.0], scales=[2.0, 2.0, 2.0], reset_xform_op_properties=True)
        stage_utils.define_prim(sensor_path, "Xform")
        sensor = XformPrim(sensor_path, translations=[1.0, 0.0, 0.0], reset_xform_op_properties=True)
        UsdGeom.Xformable(sensor.prims[0]).SetResetXformStack(True)
        await self._step(2)

        self._create_graph([sensor_path])
        await self._play_and_evaluate()

        out = self._get_outputs()
        self.assertGreater(len(out["translations"]), 0)
        trans = out["translations"][0]
        self.assertAlmostEqual(trans[0], 1.0, delta=0.1, msg="resetXformStack must discard ancestor translation/scale")
        self.assertAlmostEqual(trans[1], 0.0, delta=0.1)
        self.assertAlmostEqual(trans[2], 0.0, delta=0.1)

    async def test_non_physics_parent_discovers_physics_ancestor_not_in_targets(self):
        """parentPrim is non-physics and has a physics ancestor that is NOT in targetPrims. The ancestor
        must still be auto-discovered so the parent's world pose tracks physics motion."""
        parent_body_path = "/World/ParentBody"
        parent_mount_path = "/World/ParentBody/Mount"
        target_body_path = "/World/TargetBody"

        _add_rigid_prim(self._stage, parent_body_path, [2.0, 0.0, 0.0])
        stage_utils.define_prim(parent_mount_path, "Xform")
        XformPrim(parent_mount_path, translations=[0.0, 1.0, 0.0], reset_xform_op_properties=True)
        _add_rigid_prim(self._stage, target_body_path, [5.0, 1.0, 0.0])
        await self._step(2)

        # Only the target body is in targetPrims. The parent body's tensor pose must be
        # discovered through parentPrim's ancestor chain, not through target discovery.
        self._create_graph([target_body_path], parent_prim_path=parent_mount_path)
        await self._play_and_evaluate()

        RigidPrim(parent_body_path).set_world_poses(positions=[[4.0, 0.0, 0.0]])
        await self._step(5)
        await og.Controller.evaluate(self.GRAPH_PATH)

        out = self._get_outputs()
        self.assertGreater(len(out["translations"]), 0)
        trans = out["translations"][0]
        # Target is at (5, 1, 0); moved parent mount is at (4, 1, 0), so relative X should be ~1.
        self.assertAlmostEqual(trans[0], 1.0, delta=0.5, msg="Relative x should reflect moved parent ancestor")
        self.assertAlmostEqual(trans[1], 0.0, delta=0.5, msg="Relative y should be ~0")
        # Parent frame should be the USD leaf name "Mount" (not "world"), confirming the parent's
        # frame name resolves through the USD stage even when it's not a physics prim.
        self.assertEqual(out["parentFrames"][0], "Mount")

    # -------- Frame-name resolution edge cases --------

    async def test_nameoverride_collision_deepest_wins(self):
        """Two prims with the same `isaac:nameOverride`: the deeper path keeps the name; the
        shallower one falls back to a disambiguated name. This validates the `deepest` preference
        so sensor leaves take priority over mount parents."""
        mount_path = "/World/Mount"
        sensor_path = "/World/Mount/Sensor"

        stage_utils.define_prim(mount_path, "Xform")
        XformPrim(mount_path, translations=[0.0, 0.0, 0.0], reset_xform_op_properties=True)
        stage_utils.define_prim(sensor_path, "Xform")
        XformPrim(sensor_path, translations=[0.0, 0.0, 1.0], reset_xform_op_properties=True)
        _set_name_override(self._stage, mount_path, "shared_frame")
        _set_name_override(self._stage, sensor_path, "shared_frame")
        await self._step(2)

        self._create_graph([mount_path, sensor_path])
        await self._play_and_evaluate()

        out = self._get_outputs()
        frames = list(out["childFrames"])
        self.assertEqual(len(frames), 2)
        # Deepest (sensor) keeps the override; mount must get a different frame name.
        self.assertIn("shared_frame", frames)
        self.assertEqual(frames.count("shared_frame"), 1, "Only the deepest prim keeps the shared override")

    async def test_duplicate_leaf_name_gets_ancestor_qualified_name(self):
        """Two prims share the USD leaf name 'Link'. Disambiguation must produce an ancestor-qualified
        candidate (e.g. `RobotA_Link`) for one of them instead of the bare leaf."""
        robot_a_link = "/World/RobotA/Link"
        robot_b_link = "/World/RobotB/Link"

        _add_rigid_prim(self._stage, "/World/RobotA", [0.0, 0.0, 0.0])
        stage_utils.define_prim(robot_a_link, "Xform")
        XformPrim(robot_a_link, translations=[0.0, 0.0, 0.1], reset_xform_op_properties=True)
        _add_rigid_prim(self._stage, "/World/RobotB", [5.0, 0.0, 0.0])
        stage_utils.define_prim(robot_b_link, "Xform")
        XformPrim(robot_b_link, translations=[0.0, 0.0, 0.1], reset_xform_op_properties=True)
        await self._step(2)

        self._create_graph([robot_a_link, robot_b_link])
        await self._play_and_evaluate()

        out = self._get_outputs()
        frames = list(out["childFrames"])
        self.assertEqual(len(frames), 2)
        self.assertEqual(len(set(frames)), 2, "Duplicate leaf names must be disambiguated")
        # One of them should still be 'Link' (the deepest alphabetical winner keeps the leaf);
        # the other gets a qualified name that contains a robot ancestor.
        self.assertTrue(
            any(f == "Link" for f in frames) and any(("RobotA" in f or "RobotB" in f) for f in frames),
            f"Expected one leaf and one ancestor-qualified frame, got {frames}",
        )

    # -------- Transform correctness edge cases --------

    async def test_parent_rotation_rotates_relative_translation(self):
        """`computeRelativeTransform` rotates the world-space delta by the parent's inverse rotation.
        A 90 deg Z rotation on the parent means a child at world (+1, 0, 0) relative to parent
        origin should appear at parent-local (0, -1, 0)."""
        parent_path = "/World/RotatedParent"
        child_path = "/World/Child"

        # 90 deg around Z: (w, x, y, z) = (sqrt(2)/2, 0, 0, sqrt(2)/2)
        stage_utils.define_prim(parent_path, "Xform")
        parent = XformPrim(parent_path, translations=[0.0, 0.0, 0.0], reset_xform_op_properties=True)
        parent.set_local_poses(orientations=[0.7071068, 0.0, 0.0, 0.7071068])
        _add_rigid_prim(self._stage, child_path, [1.0, 0.0, 0.0])
        await self._step(2)

        self._create_graph([child_path], parent_prim_path=parent_path)
        await self._play_and_evaluate()

        out = self._get_outputs()
        self.assertGreater(len(out["translations"]), 0)
        trans = out["translations"][0]
        # World delta is (+1, 0, 0). Inverse of a +90 Z rotation maps +x -> -y.
        self.assertAlmostEqual(trans[0], 0.0, delta=0.2, msg="Parent-inverse rotation: x component")
        self.assertAlmostEqual(trans[1], -1.0, delta=0.2, msg="Parent-inverse rotation: y component ≈ -1")

    async def test_physics_target_tracks_reauthored_non_physics_parent_prim(self):
        """When all targets are physics, a non-physics parentPrim must still recompute its USD local transform each frame."""
        parent_path = "/World/Frame"
        body_path = "/World/Body"

        stage_utils.define_prim(parent_path, "Xform")
        XformPrim(parent_path, translations=[0.0, 0.0, 0.0], reset_xform_op_properties=True)
        _add_rigid_prim(self._stage, body_path, [5.0, 0.0, 0.0])
        await self._step(2)

        self._create_graph([body_path], parent_prim_path=parent_path)
        await self._play_and_evaluate()

        out = self._get_outputs()
        self.assertGreater(len(out["translations"]), 0)
        self.assertAlmostEqual(out["translations"][0][0], 5.0, delta=0.2, msg="Initial body x relative to parent")

        await _set_xform_translation(self._stage, parent_path, [2.0, 0.0, 0.0])
        await self._step(5)
        await og.Controller.evaluate(self.GRAPH_PATH)

        out = self._get_outputs()
        self.assertGreater(len(out["translations"]), 0)
        trans = out["translations"][0]
        self.assertAlmostEqual(trans[0], 3.0, delta=0.2, msg="Body x must reflect reauthored parentPrim")
        self.assertAlmostEqual(trans[1], 0.0, delta=0.1)

    async def test_scale_ancestor_keeps_output_quaternion_unit(self):
        """A scaled non-physics ancestor must NOT leak scale into the output quaternion. The rotation
        sub-matrix is orthonormalized before decomposition so the quaternion stays unit length."""
        mount_path = "/World/ScaledMount"
        sensor_path = "/World/ScaledMount/Sensor"

        stage_utils.define_prim(mount_path, "Xform")
        XformPrim(mount_path, translations=[0.0, 0.0, 0.0], scales=[3.0, 3.0, 3.0], reset_xform_op_properties=True)
        stage_utils.define_prim(sensor_path, "Xform")
        XformPrim(sensor_path, translations=[1.0, 0.0, 0.0], reset_xform_op_properties=True)
        await self._step(2)

        self._create_graph([sensor_path])
        await self._play_and_evaluate()

        out = self._get_outputs()
        self.assertGreater(len(out["orientations"]), 0)
        ori = out["orientations"][0]  # (x, y, z, w)
        norm_sq = sum(float(v) * float(v) for v in ori)
        self.assertAlmostEqual(norm_sq, 1.0, delta=0.01, msg="Scaled ancestor must not leak scale into quaternion")

    async def test_non_physics_sensor_tracks_moving_physics_parent(self):
        """Moving a physics rigid body between evaluations must propagate to its non-physics child
        sensor's world pose. This confirms the per-frame physics-pose read path plus non-physics
        local-chain composition correctly re-runs each frame (not cached)."""
        body_path = "/World/Body"
        sensor_path = "/World/Body/Sensor"

        _add_rigid_prim(self._stage, body_path, [0.0, 0.0, 0.0])
        stage_utils.define_prim(sensor_path, "Xform")
        XformPrim(sensor_path, translations=[0.0, 0.0, 1.0], reset_xform_op_properties=True)
        await self._step(2)

        self._create_graph([sensor_path])
        await self._play_and_evaluate()

        # Sanity: sensor world ≈ (0, 0, 1) before motion.
        out = self._get_outputs()
        sensor_idx = next(i for i, cf in enumerate(out["childFrames"]) if cf == "Sensor")
        self.assertAlmostEqual(out["translations"][sensor_idx][0], 0.0, delta=0.5)
        self.assertAlmostEqual(out["translations"][sensor_idx][2], 1.0, delta=0.5)

        # Teleport the rigid body; sensor world must follow.
        RigidPrim(body_path).set_world_poses(positions=[[4.0, 0.0, 0.0]])
        await self._step(5)
        await og.Controller.evaluate(self.GRAPH_PATH)

        out = self._get_outputs()
        sensor_idx = next(i for i, cf in enumerate(out["childFrames"]) if cf == "Sensor")
        trans = out["translations"][sensor_idx]
        self.assertAlmostEqual(trans[0], 4.0, delta=0.5, msg="Sensor x must track physics parent motion")
        self.assertAlmostEqual(trans[2], 1.0, delta=0.5, msg="Sensor z offset preserved across parent motion")

    async def test_reset_xform_stack_on_intermediate_ancestor(self):
        """`!resetXformStack!` on a middle prim of a chain must discard everything *above* it
        while preserving composition with descendants below. Chain: outer(+10 x) → reset_mid(+5 x, reset)
        → sensor(+1 x). Without reset, sensor would land at x=16; with reset_mid the outer is
        discarded and sensor world x = 5 + 1 = 6."""
        outer_path = "/World/Outer"
        mid_path = "/World/Outer/ResetMid"
        sensor_path = "/World/Outer/ResetMid/Sensor"

        stage_utils.define_prim(outer_path, "Xform")
        XformPrim(outer_path, translations=[10.0, 0.0, 0.0], reset_xform_op_properties=True)
        stage_utils.define_prim(mid_path, "Xform")
        mid = XformPrim(mid_path, translations=[5.0, 0.0, 0.0], reset_xform_op_properties=True)
        UsdGeom.Xformable(mid.prims[0]).SetResetXformStack(True)
        stage_utils.define_prim(sensor_path, "Xform")
        XformPrim(sensor_path, translations=[1.0, 0.0, 0.0], reset_xform_op_properties=True)
        await self._step(2)

        self._create_graph([sensor_path])
        await self._play_and_evaluate()

        out = self._get_outputs()
        self.assertGreater(len(out["translations"]), 0)
        trans = out["translations"][0]
        self.assertAlmostEqual(
            trans[0], 6.0, delta=0.1, msg="Intermediate reset must discard outer ancestor but keep composition below"
        )
        self.assertAlmostEqual(trans[1], 0.0, delta=0.1)
        self.assertAlmostEqual(trans[2], 0.0, delta=0.1)

    async def test_shared_non_physics_ancestor_between_sibling_targets(self):
        """Two non-physics sibling targets share the same mount ancestor chain. Both sensors
        must resolve to correct world poses — and the per-frame local-matrix memo + shared
        xformable cache must not confuse one sibling's world with the other's."""
        body_path = "/World/Body"
        mount_path = "/World/Body/Mount"  # translate (1, 2, 0), shared by both sensors
        sensor_a_path = "/World/Body/Mount/SensorA"  # local (0, 0, 1)
        sensor_b_path = "/World/Body/Mount/SensorB"  # local (0, 0, -1)

        _add_rigid_prim(self._stage, body_path, [0.0, 0.0, 0.0])
        stage_utils.define_prim(mount_path, "Xform")
        XformPrim(mount_path, translations=[1.0, 2.0, 0.0], reset_xform_op_properties=True)
        stage_utils.define_prim(sensor_a_path, "Xform")
        XformPrim(sensor_a_path, translations=[0.0, 0.0, 1.0], reset_xform_op_properties=True)
        stage_utils.define_prim(sensor_b_path, "Xform")
        XformPrim(sensor_b_path, translations=[0.0, 0.0, -1.0], reset_xform_op_properties=True)
        await self._step(2)

        self._create_graph([sensor_a_path, sensor_b_path])
        await self._play_and_evaluate()

        out = self._get_outputs()
        frames = {out["childFrames"][i]: out["translations"][i] for i in range(len(out["childFrames"]))}
        self.assertIn("SensorA", frames)
        self.assertIn("SensorB", frames)
        # Body(0,0,0) + mount(1,2,0) + sensorA(0,0,1) = (1,2,1)
        self.assertAlmostEqual(frames["SensorA"][0], 1.0, delta=0.5)
        self.assertAlmostEqual(frames["SensorA"][1], 2.0, delta=0.5)
        self.assertAlmostEqual(frames["SensorA"][2], 1.0, delta=0.5)
        # Body(0,0,0) + mount(1,2,0) + sensorB(0,0,-1) = (1,2,-1) — memo must not reuse SensorA's value.
        self.assertAlmostEqual(frames["SensorB"][0], 1.0, delta=0.5)
        self.assertAlmostEqual(frames["SensorB"][1], 2.0, delta=0.5)
        self.assertAlmostEqual(frames["SensorB"][2], -1.0, delta=0.5)

    # -------- parentPrim input edge cases --------

    async def test_parent_prim_equals_physics_target_reuses_view(self):
        """When parentPrim equals a physics target already in targetPrims, classifyParentPrim must
        reuse the existing physics view index rather than adding a duplicate entry. The output
        should be the local offset (child relative to itself-as-parent)."""
        body_path = "/World/Body"
        sensor_path = "/World/Body/Sensor"

        _add_rigid_prim(self._stage, body_path, [7.0, 0.0, 0.0])
        stage_utils.define_prim(sensor_path, "Xform")
        XformPrim(sensor_path, translations=[0.0, 0.0, 0.3], reset_xform_op_properties=True)
        await self._step(2)

        # Body is both in targets AND used as parentPrim.
        self._create_graph([body_path, sensor_path], parent_prim_path=body_path)
        await self._play_and_evaluate()

        out = self._get_outputs()
        frames = {out["childFrames"][i]: out["translations"][i] for i in range(len(out["childFrames"]))}
        # Sensor relative to body == local offset (0, 0, 0.3).
        self.assertIn("Sensor", frames)
        self.assertAlmostEqual(frames["Sensor"][2], 0.3, delta=0.2, msg="Sensor z relative to body")
        # Body relative to itself == (0, 0, 0).
        self.assertIn("Body", frames)
        self.assertAlmostEqual(frames["Body"][0], 0.0, delta=0.2, msg="Body relative to itself x")
        self.assertAlmostEqual(frames["Body"][1], 0.0, delta=0.2, msg="Body relative to itself y")
        self.assertAlmostEqual(frames["Body"][2], 0.0, delta=0.2, msg="Body relative to itself z")

    # -------- Sensor-schema edge cases --------

    async def test_new_schema_omni_lidar_prim_skips_180_degree_rotation(self):
        """New-schema RTX lidars use the `OmniLidar` prim type (not `UsdGeomCamera`) from
        `isaacsim.sensors.experimental.rtx`. Since the node's 180° x-axis rotation is gated on
        `IsA<UsdGeomCamera>`, an `OmniLidar` prim must pass through with no rotation. Also
        verifies the non-physics world-pose composition still resolves the lidar's position
        relative to a rigid parent."""
        body_path = "/World/Body"
        lidar_path = "/World/Body/Lidar"

        _add_rigid_prim(self._stage, body_path, [0.0, 0.0, 0.0])
        lidar = stage_utils.define_prim(lidar_path, "OmniLidar")
        lidar.AddAppliedSchema("OmniSensorGenericLidarCoreAPI")
        XformPrim(lidar_path, translations=[0.0, 0.0, 0.5], reset_xform_op_properties=True)
        await self._step(2)

        self._create_graph([body_path, lidar_path])
        await self._play_and_evaluate()

        out = self._get_outputs()
        for i, cf in enumerate(out["childFrames"]):
            if cf == "Lidar":
                ori = out["orientations"][i]  # (x, y, z, w)
                trans = out["translations"][i]
                self.assertAlmostEqual(ori[0], 0.0, delta=0.1, msg="OmniLidar ori x ≈ 0 (no ROS rotation)")
                self.assertAlmostEqual(ori[3], 1.0, delta=0.1, msg="OmniLidar ori w ≈ 1 (identity)")
                self.assertAlmostEqual(trans[2], 0.5, delta=0.2, msg="OmniLidar z ≈ 0.5 (local offset from body)")
                break
        else:
            self.fail("Lidar frame not found")

    async def test_new_schema_omni_sensor_camera_gets_180_degree_rotation(self):
        """New-schema RTX cameras (`isaacsim.sensors.experimental.rtx.RtxCamera`) are still
        `UsdGeomCamera` prims with `OmniSensorAPI` applied. The ROS optical-frame 180° x-axis
        rotation must still be applied for them (they remain cameras, not lidars). Only prims
        carrying the legacy `IsaacRtxLidarSensorAPI` opt out."""
        body_path = "/World/Body"
        cam_path = "/World/Body/RtxCam"

        _add_rigid_prim(self._stage, body_path, [0.0, 0.0, 0.0])
        camera = Camera(cam_path, translations=[0.0, 0.0, 0.5], reset_xform_op_properties=True)
        camera.prims[0].AddAppliedSchema("OmniSensorAPI")
        await self._step(2)

        self._create_graph([body_path, cam_path])
        await self._play_and_evaluate()

        out = self._get_outputs()
        for i, cf in enumerate(out["childFrames"]):
            if cf == "RtxCam":
                ori = out["orientations"][i]  # (x, y, z, w)
                self.assertAlmostEqual(abs(ori[0]), 1.0, delta=0.1, msg="RTX camera still gets 180° rotation: |x| ≈ 1")
                self.assertAlmostEqual(ori[3], 0.0, delta=0.1, msg="RTX camera still gets 180° rotation: w ≈ 0")
                break
        else:
            self.fail("RtxCam frame not found")

    async def test_rtx_lidar_camera_skips_180_degree_rotation(self):
        """Legacy RTX lidar schema: a UsdGeomCamera with `IsaacRtxLidarSensorAPI` applied must
        NOT receive the 180° x-axis rotation intended for RGB cameras. Its output orientation
        should stay near identity. Complementary to the new-schema test above which uses the
        `OmniLidar` prim type instead of reusing `UsdGeomCamera`."""
        body_path = "/World/Body"
        lidar_path = "/World/Body/Lidar"

        _add_rigid_prim(self._stage, body_path, [0.0, 0.0, 0.0])
        camera = Camera(lidar_path, translations=[0.0, 0.0, 0.5], reset_xform_op_properties=True)
        # Mark the camera as an RTX Lidar to opt out of the ROS optical-frame convention rotation.
        camera.prims[0].AddAppliedSchema("IsaacRtxLidarSensorAPI")
        await self._step(2)

        self._create_graph([body_path, lidar_path])
        await self._play_and_evaluate()

        out = self._get_outputs()
        for i, cf in enumerate(out["childFrames"]):
            if cf == "Lidar":
                ori = out["orientations"][i]  # (x, y, z, w)
                # With no base rotation, orientation should be near identity (w ≈ 1, x,y,z ≈ 0).
                self.assertAlmostEqual(ori[0], 0.0, delta=0.1, msg="RTX Lidar ori x ≈ 0")
                self.assertAlmostEqual(ori[3], 1.0, delta=0.1, msg="RTX Lidar ori w ≈ 1")
                break
        else:
            self.fail("Lidar frame not found")

    async def test_camera_under_scaled_ancestor_composes_scale_and_rotation(self):
        """A camera under a scaled non-physics mount must (a) have its position scaled by the
        ancestor, (b) still receive the ROS 180° x-axis rotation, and (c) emit a unit-length
        output quaternion (scale must not leak into the rotation)."""
        mount_path = "/World/Mount"
        cam_path = "/World/Mount/Camera"

        stage_utils.define_prim(mount_path, "Xform")
        XformPrim(mount_path, translations=[0.0, 0.0, 0.0], scales=[2.0, 2.0, 2.0], reset_xform_op_properties=True)
        Camera(cam_path, translations=[0.0, 0.0, 1.0], reset_xform_op_properties=True)
        await self._step(2)

        self._create_graph([cam_path])
        await self._play_and_evaluate()

        out = self._get_outputs()
        self.assertGreater(len(out["translations"]), 0)
        trans = out["translations"][0]
        ori = out["orientations"][0]  # (x, y, z, w)

        # Position: camera local z=1, mount scales 2x → world z = 2.
        self.assertAlmostEqual(trans[2], 2.0, delta=0.1, msg="Scaled ancestor must scale camera z offset")
        # Rotation: camera still receives the 180° x-axis rotation → (x, y, z, w) ≈ (±1, 0, 0, 0).
        self.assertAlmostEqual(abs(ori[0]), 1.0, delta=0.1, msg="Camera still gets ROS 180° x rotation: |x| ≈ 1")
        self.assertAlmostEqual(ori[3], 0.0, delta=0.1, msg="Camera still gets ROS 180° x rotation: w ≈ 0")
        # Unit length guarantee — scale must not contaminate the quaternion.
        norm_sq = sum(float(v) * float(v) for v in ori)
        self.assertAlmostEqual(
            norm_sq, 1.0, delta=0.01, msg="Camera quaternion must stay unit length under ancestor scale"
        )

    async def test_name_override_on_camera_honored_with_rotation(self):
        """A camera target with `isaac:nameOverride` must surface both the override as its
        childFrame AND still receive the ROS 180° x-axis rotation. Confirms frame-name
        resolution and camera rotation are independent paths."""
        body_path = "/World/Body"
        cam_path = "/World/Body/Camera"

        _add_rigid_prim(self._stage, body_path, [0.0, 0.0, 0.0])
        Camera(cam_path, translations=[0.0, 0.0, 0.5], reset_xform_op_properties=True)
        _set_name_override(self._stage, cam_path, "front_rgb")
        await self._step(2)

        self._create_graph([body_path, cam_path])
        await self._play_and_evaluate()

        out = self._get_outputs()
        self.assertIn("front_rgb", out["childFrames"], "Camera nameOverride must be honored")
        cam_idx = list(out["childFrames"]).index("front_rgb")
        ori = out["orientations"][cam_idx]  # (x, y, z, w)
        self.assertAlmostEqual(abs(ori[0]), 1.0, delta=0.1, msg="Camera with override still gets 180° rotation")
        self.assertAlmostEqual(ori[3], 0.0, delta=0.1, msg="Camera with override still gets 180° rotation")

    # -------- Lifecycle edge cases --------

    async def test_compute_returns_no_output_before_timeline_play(self):
        """Before `timeline.play()`, the simulation manager reports not simulating and compute
        returns false. No output pairs should be written."""
        cube_path = "/World/Cube"
        _add_rigid_prim(self._stage, cube_path, [1.0, 0.0, 0.0])
        await self._step(2)

        self._create_graph([cube_path])
        await self._step()
        # Deliberately do NOT call self._timeline.play().
        await og.Controller.evaluate(self.GRAPH_PATH)

        out = self._get_outputs()
        self.assertEqual(len(out["childFrames"]), 0, "No output expected before timeline.play()")
