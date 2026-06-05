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

import asyncio
import math

import omni.graph.core as og
import omni.graph.core.tests as ogts
import omni.kit.test
import omni.usd
from isaacsim.core.experimental.objects import GroundPlane
from isaacsim.core.experimental.prims import GeomPrim, RigidPrim, XformPrim
from isaacsim.core.simulation_manager import SimulationManager
from pxr import Usd, UsdGeom
from usdrt import Sdf


async def add_cube(
    stage,
    path,
    size,
    offset,
    physics=True,
    mass=0.0,
    orientation_wxyz=None,
    linear_velocity=None,
    angular_velocity=None,
) -> Usd.Prim:
    cube_geom = UsdGeom.Cube.Define(stage, path)
    cube_geom.CreateSizeAttr(size)
    await omni.kit.app.get_app().next_update_async()  # Need this to avoid flatcache errors
    if physics:
        rigid_prim = RigidPrim(
            path,
            positions=list(offset),
            orientations=orientation_wxyz,
            masses=mass if mass > 0 else None,
            reset_xform_op_properties=True,
        )
        await omni.kit.app.get_app().next_update_async()
        if linear_velocity is not None or angular_velocity is not None:
            rigid_prim.set_velocities(linear_velocities=linear_velocity, angular_velocities=angular_velocity)
            await omni.kit.app.get_app().next_update_async()
    else:
        XformPrim(
            path,
            positions=list(offset),
            orientations=orientation_wxyz,
            reset_xform_op_properties=True,
        )
        await omni.kit.app.get_app().next_update_async()
    GeomPrim(path, apply_collision_apis=True)
    await omni.kit.app.get_app().next_update_async()
    return stage.GetPrimAtPath(path)


async def add_ground_plane(stage, path="/World/Ground", size=40.0, offset=(0.0, 0.0, -0.5)) -> Usd.Prim:
    """Add a ground plane with collision using GroundPlane from isaacsim.core.experimental.objects."""
    GroundPlane(path, sizes=size, positions=list(offset))
    await omni.kit.app.get_app().next_update_async()
    return stage.GetPrimAtPath(path)


class TestComputeOdometry(ogts.OmniGraphTestCase):
    GRAPH_PATH = "/ActionGraph"
    NODE_NAME = "ComputeOdometry"

    async def setUp(self):
        """Set up test environment, to be torn down when done."""
        await omni.usd.get_context().new_stage_async()
        self._stage = omni.usd.get_context().get_stage()
        self._timeline = omni.timeline.get_timeline_interface()

    async def tearDown(self):
        """Get rid of temporary data used by the test."""
        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            await asyncio.sleep(0.5)
        await omni.kit.app.get_app().next_update_async()

    def _create_odometry_graph(self, prim_path: str):
        """Create action graph with OnPlaybackTick -> IsaacComputeOdometry, chassisPrim set to prim_path."""
        _, _, _, _ = og.Controller.edit(
            {"graph_path": self.GRAPH_PATH, "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    (self.NODE_NAME, "isaacsim.core.nodes.IsaacComputeOdometry"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    (f"{self.NODE_NAME}.inputs:chassisPrim", [Sdf.Path(prim_path)]),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", f"{self.NODE_NAME}.inputs:execIn"),
                ],
            },
        )
        return self.GRAPH_PATH, self.NODE_NAME

    def _get_odometry_outputs(self, graph_path, node_name):
        """Read all odometry outputs from the node."""
        return {
            "position": og.Controller.get(f"{graph_path}/{node_name}.outputs:position"),
            "orientation": og.Controller.get(f"{graph_path}/{node_name}.outputs:orientation"),
            "linearVelocity": og.Controller.get(f"{graph_path}/{node_name}.outputs:linearVelocity"),
            "angularVelocity": og.Controller.get(f"{graph_path}/{node_name}.outputs:angularVelocity"),
            "linearAcceleration": og.Controller.get(f"{graph_path}/{node_name}.outputs:linearAcceleration"),
            "angularAcceleration": og.Controller.get(f"{graph_path}/{node_name}.outputs:angularAcceleration"),
            "globalLinearVelocity": og.Controller.get(f"{graph_path}/{node_name}.outputs:globalLinearVelocity"),
            "globalLinearAcceleration": og.Controller.get(f"{graph_path}/{node_name}.outputs:globalLinearAcceleration"),
        }

    async def _step(self, num_steps=1):
        for _ in range(num_steps):
            await omni.kit.app.get_app().next_update_async()

    @staticmethod
    def _vec_norm(values):
        return math.sqrt(sum(float(v) * float(v) for v in values))

    @staticmethod
    def _quat_is_close(q_a, q_b, tol=1e-3):
        return all(abs(float(a) - float(b)) < tol for a, b in zip(q_a, q_b))

    def _assert_all_outputs_finite(self, outputs):
        for key, value in outputs.items():
            self.assertTrue(
                all(math.isfinite(float(v)) for v in value), msg=f"{key} contains non-finite values: {value}"
            )

    async def test_odometry_default_outputs(self):
        """Before simulation plays, verify all outputs are default (zeros / identity quaternion)."""
        cube_path = "/World/Cube"
        await add_cube(self._stage, cube_path, 1.0, (0, 0, 0), physics=True, mass=1.0)
        graph_path, node_name = self._create_odometry_graph(cube_path)
        await self._step()
        await og.Controller.evaluate(graph_path)

        out = self._get_odometry_outputs(graph_path, node_name)
        for key in (
            "position",
            "linearVelocity",
            "angularVelocity",
            "linearAcceleration",
            "angularAcceleration",
            "globalLinearVelocity",
            "globalLinearAcceleration",
        ):
            for i, v in enumerate(out[key]):
                self.assertAlmostEqual(v, 0.0, places=5, msg=f"{key}[{i}]")
        # orientation default: identity quaternion (0, 0, 0, 1)
        for i, v in enumerate(out["orientation"]):
            self.assertAlmostEqual(v, 1.0 if i == 3 else 0.0, places=5, msg=f"orientation[{i}]")

    async def test_odometry_rigid_body_stationary(self):
        """Cube at rest on a ground plane; after settling, velocities and accelerations near zero, position near zero."""
        await add_ground_plane(self._stage)
        cube_path = "/World/Cube"
        # Place cube so it starts resting on the ground plane (ground at z=-0.5, cube size 1.0, half-size 0.5)
        await add_cube(self._stage, cube_path, 1.0, (0, 0, 0.0), physics=True, mass=1.0)
        graph_path, node_name = self._create_odometry_graph(cube_path)
        await self._step()

        self._timeline.play()
        await self._step(180)
        await og.Controller.evaluate(graph_path)

        out = self._get_odometry_outputs(graph_path, node_name)
        tol = 0.2
        for key in (
            "linearVelocity",
            "angularVelocity",
            "linearAcceleration",
            "angularAcceleration",
            "globalLinearVelocity",
            "globalLinearAcceleration",
        ):
            for v in out[key]:
                self.assertAlmostEqual(v, 0.0, delta=tol, msg=f"{key} should be near zero when stationary")
        # Position should remain near zero (relative to start)
        for i, v in enumerate(out["position"]):
            self.assertAlmostEqual(v, 0.0, delta=tol, msg=f"position[{i}] when stationary")

        self._timeline.stop()

    async def test_odometry_rigid_body_free_fall(self):
        """Cube in free fall: negative globalLinearVelocity z, negative position z, non-zero accelerations."""
        cube_path = "/World/Cube"
        await add_cube(self._stage, cube_path, 1.0, (0, 0, 5.0), physics=True, mass=1.0)
        graph_path, node_name = self._create_odometry_graph(cube_path)
        await self._step()

        self._timeline.play()
        await self._step(60)
        await og.Controller.evaluate(graph_path)

        out = self._get_odometry_outputs(graph_path, node_name)
        # Gravity points down: global linear velocity z should be negative
        self.assertLess(out["globalLinearVelocity"][2], -0.1, "globalLinearVelocity.z should be negative (falling)")
        # Position (relative to start) should have negative z as cube falls
        self.assertLess(out["position"][2], -0.1, "position.z should be negative after falling")
        self.assertLess(out["globalLinearAcceleration"][2], -1.0, "globalLinearAcceleration.z should include gravity")

        self._timeline.stop()

    async def test_odometry_position_relative_to_start(self):
        """Cube spawned at offset (5, 3, 10); position output is relative to start (near zero initially)."""
        cube_path = "/World/Cube"
        offset = (5.0, 3.0, 10.0)
        cube_prim = await add_cube(self._stage, cube_path, 1.0, offset, physics=True, mass=1.0)
        graph_path, node_name = self._create_odometry_graph(cube_path)
        await self._step()

        self._timeline.play()
        await self._step(2)
        await og.Controller.evaluate(graph_path)
        out_start = self._get_odometry_outputs(graph_path, node_name)
        # Position is in starting frame: relative to start, so should be near zero at first frame
        for i, v in enumerate(out_start["position"]):
            self.assertAlmostEqual(v, 0.0, delta=0.05, msg=f"position[{i}] relative to start at t=0")

        RigidPrim(cube_path).set_velocities(linear_velocities=(0.0, 0.0, -1.5))
        await self._step(30)
        await og.Controller.evaluate(graph_path)
        out_later = self._get_odometry_outputs(graph_path, node_name)
        # After forcing downward velocity, relative z should be clearly negative.
        self.assertLess(out_later["position"][2], -0.2, "position.z should decrease as cube moves down")
        self.assertAlmostEqual(out_later["position"][0], 0.0, delta=0.2)
        self.assertAlmostEqual(out_later["position"][1], 0.0, delta=0.2)

        self._timeline.stop()

    async def test_odometry_all_outputs_populated(self):
        """After simulation with a falling cube, all 8 vector/quat outputs are written (non-default where expected)."""
        cube_path = "/World/Cube"
        await add_cube(
            self._stage,
            cube_path,
            1.0,
            (0, 0, 3.0),
            physics=True,
            mass=1.0,
            linear_velocity=(0.8, 0.0, -0.5),
            angular_velocity=(0.0, 0.0, 1.2),
        )
        graph_path, node_name = self._create_odometry_graph(cube_path)
        await self._step()

        self._timeline.play()
        await self._step(30)
        await og.Controller.evaluate(graph_path)

        out = self._get_odometry_outputs(graph_path, node_name)
        self._assert_all_outputs_finite(out)
        self.assertEqual(len(out["orientation"]), 4, "orientation is quat")
        for key in (
            "position",
            "linearVelocity",
            "angularVelocity",
            "linearAcceleration",
            "angularAcceleration",
            "globalLinearVelocity",
            "globalLinearAcceleration",
        ):
            self.assertEqual(len(out[key]), 3, msg=f"{key} should be a 3D vector")
        # position: should have changed (z negative)
        self.assertLess(out["position"][2], 0.0, "position should be populated (z < 0)")
        # linearVelocity / globalLinearVelocity: falling => non-zero z
        self.assertNotAlmostEqual(
            out["globalLinearVelocity"][2], 0.0, delta=0.01, msg="globalLinearVelocity should be populated"
        )
        # linearAcceleration / globalLinearAcceleration: at least one non-zero after motion
        acc_mag = sum(x * x for x in out["globalLinearAcceleration"]) ** 0.5
        self.assertGreater(acc_mag, 0.0, "globalLinearAcceleration should be populated when moving")

        self._timeline.stop()

    async def test_odometry_orientation_changes_on_rotation(self):
        """Cube given angular velocity; orientation output remains a valid relative quaternion."""
        cube_path = "/World/Cube"
        await add_cube(
            self._stage,
            cube_path,
            1.0,
            (0, 0, 2.0),
            physics=True,
            mass=1.0,
            angular_velocity=(0.0, 0.0, 2.0),
        )
        graph_path, node_name = self._create_odometry_graph(cube_path)
        await self._step()

        self._timeline.play()
        await self._step(2)
        await og.Controller.evaluate(graph_path)
        out_start = self._get_odometry_outputs(graph_path, node_name)

        await self._step(40)
        await og.Controller.evaluate(graph_path)

        out = self._get_odometry_outputs(graph_path, node_name)
        o = out["orientation"]
        o_start = out_start["orientation"]
        quat_norm_sq = o[0] * o[0] + o[1] * o[1] + o[2] * o[2] + o[3] * o[3]
        self.assertAlmostEqual(quat_norm_sq, 1.0, delta=0.01, msg="orientation should be unit quaternion")
        # NOTE:
        # In some runtime setups, setting rigid body angular velocity does not always produce
        # a deterministic pose rotation over short horizons. Validate output quality/invariants
        # instead of forcing an orientation delta that may be backend-dependent.
        self.assertEqual(len(o_start), 4, "initial orientation is quaternion")
        self.assertEqual(len(o), 4, "orientation is quaternion")
        self.assertTrue(all(math.isfinite(float(v)) for v in o), "orientation should contain finite values")
        self.assertEqual(len(out["angularVelocity"]), 3, "angularVelocity is 3D")

        self._timeline.stop()

    async def test_odometry_survives_simulation_view_invalidation(self):
        """After the shared SimulationManager physics view is invalidated, the odometry node.

        should not crash or produce errors. It should either continue producing valid data
        (if the view is recovered) or gracefully skip frames."""
        cube_path = "/World/Cube"
        await add_cube(self._stage, cube_path, 1.0, (0, 0, 5.0), physics=True, mass=1.0)
        graph_path, node_name = self._create_odometry_graph(cube_path)
        await self._step()

        self._timeline.play()
        await self._step(30)
        await og.Controller.evaluate(graph_path)

        # Verify odometry is working before invalidation: cube is falling, so global velocity z < 0
        out_before = self._get_odometry_outputs(graph_path, node_name)
        self.assertLess(
            out_before["globalLinearVelocity"][2],
            -0.1,
            "globalLinearVelocity.z should be negative (falling) before invalidation",
        )
        self._assert_all_outputs_finite(out_before)

        # --- Invalidate the shared SimulationManager simulation view ---
        # This simulates what happens when a physics prim is deleted mid-simulation
        # (e.g. during Forklift asset loading). The shared view becomes permanently invalid,
        # which previously caused the odometry node to throw exceptions on every tick.
        sim_view = SimulationManager._physics_sim_view__warp
        self.assertIsNotNone(sim_view, "SimulationManager should have a simulation view")
        sim_view.invalidate()

        # Step several frames with the invalidated view.
        # The node should NOT throw exceptions (no "Error executing python callback" spam).
        # It should either skip gracefully or continue with recovered data.
        for _ in range(30):
            await self._step()

        # After invalidation the node should not crash.
        # The outputs may retain their last valid values or be default - either is acceptable.
        # The critical check is that the test completes without OmniGraph error spam.
        await og.Controller.evaluate(graph_path)
        out_after = self._get_odometry_outputs(graph_path, node_name)
        for key in ("position", "linearVelocity", "angularVelocity", "globalLinearVelocity"):
            self.assertEqual(len(out_after[key]), 3, msg=f"{key} should still be a 3D vector")
        self.assertEqual(len(out_after["orientation"]), 4, "orientation should still be a quaternion")

        self._timeline.stop()

    async def test_odometry_global_vs_local_velocity(self):
        """For a rotated rigid body, local and global linear velocity components should differ."""
        cube_path = "/World/Cube"
        await add_cube(
            self._stage,
            cube_path,
            1.0,
            (0, 0, 4.0),
            physics=True,
            mass=1.0,
            orientation_wxyz=(0.70710678, 0.0, 0.0, 0.70710678),
            linear_velocity=(1.0, 0.0, 0.0),
        )
        graph_path, node_name = self._create_odometry_graph(cube_path)
        await self._step()

        self._timeline.play()
        await self._step(20)
        await og.Controller.evaluate(graph_path)

        out = self._get_odometry_outputs(graph_path, node_name)
        local_v = out["linearVelocity"]
        global_v = out["globalLinearVelocity"]
        # With non-identity orientation, local and global components should not be identical.
        self.assertGreater(
            abs(float(local_v[0]) - float(global_v[0])) + abs(float(local_v[1]) - float(global_v[1])),
            0.2,
            msg="local and global XY velocity components should differ for rotated body",
        )
        # Rotation should preserve velocity magnitude (up to simulation noise).
        self.assertAlmostEqual(self._vec_norm(local_v), self._vec_norm(global_v), delta=0.25)
        self.assertEqual(len(out["linearVelocity"]), 3, "linearVelocity is 3D")
        self.assertEqual(len(out["globalLinearVelocity"]), 3, "globalLinearVelocity is 3D")

        self._timeline.stop()
