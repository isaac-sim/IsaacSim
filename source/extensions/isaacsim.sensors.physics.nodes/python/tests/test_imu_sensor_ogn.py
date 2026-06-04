# SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
import numpy as np
import omni.graph.core as og
import omni.kit.test
import omni.timeline
import usdrt.Sdf
from isaacsim.core.experimental.objects import Cube, GroundPlane
from isaacsim.core.experimental.prims import GeomPrim, RigidPrim
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.sensors.experimental.physics import IMU

from .common import (
    ANGULAR_VEL_TOLERANCE,
    EARTH_GRAVITY,
    ORIENTATION_TOLERANCE,
    SMALL_TOLERANCE,
    step_simulation,
)


class TestIMUSensorOgn(omni.kit.test.AsyncTestCase):
    async def setUp(self):
        await stage_utils.create_new_stage_async()
        physics_rate = 60
        SimulationManager.setup_simulation(dt=1.0 / physics_rate)
        self._timeline = omni.timeline.get_timeline_interface()
        await self.setup_environment()
        await self.setup_ogn()
        self._physics_rate = physics_rate

    async def tearDown(self):
        if self._timeline.is_playing():
            self._timeline.stop()
        SimulationManager.invalidate_physics()
        await omni.kit.app.get_app().next_update_async()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            # print("tearDown, assets still loading, waiting to finish...")
            await asyncio.sleep(1.0)
        await omni.kit.app.get_app().next_update_async()
        pass

    async def setup_environment(self):
        GroundPlane("/World/GroundPlane", positions=[0.0, 0.0, 0.0])
        Cube("/World/Cube", sizes=1.0, positions=[0.0, 0.0, 1.0])
        GeomPrim("/World/Cube", apply_collision_apis=True)
        RigidPrim("/World/Cube", masses=[1.0])

        IMU.create(
            "/World/Cube/imu_sensor",
        )
        prim_utils.get_prim_at_path("/World/Cube/imu_sensor").GetAttribute("linearAccelerationFilterWidth").Set(10)
        pass

    async def setup_ogn(self):
        self.graph_path = "/TestGraph"
        self.prim_path = "/World/Cube/imu_sensor"

        if prim_utils.get_prim_at_path(self.graph_path).IsValid():
            stage_utils.delete_prim(self.graph_path)

        keys = og.Controller.Keys
        try:
            og.Controller.edit(
                {"graph_path": self.graph_path, "evaluator_name": "execution"},
                {
                    keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("ReadIMUNode", "isaacsim.sensors.physics.IsaacReadIMU"),
                    ],
                    keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "ReadIMUNode.inputs:execIn"),
                    ],
                },
            )
        except Exception as e:
            print(e)

    # verifying linear acceleration values are approximately equal to  zero for x and y and 9.81 for z in valid case
    # verifying sensor time is non-zero for the valid case
    async def test_valid_imu_sensor_ogn(self):
        og.Controller.set(
            og.Controller.attribute(self.graph_path + "/ReadIMUNode.inputs:imuPrim"),
            [usdrt.Sdf.Path("/World/Cube/imu_sensor")],
        )

        self._timeline.play()
        await step_simulation(1.5)
        lin_acc = og.Controller.attribute(self.graph_path + "/ReadIMUNode.outputs:linAcc").get()
        self.assertAlmostEqual(lin_acc[2], EARTH_GRAVITY, delta=SMALL_TOLERANCE)
        self.assertAlmostEqual(lin_acc[0], 0.0, delta=SMALL_TOLERANCE)
        self.assertAlmostEqual(lin_acc[1], 0.0, delta=SMALL_TOLERANCE)

        ang_vel = og.Controller.attribute(self.graph_path + "/ReadIMUNode.outputs:angVel").get()
        self.assertAlmostEqual(ang_vel[0], 0.0, delta=ANGULAR_VEL_TOLERANCE)
        self.assertAlmostEqual(ang_vel[1], 0.0, delta=ANGULAR_VEL_TOLERANCE)
        self.assertAlmostEqual(ang_vel[2], 0.0, delta=ANGULAR_VEL_TOLERANCE)

        orientation = og.Controller.attribute(self.graph_path + "/ReadIMUNode.outputs:orientation").get()
        orientation_norm = float(np.linalg.norm(np.array(orientation)))
        self.assertAlmostEqual(orientation_norm, 1.0, delta=ORIENTATION_TOLERANCE)

        sensor_time = og.Controller.attribute(self.graph_path + "/ReadIMUNode.outputs:sensorTime").get()
        self.assertNotEqual(sensor_time, 0.0)

    async def test_read_gravity_toggle_imu_sensor_ogn(self):
        og.Controller.set(
            og.Controller.attribute(self.graph_path + "/ReadIMUNode.inputs:imuPrim"),
            [usdrt.Sdf.Path("/World/Cube/imu_sensor")],
        )
        og.Controller.set(
            og.Controller.attribute(self.graph_path + "/ReadIMUNode.inputs:readGravity"),
            False,
        )

        self._timeline.play()
        await step_simulation(1.0)
        lin_acc = og.Controller.attribute(self.graph_path + "/ReadIMUNode.outputs:linAcc").get()
        self.assertAlmostEqual(lin_acc[2], 0.0, delta=SMALL_TOLERANCE)

    async def test_use_latest_data_imu_sensor_ogn(self):
        og.Controller.set(
            og.Controller.attribute(self.graph_path + "/ReadIMUNode.inputs:imuPrim"),
            [usdrt.Sdf.Path("/World/Cube/imu_sensor")],
        )
        og.Controller.set(
            og.Controller.attribute(self.graph_path + "/ReadIMUNode.inputs:useLatestData"),
            True,
        )

        self._timeline.play()
        await step_simulation(1.0)
        sensor_time = og.Controller.attribute(self.graph_path + "/ReadIMUNode.outputs:sensorTime").get()
        self.assertNotEqual(sensor_time, 0.0)

    async def test_imu_sensor_stop_play_lifecycle(self):
        """IMU outputs recover valid data after a stop/play cycle."""
        og.Controller.set(
            og.Controller.attribute(self.graph_path + "/ReadIMUNode.inputs:imuPrim"),
            [usdrt.Sdf.Path("/World/Cube/imu_sensor")],
        )

        self._timeline.play()
        await step_simulation(1.5)

        lin_acc = og.Controller.attribute(self.graph_path + "/ReadIMUNode.outputs:linAcc").get()
        self.assertAlmostEqual(lin_acc[2], EARTH_GRAVITY, delta=SMALL_TOLERANCE)

        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()

        self._timeline.play()
        await step_simulation(1.5)

        lin_acc_after = og.Controller.attribute(self.graph_path + "/ReadIMUNode.outputs:linAcc").get()
        self.assertAlmostEqual(
            lin_acc_after[2],
            EARTH_GRAVITY,
            delta=SMALL_TOLERANCE,
            msg="Should recover valid gravity reading after stop/play",
        )

        sensor_time_after = og.Controller.attribute(self.graph_path + "/ReadIMUNode.outputs:sensorTime").get()
        self.assertNotEqual(sensor_time_after, 0.0, "Should have non-zero time after stop/play")

    async def test_invalid_imu_sensor_ogn(self):
        og.Controller.set(
            og.Controller.attribute(self.graph_path + "/ReadIMUNode.inputs:imuPrim"),
            [usdrt.Sdf.Path("/World/Cube")],
        )
        self._timeline.play()
        await step_simulation(0.5)

        lin_acc = og.Controller.attribute(self.graph_path + "/ReadIMUNode.outputs:linAcc").get()
        self.assertEqual(lin_acc[2], 0.0)
        self.assertEqual(lin_acc[0], 0.0)
        self.assertEqual(lin_acc[1], 0.0)

        ang_vel = og.Controller.attribute(self.graph_path + "/ReadIMUNode.outputs:angVel").get()
        self.assertEqual(ang_vel[0], 0.0)
        self.assertEqual(ang_vel[1], 0.0)
        self.assertEqual(ang_vel[2], 0.0)

        orientation = og.Controller.attribute(self.graph_path + "/ReadIMUNode.outputs:orientation").get()
        self.assertEqual(orientation[0], 0.0)
        self.assertEqual(orientation[1], 0.0)
        self.assertEqual(orientation[2], 0.0)
        self.assertEqual(orientation[3], 1.0)

        sensor_time = og.Controller.attribute(self.graph_path + "/ReadIMUNode.outputs:sensorTime").get()
        self.assertAlmostEqual(sensor_time, 0.0)
