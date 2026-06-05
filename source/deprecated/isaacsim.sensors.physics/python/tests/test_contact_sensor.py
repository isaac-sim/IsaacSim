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

"""Test contact sensor functionality."""

import asyncio
from typing import Any

import carb.tokens
import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
import omni
import omni.kit.commands

# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add suport for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
import omni.kit.test
import omni.timeline
from isaacsim.core.experimental.objects import Cube, GroundPlane
from isaacsim.core.experimental.prims import GeomPrim, RigidPrim, XformPrim
from isaacsim.core.experimental.utils.stage import add_reference_to_stage
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.sensors.physics import ContactSensor, _sensor
from isaacsim.storage.native import get_assets_root_path_async
from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics

from .common import reset_timeline, setup_ant_scene, step_simulation

# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test


async def add_cube(stage: Any, path: Any, size: Any, offset: Any, physics: bool = True, mass: float = 0.0) -> Usd.Prim:
    """Add a cube prim to the stage.

    Args:
        stage: The USD stage to add the cube to.
        path: USD path for the cube prim.
        size: Size of the cube.
        offset: Translation offset for the cube.
        physics: Whether to apply rigid body physics to the cube.
        mass: Mass of the cube in kg. Only used when physics is True and mass > 0.

    Returns:
        The created USD prim for the cube.
    """
    cube_geom = UsdGeom.Cube.Define(stage, path)
    cube_prim = stage.GetPrimAtPath(path)
    cube_geom.CreateSizeAttr(size)
    cube_geom.AddTranslateOp().Set(offset)
    await omni.kit.app.get_app().next_update_async()  # Need this to avoid flatcache errors
    if physics:
        rigid_api = UsdPhysics.RigidBodyAPI.Apply(cube_prim)
        await omni.kit.app.get_app().next_update_async()
        rigid_api.CreateRigidBodyEnabledAttr(True)
        await omni.kit.app.get_app().next_update_async()
        if mass > 0:
            mass_api = UsdPhysics.MassAPI.Apply(cube_prim)
            await omni.kit.app.get_app().next_update_async()
            mass_api.CreateMassAttr(mass)
            await omni.kit.app.get_app().next_update_async()
    UsdPhysics.CollisionAPI.Apply(cube_prim)
    await omni.kit.app.get_app().next_update_async()
    return cube_prim


class TestContactSensor(omni.kit.test.AsyncTestCase):
    """Test contact sensor."""

    # Before running each test
    async def setUp(self) -> None:
        """Set up test fixtures."""
        # This needs to be set so that kit updates match physics updates
        self._physics_rate = 60
        self._cs = _sensor.acquire_contact_sensor_interface()
        self._timeline = omni.timeline.get_timeline_interface()
        self._ant_config = None

        self._assets_root_path = await get_assets_root_path_async()
        if self._assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return

    async def _setup_ant(self) -> None:
        """Load the ant scene and configure ant-specific test data."""
        self._ant_config = await setup_ant_scene(self._physics_rate)
        self._stage = stage_utils.get_current_stage()

    # Convenience properties for ant configuration
    @property
    def leg_paths(self) -> list:
        """Return leg paths."""
        return self._ant_config.leg_paths

    @property
    def sensor_offsets(self) -> list:
        """Return sensor offsets."""
        return self._ant_config.sensor_offsets

    @property
    def color(self) -> list:
        """Return color values."""
        return self._ant_config.colors

    # After running each test
    async def tearDown(self) -> None:
        """Tear down test fixtures."""
        await omni.kit.app.get_app().next_update_async()
        if self._timeline.is_playing():
            self._timeline.stop()
        SimulationManager.invalidate_physics()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            carb.log_warn("tearDown, assets still loading, waiting to finish...")
            await asyncio.sleep(1.0)
        await omni.kit.app.get_app().next_update_async()

    async def _add_sensor_prims(self) -> None:
        """Helper to add contact sensors to ant legs. Requires ant to be loaded."""
        self.sensorGeoms = []
        for i in range(4):
            result, sensor = omni.kit.commands.execute(
                "IsaacSensorCreateContactSensor",
                path="/sensor",
                parent=self.leg_paths[i],
                min_threshold=0,
                max_threshold=10000000,
                color=self.color[i],
                radius=0.12,
                sensor_period=-1,
                translation=self.sensor_offsets[i],
            )
            self.sensorGeoms.append(sensor)
            self.assertTrue(result)
            self.assertIsNotNone(sensor)

    async def test_add_sensor_prim(self) -> None:
        """Ensure contact sensors can be created on ant legs."""
        await self._setup_ant()
        await self._add_sensor_prims()

    # test plan:
    # move the ground to -10, simulate 10 steps, test for no contact
    # move the ground to -0.78, simulate 60 steps, test for contact
    # test raw contact value, z-normal ~ 1.0
    # move teh ground to -15, simulate 30 steps, test for no contact
    async def test_lost_contacts(self) -> None:
        """Validate contact detection when ground moves in/out of reach."""
        await self._setup_ant()
        await self._add_sensor_prims()
        xform = UsdGeom.Xformable(self._stage.GetPrimAtPath("/World/GroundPlane"))
        xform_op = xform.GetOrderedXformOps()[0]
        xform_op.Set(Gf.Vec3d(0, 0, -10))

        await omni.kit.app.get_app().next_update_async()
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()  # ant should be in the air and have no contact
        contacts_raw = self._cs.get_contact_sensor_raw_data(self.leg_paths[0] + "/sensor")
        self.assertEqual(len(contacts_raw), 0)

        xform = UsdGeom.Xformable(self._stage.GetPrimAtPath("/World/GroundPlane"))
        xform_op = xform.GetOrderedXformOps()[0]
        xform_op.Set(Gf.Vec3d(0, 0, -0.78))
        await step_simulation(1)  # simulate 60 steps, ant should touch ground
        contacts_raw = self._cs.get_contact_sensor_raw_data(self.leg_paths[0] + "/sensor")
        self.assertEqual(len(contacts_raw), 1)

        c = contacts_raw[0]
        body0 = self._cs.decode_body_name(c["body0"])
        body1 = self._cs.decode_body_name(c["body1"])
        if self.leg_paths[0] not in [body0, body1]:
            self.fail(f"Raw contact does not contain queried body {self.leg_paths[0]} ({body0},{body1})")
        self.assertAlmostEqual(1.0, c["normal"]["z"], delta=0.1)
        # # print(c)

        # move the ground to lose the contacts
        xform = UsdGeom.Xformable(self._stage.GetPrimAtPath("/World/GroundPlane"))
        xform_op = xform.GetOrderedXformOps()[0]
        xform_op.Set(Gf.Vec3d(0, 0, -15))
        await step_simulation(0.5)
        contacts_raw = self._cs.get_contact_sensor_raw_data(self.leg_paths[0] + "/sensor")
        self.assertEqual(len(contacts_raw), 0)

    async def test_get_body_raw_data(self) -> None:
        """Test raw contact data retrieval between rigid bodies without using ant."""
        await stage_utils.create_new_stage_async()
        stage_utils.set_stage_units(meters_per_unit=1.0)
        SimulationManager.setup_simulation(dt=1.0 / self._physics_rate)

        # Create ground plane for the blocks to land on
        GroundPlane("/World/GroundPlane", sizes=10.0)

        block_0_prim = add_reference_to_stage(
            usd_path=self._assets_root_path + "/Isaac/Props/Blocks/basic_block.usd", path="/World/block_0"
        )
        block_0 = RigidPrim(
            "/World/block_0/Cube", positions=[10, 0, 5.0], scales=np.ones(3) * 1.0, reset_xform_op_properties=True
        )
        PhysxSchema.PhysxContactReportAPI.Apply(block_0.prims[0])

        block_1_prim = add_reference_to_stage(
            usd_path=self._assets_root_path + "/Isaac/Props/Blocks/basic_block.usd", path="/World/block_1"
        )
        block_1 = RigidPrim(
            "/World/block_1/Cube", positions=[10, 0, 10.0], scales=np.ones(3) * 1.0, reset_xform_op_properties=True
        )
        PhysxSchema.PhysxContactReportAPI.Apply(block_1.prims[0])

        def block_1_is_contacting_block_0() -> bool:
            raw_data = self._cs.get_rigid_body_raw_data(block_1.paths[0])
            in_contact = False
            for c in raw_data:
                if block_0.paths[0] in {self._cs.decode_body_name(c["body0"]), self._cs.decode_body_name(c["body1"])}:
                    in_contact = True
                    break

            return in_contact

        await omni.kit.app.get_app().next_update_async()
        self._timeline.play()

        count = 0

        while not block_1_is_contacting_block_0() and count < 500:
            count += 1
            await omni.kit.app.get_app().next_update_async()

        self.assertTrue(count < 500)

    async def test_get_raw_data(self) -> None:
        """Validate raw contact data from a sensor when the ant contacts ground."""
        await self._setup_ant()
        await self._add_sensor_prims()
        await omni.kit.app.get_app().next_update_async()
        self._timeline.play()
        await step_simulation(1)  # simulate 60 steps, ant should touch ground
        contacts_raw = self._cs.get_contact_sensor_raw_data(self.leg_paths[0] + "/sensor")
        self.assertEqual(len(contacts_raw), 1)

        c = contacts_raw[0]
        body0 = self._cs.decode_body_name(c["body0"])
        body1 = self._cs.decode_body_name(c["body1"])
        if self.leg_paths[0] not in [body0, body1]:
            self.fail(f"Raw contact does not contain queried body {self.leg_paths[0]} ({body0},{body1})")
        self.assertAlmostEqual(1.0, c["normal"]["z"], delta=0.1)
        ## print(c)

    async def test_persistent_raw_data(self) -> None:
        """Ensure raw contact data remains available during persistent contacts."""
        await self._setup_ant()
        await self._add_sensor_prims()
        self._timeline.play()
        await step_simulation(2.0)  # simulate long enough that physx stops sending persistent contact raw data
        contacts_raw = self._cs.get_contact_sensor_raw_data(self.leg_paths[0] + "/sensor")
        self.assertEqual(len(contacts_raw), 1)

        c = contacts_raw[0]
        body0 = self._cs.decode_body_name(c["body0"])
        body1 = self._cs.decode_body_name(c["body1"])
        if self.leg_paths[0] not in [body0, body1]:
            self.fail(f"Raw contact does not contain queried body {self.leg_paths[0]} ({body0},{body1})")
        self.assertAlmostEqual(1.0, c["normal"]["z"], delta=6)
        ## print(c)

    async def test_delayed_get_sensor_reading(self) -> None:
        """Compare delayed sensor readings against raw impulse-derived force."""
        await self._setup_ant()
        await self._add_sensor_prims()
        await omni.kit.app.get_app().next_update_async()
        self._timeline.play()
        await step_simulation(1.0)

        for i in range(120):
            await omni.kit.app.get_app().next_update_async()

        contacts_raw = self._cs.get_contact_sensor_raw_data(self.leg_paths[0] + "/sensor")
        sensor_reading = self._cs.get_sensor_reading(self.leg_paths[0] + "/sensor")
        self.assertTrue(sensor_reading.is_valid)

        if len(contacts_raw):
            # there is a contact, compute force from impulse, compare to sensor reading
            force = (
                np.linalg.norm(
                    [contacts_raw[0]["impulse"]["x"], contacts_raw[0]["impulse"]["y"], contacts_raw[0]["impulse"]["z"]]
                )
                * 60.0
            )  # dt is 1/60
            self.assertAlmostEqual(force, sensor_reading.value, 2)
        else:
            # No contact, reading should be zero
            self.assertEqual(sensor_reading.value, 0)

    async def test_contact_outside_range(self) -> None:
        """Ensure sensors out of contact range report zero readings."""
        await self._setup_ant()
        self.sensorGeoms = []

        # create 4 sensors at the center of the leg
        for i in range(4):
            result, sensor = omni.kit.commands.execute(
                "IsaacSensorCreateContactSensor",
                path="/sensor",
                parent=self.leg_paths[i],
                min_threshold=0,
                max_threshold=10000000,
                color=self.color[i],
                radius=0.12,
                sensor_period=-1,
                translation=Gf.Vec3f(0, 0, 0),
            )
            self.sensorGeoms.append(sensor)
            self.assertTrue(result)
            self.assertIsNotNone(sensor)

        await omni.kit.app.get_app().next_update_async()
        self._timeline.play()
        await step_simulation(1)

        for i in range(40):
            await omni.kit.app.get_app().next_update_async()
            sensor_reading = self._cs.get_sensor_reading(self.leg_paths[0] + "/sensor")
            self.assertTrue(sensor_reading.is_valid)
            self.assertEqual(sensor_reading.value, 0)

    async def test_sensor_period(self) -> None:
        """Verify sensor outputs update at the configured frequency."""
        await self._setup_ant()
        # create four sensors that run at 30hz
        for i in range(4):
            result, sensor = omni.kit.commands.execute(
                "IsaacSensorCreateContactSensor",
                path="/sensor",
                parent=self.leg_paths[i],
                min_threshold=0,
                max_threshold=10000000,
                color=self.color[i],
                radius=0.12,
                sensor_period=1.0 / 30.0,
                translation=self.sensor_offsets[i],
            )
            self.assertTrue(result)
            self.assertIsNotNone(sensor)

        await omni.kit.app.get_app().next_update_async()
        self._timeline.play()
        # give it some time to reach the ground first
        await step_simulation(1.5)
        await omni.kit.app.get_app().next_update_async()
        readings = []

        for i in range(60):  # Simulate for one second
            await omni.kit.app.get_app().next_update_async()
            raw = self._cs.get_contact_sensor_raw_data(self.leg_paths[0] + "/sensor")
            # print(str(raw))
            sensor_reading = self._cs.get_sensor_reading(self.leg_paths[0] + "/sensor")
            # print(str(sensor_reading))

            # the sensor is running at 30hz, while the sim is 60hz, so expecting every other reading to be new,
            # old reading should be identical, and have the same timestamp
            if not readings or readings[-1].time != sensor_reading.time:
                readings.append(sensor_reading)

        # tolerance +-1 reading (29, 30, 31 will be accepted)
        # print(len(readings))
        self.assertTrue(abs(len(readings) - 30) <= 1)

    async def test_stop_start(self) -> None:
        """Verify sensor readings are consistent across timeline stop/start."""
        await self._setup_ant()
        await self._add_sensor_prims()

        await omni.kit.app.get_app().next_update_async()

        self._timeline.play()
        await step_simulation(0.5)

        init_reading = self._cs.get_sensor_reading(self.leg_paths[0] + "/sensor")

        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()

        self._timeline.play()
        await step_simulation(0.5)

        sensor_reading = self._cs.get_sensor_reading(self.leg_paths[0] + "/sensor")

        self.assertEqual(init_reading.in_contact, sensor_reading.in_contact)
        self.assertEqual(init_reading.value, sensor_reading.value)
        self.assertEqual(init_reading.time, sensor_reading.time)

    async def test_sensor_latest_data(self) -> None:
        """Ensure latest-data reads return monotonically increasing times."""
        await self._setup_ant()
        # create four sensors that run at 30hz
        for i in range(4):
            result, sensor = omni.kit.commands.execute(
                "IsaacSensorCreateContactSensor",
                path="/custom_sensor",
                parent=self.leg_paths[i],
                min_threshold=0,
                max_threshold=10000000,
                color=self.color[i],
                radius=0.12,
                sensor_period=1.0 / 30.0,
                translation=self.sensor_offsets[i],
            )
            self.assertTrue(result)
            self.assertIsNotNone(sensor)

        await omni.kit.app.get_app().next_update_async()
        self._timeline.play()
        # give it some time to reach the ground first
        await omni.kit.app.get_app().next_update_async()
        old_time = -1

        for i in range(10):  # Simulate for 10 steps
            await omni.kit.app.get_app().next_update_async()
            latest_sensor_reading = self._cs.get_sensor_reading(self.leg_paths[0] + "/custom_sensor", True)

            self.assertTrue(latest_sensor_reading.time > old_time)
            old_time = latest_sensor_reading.time

    async def test_wrong_sensor_path(self) -> None:
        """Ensure invalid sensor paths return invalid readings."""
        await self._setup_ant()
        await self._add_sensor_prims()
        await omni.kit.app.get_app().next_update_async()
        self._timeline.play()
        # give it some time to reach the ground first
        await omni.kit.app.get_app().next_update_async()
        for i in range(10):  # Simulate for 10 steps
            await omni.kit.app.get_app().next_update_async()
            latest_sensor_reading = self._cs.get_sensor_reading(self.leg_paths[0] + "/wrong_sensor", True)

            self.assertFalse(latest_sensor_reading.is_valid)
            self.assertEqual(latest_sensor_reading.time, 0)

    async def test_sensor_threshold(self) -> None:
        """Verify min/max thresholds gate contact readings as expected."""
        await self._setup_ant()
        await self._add_sensor_prims()
        await omni.kit.app.get_app().next_update_async()

        # note, expected reading is around 0.57
        min_threshold = [0.0, 50.0, 0.0, 0.0]
        max_threshold = [100.0, 100.0, 100.0, 0.1]

        # create four sensors with custom thresholds
        for i in range(4):
            result, sensor = omni.kit.commands.execute(
                "IsaacSensorCreateContactSensor",
                path="/custom_sensor",
                parent=self.leg_paths[i],
                min_threshold=min_threshold[i],
                max_threshold=max_threshold[i],
                color=self.color[i],
                radius=0.12,
                sensor_period=0,
                translation=self.sensor_offsets[i],
            )
            self.assertTrue(result)
            self.assertIsNotNone(sensor)

        await omni.kit.app.get_app().next_update_async()
        self._timeline.play()
        # give it some time to reach the ground first
        await step_simulation(1.0)

        sensor_0 = self._cs.get_sensor_reading(self.leg_paths[0] + "/custom_sensor")  # expect contact
        sensor_1 = self._cs.get_sensor_reading(self.leg_paths[1] + "/custom_sensor")  # expect no contact
        sensor_2 = self._cs.get_sensor_reading(self.leg_paths[2] + "/custom_sensor")  # expect proper reading
        sensor_3 = self._cs.get_sensor_reading(self.leg_paths[3] + "/custom_sensor")  # expect 0.1

        # print(f"sensor 0: {float(sensor_0.value)}")  # val
        # print(f"sensor 1: {float(sensor_1.value)}")  # no val
        # print(f"sensor 2: {float(sensor_2.value)}")  # val
        # print(f"sensor 3: {float(sensor_3.value)}")  # val
        self.assertTrue(sensor_0.inContact)
        self.assertFalse(sensor_1.inContact)
        self.assertGreater(float(sensor_2.value), 0.1)
        self.assertAlmostEqual(float(sensor_3.value), 0.1, delta=1e-5)

    async def test_sensor_with_skip_parents(self) -> None:
        """Verify sensors work when inserted under intermediate Xform prims."""
        await self._setup_ant()
        await self._add_sensor_prims()
        await omni.kit.app.get_app().next_update_async()

        # create four sensors with in an Xform. The offset needed to reach the tip of the leg is (40,0,0)
        # Break this down to (20,0,0) to the xform, (20,0,0) to the sensor.
        for i in range(4):
            xform_path = self.leg_paths[i] + "/xform"
            stage_utils.define_prim(xform_path, "Xform")
            xform = XformPrim(xform_path, translations=[20, 0, 0], reset_xform_op_properties=True)

            result, sensor = omni.kit.commands.execute(
                "IsaacSensorCreateContactSensor",
                path="/xform/custom_sensor",
                parent=self.leg_paths[i],
                min_threshold=0.0,
                max_threshold=100.0,
                color=self.color[i],
                radius=0.12,
                sensor_period=0,
                translation=Gf.Vec3d(20, 0, 0),
            )
            self.assertTrue(result)
            self.assertIsNotNone(sensor)

        await omni.kit.app.get_app().next_update_async()
        self._timeline.play()
        # give it some time to reach the ground first
        await step_simulation(1.0)

        # all four sensors should have proper reading
        for i in range(4):
            sensor_reading = self._cs.get_sensor_reading(self.leg_paths[i] + "/xform/custom_sensor")  # expect contact
            self.assertTrue(sensor_reading.inContact)
            self.assertGreater(float(sensor_reading.value), 0.1)

    async def test_stacked_cubes_contact_forces(self) -> None:
        """Test contact sensor force calculations with stacked cubes in equilibrium."""
        await stage_utils.create_new_stage_async()
        stage_utils.set_stage_units(meters_per_unit=1.0)
        SimulationManager.setup_simulation(dt=1.0 / 60.0)
        physics_scene = SimulationManager.get_physics_scenes()[0]
        GroundPlane("/World/groundPlane", sizes=10.0, colors=[0.5, 0.5, 0.5])
        physics_scene.set_gravity(Gf.Vec3f(0.0, 0.0, -10.0))

        # Create stacked cubes
        Cube(
            "/World/Cube",
            sizes=1.0,
            positions=[-20.0, -0.2, 0.25],
            scales=[0.5, 0.5, 0.5],
        )
        GeomPrim("/World/Cube", apply_collision_apis=True)
        RigidPrim("/World/Cube", masses=[1.0])

        Cube(
            "/World/Cube2",
            sizes=1.0,
            positions=[-20.0, -0.2, 0.75],
            scales=[0.5, 0.5, 0.5],
        )
        GeomPrim("/World/Cube2", apply_collision_apis=True)
        RigidPrim("/World/Cube2", masses=[1.0])

        # Create contact sensors
        bottom_sensor = ContactSensor(
            prim_path="/World/Cube/Contact_Sensor",
            name="BottomContactSensor",
            frequency=60,
            translation=np.array([0, 0, 0]),
            min_threshold=0,
            max_threshold=100,
            radius=-1,
        )

        top_sensor = ContactSensor(
            prim_path="/World/Cube2/Contact_Sensor",
            name="TopContactSensor",
            frequency=60,
            translation=np.array([0, 0, 0]),
            min_threshold=0,
            max_threshold=100,
            radius=-1,
        )

        # Initialize timeline
        await reset_timeline(self._timeline)

        # Allow settling time for cubes to come into contact
        await step_simulation(10 * SimulationManager.get_physics_dt())

        # Get latest contact data
        bottom_data = bottom_sensor.get_current_frame()
        top_data = top_sensor.get_current_frame()

        bottom_force = bottom_data.get("force", 0)
        top_force = top_data.get("force", 0)
        bottom_contacts = bottom_data.get("number_of_contacts", 0)
        top_contacts = top_data.get("number_of_contacts", 0)

        # Validate that contacts are detected
        self.assertTrue(bottom_data.get("in_contact", False), "Bottom cube should be in contact")
        self.assertTrue(top_data.get("in_contact", False), "Top cube should be in contact")

        # Validate contact counts
        self.assertGreater(bottom_contacts, 1, f"Bottom cube should have multiple contacts (got {bottom_contacts})")
        self.assertGreaterEqual(top_contacts, 1, f"Top cube should have at least 1 contact (got {top_contacts})")

        # Validate equilibrium physics: contact forces should be approximately m*g for each cube
        cube_mass = 1.0  # kg
        gravity = 10.0  # m/s^2 (magnitude)
        expected_force = cube_mass * gravity  # 10.0 N
        tolerance = 0.05

        # Each cube should have contact force approximately equal to its weight
        self.assertAlmostEqual(
            bottom_force,
            expected_force,
            delta=expected_force * tolerance,
            msg=f"Bottom cube force {bottom_force:.3f} should be approximately {expected_force:.3f} N (m*g)",
        )
        self.assertAlmostEqual(
            top_force,
            expected_force,
            delta=expected_force * tolerance,
            msg=f"Top cube force {top_force:.3f} should be approximately {expected_force:.3f} N (m*g)",
        )

    async def test_is_contact_sensor(self) -> None:
        """Test is_contact_sensor returns correct values for valid sensor, invalid prim, and non-existent prim."""
        await stage_utils.create_new_stage_async()
        SimulationManager.setup_simulation(dt=1.0 / 60.0)

        # Create a cube with rigid body
        Cube("/World/Cube", sizes=1.0, positions=[0.0, 0.0, 1.0])
        GeomPrim("/World/Cube", apply_collision_apis=True)
        RigidPrim("/World/Cube", masses=[1.0])
        await omni.kit.app.get_app().next_update_async()

        # Create a contact sensor on the cube
        result, sensor = omni.kit.commands.execute(
            "IsaacSensorCreateContactSensor",
            path="/contact_sensor",
            parent="/World/Cube",
            min_threshold=0,
            max_threshold=10000000,
            radius=0.12,
            sensor_period=-1,
            translation=Gf.Vec3d(0, 0, 0),
        )
        self.assertTrue(result)
        self.assertIsNotNone(sensor)
        await omni.kit.app.get_app().next_update_async()

        # Before simulation starts, sensor should not be registered with the manager
        self.assertFalse(self._cs.is_contact_sensor("/World/Cube/contact_sensor"))

        # Start the timeline to register sensors with the manager
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()

        # After simulation starts, valid contact sensor path should return True
        self.assertTrue(self._cs.is_contact_sensor("/World/Cube/contact_sensor"))

        # Test invalid prim (cube itself, not a sensor) - should return False
        self.assertFalse(self._cs.is_contact_sensor("/World/Cube"))

        # Test non-existent prim path - should return False
        self.assertFalse(self._cs.is_contact_sensor("/World/NonExistent/sensor"))

    async def test_raw_data_all_fields(self) -> None:
        """Validate CsRawData fields include dt and position."""
        await self._setup_ant()
        await self._add_sensor_prims()
        await omni.kit.app.get_app().next_update_async()
        self._timeline.play()
        await step_simulation(1.0)

        contacts_raw = self._cs.get_contact_sensor_raw_data(self.leg_paths[0] + "/sensor")
        self.assertGreater(len(contacts_raw), 0)
        c = contacts_raw[0]

        self.assertGreater(float(c["dt"]), 0.0)
        self.assertLessEqual(float(c["dt"]), SimulationManager.get_physics_dt() * 1.1)

        position = c["position"]
        for axis in ("x", "y", "z"):
            self.assertTrue(np.isfinite(position[axis]))

    async def test_cs_sensor_reading_defaults(self) -> None:
        """Verify CsSensorReading default construction values."""
        reading = _sensor.CsSensorReading()
        self.assertEqual(reading.time, 0.0)
        self.assertEqual(reading.value, 0.0)
        self.assertFalse(reading.inContact)
        self.assertFalse(reading.in_contact)
        self.assertFalse(reading.is_valid)
