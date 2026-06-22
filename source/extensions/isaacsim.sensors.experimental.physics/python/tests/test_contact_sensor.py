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

"""Verifies contact sensor authoring and runtime data, including contact readings, raw and persistent contact data, thresholds, range filtering, timeline lifecycle, deleted prims, stacked-cube forces, and default reading fields."""

import asyncio
from typing import Any

import carb.tokens
import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
import omni

# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add suport for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
import omni.kit.test
import omni.timeline
from isaacsim.core.experimental.objects import Cube, GroundPlane
from isaacsim.core.experimental.prims import GeomPrim, RigidPrim, XformPrim
from isaacsim.core.experimental.utils.stage import add_reference_to_stage
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.sensors.experimental.physics import Contact, ContactSensor, ContactSensorReading
from isaacsim.storage.native import get_assets_root_path_async
from pxr import Gf, PhysicsSchemaTools, Usd, UsdGeom, UsdPhysics

from .common import reset_timeline, setup_ant_scene, step_simulation

# Having a test class dervived from omni.kit.test.AsyncTestCase declared on the root of module will make it auto-discoverable by omni.kit.test


async def add_cube(stage: Any, path: Any, size: Any, offset: Any, physics: Any = True, mass: Any = 0.0) -> Usd.Prim:
    """Add a cube prim to the stage.

    Args:
        stage: USD stage to author into.
        path: USD path for the cube prim.
        size: Cube size value.
        offset: Translation to apply to the cube.
        physics: Whether to apply rigid-body physics.
        mass: Mass to author when greater than zero.

    Returns:
        The created cube prim.
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
        """Initialize contact sensor caches, timeline access, physics rate, and asset root."""
        # This needs to be set so that kit updates match physics updates
        self._physics_rate = 60
        self._contact_sensors: dict[str, ContactSensor] = {}
        self._timeline = omni.timeline.get_timeline_interface()
        self._ant_config = None

        self._assets_root_path = await get_assets_root_path_async()
        if self._assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            return

    def _get_contact_sensor(self, prim_path: str) -> ContactSensor:
        """Get or create a cached ContactSensor for the given path.

        Args:
            prim_path: USD path to the contact sensor prim.

        Returns:
            Cached contact sensor for ``prim_path``.
        """
        if prim_path not in self._contact_sensors:
            self._contact_sensors[prim_path] = ContactSensor(prim_path)
        return self._contact_sensors[prim_path]

    async def _setup_ant(self) -> None:
        """Load the ant scene and configure ant-specific test data."""
        self._ant_config = await setup_ant_scene(self._physics_rate)
        self._stage = stage_utils.get_current_stage()

    # Convenience properties for ant configuration
    @property
    def leg_paths(self) -> Any:
        """Return leg paths."""
        return self._ant_config.leg_paths

    @property
    def sensor_offsets(self) -> Any:
        """Return sensor offsets."""
        return self._ant_config.sensor_offsets

    @property
    def color(self) -> Any:
        """Return color values."""
        return self._ant_config.colors

    # After running each test
    async def tearDown(self) -> None:
        """Stop playback, clear physics state, and wait for pending stage loads."""
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
        self.sensor_geoms = []
        for i in range(4):
            sensor = ContactSensor(
                Contact.create(
                    self.leg_paths[i] + "/sensor",
                    min_threshold=0,
                    max_threshold=10000000,
                    color=self.color[i],
                    radius=0.12,
                    translations=self.sensor_offsets[i],
                )
            )
            self.sensor_geoms.append(sensor)
            self.assertIsNotNone(sensor)

    async def test_add_sensor_prim(self) -> None:
        """Ensure contact sensors can be created on ant legs."""
        await self._setup_ant()
        await self._add_sensor_prims()

    async def test_physics_only_step_outputs_contact_data(self) -> None:
        """ContactSensor produces data when stepping physics without app/render updates."""
        await stage_utils.create_new_stage_async()
        stage_utils.set_stage_units(meters_per_unit=1.0)
        SimulationManager.setup_simulation(dt=1.0 / self._physics_rate)

        GroundPlane("/World/GroundPlane", sizes=10.0)
        Cube("/World/Cube", sizes=1.0, positions=[0.0, 0.0, 0.5])
        GeomPrim("/World/Cube", apply_collision_apis=True)
        RigidPrim("/World/Cube", masses=[1.0])

        sensor = ContactSensor(
            Contact.create(
                "/World/Cube/contact_sensor",
                min_threshold=0,
                max_threshold=10000000,
                radius=-1,
            )
        )
        self.assertFalse(sensor.get_sensor_reading().is_valid, "Reading should be invalid before physics steps")

        try:
            self._timeline.play()
            SimulationManager.initialize_physics()
            SimulationManager.step(steps=10, update_fabric=False)

            reading = sensor.get_sensor_reading()
            self.assertTrue(reading.is_valid, "Reading should be valid after physics-only steps")
            self.assertTrue(reading.in_contact, "Cube should contact the ground")
            self.assertGreater(reading.value, 0.0)
        finally:
            sensor.reset()
            if self._timeline.is_playing():
                self._timeline.stop()
                await omni.kit.app.get_app().next_update_async()

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
        contacts_raw = self._get_contact_sensor(self.leg_paths[0] + "/sensor").get_raw_data()
        self.assertEqual(len(contacts_raw), 0)

        xform = UsdGeom.Xformable(self._stage.GetPrimAtPath("/World/GroundPlane"))
        xform_op = xform.GetOrderedXformOps()[0]
        xform_op.Set(Gf.Vec3d(0, 0, -0.78))
        await step_simulation(1)  # simulate 60 steps, ant should touch ground
        contacts_raw = self._get_contact_sensor(self.leg_paths[0] + "/sensor").get_raw_data()
        self.assertEqual(len(contacts_raw), 1)

        contact = contacts_raw[0]
        body0 = str(PhysicsSchemaTools.intToSdfPath(contact["body0"]))
        body1 = str(PhysicsSchemaTools.intToSdfPath(contact["body1"]))
        if self.leg_paths[0] not in [body0, body1]:
            self.fail(f"Raw contact does not contain queried body {self.leg_paths[0]} ({body0},{body1})")
        self.assertAlmostEqual(1.0, contact["normal"]["z"], delta=0.1)

        # move the ground to lose the contacts
        xform = UsdGeom.Xformable(self._stage.GetPrimAtPath("/World/GroundPlane"))
        xform_op = xform.GetOrderedXformOps()[0]
        xform_op.Set(Gf.Vec3d(0, 0, -15))
        await step_simulation(0.5)
        contacts_raw = self._get_contact_sensor(self.leg_paths[0] + "/sensor").get_raw_data()
        self.assertEqual(len(contacts_raw), 0)

    async def test_get_body_raw_data(self) -> None:
        """Test raw contact data retrieval between rigid bodies without using ant."""
        await stage_utils.create_new_stage_async()
        stage_utils.set_stage_units(meters_per_unit=1.0)
        SimulationManager.setup_simulation(dt=1.0 / self._physics_rate)

        GroundPlane("/World/GroundPlane", sizes=10.0)

        add_reference_to_stage(
            usd_path=self._assets_root_path + "/Isaac/Props/Blocks/basic_block.usd", path="/World/block_0"
        )
        RigidPrim(
            "/World/block_0/Cube", positions=[10, 0, 5.0], scales=np.ones(3) * 1.0, reset_xform_op_properties=True
        )

        add_reference_to_stage(
            usd_path=self._assets_root_path + "/Isaac/Props/Blocks/basic_block.usd", path="/World/block_1"
        )
        RigidPrim(
            "/World/block_1/Cube", positions=[10, 0, 10.0], scales=np.ones(3) * 1.0, reset_xform_op_properties=True
        )

        ContactSensor("/World/block_1/Cube/contact_sensor")
        sensor = self._get_contact_sensor("/World/block_1/Cube/contact_sensor")

        await omni.kit.app.get_app().next_update_async()
        self._timeline.play()

        count = 0
        while count < 500:
            count += 1
            await omni.kit.app.get_app().next_update_async()
            reading = sensor.get_sensor_reading()
            if reading.is_valid and reading.in_contact:
                break

        self.assertTrue(count < 500, "Block 1 never detected contact via C++ contact sensor")

    async def test_get_raw_data(self) -> None:
        """Validate raw contact data from a sensor when the ant contacts ground."""
        await self._setup_ant()
        await self._add_sensor_prims()
        await omni.kit.app.get_app().next_update_async()
        self._timeline.play()
        await step_simulation(1)  # simulate 60 steps, ant should touch ground
        contacts_raw = self._get_contact_sensor(self.leg_paths[0] + "/sensor").get_raw_data()
        self.assertEqual(len(contacts_raw), 1)

        contact = contacts_raw[0]
        body0 = str(PhysicsSchemaTools.intToSdfPath(contact["body0"]))
        body1 = str(PhysicsSchemaTools.intToSdfPath(contact["body1"]))
        if self.leg_paths[0] not in [body0, body1]:
            self.fail(f"Raw contact does not contain queried body {self.leg_paths[0]} ({body0},{body1})")
        self.assertAlmostEqual(1.0, contact["normal"]["z"], delta=0.1)

    async def test_persistent_raw_data(self) -> None:
        """Ensure raw contact data remains available during persistent contacts."""
        await self._setup_ant()
        await self._add_sensor_prims()
        self._timeline.play()
        await step_simulation(2.0)  # simulate long enough that physx stops sending persistent contact raw data
        contacts_raw = self._get_contact_sensor(self.leg_paths[0] + "/sensor").get_raw_data()
        self.assertEqual(len(contacts_raw), 1)

        contact = contacts_raw[0]
        body0 = str(PhysicsSchemaTools.intToSdfPath(contact["body0"]))
        body1 = str(PhysicsSchemaTools.intToSdfPath(contact["body1"]))
        if self.leg_paths[0] not in [body0, body1]:
            self.fail(f"Raw contact does not contain queried body {self.leg_paths[0]} ({body0},{body1})")
        self.assertAlmostEqual(1.0, contact["normal"]["z"], delta=6)

    async def test_delayed_get_sensor_reading(self) -> None:
        """Compare delayed sensor readings against raw impulse-derived force."""
        await self._setup_ant()
        await self._add_sensor_prims()
        await omni.kit.app.get_app().next_update_async()
        self._timeline.play()
        await step_simulation(1.0)

        for i in range(120):
            await omni.kit.app.get_app().next_update_async()

        sensor = self._get_contact_sensor(self.leg_paths[0] + "/sensor")
        contacts_raw = sensor.get_raw_data()
        sensor_reading = sensor.get_sensor_reading()
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
        self.sensor_geoms = []

        # create 4 sensors at the center of the leg
        for i in range(4):
            sensor = ContactSensor(
                Contact.create(
                    self.leg_paths[i] + "/sensor",
                    min_threshold=0,
                    max_threshold=10000000,
                    color=self.color[i],
                    radius=0.12,
                    translations=[[0.0, 0.0, 0.0]],
                )
            )
            self.sensor_geoms.append(sensor)
            self.assertIsNotNone(sensor)

        await omni.kit.app.get_app().next_update_async()
        self._timeline.play()
        await step_simulation(1)

        for i in range(40):
            await omni.kit.app.get_app().next_update_async()
            sensor_reading = self._get_contact_sensor(self.leg_paths[0] + "/sensor").get_sensor_reading()
            self.assertTrue(sensor_reading.is_valid)
            self.assertEqual(sensor_reading.value, 0)

    async def test_stop_start(self) -> None:
        """Verify sensor readings are consistent across timeline stop/start."""
        await self._setup_ant()
        await self._add_sensor_prims()

        await omni.kit.app.get_app().next_update_async()

        self._timeline.play()
        await step_simulation(0.5)

        init_reading = self._get_contact_sensor(self.leg_paths[0] + "/sensor").get_sensor_reading()

        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()

        self._timeline.play()
        await step_simulation(0.5)

        sensor_reading = self._get_contact_sensor(self.leg_paths[0] + "/sensor").get_sensor_reading()

        self.assertEqual(init_reading.in_contact, sensor_reading.in_contact)
        self.assertEqual(init_reading.value, sensor_reading.value)
        self.assertEqual(init_reading.time, sensor_reading.time)

    # number of readings aggregated from node is same as number output from sensor

    # if time permits, add currently in contact with functionality to contact sensor node

    async def test_sensor_latest_data(self) -> None:
        """Ensure sensor reads return monotonically increasing times."""
        await self._setup_ant()
        for i in range(4):
            sensor = ContactSensor(
                Contact.create(
                    self.leg_paths[i] + "/custom_sensor",
                    min_threshold=0,
                    max_threshold=10000000,
                    color=self.color[i],
                    radius=0.12,
                    translations=self.sensor_offsets[i],
                )
            )
            self.assertIsNotNone(sensor)

        await omni.kit.app.get_app().next_update_async()
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        old_time = -1

        for i in range(10):
            await omni.kit.app.get_app().next_update_async()
            latest_sensor_reading = self._get_contact_sensor(self.leg_paths[0] + "/custom_sensor").get_sensor_reading()
            self.assertTrue(latest_sensor_reading.time > old_time)
            old_time = latest_sensor_reading.time

    async def test_invalid_after_prim_delete(self) -> None:
        """Reading a sensor whose prim was deleted mid-simulation returns invalid.

        Replaces the legacy ``test_wrong_sensor_path`` (which constructed a
        backend at a non-existent path); after collapsing the backend layer,
        the equivalent way to exercise the invalid-reading branch is to
        invalidate the prim after creation.
        """
        await self._setup_ant()
        await self._add_sensor_prims()
        sensor_path = self.leg_paths[0] + "/disposable_sensor"
        sensor = ContactSensor(
            Contact.create(
                sensor_path,
                min_threshold=0,
                max_threshold=10000000,
                color=self.color[0],
                radius=0.12,
                translations=self.sensor_offsets[0],
            )
        )
        self.assertIsNotNone(sensor)

        await omni.kit.app.get_app().next_update_async()
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        for _ in range(10):
            await omni.kit.app.get_app().next_update_async()

        valid_reading = sensor.get_sensor_reading()
        self.assertTrue(valid_reading.is_valid, "Sensor should produce valid readings while the prim exists")

        stage_utils.delete_prim(sensor_path)
        await omni.kit.app.get_app().next_update_async()

        for _ in range(5):
            await omni.kit.app.get_app().next_update_async()

        invalid_reading = sensor.get_sensor_reading()
        self.assertFalse(invalid_reading.is_valid, "Reading should be invalid after the sensor prim is deleted")

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
            sensor = ContactSensor(
                Contact.create(
                    self.leg_paths[i] + "/custom_sensor",
                    min_threshold=min_threshold[i],
                    max_threshold=max_threshold[i],
                    color=self.color[i],
                    radius=0.12,
                    translations=self.sensor_offsets[i],
                )
            )
            self.assertIsNotNone(sensor)

        await omni.kit.app.get_app().next_update_async()
        self._timeline.play()
        # give it some time to reach the ground first
        await step_simulation(1.0)

        sensor_0 = self._get_contact_sensor(self.leg_paths[0] + "/custom_sensor").get_sensor_reading()  # expect contact
        sensor_1 = self._get_contact_sensor(
            self.leg_paths[1] + "/custom_sensor"
        ).get_sensor_reading()  # expect no contact
        sensor_2 = self._get_contact_sensor(
            self.leg_paths[2] + "/custom_sensor"
        ).get_sensor_reading()  # expect proper reading
        sensor_3 = self._get_contact_sensor(self.leg_paths[3] + "/custom_sensor").get_sensor_reading()  # expect 0.1

        # print(f"sensor 0: {float(sensor_0.value)}")  # val
        # print(f"sensor 1: {float(sensor_1.value)}")  # no val
        # print(f"sensor 2: {float(sensor_2.value)}")  # val
        # print(f"sensor 3: {float(sensor_3.value)}")  # val
        self.assertTrue(sensor_0.in_contact)
        self.assertFalse(sensor_1.in_contact)
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
            XformPrim(xform_path, translations=[20, 0, 0], reset_xform_op_properties=True)

            sensor = ContactSensor(
                Contact.create(
                    self.leg_paths[i] + "/xform/custom_sensor",
                    min_threshold=0.0,
                    max_threshold=100.0,
                    color=self.color[i],
                    radius=0.12,
                    translations=[[20.0, 0.0, 0.0]],
                )
            )
            self.assertIsNotNone(sensor)

        await omni.kit.app.get_app().next_update_async()
        self._timeline.play()
        # give it some time to reach the ground first
        await step_simulation(1.0)

        # all four sensors should have proper reading
        for i in range(4):
            sensor_reading = self._get_contact_sensor(
                self.leg_paths[i] + "/xform/custom_sensor"
            ).get_sensor_reading()  # expect contact
            self.assertTrue(sensor_reading.in_contact)
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
            Contact.create(
                "/World/Cube/Contact_Sensor",
                translations=np.array([[0.0, 0.0, 0.0]]),
                min_threshold=0,
                max_threshold=100,
                radius=-1,
            )
        )

        top_sensor = ContactSensor(
            Contact.create(
                "/World/Cube2/Contact_Sensor",
                translations=np.array([[0.0, 0.0, 0.0]]),
                min_threshold=0,
                max_threshold=100,
                radius=-1,
            )
        )

        # Initialize timeline
        await reset_timeline(self._timeline)

        # Allow settling time for cubes to come into contact
        await step_simulation(10 * SimulationManager.get_physics_dt())

        # Get latest contact data
        bottom_data = bottom_sensor.get_data()
        top_data = top_sensor.get_data()

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
        sensor = ContactSensor(
            Contact.create(
                "/World/Cube/contact_sensor",
                min_threshold=0,
                max_threshold=10000000,
                radius=0.12,
                translations=[[0.0, 0.0, 0.0]],
            )
        )
        self.assertIsNotNone(sensor)
        await omni.kit.app.get_app().next_update_async()

        stage = stage_utils.get_current_stage()

        # Before simulation starts, sensor should not be registered with the manager
        self.assertFalse(self._timeline.is_playing())

        # Start the timeline to register sensors with the manager
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()

        # After simulation starts, valid contact sensor path should return True
        prim = stage.GetPrimAtPath("/World/Cube/contact_sensor")
        self.assertTrue(prim.IsValid() and prim.GetTypeName() == "IsaacContactSensor")

        # Test invalid prim (cube itself, not a sensor) - should return False
        prim = stage.GetPrimAtPath("/World/Cube")
        self.assertFalse(prim.GetTypeName() == "IsaacContactSensor")

        # Test non-existent prim path - should return False
        prim = stage.GetPrimAtPath("/World/NonExistent/sensor")
        self.assertFalse(prim.IsValid())

    async def test_raw_data_all_fields(self) -> None:
        """Validate CsRawData fields include dt and position."""
        await self._setup_ant()
        await self._add_sensor_prims()
        await omni.kit.app.get_app().next_update_async()
        self._timeline.play()
        await step_simulation(1.0)

        contacts_raw = self._get_contact_sensor(self.leg_paths[0] + "/sensor").get_raw_data()
        self.assertGreater(len(contacts_raw), 0)
        contact = contacts_raw[0]

        self.assertGreater(float(contact["dt"]), 0.0)
        self.assertLessEqual(float(contact["dt"]), SimulationManager.get_physics_dt() * 1.1)

        position = contact["position"]
        for axis in ("x", "y", "z"):
            self.assertTrue(np.isfinite(position[axis]))

    async def test_cs_sensor_reading_defaults(self) -> None:
        """Verify ContactSensorReading default construction values."""
        reading = ContactSensorReading()
        self.assertEqual(reading.time, 0.0)
        self.assertEqual(reading.value, 0.0)
        self.assertFalse(reading.in_contact)
        self.assertFalse(reading.in_contact)
        self.assertFalse(reading.is_valid)


class TestContactSensorRuntimeData(omni.kit.test.AsyncTestCase):
    """Test ContactSensor runtime data helpers."""

    # Before running each test
    async def setUp(self) -> None:
        """Create a cube-on-ground contact sensor scene for runtime data tests."""
        await stage_utils.create_new_stage_async()
        SimulationManager.setup_simulation(dt=1.0 / 60.0)
        self._timeline = omni.timeline.get_timeline_interface()
        GroundPlane("/World/defaultGroundPlane", positions=[0.0, 0.0, 0.0])
        Cube(
            "/World/new_cube_2",
            sizes=1.0,
            positions=[0.0, 0.0, 1.0],
            scales=[0.6, 0.5, 0.2],
        )
        GeomPrim("/World/new_cube_2", apply_collision_apis=True)
        RigidPrim("/World/new_cube_2", masses=[1.0])
        self._contact_sensor = ContactSensor(
            Contact(
                "/World/new_cube_2/contact_sensor",
                min_threshold=0,
                max_threshold=10000000,
                radius=-1,
            )
        )
        await reset_timeline(self._timeline, steps=1)
        return

    # After running each test
    async def tearDown(self) -> None:
        """Stop playback, clear physics state, and wait for pending stage loads."""
        if self._timeline.is_playing():
            self._timeline.stop()
        SimulationManager.invalidate_physics()
        await omni.kit.app.get_app().next_update_async()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            # print("tearDown, assets still loading, waiting to finish...")
            await asyncio.sleep(1.0)
        await omni.kit.app.get_app().next_update_async()
        return

    async def test_data_acquisition(self) -> None:
        """Verify current frame contains expected fields and optional contacts."""
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        data = self._contact_sensor.get_data()
        for key in ["time", "physics_step", "in_contact", "force", "number_of_contacts"]:
            self.assertTrue(key in data)
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        self.assertTrue("contacts" not in data)
        self._contact_sensor.add_raw_contact_data_to_frame()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        self.assertTrue("contacts" in data)
        self._contact_sensor.remove_raw_contact_data_from_frame()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        self.assertTrue("contacts" not in data)
        return

    async def test_timeline_reset(self) -> None:
        """Verify frame updates are consistent across timeline stop/start."""
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        data = self._contact_sensor.get_data()
        self.assertGreater(data["physics_step"], 0)
        self.assertGreater(data["time"], 0)

        await reset_timeline(self._timeline, steps=1)
        data = self._contact_sensor.get_data()
        self.assertEqual(data["physics_step"], 3)
        self.assertAlmostEqual(data["time"], 0.05, delta=0.01)
        return

    async def test_properties(self) -> None:
        """Verify contact sensor property setters update the underlying values."""
        self._contact_sensor.contact.set_radius(0.1)
        self.assertAlmostEqual(0.1, self._contact_sensor.contact.get_radius(), delta=0.01)
        self._contact_sensor.contact.set_min_threshold(0.1)
        self.assertAlmostEqual(0.1, self._contact_sensor.contact.get_min_threshold(), delta=0.01)
        self._contact_sensor.contact.set_max_threshold(100000)
        self.assertAlmostEqual(100000, self._contact_sensor.contact.get_max_threshold(), delta=0.01)
        return
