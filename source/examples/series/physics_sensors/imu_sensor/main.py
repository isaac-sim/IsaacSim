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

"""Derive orientation, angular velocity, and specific force for a mounted IMU."""

from __future__ import annotations

import math
from collections.abc import Sequence

import isaacsim.physics_engines.ovphysx  # noqa: F401
from imu import BodyState, ImuReading, ImuSensor
from isaacsim.common.logging import Logger
from isaacsim.foundation.objects import Cube, PhysicsScene, Stage, Xform
from isaacsim.foundation.prims import ColliderBody, GroundPlane, RigidBody
from isaacsim.physics.entities import RigidBodyEntity
from isaacsim.physics.manager import PhysicsManager

ENGINE_NAME = "ovphysx"

LOGGER = Logger("isaacsim.examples.physics_sensors.imu_sensor")

BODY_PATH = "/World/Body"
SENSOR_PATH = "/World/Body/Imu"

# Mount in the body frame: raised along body Z and yawed a quarter turn about it, so the sensor
# frame differs from the body frame while sensor +Z stays along world up.
MOUNT_TRANSLATION = (0.0, 0.0, 0.25)
# Ordered xyzw, the library default, matching the sensor.
MOUNT_ORIENTATION = (0.0, 0.0, math.sin(math.pi / 4.0), math.cos(math.pi / 4.0))

# Authored on the stage, and reused as the sensor's gravity vector.
GRAVITY = (0.0, 0.0, -9.81)

# Dropped while spinning about world Z. It lands at step 33 and settles before the last step.
TIME_STEP = 1.0 / 60.0
STEP_COUNT = 180
BODY_SIZE = 1.0
DROP_HEIGHT = 2.0
INITIAL_ANGULAR_VELOCITY = (0.0, 0.0, 3.0)

# Rolling average window, equivalent to the IsaacImuSensor filter width attributes.
FILTER_WIDTH = 5

# Airborne, and late enough for the sample buffer to be full.
FREE_FALL_STEP = 25


def _author_stage() -> tuple[str, list[float], list[float]]:
    """Author the scene and read the sensor mount back through Foundation.

    Returns:
        Serialized stage data, the mount translation, and the mount orientation in ``xyzw`` order.

    """
    stage = Stage("openusd").create_stage()
    try:
        stage.define_prim("/World")

        GroundPlane("/World/Ground")
        Cube(BODY_PATH, sizes=BODY_SIZE).set_local_poses(translations=[0.0, 0.0, DROP_HEIGHT])

        PhysicsScene("/World/PhysicsScene").set_gravities(GRAVITY)
        ColliderBody(BODY_PATH)
        RigidBody(BODY_PATH)

        # A plain Xform child of the body: not simulated, so its pose is a fixed offset.
        sensor = Xform(SENSOR_PATH)
        sensor.set_local_poses(translations=list(MOUNT_TRANSLATION), orientations=list(MOUNT_ORIENTATION))
        mount_translations, mount_orientations = sensor.get_local_poses()
        return stage.export_stage_to_string(), mount_translations[0].list(), mount_orientations[0].list()
    finally:
        stage.close_stage()


def _read_body_state(body: RigidBodyEntity) -> BodyState:
    """Read the world-frame state of a single rigid body.

    Args:
        body: Rigid body entity wrapping exactly one prim.

    Returns:
        The body state, orientation included unreordered.

    """
    # The entity hands back the engine's xyzw ordering, which is the sensor's, so nothing moves.
    _, orientations = body.get_world_poses()
    linear_velocities, angular_velocities = body.get_velocities()
    return BodyState(
        orientation=orientations[0].list(),
        linear_velocity=linear_velocities[0].list(),
        angular_velocity=angular_velocities[0].list(),
    )


def _format_vector(values: Sequence[float]) -> str:
    """Format a short sequence of numbers for display.

    Args:
        values: Numbers to format.

    Returns:
        A bracketed, comma-separated string.

    """
    return "[" + ", ".join(f"{float(value): .3f}" for value in values) + "]"


def _print_reading(label: str, reading: ImuReading) -> None:
    """Print one reading.

    Args:
        label: Phase name shown above the reading.
        reading: Reading to print.

    """
    LOGGER.report(f"{label} at t = {reading.time:.3f} s:")
    LOGGER.report(f"  world orientation (xyzw)     {_format_vector(reading.orientation)}")
    LOGGER.report(f"  sensor angular velocity      {_format_vector(reading.angular_velocity)} rad/s")
    LOGGER.report(f"  sensor specific force        {_format_vector(reading.linear_acceleration)} m/s^2")


def main() -> None:
    """Simulate a spinning falling body and report IMU readings for a sensor mounted on it."""
    ovstage_stage = None
    manager = None
    try:
        stage_text, mount_translation, mount_orientation = _author_stage()
        ovstage_stage = Stage("ovstage").import_stage_from_string(stage_text, make_default=False)
        manager = PhysicsManager.get_instance()
        if not manager.switch_physics_engine(ENGINE_NAME):
            raise RuntimeError("OvPhysX physics engine is unavailable.")

        manager.setup(dt=TIME_STEP)
        # Held open for the run: the manager takes a pointer to the stage and does not own it.
        if not manager.initialize(ovstage_stage.get_stage_ptr(), ovstage_stage.get_stage_id()):
            raise RuntimeError("Physics initialization failed.")
        body = RigidBodyEntity(ENGINE_NAME, BODY_PATH)
        body.set_velocities(angular_velocities=list(INITIAL_ANGULAR_VELOCITY))

        sensor = ImuSensor(
            gravity=GRAVITY,
            mount_translation=mount_translation,
            mount_orientation=mount_orientation,
            filter_width=FILTER_WIDTH,
        )

        LOGGER.report(f"IMU at {SENSOR_PATH} mounted on {BODY_PATH}.")
        LOGGER.report("Reporting specific force, the proper acceleration.")

        free_fall_reading: ImuReading | None = None
        for step in range(STEP_COUNT + 1):
            manager.step()
            # The buffers hold the state after the step, so the manager's clock stamps it.
            sensor.sample(manager.get_simulated_time(), _read_body_state(body))
            if step == FREE_FALL_STEP:
                free_fall_reading = sensor.read()

        resting_reading = sensor.read()
        if free_fall_reading is None or resting_reading is None:
            raise RuntimeError("The sensor did not buffer enough samples to produce a reading.")
        _print_reading("Free fall", free_fall_reading)
        _print_reading("At rest", resting_reading)
    finally:
        if manager is not None and manager.is_initialized():
            manager.invalidate()
        if ovstage_stage is not None:
            ovstage_stage.close_stage()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
