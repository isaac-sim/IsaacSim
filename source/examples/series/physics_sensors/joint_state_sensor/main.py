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

"""Read the position, velocity, and effort of every degree of freedom of an articulation."""

from __future__ import annotations

import os

import isaacsim.physics_engines.ovphysx  # noqa: F401
import warp as wp
from isaacsim.common.logging import Logger
from isaacsim.foundation.objects import Stage
from isaacsim.physics.entities import ArticulationEntity
from isaacsim.physics.manager import PhysicsManager
from joint_state import ArticulationState, Dof, DofType, JointStateReading, JointStateSensor

# SimpleArticulation has one revolute and one prismatic DOF on a meter-and-kilogram stage. It
# includes its own physics scene, so the example opens it directly.
_DEFAULT_ASSET_ROOT = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.1"
ASSET_ROOT = (os.environ.get("ISAACSIM_ASSET_ROOT") or _DEFAULT_ASSET_ROOT).rstrip("/\\")
ROBOT_USD = f"{ASSET_ROOT}/Isaac/Robots/IsaacSim/SimpleArticulation/simple_articulation.usd"

ROBOT_PATH = "/Articulation"

LOGGER = Logger("isaacsim.examples.physics_sensors.joint_state_sensor")
ENGINE = "ovphysx"

SPIN_DOF = "RevoluteJoint"
SLIDE_DOF = "PrismaticJoint"

TIME_STEP = 1.0 / 60.0
STEP_COUNT = 180
# Both drives are commanded at a constant rate, so both DOFs have a closed form to read against.
SPIN_RATE = 0.5
SLIDE_RATE = 0.5
# Stiff enough that each drive holds its rate against the load the other one puts on it.
DRIVE_DAMPING = 1.0e6
# The slider covers its meter of travel in two seconds, then parks on its limit for the rest.
MOVING_STEP = 40


def _dof_type_from_name(name: str) -> DofType:
    """Read a DOF's kind off its joint name, which only this robot's naming makes possible.

    Args:
        name: DOF name, as the articulation reports it.

    Returns:
        The DOF's kind.

    Raises:
        ValueError: If the name matches neither joint kind.

    """
    for keyword, dof_type in (("Revolute", DofType.ROTATION), ("Prismatic", DofType.TRANSLATION)):
        if keyword in name:
            return dof_type
    raise ValueError(f"DOF {name!r} names neither a revolute nor a prismatic joint.")


def _read_articulation_state(robot: ArticulationEntity) -> ArticulationState:
    """Read the raw DOF state of a single articulation.

    Args:
        robot: Articulation entity wrapping exactly one prim.

    Returns:
        The state, in stage units and articulation order, exactly as the engine reports it.

    """
    return ArticulationState(
        positions=robot.get_dof_positions()[0],
        velocities=robot.get_dof_velocities()[0],
        # Not `get_dof_efforts()`, the actuation force a drive commands. The shipped sensor's
        # `efforts` is the measured joint force on the DOF axis, which the Newton backend does not
        # report at all: a caller there passes `efforts=None`.
        efforts=robot.get_dof_projected_joint_forces()[0],
    )


def _report(sensor: JointStateSensor, robot: ArticulationEntity, manager: PhysicsManager, label: str) -> None:
    """Read the articulation through a sensor and report the reading, one line per DOF.

    Args:
        sensor: Sensor to read through.
        robot: Articulation entity wrapping exactly one prim.
        manager: Physics manager, whose clock stamps the reading.
        label: Phase name shown above the reading.

    """
    reading = sensor.read(manager.get_simulated_time(), _read_articulation_state(robot))
    _print_reading(label, reading)


def _print_reading(label: str, reading: JointStateReading) -> None:
    """Print one reading, one line per DOF.

    Args:
        label: Phase name shown above the reading.
        reading: Reading to print.

    """
    LOGGER.report(f"{label} at t = {reading.time:.3f} s:")
    units = {
        DofType.ROTATION: ("rad", "rad/s", "N*m"),
        DofType.TRANSLATION: ("m", "m/s", "N"),
    }
    for index, name in enumerate(reading.dof_names):
        position, velocity, effort = units[reading.dof_types[index]]
        LOGGER.report(
            f"  {name:<14} ({reading.dof_types[index].name.lower():<11})"
            f"  position {reading.positions[index]: .4f} {position:<8}"
            f"  velocity {reading.velocities[index]: .4f} {velocity:<8}"
            f"  effort {reading.efforts[index]: .4f} {effort}"
        )


def main() -> None:
    """Simulate a two-DOF robot and report the joint state of both of its degrees of freedom."""
    ovstage_stage = None
    manager = None
    try:
        ovstage_stage = Stage("ovstage").open_stage(ROBOT_USD, make_default=False)
        # Everything goes through the manager. The module-level `initialize`, `simulate` and
        # `close` do the same job but are on their way out of the API.
        manager = PhysicsManager.get_instance()
        if not manager.switch_physics_engine(ENGINE):
            raise RuntimeError("OvPhysX physics engine is unavailable.")

        # The step size is configured once here rather than passed to every step.
        manager.setup(dt=TIME_STEP)
        if not manager.initialize(ovstage_stage.get_stage_ptr(), ovstage_stage.get_stage_id()):
            raise RuntimeError("Physics initialization failed.")
        robot = ArticulationEntity(ENGINE, ROBOT_PATH)

        dofs = [Dof(name, _dof_type_from_name(name)) for name in robot.dof_names]
        sensor = JointStateSensor(dofs)

        # The asset parks both drives on a stiff position gain. Zeroing the stiffness and leaning
        # on the damping turns each into a velocity drive, which is what the sensor documentation
        # has you do by hand in the property panel.
        order = [dof.name for dof in dofs]
        targets = [0.0] * len(dofs)
        targets[order.index(SPIN_DOF)] = SPIN_RATE
        targets[order.index(SLIDE_DOF)] = SLIDE_RATE
        robot.set_dof_gains(
            stiffnesses=wp.array([[0.0] * len(dofs)], dtype=wp.float32, device="cpu"),
            dampings=wp.array([[DRIVE_DAMPING] * len(dofs)], dtype=wp.float32, device="cpu"),
        )
        robot.set_dof_velocity_targets(wp.array([targets], dtype=wp.float32, device="cpu"))

        LOGGER.report(f"Joint state of the articulation at {ROBOT_PATH}.")
        LOGGER.report("Reporting measured joint effort, not commanded actuation.")
        LOGGER.report("The stage is authored in meters, so the raw DOF values are already SI.")

        manager.step(steps=MOVING_STEP + 1)
        _report(sensor, robot, manager, "Moving")
        manager.step(steps=STEP_COUNT - MOVING_STEP)
        _report(sensor, robot, manager, "Slider parked on its limit")
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
