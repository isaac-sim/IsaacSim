# SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""
================================================================================
This file contains code snippets that are displayed in the Newton Actuators
"Adding Actuators from Python" tutorial.  Keep the
``<start-...-snippet>`` / ``<end-...-snippet>`` markers in sync with
``docs/isaacsim/newton_actuators_tutorials/newton_actuators_python.rst``.
================================================================================

Runs end-to-end as a standalone script:

    ./python.sh standalone_examples/api/isaacsim.core.experimental.actuators/newton_actuators_python_example.py

Pass ``--non-ideal`` to swap the default ideal PD actuators for the
non-ideal variant (PD + per-joint effort clamp + 2-step input delay):

    ./python.sh standalone_examples/api/isaacsim.core.experimental.actuators/newton_actuators_python_example.py --non-ideal
"""

from __future__ import annotations

# ============================================================================
# 1. Parse arguments and launch Simulation App
# ============================================================================
import argparse

_parser = argparse.ArgumentParser(description="Newton actuators Python tutorial example.")
_parser.add_argument(
    "--non-ideal",
    action="store_true",
    help=(
        "Use non-ideal actuators (PD + per-joint effort clamp + 2-step input delay) "
        "instead of the default ideal PD actuators."
    ),
)
args, _ = _parser.parse_known_args()

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
import omni.kit.app
import omni.timeline
import warp as wp
from isaacsim.core.experimental.utils.stage import add_reference_to_stage
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.storage.native import get_assets_root_path_async

FRANKA_USD_REL_PATH = "Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
FRANKA_PRIM_PATH = "/World/Franka"
ARM_JOINTS = [f"panda_joint{i}" for i in range(1, 8)]


async def setup_stage_with_franka() -> None:
    """Create a fresh stage with a PhysicsScene and reference in a Franka Panda.

    Async so the kit app stays responsive while the Franka USD is fetched
    (especially from Nucleus).  Drive it via ``simulation_app.run_coroutine``.
    """
    await stage_utils.create_new_stage_async(template="default stage")
    stage_utils.define_prim("/World/PhysicsScene", "PhysicsScene")
    assets_root_path = await get_assets_root_path_async()
    usd_path = f"{assets_root_path}/{FRANKA_USD_REL_PATH}"
    add_reference_to_stage(usd_path, FRANKA_PRIM_PATH)
    await omni.kit.app.get_app().next_update_async()


# ============================================================================
# 2. Building a stock PD actuator config
# ============================================================================
def build_pd_actuator_config(n_robots: int, kp: float, kd: float) -> ActuatorConfig:
    """Build an ActuatorConfig with a Newton ``ControllerPD``."""
    # <start-build-pd-config-snippet>
    import warp as wp
    from isaacsim.core.experimental.actuators import ActuatorConfig
    from newton.actuators import ControllerPD

    # Per-robot Warp arrays.  Even a single-robot scene must size them with
    # n_robots = len(Articulation(paths)).
    pd_config = ActuatorConfig(
        controller=ControllerPD(
            kp=wp.array([kp] * n_robots, dtype=wp.float32),
            kd=wp.array([kd] * n_robots, dtype=wp.float32),
        )
    )
    # <end-build-pd-config-snippet>
    return pd_config


# ============================================================================
# 3. Adding clamping and delay to an actuator config
# ============================================================================
def build_pd_with_clamping_and_delay(n_robots: int, kp: float, kd: float, max_effort: float) -> ActuatorConfig:
    """Build an ActuatorConfig with PD control plus a max-effort clamp and an input delay.

    ``kp``, ``kd``, and ``max_effort`` are single-joint scalars; each is fanned
    out across ``n_robots`` instances.
    """
    # <start-build-pd-with-clamping-snippet>
    import warp as wp
    from isaacsim.core.experimental.actuators import ActuatorConfig
    from newton.actuators import ClampingMaxEffort, ControllerPD, Delay

    config = ActuatorConfig(
        controller=ControllerPD(
            kp=wp.array([kp] * n_robots, dtype=wp.float32),
            kd=wp.array([kd] * n_robots, dtype=wp.float32),
        ),
        clamping=[
            ClampingMaxEffort(max_effort=wp.array([max_effort] * n_robots, dtype=wp.float32)),
        ],
        delay=Delay(
            delay_steps=wp.array([2] * n_robots, dtype=wp.int32),
            max_delay=2,
        ),
    )
    # <end-build-pd-with-clamping-snippet>
    return config


# ============================================================================
# 4. Constructing ArticulationActuators with the configs
# ============================================================================
# Per-joint PD gains tuned for a single Franka.  Joints 1-4 are the heavy arm
# joints; joints 5-7 are the wrist.
KP_GAINS = [32.0, 32.0, 32.0, 16.0, 32.0, 32.0, 32.0]
KD_GAINS = [8.0, 8.0, 8.0, 8.0, 8.0, 8.0, 8.0]
MAX_EFFORTS = [1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0]


def construct_articulation_actuators() -> ArticulationActuators:
    """Attach a per-DOF ideal PD actuator to each of the seven Franka arm joints."""
    # <start-construct-from-actuators-snippet>
    from isaacsim.core.experimental.actuators import ArticulationActuators
    from isaacsim.core.experimental.prims import Articulation

    # FRANKA_PRIM_PATH = "/World/Franka"
    # ARM_JOINTS       = ["panda_joint1", "panda_joint2", ..., "panda_joint7"]
    # Single Franka instance, so n_robots = 1.
    n_robots = len(Articulation(FRANKA_PRIM_PATH))

    actuators = [
        (build_pd_actuator_config(n_robots, kp, kd), name) for name, kp, kd in zip(ARM_JOINTS, KP_GAINS, KD_GAINS)
    ]

    actuated = ArticulationActuators.from_actuators(
        FRANKA_PRIM_PATH,
        actuators=actuators,
    )

    # A small armature on every DOF improves numerical stability when joints
    # are driven externally (the Newton-actuator effort can excite high-frequency
    # modes that the implicit USD drive would normally damp).  Real motors and
    # gearboxes also carry rotor inertia, so a non-zero armature is closer to
    # physical reality regardless of stability concerns.
    actuated.articulation.set_dof_armatures(0.1)
    # <end-construct-from-actuators-snippet>
    return actuated


def construct_articulation_actuators_non_ideal() -> ArticulationActuators:
    """Construct `ArticulationActuators` with per-joint clamping and a 2-step input delay."""
    from isaacsim.core.experimental.actuators import ArticulationActuators
    from isaacsim.core.experimental.prims import Articulation

    n_robots = len(Articulation(FRANKA_PRIM_PATH))

    actuators = [
        (build_pd_with_clamping_and_delay(n_robots, kp, kd, max_effort), name)
        for name, kp, kd, max_effort in zip(ARM_JOINTS, KP_GAINS, KD_GAINS, MAX_EFFORTS)
    ]

    actuated = ArticulationActuators.from_actuators(
        FRANKA_PRIM_PATH,
        actuators=actuators,
    )
    actuated.articulation.set_dof_armatures(0.1)
    return actuated


# ============================================================================
# 5. Driving the robot to a position target
# ============================================================================
def drive_to_target(actuated, num_steps: int = 240) -> None:
    """Set position targets and step the simulation to watch the robot converge."""
    # <start-drive-to-target-snippet>
    articulation = actuated.articulation

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    simulation_app.update()

    # Send a per-DOF position target.  The PD controller reads it each tick.
    # Values match the Franka robot descriptor's default_q (home pose).
    target_positions = [0.8, -1.3, 0.0, -2.87, 0.0, 2.0, 0.75]
    arm_dof_indices = articulation.get_dof_indices(ARM_JOINTS)
    articulation.set_dof_position_targets(
        positions=target_positions,
        dof_indices=arm_dof_indices,
    )

    for _ in range(num_steps):  # ~4 seconds at 60 Hz
        simulation_app.update()
    # <end-drive-to-target-snippet>

    # <start-feedforward-effort-snippet>
    # Feedforward effort is added on top of the controller output every tick.
    # With kp = kd = 0 it becomes the entire output — a pure open-loop torque drive.
    feedforward_efforts = [50.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    actuated.set_dof_feedforward_effort_targets(
        feedforward_efforts,
        dof_indices=arm_dof_indices,
    )
    # <end-feedforward-effort-snippet>

    for _ in range(num_steps):
        simulation_app.update()


# ============================================================================
# Entry point
# ============================================================================
def main() -> None:
    SimulationManager.set_physics_dt(1.0 / 60.0)
    simulation_app.run_coroutine(setup_stage_with_franka())
    if args.non_ideal:
        print("Using non-ideal actuators (PD + per-joint effort clamp + 2-step delay).")
        construct = construct_articulation_actuators_non_ideal
    else:
        construct = construct_articulation_actuators
    try:
        # <start-context-manager-snippet>
        # Recommended teardown pattern: construct ``ArticulationActuators`` inside
        # the ``with`` statement so the wrapper's lifetime is bounded by the block.
        # ``__exit__`` calls ``actuated.close()``, which deregisters every
        # ``SimulationManager`` lifecycle callback owned by the instance and is
        # guaranteed to run even if the body raises.
        with construct() as actuated:
            drive_to_target(actuated)
        # <end-context-manager-snippet>
    finally:
        omni.timeline.get_timeline_interface().stop()
    print("Newton actuators Python example complete.")


if __name__ == "__main__":
    main()
    simulation_app.close()
