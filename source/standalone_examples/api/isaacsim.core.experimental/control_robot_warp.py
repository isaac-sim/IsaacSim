# SPDX-FileCopyrightText: Copyright (c) 2021-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

from __future__ import annotations

from typing import Any

"""
This example demonstrates how to control the Franka Panda's arm DOFs (via differential Inverse Kinematics (IK))
to follow a randomly positioned sphere in 3D space using Warp.

The example serves to illustrate the following concepts:
- How to use the Isaac Sim core (experimental) API to compose a stage, and wrap and operate on prims.
- How to use Warp (native to the core experimental API) for robot control computations.

The source code is organized into 4 main sections:
1. Command-line argument parsing and SimulationApp launch (common to all standalone examples).
2. Helper function definitions (using Warp).
3. Stage creation and population.
4. Example logic.
"""

# 1. --------------------------------------------------------------------

# Parse any command-line arguments specific to the standalone application (only known arguments).
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--device", type=str, choices=["cpu", "cuda"], default="cpu", help="Simulation device")
parser.add_argument(
    "--ik-method",
    type=str,
    choices=["singular-value-decomposition", "pseudoinverse", "transpose", "damped-least-squares"],
    default="transpose",
    help="Differential inverse kinematics method",
)
args, _ = parser.parse_known_args()

# Launch the `SimulationApp` (see DEFAULT_LAUNCHER_CONFIG for available configuration):
# https://docs.isaacsim.omniverse.nvidia.com/latest/py/source/extensions/isaacsim.simulation_app/docs/index.html
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

# Any Omniverse level imports must occur after the `SimulationApp` class is instantiated (because APIs are provided
# by the extension/runtime plugin system, it must be loaded before they will be available to import).
import isaacsim.core.experimental.utils.stage as stage_utils
import omni.timeline
import warp as wp
from isaacsim.core.experimental.materials import PreviewSurfaceMaterial
from isaacsim.core.experimental.objects import Sphere
from isaacsim.core.experimental.prims import Articulation, RigidPrim
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.storage.native import get_assets_root_path

# 2. --------------------------------------------------------------------

# Warp-kernel related variables.
TILE_THREADS = 64


@wp.kernel
def _sample_random_position_kernel(center: wp.array2d(dtype=Any), scale: float, seed: int, out: wp.array2d(dtype=Any)):
    i = wp.tid()
    for axis in range(3):
        state = wp.rand_init(seed, offset=i + axis)
        sample = 2.0 * (wp.randf(state) - 0.5)  # [-1, 1)
        out[i, axis] = center[i, axis] + scale * sample


def sample_random_position(*, center: wp.array, scale: float, seed: int) -> wp.array:
    out = wp.zeros_like(center)
    wp.launch(kernel=_sample_random_position_kernel, dim=center.shape[0], inputs=[center, scale, seed], outputs=[out])
    return out


@wp.kernel
def _add_kernel(a: wp.array2d(dtype=Any), b: wp.array2d(dtype=Any), out: wp.array2d(dtype=Any)):
    i, j = wp.tid()
    out[i, j] = a[i, j] + b[i, j]


def add(a: wp.array, b: wp.array) -> wp.array:
    out = wp.zeros_like(a)
    wp.launch(kernel=_add_kernel, dim=a.shape, inputs=[a, b], outputs=[out], device=a.device)
    return out


@wp.kernel
def _compute_error_kernel(
    current_position: wp.array2d(dtype=Any),  # shape: (N, 3)
    current_orientation: wp.array2d(dtype=Any),  # shape: (N, 4)
    goal_position: wp.array2d(dtype=Any),  # shape: (N, 3)
    goal_orientation: wp.array2d(dtype=Any),  # shape: (N, 4)
    out: wp.array3d(dtype=Any),  # shape: (N, 6, 1)
):
    i = wp.tid()
    # Convert Isaac Sim quaternion (wxyz) to Warp quaternion (xyzw)
    q_goal = wp.quat(goal_orientation[i, 1], goal_orientation[i, 2], goal_orientation[i, 3], goal_orientation[i, 0])
    q_current = wp.quat(
        current_orientation[i, 1], current_orientation[i, 2], current_orientation[i, 3], current_orientation[i, 0]
    )
    # Compute linear velocity error
    out[i, 0, 0] = goal_position[i, 0] - current_position[i, 0]
    out[i, 1, 0] = goal_position[i, 1] - current_position[i, 1]
    out[i, 2, 0] = goal_position[i, 2] - current_position[i, 2]
    # Compute angular velocity error
    q = wp.mul(q_goal, wp.quat_inverse(q_current))  # xyzw
    out[i, 3, 0] = q[0] * wp.sign(q[3])  # x
    out[i, 4, 0] = q[1] * wp.sign(q[3])  # y
    out[i, 5, 0] = q[2] * wp.sign(q[3])  # z


@wp.kernel
def _transpose_kernel(
    jacobian: wp.array3d(dtype=wp.float32),  # shape: (N, 6, 7)
    error: wp.array3d(dtype=wp.float32),  # shape: (N, 6, 1)
    scale: wp.float32,
    output: wp.array3d(dtype=wp.float32),  # shape: (N, 7, 1)
):
    i = wp.tid()
    _jacobian = wp.tile_load(jacobian[i], shape=(6, 7))
    _error = wp.tile_load(error[i], shape=(6, 1))
    out = wp.mul(wp.tile_matmul(wp.tile_transpose(_jacobian), _error), scale)
    wp.tile_store(output[i], out)


def differential_inverse_kinematics(
    jacobian_end_effector: wp.array,
    current_position: wp.array,
    current_orientation: wp.array,
    goal_position: wp.array,
    goal_orientation: wp.array | None = None,
    method: str = "damped-least-squares",
    method_cfg: dict[str, float] = {"scale": 1.0, "damping": 0.05, "min_singular_value": 1e-5},
) -> wp.array:
    batch_size = jacobian_end_effector.shape[0]
    device = jacobian_end_effector.device
    scale = method_cfg.get("scale", 1.0)
    # Compute velocity error
    error = wp.empty(shape=(batch_size, 6, 1), dtype=wp.float32, device=device)
    goal_orientation = current_orientation if goal_orientation is None else goal_orientation
    wp.launch(
        kernel=_compute_error_kernel,
        dim=batch_size,
        inputs=[current_position, current_orientation, goal_position, goal_orientation],
        outputs=[error],
        device=device,
    )
    # Compute delta DOF positions
    # - Transpose of matrix
    if method == "transpose":
        output = wp.zeros((1, 7, 1), dtype=wp.float32, device=device)
        wp.launch_tiled(
            _transpose_kernel,
            dim=[batch_size],
            inputs=[jacobian_end_effector, error, scale, output],
            block_dim=TILE_THREADS,
            device=device,
        )
        return output.reshape((batch_size, jacobian_end_effector.shape[-1]))
    elif method in ["singular-value-decomposition", "pseudoinverse", "damped-least-squares"]:
        raise NotImplementedError("Not implemented for Warp")
    else:
        raise ValueError("Invalid IK method")


# 3. --------------------------------------------------------------------

# Configure simulation.
SimulationManager.set_physics_sim_device(args.device)
simulation_app.update()  # allow configuration to take effect

# Setup stage programatically:
# - Create a new stage
stage_utils.create_new_stage(template="sunlight")
# - Add robot (Franka Panda)
robot_prim = stage_utils.add_reference_to_stage(
    usd_path=get_assets_root_path() + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd",
    path="/World/robot",
    variants=[("Gripper", "AlternateFinger"), ("Mesh", "Performance")],
)
# - Add red sphere
visual_material = PreviewSurfaceMaterial("/Visual_materials/red")
visual_material.set_input_values("diffuseColor", [1.0, 0.0, 0.0])
sphere = Sphere("/World/sphere", radii=[0.05], reset_xform_op_properties=True)
sphere.apply_visual_materials(visual_material)

# 4. --------------------------------------------------------------------

# Get high-level wrappers for the involved prims.
robot = Articulation("/World/robot")
end_effector_link = RigidPrim("/World/robot/panda_hand")
end_effector_link_index = robot.get_link_indices("panda_hand").list()[0]

robot.set_default_state(dof_positions=[0.012, -0.568, 0.0, -2.811, 0.0, 3.037, 0.741, 0.0, 0.0])

# Play the simulation.
omni.timeline.get_timeline_interface().play()
simulation_app.update()

# Run 10 trials of the robot reaching a random goal pose example.
for i in range(10):

    # Define a different random seed on each trial.
    seed = i

    # Set the sphere's position to a random value within a cube of side length 0.2 centered at (0.5, 0.0, 0.5).
    sphere_position = sample_random_position(center=wp.array([[0.5, 0.0, 0.5]], dtype=wp.float32), scale=0.1, seed=seed)
    sphere.set_world_poses(positions=sphere_position)

    # Reset the robot to the predefined default state.
    robot.reset_to_default_state()

    simulation_app.update()

    # Run the simulation in a loop for several iterations to allow the robot to reach the goal pose.
    # Differential IK deals with the change of robot DOF positions over time.
    for i in range(100):

        # Get intermediate values.
        current_dof_positions = robot.get_dof_positions()
        current_end_effector_position, current_end_effector_orientation = end_effector_link.get_world_poses()
        goal_position = sphere.get_world_poses()[0]

        # Get the Jacobian matrix related to the end-effector link.
        # The Franka Panda has 9 DOFs (arm: 7, hand: 2). Since we are controlling the arm, we only need the
        # first 7 values of the Jacobian matrix's last dimension (with length `num_dofs`, for fixed articulation base).
        jacobian_matrices = robot.get_jacobian_matrices()
        jacobian_end_effector = jacobian_matrices[:, end_effector_link_index - 1, :, :7]

        # Compute the change in (delta) DOF positions that contributes to reaching the desired goal pose.
        delta_dof_positions = differential_inverse_kinematics(
            jacobian_end_effector=jacobian_end_effector,
            current_position=current_end_effector_position,
            current_orientation=current_end_effector_orientation,
            goal_position=goal_position,
            goal_orientation=None,
            method=args.ik_method,
        )

        # Set the DOF position targets for the arm (current DOF positions + delta DOF positions).
        dof_position_targets = add(current_dof_positions[:, :7], delta_dof_positions)
        robot.set_dof_position_targets(dof_position_targets, dof_indices=list(range(7)))

        simulation_app.update()

# Close the `SimulationApp`.
simulation_app.close()
