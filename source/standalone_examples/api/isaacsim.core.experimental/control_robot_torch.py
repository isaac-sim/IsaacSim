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

"""
This example demonstrates how to control the Franka Panda's arm DOFs (via differential Inverse Kinematics (IK))
to follow a randomly positioned sphere in 3D space using PyTorch.

The example serves to illustrate the following concepts:
- How to use the Isaac Sim core (experimental) API to compose a stage, and wrap and operate on prims.
- How to perform (zero-copy) interoperability between Warp (native to the core experimental API) and PyTorch.

The source code is organized into 4 main sections:
1. Command-line argument parsing and SimulationApp launch (common to all standalone examples).
2. Helper function definitions (using PyTorch).
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
    default="damped-least-squares",
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
import torch
import warp as wp
from isaacsim.core.experimental.materials import PreviewSurfaceMaterial
from isaacsim.core.experimental.objects import Sphere
from isaacsim.core.experimental.prims import Articulation, RigidPrim
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.storage.native import get_assets_root_path

# 2. --------------------------------------------------------------------


def quat_mul(a: torch.Tensor, b: torch.Tensor) -> torch.Tensor:
    w1, x1, y1, z1 = a[:, 0], a[:, 1], a[:, 2], a[:, 3]
    w2, x2, y2, z2 = b[:, 0], b[:, 1], b[:, 2], b[:, 3]
    ww = (z1 + x1) * (x2 + y2)
    yy = (w1 - y1) * (w2 + z2)
    zz = (w1 + y1) * (w2 - z2)
    xx = ww + yy + zz
    qq = 0.5 * (xx + (z1 - x1) * (x2 - y2))
    w = qq - ww + (z1 - y1) * (y2 - z2)
    x = qq - xx + (x1 + w1) * (x2 + w2)
    y = qq - yy + (w1 - x1) * (y2 + z2)
    z = qq - zz + (z1 + y1) * (w2 - x2)
    return torch.stack([w, x, y, z], dim=-1)


def quat_conjugate(q: torch.Tensor) -> torch.Tensor:
    return torch.cat((q[:, :1], -q[:, 1:]), dim=-1)


def differential_inverse_kinematics(
    jacobian_end_effector: torch.Tensor,
    current_position: torch.Tensor,
    current_orientation: torch.Tensor,
    goal_position: torch.Tensor,
    goal_orientation: torch.Tensor | None = None,
    method: str = "damped-least-squares",
    method_cfg: dict[str, float] = {"scale": 1.0, "damping": 0.05, "min_singular_value": 1e-5},
) -> torch.Tensor:
    scale = method_cfg.get("scale", 1.0)
    # Compute velocity error
    goal_orientation = current_orientation if goal_orientation is None else goal_orientation
    q = quat_mul(goal_orientation, quat_conjugate(current_orientation))
    error = torch.cat([goal_position - current_position, q[:, 1:] * torch.sign(q[:, [0]])], dim=-1).unsqueeze(-1)
    # Compute delta DOF positions
    # - Adaptive Singular Value Decomposition (SVD)
    if method == "singular-value-decomposition":
        min_singular_value = method_cfg.get("min_singular_value", 1e-5)
        U, S, Vh = torch.linalg.svd(jacobian_end_effector)
        inv_s = torch.where(S > min_singular_value, 1.0 / S, torch.zeros_like(S))
        pseudoinverse = torch.transpose(Vh, 1, 2)[:, :, :6] @ torch.diagflat(inv_s) @ torch.transpose(U, 1, 2)
        return (scale * pseudoinverse @ error).squeeze(-1)
    # - Moore-Penrose pseudoinverse
    elif method == "pseudoinverse":
        pseudoinverse = torch.linalg.pinv(jacobian_end_effector)
        return (scale * pseudoinverse @ error).squeeze(-1)
    # - Transpose of matrix
    elif method == "transpose":
        transpose = torch.transpose(jacobian_end_effector, 1, 2)
        return (scale * transpose @ error).squeeze(-1)
    # - Damped Least-Squares
    elif method == "damped-least-squares":
        damping = method_cfg.get("damping", 0.05)
        transpose = torch.transpose(jacobian_end_effector, 1, 2)
        lmbda = torch.eye(jacobian_end_effector.shape[1], device=jacobian_end_effector.device) * (damping**2)
        return (scale * transpose @ torch.inverse(jacobian_end_effector @ transpose + lmbda) @ error).squeeze(-1)
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

    # Set the sphere's position to a random value within a cube of side length 0.2 centered at (0.5, 0.0, 0.5).
    random_sample = 2 * (torch.rand((3,), device=args.device) - 0.5)  # [-1, 1)
    sphere_position = torch.tensor([0.5, 0.0, 0.5], device=args.device) + 0.1 * random_sample
    sphere.set_world_poses(positions=wp.from_torch(sphere_position))

    # Reset the robot to the predefined default state.
    robot.reset_to_default_state()

    simulation_app.update()

    # Run the simulation in a loop for several iterations to allow the robot to reach the goal pose.
    # Differential IK deals with the change of robot DOF positions over time.
    for i in range(100):

        # Get intermediate values.
        current_dof_positions = wp.to_torch(robot.get_dof_positions())
        current_end_effector_position, current_end_effector_orientation = end_effector_link.get_world_poses()
        current_end_effector_position = wp.to_torch(current_end_effector_position)
        current_end_effector_orientation = wp.to_torch(current_end_effector_orientation)
        goal_position = wp.to_torch(sphere.get_world_poses()[0])

        # Get the Jacobian matrix related to the end-effector link.
        # The Franka Panda has 9 DOFs (arm: 7, hand: 2). Since we are controlling the arm, we only need the
        # first 7 values of the Jacobian matrix's last dimension (with length `num_dofs`, for fixed articulation base).
        jacobian_matrices = wp.to_torch(robot.get_jacobian_matrices())
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
        dof_position_targets = wp.from_torch(current_dof_positions[:, :7] + delta_dof_positions)
        robot.set_dof_position_targets(dof_position_targets, dof_indices=list(range(7)))

        simulation_app.update()

# Close the `SimulationApp`.
simulation_app.close()
