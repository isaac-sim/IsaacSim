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

"""Module defining attribute constants for domain randomization across simulation contexts, rigid bodies, articulations, and tendons."""

SIMULATION_CONTEXT_ATTRIBUTES = ["gravity"]

RIGID_PRIM_ATTRIBUTES = [
    "angular_velocity",
    "linear_velocity",
    "velocity",
    "position",
    "orientation",
    "force",
    "mass",
    "inertia",
    "material_properties",
    "contact_offset",
    "rest_offset",
]

ARTICULATION_ATTRIBUTES = [
    "stiffness",
    "damping",
    "joint_friction",
    "position",
    "orientation",
    "linear_velocity",
    "angular_velocity",
    "velocity",
    "joint_positions",
    "joint_velocities",
    "lower_dof_limits",
    "upper_dof_limits",
    "max_efforts",
    "joint_armatures",
    "joint_max_velocities",
    "joint_efforts",
    "body_masses",
    "body_inertias",
    "material_properties",
    "contact_offset",
    "rest_offset",
    "tendon_stiffnesses",
    "tendon_dampings",
    "tendon_limit_stiffnesses",
    "tendon_lower_limits",
    "tendon_upper_limits",
    "tendon_rest_lengths",
    "tendon_offsets",
]

TENDON_ATTRIBUTES = [
    "tendon_stiffnesses",
    "tendon_dampings",
    "tendon_limit_stiffnesses",
    "tendon_lower_limits",
    "tendon_upper_limits",
    "tendon_rest_lengths",
    "tendon_offsets",
]
