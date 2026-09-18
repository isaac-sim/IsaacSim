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

"""Provide the physics engine registration API.

Nanobind bindings expose shared data types, engine callback groups, and
registration operations for both runtime simulations and tensor entity or
simulation-view factories.
"""

from isaacsim.physics.registration.bindings._bindings import *  # noqa: F401,F403

__all__ = [
    # Runtime register API + shared vocabulary
    "register_simulation",
    "unregister_simulation",
    "get_simulation",
    "get_simulation_name",
    "get_num_simulations",
    "get_simulation_ids",
    "activate_simulation",
    "deactivate_simulation",
    "is_simulation_active",
    "get_active_simulation_id",
    "subscribe_simulation_registry_events",
    "Simulation",
    "SimulationId",
    "SimulationFns",
    "SceneQueryFns",
    "InteractionFns",
    "BenchmarkFns",
    "SimulationRegistryEventType",
    "ContactEventType",
    "ContactEventHeader",
    "ContactData",
    "FrictionAnchor",
    "ContactEventHeaderVector",
    "ContactDataVector",
    "FrictionAnchorsDataVector",
    "ForceMode",
    "PhysicsStepContext",
    "SceneQueryHitObject",
    "SceneQueryHitLocation",
    "OverlapHit",
    "RaycastHit",
    "SweepHit",
    "PhysicsProfileStats",
    "Subscription",
    "Float3",
    "Float4",
    "k_invalid_simulation_id",
    "k_invalid_subscription_id",
    # Tensor register API + vocabulary
    "DType",
    "DeviceKind",
    "ImplKind",
    "ObjectType",
    "JointType",
    "DofType",
    "DofMotion",
    "DofDriveType",
    "TensorSpec",
    "TensorDesc",
    "TensorRegistry",
    "get_registry",
    "register_entity",
]
