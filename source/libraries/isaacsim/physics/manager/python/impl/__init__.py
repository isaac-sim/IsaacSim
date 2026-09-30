# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Provide the backend-neutral physics simulation manager.

Nanobind bindings provide simulation lifecycle, stepping, scene queries,
interaction, benchmarking, and tensor-view creation. Shared data types and the
engine registration API live in ``isaacsim.physics.registration``.

The ``create_entity`` exposed here is the raw binding: it returns a view whose
reads hand back DLPack arrays. Applications working in Warp should use
``isaacsim.physics.manager.impl.tensors.create_entity`` instead, which wraps the
same factory with Warp dispatch, device handling, and path validation.
"""

from isaacsim.physics.manager.bindings._bindings import *  # noqa: F401,F403

# Re-export the registration API and shared vocabulary into this namespace, but
# not into `__all__`, so `isaacsim.physics.manager.impl.<name>` remains
# import-compatible. Their canonical public home is
# `isaacsim.physics.registration`; the public `isaacsim.physics.manager` facade
# exposes only application-facing operations.
from isaacsim.physics.registration.impl import *  # noqa: F401,F403,E402

from .physics_event import PhysicsEvent
from .physics_manager import PhysicsManager

__all__ = [
    "PhysicsManager",
    "PhysicsEvent",
    "initialize",
    "close",
    "get_attached_stage",
    "simulate_async",
    "simulate",
    "fetch_results",
    "check_results",
    "publish_transforms_to_stage",
    "flush_changes",
    "pause_change_tracking",
    "is_change_tracking_paused",
    "subscribe_physics_contact_report_events",
    "get_simulation_time_steps_per_second",
    "get_simulation_timestamp",
    "get_simulation_step_count",
    "subscribe_physics_on_step_events",
    "is_capable_of_simulating",
    "raycast_closest",
    "raycast_any",
    "raycast_all",
    "sweep_sphere_closest",
    "sweep_sphere_any",
    "sweep_sphere_all",
    "overlap_sphere",
    "overlap_sphere_any",
    "overlap_box",
    "overlap_box_any",
    "overlap_shape",
    "overlap_shape_any",
    "sweep_box_closest",
    "sweep_shape_closest",
    "sweep_box_any",
    "sweep_shape_any",
    "sweep_box_all",
    "sweep_shape_all",
    "handle_raycast",
    "get_prim_debug_data",
    "subscribe_profile_stats_events",
    "PhysicsProfileStats",
    "DebugDataItemType",
    "create_entity",
]
