# SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Testing utilities for comparing articulated USD assets."""

from __future__ import annotations

import gc
import itertools
from collections.abc import Callable
from typing import Any

import carb
import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
import omni.timeline
import warp as wp
from isaacsim.core.experimental.prims import Articulation
from pxr import Gf, UsdGeom


async def compare_usd_files(paths: list[str]) -> bool:
    """Compare USD files by loading and validating articulation properties.

    Args:
        paths: List of USD file paths to compare.

    Returns:
        True if all compared articulation properties and values match, False otherwise.

    Raises:
        RuntimeError: If a USD reference cannot be added to the comparison stage.
    """
    stage = stage_utils.create_new_stage()
    prims = []
    for i in range(len(paths)):
        prim = stage_utils.add_reference_to_stage(paths[i], f"/robot_{i}", variants=[("Physics", "physx")])
        if not prim or not prim.IsValid():
            raise RuntimeError(f"Failed to add reference for path: {paths[i]}")
        xform = UsdGeom.Xformable(prim)
        xform.ClearXformOpOrder()
        xform.AddTranslateOp().Set(Gf.Vec3d(0.0, 10.0 * i, 0.0))
        await omni.kit.app.get_app().next_update_async()
        prims.append(Articulation(f"/robot_{i}"))

    omni.timeline.get_timeline_interface().play()
    await omni.kit.app.get_app().next_update_async()
    await omni.kit.app.get_app().next_update_async()

    status = True

    carb.log_info("\n-----------------------------")
    carb.log_info("Checking common properties...")
    carb.log_info("-----------------------------")

    status &= compare_articulation_properties(prims, Articulation.num_dofs)
    status &= compare_articulation_properties(prims, Articulation.dof_names)
    status &= compare_articulation_properties(prims, Articulation.dof_types)
    # - joints
    status &= compare_articulation_properties(prims, Articulation.num_joints)
    status &= compare_articulation_properties(prims, Articulation.joint_names)
    status &= compare_articulation_properties(prims, Articulation.joint_types)
    # - links
    status &= compare_articulation_properties(prims, Articulation.num_links)
    status &= compare_articulation_properties(prims, Articulation.link_names)

    # Check per-DOF values
    carb.log_info("\n--------------------------")
    carb.log_info("Checking per-DOF values...")
    carb.log_info("--------------------------")
    common_dof_names = set(prims[0].dof_names)
    for prim in prims[1:]:
        common_dof_names = common_dof_names.intersection(set(prim.dof_names))
    if not common_dof_names:
        carb.log_info("No common DOF names found")
    for dof_name in common_dof_names:
        indices = [{"dof_indices": prim.get_dof_indices(dof_name)} for prim in prims]
        msg = [f"{dof_name}:{index['dof_indices']}" for index in indices]
        # methods
        status &= compare_articulation_properties(prims, Articulation.get_dof_armatures, member_kwargs=indices, msg=msg)
        status &= compare_articulation_properties(
            prims, Articulation.get_dof_drive_model_properties, member_kwargs=indices, msg=msg
        )
        status &= compare_articulation_properties(
            prims, Articulation.get_dof_drive_types, member_kwargs=indices, msg=msg
        )
        status &= compare_articulation_properties(
            prims, Articulation.get_dof_friction_properties, member_kwargs=indices, msg=msg
        )
        status &= compare_articulation_properties(prims, Articulation.get_dof_gains, member_kwargs=indices, msg=msg)
        status &= compare_articulation_properties(prims, Articulation.get_dof_limits, member_kwargs=indices, msg=msg)
        status &= compare_articulation_properties(
            prims, Articulation.get_dof_max_efforts, member_kwargs=indices, msg=msg
        )
        status &= compare_articulation_properties(
            prims, Articulation.get_dof_max_velocities, member_kwargs=indices, msg=msg
        )

    # Check per-link values
    carb.log_info("\n---------------------------")
    carb.log_info("Checking per-link values...")
    carb.log_info("---------------------------")
    common_link_names = set(prims[0].link_names)
    for prim in prims[1:]:
        common_link_names = common_link_names.intersection(set(prim.link_names))
    if not common_link_names:
        carb.log_info("No common link names found")
    for link_name in common_link_names:
        indices = [{"link_indices": prim.get_link_indices(link_name)} for prim in prims]
        msg = [f"{link_name}:{index['link_indices']}" for index in indices]
        # methods
        status &= compare_articulation_properties(prims, Articulation.get_link_coms, member_kwargs=indices, msg=msg)
        status &= compare_articulation_properties(
            prims, Articulation.get_link_enabled_gravities, member_kwargs=indices, msg=msg
        )
        status &= compare_articulation_properties(prims, Articulation.get_link_inertias, member_kwargs=indices, msg=msg)
        status &= compare_articulation_properties(prims, Articulation.get_link_masses, member_kwargs=indices, msg=msg)

    # Check other values
    carb.log_info("\n------------------------")
    carb.log_info("Checking other values...")
    carb.log_info("------------------------")

    # Dispatch the self-collision check based on which articulation schema is authored:
    # ``PhysxArticulationAPI`` is read via the Articulation view; ``NewtonArticulationRootAPI``
    # exposes the value as the ``newton:selfCollisionEnabled`` attribute on the root prim.
    stage = stage_utils.get_current_stage()

    def _articulation_has_api(articulation: Articulation, api_name: str) -> bool:
        for path in articulation.paths:
            prim = stage.GetPrimAtPath(path)
            if prim and prim.IsValid() and prim.HasAPI(api_name):
                return True
        return False

    if all(_articulation_has_api(p, "PhysxArticulationAPI") for p in prims):
        status &= compare_articulation_properties(prims, Articulation.get_enabled_self_collisions)
        status &= compare_articulation_properties(prims, Articulation.get_sleep_thresholds)
        status &= compare_articulation_properties(prims, Articulation.get_solver_iteration_counts)
        status &= compare_articulation_properties(prims, Articulation.get_stabilization_thresholds)

    elif all(_articulation_has_api(p, "NewtonArticulationRootAPI") for p in prims):

        def get_newton_self_collisions(articulation: Articulation) -> list[bool | None]:
            values: list[bool | None] = []
            for path in articulation.paths:
                prim = stage.GetPrimAtPath(path)
                attr = prim.GetAttribute("newton:selfCollisionEnabled") if prim and prim.IsValid() else None
                values.append(bool(attr.Get()) if attr and attr.IsValid() else None)
            return values

        status &= compare_articulation_properties(prims, get_newton_self_collisions)
    else:
        carb.log_info("Skipping self-collision comparison: no PhysxArticulationAPI or NewtonArticulationRootAPI found")

    stage = None
    gc.collect()
    return status


def compare_articulation_properties(
    prims: list[Articulation],
    member: property | Callable[..., Any],
    member_args: list[tuple] | None = None,
    member_kwargs: list[dict[str, Any]] | None = None,
    msg: list[str] | None = None,
) -> bool:
    """Compare a member value/method result across articulation prims.

    Args:
        prims: Articulation wrapper objects to compare.
        member: Property or bound method descriptor to evaluate on each prim.
        member_args: Optional positional args per prim for method calls.
        member_kwargs: Optional keyword args per prim for method calls.
        msg: Optional labels used in mismatch logging.

    Returns:
        True if all pairwise comparisons match, False otherwise.
    """

    def show_mismatch(member_name: str, *, i: int, x: Any, j: int, y: Any) -> None:
        """Log a property mismatch between two articulation prims.

        Args:
            member_name: Name of the property that mismatched.
            i: Index of the first prim.
            x: Value from the first prim.
            j: Index of the second prim.
            y: Value from the second prim.
        """
        if msg is None:
            carb.log_info(f"\n'{member_name}' mismatch")
        else:
            carb.log_info(f"\n'{member_name}' mismatch for {msg[i]} and {msg[j]}")
        carb.log_info(f"  - {x}")
        carb.log_info(f"  - {y}")

    def check_values(x: Any, y: Any) -> tuple[bool, Any, Any]:
        """Recursively compare two values, handling tuples and warp arrays.

        Args:
            x: First value to compare.
            y: Second value to compare.

        Returns:
            Tuple of (match_status, processed_x, processed_y).
        """
        # tuple
        if isinstance(x, tuple):
            status = True
            new_x, new_y = [], []
            for xx, yy in zip(x, y):
                result, xx, yy = check_values(xx, yy)
                new_x.append(xx)
                new_y.append(yy)
                if not result:
                    status = False
                    break
            return status, tuple(new_x), tuple(new_y)
        # Warp arrays
        elif isinstance(x, wp.array):
            x = x.numpy()
            y = y.numpy()
            return np.allclose(x, y, rtol=1e-03, atol=1e-05), x, y
        # generic Python types
        elif x != y:
            return False, x, y
        return True, x, y

    status = True
    member_getter = member.fget if isinstance(member, property) else member
    if member_getter is None:
        return False
    member_name = member_getter.__name__
    member_args = [()] * len(prims) if member_args is None else member_args
    member_kwargs = [{}] * len(prims) if member_kwargs is None else member_kwargs
    results = [member_getter(prim, *member_args[i], **member_kwargs[i]) for i, prim in enumerate(prims)]
    for (i, x), (j, y) in itertools.combinations(enumerate(results), 2):
        result, x, y = check_values(x, y)
        if not result:
            status = False
            show_mismatch(member_name, i=i, x=x, j=j, y=y)
    return status
