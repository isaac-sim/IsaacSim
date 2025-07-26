# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
from dataclasses import dataclass

import carb
from omni.physx import get_physx_property_query_interface
from omni.physx.bindings._physx import PhysxPropertyQueryMode, PhysxPropertyQueryResult
from pxr import Gf, PhysicsSchemaTools, Sdf, Usd, UsdUtils


@dataclass
class BodyInfo:
    """Class to query and store mass properties of a rigid body prim.

    Attributes:
        valid (bool): Whether the query was successful
        name (str): Name of the rigid body
        mass (float): Mass of the rigid body
        diagonal_inertia (Gf.Vec3d): Diagonal elements of the inertia tensor
        principal_axes (Gf.Quatf): Quaternion representing principal axes orientation
        center_of_mass (Gf.Vec3d): Center of mass position
        done (bool): Whether the query has completed
        stage (Usd.Stage): USD stage containing the prim
        prim_path (Sdf.Path): Path to the rigid body prim
    """

    stage: Usd.Stage
    prim_path: Sdf.Path
    valid: bool = False
    name: str = None
    mass: float = None
    diagonal_inertia: Gf.Vec3d = None
    principal_axes: Gf.Quatf = None
    center_of_mass: Gf.Vec3d = None
    done: bool = False

    def __post_init__(self):
        """Initialize the object by querying mass properties."""
        self.query_prim_mass()

    def query_prim_mass(self):
        """Query mass properties of the rigid body prim.

        Uses PhysX property query interface to get mass, inertia, principal axes,
        and center of mass of the rigid body. Updates the object attributes with
        the query results.
        """
        stage_id = UsdUtils.StageCache().Get().GetId(self.stage).ToLongInt()
        rigid_body_prim = self.stage.GetPrimAtPath(self.prim_path)
        query_prim_id = PhysicsSchemaTools.sdfPathToInt(rigid_body_prim.GetPath())

        def rigid_body_fn(rigid_info):

            if rigid_info.result == PhysxPropertyQueryResult.VALID:
                self.mass = rigid_info.mass
                self.diagonal_inertia = Gf.Vec3d(*rigid_info.inertia)
                self.principal_axes = Gf.Quatf(rigid_info.principal_axes[3], *[rigid_info.principal_axes[0:3]])
                self.center_of_mass = Gf.Vec3d(*rigid_info.center_of_mass)
                self.valid = True
            else:
                carb.log_error(f"PhysX rigid body property query failed for prim {self}")
            self.done = True

        get_physx_property_query_interface().query_prim(
            stage_id=stage_id,
            prim_id=query_prim_id,
            query_mode=PhysxPropertyQueryMode.QUERY_RIGID_BODY_WITH_COLLIDERS,
            rigid_body_fn=rigid_body_fn,
        )
        return self


def query_prims(stage: Usd.Stage, prim_paths: list[Sdf.Path]):
    """Query mass properties for multiple prims.

    Args:
        stage (Usd.Stage): USD stage containing the prims
        prim_paths (list[Sdf.Path]): List of prim paths to query

    Returns:
        dict: Dictionary mapping prim paths to BodyInfo objects containing mass properties
    """
    return {prim_path: BodyInfo(stage, prim_path) for prim_path in prim_paths}
