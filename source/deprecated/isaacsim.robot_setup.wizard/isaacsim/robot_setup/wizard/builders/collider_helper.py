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

"""Backend of "Add Colliders".

- parse colliders that are already on stage
- update colliders if users made changes in the UI

"""

import omni.usd
from pxr import PhysxSchema, Usd, UsdGeom, UsdPhysics

MESH_TYPES = ["Mesh", "Cube", "Sphere", "Cylinder", "Cone", "Capsule"]
MESH_APPROXIMATIONS = {
    "triangleMesh": PhysxSchema.PhysxTriangleMeshCollisionAPI,
    "convexHull": PhysxSchema.PhysxConvexHullCollisionAPI,
    "convexDecomposition": PhysxSchema.PhysxConvexDecompositionCollisionAPI,
    "meshSimplification": PhysxSchema.PhysxTriangleMeshSimplificationCollisionAPI,
    "convexMeshSimplification": PhysxSchema.PhysxTriangleMeshSimplificationCollisionAPI,
    "boundingCube": None,
    "boundingSphere": None,
    "sphereFill": PhysxSchema.PhysxSphereFillCollisionAPI,
    "sdf": PhysxSchema.PhysxSDFMeshCollisionAPI,
}


def apply_collider(link_name: str, approximation_type: str) -> None:
    """Applies collision detection to a mesh prim based on the specified approximation type.

    Retrieves the mesh prim from the colliders path and applies the appropriate collision APIs,
    including specialized approximation methods like convex hull or triangle mesh.

    Args:
        link_name: Name of the robot link to apply collision to.
        approximation_type: Type of collision approximation to use (e.g., "triangleMesh", "convexHull").

    Raises:
        ValueError: If the mesh prim does not exist at the expected path.
    """
    # get the prim from the link name, if doesn't exist, throw an error for now,
    ## TODO: in the future, you can create a new one of a limit shape (need to add scale accordingly) and redo the referencing inside the robot prim
    stage = omni.usd.get_context().get_stage()
    mesh_prim_path = f"/colliders/{link_name}"
    mesh_prim = stage.GetPrimAtPath(mesh_prim_path)
    if not mesh_prim:
        raise ValueError(f"Mesh prim {mesh_prim_path} does not exist")

    # apply Collision API to the mesh prim
    UsdPhysics.CollisionAPI.Apply(mesh_prim)
    PhysxSchema.PhysxCollisionAPI.Apply(mesh_prim)

    # apply any special collider API if any, otherwise the generic collision API applies.
    if approximation_type in MESH_APPROXIMATIONS and approximation_type not in [
        "boundingCube",
        "boundingSphere",
        "Mesh",
    ]:
        mesh_prim.ApplyAPI(MESH_APPROXIMATIONS[approximation_type])


### copied from omni.extensions.runtime: /omni/physx/utils.py
def remove_collider(prim: Usd.Prim) -> None:
    """Removes all collision-related APIs from a USD prim.

    Removes the CollisionAPI, PhysxCollisionAPI, and any mesh-specific collision APIs
    like convex hull or triangle mesh collision from the specified prim.

    Args:
        prim: The USD prim to remove collision APIs from.
    """
    ret = prim.RemoveAPI(UsdPhysics.CollisionAPI)
    prim.RemoveAPI(PhysxSchema.PhysxCollisionAPI)
    if prim.IsA(UsdGeom.Mesh):
        prim.RemoveAPI(UsdPhysics.MeshCollisionAPI)
        prim.RemoveAPI(PhysxSchema.PhysxConvexHullCollisionAPI)
        prim.RemoveAPI(PhysxSchema.PhysxConvexDecompositionCollisionAPI)
        prim.RemoveAPI(PhysxSchema.PhysxTriangleMeshSimplificationCollisionAPI)
        prim.RemoveAPI(PhysxSchema.PhysxTriangleMeshCollisionAPI)
