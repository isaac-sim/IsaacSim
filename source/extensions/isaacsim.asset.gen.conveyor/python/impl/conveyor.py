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

"""Conveyor belt creation utilities."""

from __future__ import annotations

__all__ = ["create_conveyor_belt"]

import omni
import omni.graph.core as og
import pxr
from pxr import PhysxSchema, Usd, UsdGeom, UsdPhysics


def create_conveyor_belt(
    stage: Usd.Stage,
    conveyor_prim: Usd.Prim,
    prim_name: str = "ConveyorBeltGraph",
) -> Usd.Prim:
    """Create a conveyor belt action graph on a rigid body prim.

    Creates an action graph containing OnPlaybackTick, IsaacConveyor, and
    ReadVariable nodes wired to the given prim. If the prim does not already
    have a rigid body API, the function walks up the hierarchy to find one
    or applies RigidBodyAPI, CollisionAPI, and PhysxSurfaceVelocityAPI. For
    ``UsdGeomMesh`` prims it additionally applies ``UsdPhysics.MeshCollisionAPI``
    with the ``convexHull`` approximation, because PhysX rejects the default
    ``meshSimplification`` (triangle mesh) approximation on dynamic bodies
    and emits an error per parse otherwise. Non-mesh primitives
    (Cube/Sphere/Cylinder/Capsule) resolve to their analytic collision shapes
    and do not need (or accept) ``MeshCollisionAPI``.

    Args:
        stage: The USD stage.
        conveyor_prim: The rigid body prim to apply the conveyor belt to.
        prim_name: Name for the conveyor belt graph prim.

    Returns:
        The created conveyor node prim.
    """
    if not conveyor_prim.HasAPI(UsdPhysics.RigidBodyAPI):
        found = False
        ancestor = conveyor_prim.GetParent()
        while ancestor and ancestor.IsValid():
            if ancestor.HasAPI(UsdPhysics.RigidBodyAPI):
                conveyor_prim = ancestor
                found = True
                break
            ancestor = ancestor.GetParent()
        if not found:
            UsdPhysics.RigidBodyAPI.Apply(conveyor_prim)
            UsdPhysics.CollisionAPI.Apply(conveyor_prim)
            # MeshCollisionAPI applies only to UsdGeomMesh per the schema's
            # ``appliesTo`` clause. Applying it to a Cube/Sphere/Cylinder
            # would author a spec PhysX ignores and produce noisy USD output.
            if conveyor_prim.IsA(UsdGeom.Mesh):
                mesh_collision_api = UsdPhysics.MeshCollisionAPI.Apply(conveyor_prim)
                mesh_collision_api.CreateApproximationAttr().Set("convexHull")
            PhysxSchema.PhysxSurfaceVelocityAPI.Apply(conveyor_prim)

    base_path = conveyor_prim.GetPath()
    if conveyor_prim != stage.GetDefaultPrim():
        base_path = conveyor_prim.GetParent().GetPath()
    graph_path = omni.usd.get_stage_next_free_path(
        stage, base_path.AppendChild(pxr.Tf.MakeValidIdentifier(prim_name)), True
    )

    keys = og.Controller.Keys
    conveyor_node_name = "ConveyorNode"
    og.Controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            keys.CREATE_VARIABLES: [("Velocity", "float")],
            keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnPlaybackTick"),
                (conveyor_node_name, "isaacsim.asset.gen.conveyor.IsaacConveyor"),
                ("read_speed", "omni.graph.core.ReadVariable"),
            ],
            keys.SET_VALUES: [
                ("read_speed.inputs:graph", graph_path),
                ("read_speed.inputs:variableName", "Velocity"),
            ],
            keys.CONNECT: [
                ("OnTick.outputs:tick", f"{conveyor_node_name}.inputs:onStep"),
                ("OnTick.outputs:deltaSeconds", f"{conveyor_node_name}.inputs:delta"),
                ("read_speed.outputs:value", f"{conveyor_node_name}.inputs:velocity"),
            ],
        },
    )

    conveyor_node = stage.GetPrimAtPath(graph_path + "/" + conveyor_node_name)
    input_rel = conveyor_node.GetRelationship("inputs:conveyorPrim")
    if not input_rel:
        input_rel = conveyor_node.CreateRelationship("inputs:conveyorPrim")
    input_rel.SetTargets([conveyor_prim.GetPath()])

    return conveyor_node
