# SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Command for merging multiple meshes into a single mesh."""

import omni

from .mesh_merger import MeshMerger


class MergeMeshesCommand(omni.kit.commands.Command):
    """Command to merge selected meshes and all children of given prims into a single mesh.

    This command provides options to control the merge behavior:

    - Clear Parent Transform: Sets the mesh origin at world origin, otherwise origin is
      the same as the first element.
    - Deactivate source assets: Sets source prims to Inactive after performing the merge operation.
    - Combine Materials: Redirects all assets materials to a given folder, and every geomsubset
      that shares a same material name uses the same material, each geom subset uses the original
      material from the source assets.

    Args:
        source: List of prim paths to merge.
        clear_transform: If True, sets the merged mesh origin at world origin.
            Otherwise, the origin is the same as the first element.
        deactivate_source: If True, deactivates source prims after merging.
        combine_materials: If True, redirects all materials to a single folder
            and shares materials with the same name.
        materials_destination: The prim path where combined materials will be stored.

    Example:

    .. code-block:: python

        >>> result, prim = omni.kit.commands.execute(
        ...     "MergeMeshesCommand",
        ...     source=["/World/Cube", "/World/Cone", "/World/ManyToruses"],
        ...     clear_transform=False,
        ...     deactivate_source=True,
        ...     combine_materials=True,
        ...     materials_destination="/World/Looks",
        ... )

    """

    def __init__(
        self,
        source: str,
        clear_transform: bool = False,
        deactivate_source: bool = False,
        combine_materials: bool = False,
        materials_destination: str = "/World/Looks",
    ) -> None:
        self._stage = omni.usd.get_context().get_stage()
        self.mesh_merger = MeshMerger(self._stage)
        self.mesh_merger.clear_parent_xform = clear_transform
        self.mesh_merger.deactivate_source = deactivate_source
        self.mesh_merger.combine_materials = combine_materials
        self.mesh_merger.materials_destination = materials_destination
        self.mesh_merger.update_selection(selection=source, stage=self._stage)

        self.mesh_merger.output_mesh = "/Merged/" + str(self._stage.GetPrimAtPath(source[0]).GetName())

    def do(self) -> str:
        """Execute the mesh merge operation.

        Returns:
            The prim path of the merged mesh.

        """
        self.mesh_merger.merge_meshes()

        return self.mesh_merger.output_mesh

    def undo(self) -> None:
        """Undo the mesh merge operation.

        Reactivates source prims if they were deactivated, removes the merged mesh,
        and removes any materials that were created during the merge.
        """
        self.mesh_merger.reactivate_sources()
        self._stage.RemovePrim(self.mesh_merger.output_mesh)
        self.mesh_merger.remove_created_materials()
