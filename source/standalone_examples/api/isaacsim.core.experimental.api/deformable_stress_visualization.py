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

"""Demonstrate von Mises stress visualization of a volume deformable body.

This example demonstrates how to visualize the von Mises stress of a volume deformable body during simulation.

Note: The visualization of the simulation mesh requires Fabric Scene Delegate (FSD) to be disabled.

The example illustrates the following concepts:
- Create an auto-volume deformable body and assign a deformable material to it.
- Computing and visualizing nodal von Mises stress.

The source code is organized into 4 main sections:
1. Command-line argument parsing and SimulationApp launch (common to all standalone examples).
2. Utility function for computing and visualizing nodal von Mises stress.
3. Stage creation and population.
4. Example logic.
"""

from __future__ import annotations

# Parse any command-line arguments specific to the standalone application (only known arguments).
import argparse

# 1. --------------------------------------------------------------------


parser = argparse.ArgumentParser()
parser.add_argument(
    "--show-visual-mesh",
    action="store_true",
    help="Show the visual mesh of the deformable prim rather than the simulation mesh",
)
parser.add_argument(
    "--stress-range",
    type=float,
    nargs=2,
    help=(
        "The range of stress values to use for the color gradient. "
        "If not provided, the range will be computed automatically during the simulation."
    ),
)
parser.add_argument("--test", default=False, action="store_true", help="Run in test mode")
args, _ = parser.parse_known_args()

# Launch the `SimulationApp` (see DEFAULT_LAUNCHER_CONFIG for available configuration):
# https://docs.isaacsim.omniverse.nvidia.com/latest/py/source/extensions/isaacsim.simulation_app/docs/index.html
from isaacsim import SimulationApp

extra_args = []
if not args.show_visual_mesh:
    extra_args.append("--/app/useFabricSceneDelegate=0")  # disable FSD to allow visualization of the simulation mesh
simulation_app = SimulationApp({"headless": False, "extra_args": extra_args})

# Any Omniverse level imports must occur after the `SimulationApp` class is instantiated (because APIs are provided
# by the extension/runtime plugin system, it must be loaded before they will be available to import).
import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.prim as prim_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
from isaacsim.core.experimental.materials import VolumeDeformableMaterial
from isaacsim.core.experimental.objects import GroundPlane, Mesh
from isaacsim.core.experimental.prims import DeformablePrim
from isaacsim.core.simulation_manager import SimulationManager
from pxr import Gf, Sdf, UsdGeom

# 2. --------------------------------------------------------------------

global_stress_range = [np.inf, 0.0]


def set_vertex_display_colors(
    deformable_prim: DeformablePrim, *, stress_range: tuple[float, float] | None = None
) -> None:
    """Compute and visualize per-vertex von Mises stress as a color gradient."""
    geom_prim = UsdGeom.TetMesh(prim_utils.get_prim_at_path(deformable_prim.simulation_mesh_paths[0]))
    surface_indices = UsdGeom.TetMesh.ComputeSurfaceFaces(geom_prim)
    geom_prim.CreateSurfaceFaceVertexIndicesAttr().Set(surface_indices)
    geom_prim.GetOrientationAttr().Set("leftHanded")
    points = geom_prim.GetPointsAttr().Get()
    if not points:
        raise RuntimeError(f"No points on {geom_prim.GetPath()}")

    # Compute von Mises stress per-element
    stress = deformable_prim.get_nodal_stresses().numpy()[0]
    s00, s11, s22 = stress[:, 0, 0], stress[:, 1, 1], stress[:, 2, 2]
    s01, s12, s20 = stress[:, 0, 1], stress[:, 1, 2], stress[:, 2, 0]
    von_mises = np.sqrt(0.5 * ((s00 - s11) ** 2 + (s11 - s22) ** 2 + (s22 - s00) ** 2 + 6 * (s01**2 + s12**2 + s20**2)))

    # Map von Mises stress to per-node (mesh point) values by averaging over adjacent elements
    element_indices = deformable_prim.get_element_indices()[0]
    element_indices = element_indices.numpy()[0]  # shape: (num_elements, 4)
    num_nodes = len(points)
    node_stress = np.zeros(num_nodes)
    node_count = np.zeros(num_nodes)
    np.add.at(node_stress, element_indices.ravel(), np.repeat(von_mises, 4))
    np.add.at(node_count, element_indices.ravel(), 1)
    node_stress /= np.maximum(node_count, 1)

    # Color gradient: blue (low stress) → red (high stress)
    min_stress, max_stress = node_stress.min(), node_stress.max()
    global_stress_range[0] = min(global_stress_range[0], min_stress.item())
    global_stress_range[1] = max(global_stress_range[1], max_stress.item())
    if stress_range is None:
        min_stress, max_stress = global_stress_range[0], global_stress_range[1]
    else:
        min_stress, max_stress = stress_range
    span = (max_stress - min_stress) or 1.0
    t = np.clip((node_stress - min_stress) / span, 0.0, 1.0)
    colors = [Gf.Vec3f(float(ti), 0.0, float(1.0 - ti)) for ti in t]

    primvar = UsdGeom.PrimvarsAPI(geom_prim.GetPrim()).CreatePrimvar(
        "displayColor",
        Sdf.ValueTypeNames.Color3fArray,
        UsdGeom.Tokens.vertex,  # per-vertex interpolation
    )
    primvar.Set(colors)

    # If indices were previously authored, clear them for clean vertex interpolation
    indices_attr = primvar.GetIndicesAttr()
    if indices_attr and indices_attr.HasAuthoredValueOpinion():
        indices_attr.Clear()


# 3. --------------------------------------------------------------------

# Configure simulation.
SimulationManager.set_device("cuda")
simulation_app.update()  # allow configuration to take effect

# Setup stage programatically:
# - Create a new stage
stage_utils.create_new_stage(template="sunlight")
# - Add ground plane
GroundPlane("/World/ground_plane")
# - Add the prims that compose an auto-volume deformable body (Xform prim containing a Mesh/TetMesh child prim).
stage_utils.define_prim("/World/deformable", "Xform")
mesh = Mesh("/World/deformable/mesh", primitives="Cone", positions=[0.0, 0.0, 1.5])
mesh.set_visibilities(args.show_visual_mesh)

# 4. --------------------------------------------------------------------

# Get high-level wrappers for the deformable prim.
deformable = DeformablePrim("/World/deformable", deformable_type="volume")
UsdGeom.Imageable(prim_utils.get_prim_at_path(deformable.collision_mesh_paths[0])).MakeInvisible()

# Apply a deformable physics material to support the deformable stresses computation.
material = VolumeDeformableMaterial(
    "/World/physics_material/deformable_material",
    youngs_moduli=[1e6],
    poissons_ratios=[0.3],
    dynamic_frictions=[0.5],
    static_frictions=[0.5],
)
deformable.apply_physics_materials(material)

# Play the simulation.
app_utils.play()
simulation_app.update()

for i in range(250):
    if app_utils.is_stopped():
        break
    set_vertex_display_colors(deformable, stress_range=args.stress_range)
    simulation_app.update()
    if args.test is True:
        break

print("von Mises stress range:", global_stress_range)

# Close the `SimulationApp`.
simulation_app.close()
