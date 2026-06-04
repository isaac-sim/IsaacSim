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

"""Implementation module for various prim classes including geometry, physics, articulation, and transformation primitives."""

from .articulation import Articulation as Articulation
from .cloth_prim import ClothPrim as ClothPrim
from .deformable_prim import DeformablePrim as DeformablePrim
from .geometry_prim import GeometryPrim as GeometryPrim
from .particle_system import ParticleSystem as ParticleSystem
from .rigid_prim import RigidPrim as RigidPrim
from .sdf_shape_prim import SdfShapePrim as SdfShapePrim
from .single_articulation import SingleArticulation as SingleArticulation
from .single_cloth_prim import SingleClothPrim as SingleClothPrim
from .single_deformable_prim import SingleDeformablePrim as SingleDeformablePrim
from .single_geometry_prim import SingleGeometryPrim as SingleGeometryPrim
from .single_particle_system import SingleParticleSystem as SingleParticleSystem
from .single_rigid_prim import SingleRigidPrim as SingleRigidPrim
from .single_xform_prim import SingleXFormPrim as SingleXFormPrim
from .xform_prim import XFormPrim as XFormPrim
