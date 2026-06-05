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

"""Provide implementations for physics and visual material classes in Isaac Sim."""

from .non_visual_material import NonVisualMaterial as NonVisualMaterial
from .physics_materials import PhysicsMaterial as PhysicsMaterial
from .physics_materials import RigidBodyMaterial as RigidBodyMaterial
from .physics_materials import SurfaceDeformableMaterial as SurfaceDeformableMaterial
from .physics_materials import VolumeDeformableMaterial as VolumeDeformableMaterial
from .visual_materials import OmniGlassMaterial as OmniGlassMaterial
from .visual_materials import OmniPbrMaterial as OmniPbrMaterial
from .visual_materials import PreviewSurfaceMaterial as PreviewSurfaceMaterial
from .visual_materials import VisualMaterial as VisualMaterial
