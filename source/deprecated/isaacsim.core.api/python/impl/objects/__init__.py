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

"""API for creating and managing primitive geometric objects in Isaac Sim."""

from isaacsim.core.api.objects.capsule import DynamicCapsule as DynamicCapsule
from isaacsim.core.api.objects.capsule import FixedCapsule as FixedCapsule
from isaacsim.core.api.objects.capsule import VisualCapsule as VisualCapsule
from isaacsim.core.api.objects.cone import DynamicCone as DynamicCone
from isaacsim.core.api.objects.cone import FixedCone as FixedCone
from isaacsim.core.api.objects.cone import VisualCone as VisualCone
from isaacsim.core.api.objects.cuboid import DynamicCuboid as DynamicCuboid
from isaacsim.core.api.objects.cuboid import FixedCuboid as FixedCuboid
from isaacsim.core.api.objects.cuboid import VisualCuboid as VisualCuboid
from isaacsim.core.api.objects.cylinder import DynamicCylinder as DynamicCylinder
from isaacsim.core.api.objects.cylinder import FixedCylinder as FixedCylinder
from isaacsim.core.api.objects.cylinder import VisualCylinder as VisualCylinder
from isaacsim.core.api.objects.ground_plane import GroundPlane as GroundPlane
from isaacsim.core.api.objects.sphere import DynamicSphere as DynamicSphere
from isaacsim.core.api.objects.sphere import FixedSphere as FixedSphere
from isaacsim.core.api.objects.sphere import VisualSphere as VisualSphere
