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

# from ._mesh_bounds import compute_mesh_convex_hull

"""Utilities for collision detection and geometric approximation including bounding volumes and mesh processing."""

from .bounding_geometries import AABB, OBB, ConvexHull
from .bounds import compute_obb, compute_world_aabb, create_bbox_cache
from .triangulate_mesh import triangulate_mesh
