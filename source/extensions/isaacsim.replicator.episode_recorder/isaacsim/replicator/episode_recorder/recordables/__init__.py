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

"""Register built-in recordable implementations.

This package covers sim time, articulations, xforms, rigid bodies, cameras, and arbitrary
USD attributes. Importing it registers all built-in recordables with the registry.
"""

from __future__ import annotations

from .articulation import ArticulationRecordable
from .attribute import AttributeRecordable
from .camera import CameraRecordable
from .rigid_body import RigidBodyRecordable
from .sim_time import SimTimeRecordable
from .xform import XformRecordable

__all__ = [
    "ArticulationRecordable",
    "AttributeRecordable",
    "CameraRecordable",
    "RigidBodyRecordable",
    "SimTimeRecordable",
    "XformRecordable",
]
