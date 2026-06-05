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

# New focused recorders (no legacy code)

"""Data recorders for capturing various performance and system metrics during Isaac Sim benchmark execution."""

from .app_frametime import AppFrametimeRecorder
from .cpu_continuous import CPUContinuousRecorder
from .gpu_frametime import GPUFrametimeRecorder
from .hardware import HardwareSpecRecorder
from .interface import InputContext, MeasurementData, MeasurementDataRecorder, MeasurementDataRecorderRegistry
from .memory import MemoryRecorder
from .physics_frametime import PhysicsFrametimeRecorder
from .physics_step_interval import PhysicsStepIntervalRecorder
from .render_frametime import RenderFrametimeRecorder
from .runtime import RuntimeRecorder
from .stats_utils import Stats

__all__ = [
    # Base classes
    "MeasurementDataRecorder",
    "MeasurementDataRecorderRegistry",
    "MeasurementData",
    "InputContext",
    # Frametime recorders
    "AppFrametimeRecorder",
    "PhysicsFrametimeRecorder",
    "PhysicsStepIntervalRecorder",
    "GPUFrametimeRecorder",
    "RenderFrametimeRecorder",
    # System recorders
    "CPUContinuousRecorder",
    "MemoryRecorder",
    # Utility recorders
    "RuntimeRecorder",
    "HardwareSpecRecorder",
    # Statistics
    "Stats",
]
