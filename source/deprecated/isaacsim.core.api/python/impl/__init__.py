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

import builtins

from isaacsim.core.api.physics_context.physics_context import PhysicsContext as PhysicsContext
from isaacsim.core.api.simulation_context.simulation_context import SimulationContext as SimulationContext
from isaacsim.core.api.world.world import World as World

# In case we are running from a regular kit instance and not a simulation_app, this variable is not defined.
if not hasattr(builtins, "ISAAC_LAUNCHED_FROM_TERMINAL"):
    builtins.ISAAC_LAUNCHED_FROM_TERMINAL = True
