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

# preload USD libraries
import isaacsim.foundation.usd.openusd  # noqa: F401 - initialize OpenUSD bindings
import isaacsim.foundation.usd.ovstage  # noqa: F401 - initialize OVStage bindings

# register third-party USD schemas
import newton_usd_schemas  # noqa: F401 - register Newton schemas
import physx_usd_schemas  # noqa: F401 - register PhysX schemas

from . import control, data

__all__ = [
    "control",
    "data",
]
