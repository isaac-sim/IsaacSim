# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Exercise shared tensor-view contracts against the Newton backend.

The driver selects Newton and the public Warp frontend. Entity-view operations
that Newton does not implement remain absent from the registry rather than
being represented by placeholder capability claims.
"""

from __future__ import annotations

import _physics_setup  # noqa: F401
import isaacsim.physics_engines.ovnewton.impl.tensors  # noqa: F401
from common.tensor_registry_tests import (
    LegacyMethodDelegationMixin,
    TensorRegistryBasicsMixin,
)


class TestNewtonTensorRegistryBasics(TensorRegistryBasicsMixin):
    """Run the shared tensor-registry contract against Newton and Warp."""

    backend = "newton"
    frontend = "warp"


class TestNewtonLegacyMethodDelegation(LegacyMethodDelegationMixin):
    """Verify Newton compatibility-method delegation through the Warp adapter."""

    backend = "newton"
    adapter_module = "isaacsim.physics_engines.ovnewton.impl.tensors.entity_factories"
