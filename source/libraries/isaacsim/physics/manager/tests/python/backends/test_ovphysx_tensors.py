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

"""Exercise shared tensor-view mixins against the OvPhysX backend.

Importing ``_physics_setup`` registers the OvPhysX entity factories, so the
tests use the public tensor registry directly.
"""

from __future__ import annotations

import _physics_setup  # noqa: F401  -- auto-registers ovphysx C++ factory
from common.tensor_registry_tests import TensorRegistryBasicsMixin


class TestOvPhysxTensorRegistryBasics(TensorRegistryBasicsMixin):
    """Run the shared tensor-registry contract against OvPhysX and Warp."""

    backend = "ovphysx"
    frontend = "warp"
    unsupported_entity_types = ()
