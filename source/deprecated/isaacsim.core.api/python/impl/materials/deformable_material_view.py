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

"""Stub module for the removed DeformableMaterialView class."""

from __future__ import annotations

from typing import Any

import carb

_ERROR_MSG = (
    "DeformableMaterialView is no longer available. Omniverse PhysX removed the deprecated deformable body features "
    "that this material depended on. Please use the new material APIs in isaacsim.core.experimental.materials instead."
)


class DeformableMaterialView:
    """Stub for the removed DeformableMaterialView class.

    DeformableMaterialView is no longer available because Omniverse PhysX removed
    the deprecated deformable body features it depended on. Use the new material
    APIs in isaacsim.core.experimental.materials instead.

    Args:
        *args: Positional arguments (ignored; class always raises NotImplementedError).
        **kwargs: Keyword arguments (ignored; class always raises NotImplementedError).

    """

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        carb.log_error(_ERROR_MSG)
        raise NotImplementedError(_ERROR_MSG)
