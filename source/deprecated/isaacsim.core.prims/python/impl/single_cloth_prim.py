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

"""Deprecated SingleClothPrim stub module."""

from typing import Any

import carb

_ERROR_MSG = (
    "SingleClothPrim is no longer available. Omniverse PhysX removed the deprecated particle-based cloth features. "
    "Please use the new deformable body API in isaacsim.core.experimental instead."
)


class SingleClothPrim:
    """Deprecated single cloth prim class. No longer available.

    Args:
        *args: Unused positional arguments.
        **kwargs: Unused keyword arguments.
    """

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        carb.log_error(_ERROR_MSG)
        raise NotImplementedError(_ERROR_MSG)
