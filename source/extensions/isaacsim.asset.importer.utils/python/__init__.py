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

"""Public API for the asset importer utilities.

Re-exports the contents of every implementation submodule so callers can use
short top-level imports (matching the documented ``python_api.md`` surface)::

    from isaacsim.asset.importer.utils import collision_from_visuals, parse_robot_name

The implementation submodules remain accessible via their original names::

    from isaacsim.asset.importer.utils import importer_utils
    importer_utils.collision_from_visuals(...)
"""

from .impl import asset_utils, importer_utils, merge_mesh_utils, physx_types, stage_utils
from .impl.asset_utils import *  # noqa: F401,F403
from .impl.importer_utils import *  # noqa: F401,F403
from .impl.merge_mesh_utils import *  # noqa: F401,F403
from .impl.physx_types import *  # noqa: F401,F403
from .impl.stage_utils import *  # noqa: F401,F403

# Submodule names re-exported for ``isaacsim.asset.importer.utils.<submodule>`` access.
_SUBMODULES = ("asset_utils", "importer_utils", "merge_mesh_utils", "physx_types", "stage_utils")

# De-duplicated union of every submodule's ``__all__`` plus the submodule names themselves.
__all__ = list(
    dict.fromkeys(
        [
            *_SUBMODULES,
            *asset_utils.__all__,
            *importer_utils.__all__,
            *merge_mesh_utils.__all__,
            *physx_types.__all__,
            *stage_utils.__all__,
        ]
    )
)
