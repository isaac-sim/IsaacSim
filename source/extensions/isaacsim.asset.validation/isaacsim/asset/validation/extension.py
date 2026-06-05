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

"""Isaac Sim Asset Validation Extension that provides custom validation rules for physics, robot schemas, and materials."""

import omni.ext


class IsaacSimAssetValidationExtension(omni.ext.IExt):
    """Isaac Sim Asset Validation Extension.

    This extension provides various custom rules to validate content for Isaac Sim,
    including physics validation, robot schema validation, and material validation.
    """

    def on_startup(self, ext_id: str) -> None:
        """Initialize the extension on startup.

        Args:
            ext_id: The extension identifier.
        """

    def on_shutdown(self) -> None:
        """Clean up resources when the extension is shut down."""
