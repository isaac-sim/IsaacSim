# SPDX-FileCopyrightText: Copyright (c) 2023-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

from typing import Any

import omni.ext


class Extension(omni.ext.IExt):
    def on_startup(self, ext_id: Any) -> Any:
        # Do not reload typing_extensions: this archive no longer bundles it, and reloading the live
        # module replaces classes such as TypeAliasType that other extensions have already used.
        pass

    def on_shutdown(self) -> Any:
        pass


__all__ = []
