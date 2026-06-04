-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
-- SPDX-License-Identifier: Apache-2.0
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
-- http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

-- Setup the extension.
local ext = get_current_extension_info()

project_ext(ext)

-- Link folders that should be packaged with the extension.
repo_build.prebuild_link {
    { "data", ext.target_dir.."/data" },
    { "docs", ext.target_dir.."/docs" },
    { "python/impl", ext.target_dir.."/isaacsim/physics/newton/impl" },
    { "python/impl/tensors", ext.target_dir.."/isaacsim/physics/newton/tensors" },  -- Link tensors directly to maintain module structure
    { "python/tests", ext.target_dir.."/isaacsim/physics/newton/tests" },
}

-- Copy the main __init__.py to maintain the module root
repo_build.prebuild_copy {
    { "python/__init__.py", ext.target_dir.."/isaacsim/physics/newton/__init__.py" },
}

