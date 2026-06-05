-- SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

local ext = get_current_extension_info()
ext.target_dir = deprecated_exts_path .. "/" .. ext.id

project_ext(ext, { generate_ext_project = true })

add_files("python", "*.py")
add_files("python/impl", "python/impl/**.py")
add_files("python/tests", "python/tests/**.py")
add_files("python/scripts", "python/scripts/**.py")

repo_build.prebuild_copy {
    { "python/__init__.py", ext.target_dir .. "/isaacsim/replicator/domain_randomization" },
}

repo_build.prebuild_link {
    { "docs", ext.target_dir .. "/docs" },
    { "data", ext.target_dir .. "/data" },
    { "python/tests", ext.target_dir .. "/isaacsim/replicator/domain_randomization/tests" },
    { "python/impl", ext.target_dir .. "/isaacsim/replicator/domain_randomization/impl" },
    { "python/scripts", ext.target_dir .. "/isaacsim/replicator/domain_randomization/scripts" },
}
