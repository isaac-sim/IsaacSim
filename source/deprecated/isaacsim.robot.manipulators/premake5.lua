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
local ogn = get_ogn_project_information(ext, "isaacsim/robot/manipulators")

project_ext(ext)

add_ogn_dependencies(ogn, { "python/nodes" })

project_ext_ogn(ext, ogn)

-- Add the standard dependencies all OGN projects have
repo_build.prebuild_copy {
    { "python/__init__.py", ogn.python_target_path },
}

repo_build.prebuild_link {
    { "docs", ext.target_dir .. "/docs" },
    { "data", ext.target_dir .. "/data" },
    { "python/controllers", ogn.python_target_path .. "/controllers" },
    { "python/nodes", ogn.python_target_path .. "/nodes" },
    { "python/manipulators", ogn.python_target_path .. "/manipulators" },
    { "python/grippers", ogn.python_target_path .. "/grippers" },
    { "python/impl", ogn.python_target_path .. "/impl" },
    { "python/tests", ogn.python_tests_target_path },
}
