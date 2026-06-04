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

-- Setup the basic extension variables
local ext = get_current_extension_info()
local ogn = get_ogn_project_information(ext, "isaacsim/core/experimental/actuators")

-- Generate the Python OGN database (no C++ plugin needed).
project_ext_ogn(ext, ogn)

project_ext(ext, { generate_ext_project = true })

-- Add Python source files so they appear in IDE projects.
add_files("python/nodes", "python/nodes/**.py")
add_files("python/impl", "python/impl/**.py")
add_files("python/tests", "python/tests/**.py")

-- Register the python/nodes directory as an OGN node source.
add_ogn_dependencies(ogn, { "python/nodes" })

-- -------------------------------------
-- Link/copy folders and files to be packaged with the extension
repo_build.prebuild_copy {
    { "python/__init__.py", ogn.python_target_path },
}

repo_build.prebuild_link {
    { "data", ext.target_dir .. "/data" },
    { "docs", ext.target_dir .. "/docs" },
    { "python/nodes", ogn.python_target_path .. "/nodes" },
    { "python/impl", ogn.python_target_path .. "/impl" },
    { "python/tests", ogn.python_tests_target_path },
}
