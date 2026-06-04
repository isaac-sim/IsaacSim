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

-- Use folder name to build extension name and tag.
local ext = get_current_extension_info()
local ogn = get_ogn_project_information(ext, "{{python_module_path}}")
project_ext(ext)

-- C++ Carbonite plugin
project_ext_plugin(ext, "{{extension_name}}.plugin")
add_files("impl", "plugins")
add_files("iface", "include")
add_files("ogn", ogn.nodes_path)

includedirs {
    "%{root}/source/extensions/{{extension_name}}/include",
}

filter { "configurations:debug" }
defines { "_DEBUG" }
filter { "configurations:release" }
defines { "NDEBUG" }
filter {}

add_ogn_dependencies(ogn, { "python/nodes" })

project_ext_ogn(ext, ogn)

project_ext_bindings {
    ext = ext,
    project_name = ogn.python_project,
    module = ogn.bindings_module,
    src = ogn.bindings_path,
    target_subdir = ogn.bindings_target_path,
}
add_files("bindings", "bindings/*.*")
add_files("python", "python/*.py")
add_files("python/tests", "python/tests/*.py")

includedirs {
    "%{root}/source/extensions/{{extension_name}}/include",
}

-- Add the standard dependencies all OGN projects have
repo_build.prebuild_copy {
    { "python/__init__.py", ogn.python_target_path },
}

repo_build.prebuild_link {
    { "docs", ext.target_dir .. "/docs" },
    { "data", ext.target_dir .. "/data" },
    { "include", ext.target_dir .. "/include" },
    { "python/impl", ogn.python_target_path .. "/impl" },
    { "python/tests", ogn.python_tests_target_path },
}
