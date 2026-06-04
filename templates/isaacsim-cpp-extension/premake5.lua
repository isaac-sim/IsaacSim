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
project_ext(ext)

-- C++ Carbonite plugin
project_ext_plugin(ext, "{{extension_name}}.plugin")
add_files("impl", "plugins")
add_files("iface", "include")

includedirs {
    "%{root}/source/extensions/{{extension_name}}/include",
}

filter { "configurations:debug" }
defines { "_DEBUG" }
filter { "configurations:release" }
defines { "NDEBUG" }
filter {}

-- Python Bindings
project_ext_bindings {
    ext = ext,
    project_name = "{{extension_name}}.python",
    module = "_{{binding_module}}",
    src = "bindings",
    target_subdir = "{{python_module_path}}/bindings",
}

includedirs {
    "%{root}/source/extensions/{{extension_name}}/include",
}

repo_build.prebuild_link {
    { "python/impl", ext.target_dir .. "/{{python_module_path}}/impl" },
    { "python/tests", ext.target_dir .. "/{{python_module_path}}/tests" },
    { "docs", ext.target_dir .. "/docs" },
    { "data", ext.target_dir .. "/data" },
    { "include", ext.target_dir .. "/include" },
}

repo_build.prebuild_copy {
    { "python/*.py", ext.target_dir .. "/{{python_module_path}}" },
}
