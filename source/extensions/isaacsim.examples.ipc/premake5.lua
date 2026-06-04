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

local ext = get_current_extension_info()
local ogn = get_ogn_project_information(ext, "isaacsim/examples/ipc")
project_ext(ext)

project_ext_plugin(ext, "isaacsim.examples.ipc.plugin")

add_files("impl", "plugins")
add_files("ogn", ogn.nodes_path)

add_ogn_dependencies(ogn, { "python/nodes" })
add_ogn_dependencies(ogn, { "nodes" })

cppdialect("C++17")

includedirs {
    "plugins/isaacsim.examples.ipc",
    "%{root}/source/extensions/isaacsim.core.includes/include",
    "%{root}/_build/target-deps/omni_physics/%{config}/include",
    "%{root}/_build/target-deps/usd/%{cfg.buildcfg}/include",
    "%{kit_sdk_bin_dir}/dev/fabric/include/",
}

add_usd()

filter { "system:linux" }
disablewarnings { "error=narrowing", "error=unused-but-set-variable", "error=unused-variable" }
buildoptions("-fvisibility=default")
linkoptions { "-Wl,--export-dynamic" }
filter { "system:windows" }
links { "ws2_32" }
filter {}

filter { "configurations:debug" }
defines { "_DEBUG" }
filter { "configurations:release" }
defines { "NDEBUG" }
filter {}

project_ext_ogn(ext, ogn)

project_ext_bindings {
    ext = ext,
    project_name = ogn.python_project,
    module = "_isaacsim_examples_ipc",
    src = ogn.bindings_path,
    target_subdir = ogn.bindings_target_path,
}
add_files("bindings", "bindings/*.*")
add_files("python", "python/*.py")
add_files("python/nodes", "python/nodes/*.py")
add_files("python/scripts", "python/scripts/*.py")
add_files("python/tests", "python/tests/*.py")

includedirs {
    "plugins/isaacsim.examples.ipc",
    "%{kit_sdk_bin_dir}/dev/fabric/include/",
}

project_ext_tests(ext, "isaacsim.examples.ipc.tests")
cppdialect("C++17")
add_files("source", "library/tests")

includedirs {
    "plugins/isaacsim.examples.ipc",
    "%{target_deps}/doctest/include",
    "%{root}/source/extensions/isaacsim.core.includes/include",
    "%{root}/_build/target-deps/omni_physics/%{config}/include",
    "%{root}/_build/target-deps/usd/%{cfg.buildcfg}/include",
    "%{kit_sdk_bin_dir}/dev/fabric/include/",
}

libdirs {
    extsbuild_dir .. "/omni.kit.test/bin",
}

add_usd()

filter { "system:linux" }
disablewarnings { "error=narrowing", "error=unused-but-set-variable", "error=unused-variable" }
filter { "system:windows" }
links { "ws2_32" }
filter {}

filter { "configurations:debug" }
defines { "_DEBUG" }
filter { "configurations:release" }
defines { "NDEBUG" }
filter {}

repo_build.prebuild_copy {
    { "python/__init__.py", ogn.python_target_path },
    { "python/extension.py", ogn.python_target_path },
    { "bindings/__init__.py", ogn.python_target_path .. "/bindings" },
}

repo_build.prebuild_link {
    { "docs", ext.target_dir .. "/docs" },
    { "data", ext.target_dir .. "/data" },
    { "nodes", ext.target_dir .. "/nodes" },
    { "python/nodes", ogn.python_target_path .. "/nodes" },
    { "python/scripts", ogn.python_target_path .. "/scripts" },
    { "python/tests", ogn.python_target_path .. "/tests" },
}
