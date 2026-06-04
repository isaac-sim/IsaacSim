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
ext.bin_dir = ext.target_dir .. "/bin"
local ogn = get_ogn_project_information(ext, "isaacsim/sensors/rtx")
local targetDepsDir = "%{root}/_build/target-deps"
local hostDepsDir = "%{root}/_build/host-deps"

project_ext(ext)

-- C++ Carbonite plugin
project_ext_plugin(ext, ogn.plugin_project)
targetdir(ext.bin_dir)

add_files("impl", "plugins")
add_files("nodes", ogn.nodes_path)

add_ogn_dependencies(ogn)

include_physx()

includedirs {
    target_deps .. "/cuda/include",
    "%{kit_sdk_bin_dir}/dev/fabric/include/",
    "%{root}/source/extensions/isaacsim.core.includes/include",
    "%{root}/source/extensions/isaacsim.core.nodes/include",
    "%{root}/source/deprecated/isaacsim.sensors.rtx/include",
    target_deps .. "/generic_model_output/%{platform}/%{config}/include",
    target_deps .. "/sensor-checker/%{platform}/%{config}/include",
    target_deps .. "/omni_client_library/include",
    target_deps .. "/python/include",
    target_deps .. "/rapidjson/include",
}
libdirs {
    extsbuild_dir .. "/omni.usd.core/bin",
    target_deps .. "/python/lib",
}

links {
    "omni.usd",
}

extra_usd_libs = { "usdGeom", "ts" }

-- Begin OpenUSD
add_usd(extra_usd_libs)
-- End OpenUSD

filter { "system:linux" }
includedirs {
    target_deps .. "/usd/%{cfg.buildcfg}/include/boost",
    target_deps .. "/python/include/python3.12",
}
filter { "system:windows" }
libdirs {
    target_deps .. "/tbb/lib/intel64/vc14",
}
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
    module = ogn.bindings_module,
    src = ogn.bindings_path,
    target_subdir = ogn.bindings_target_path,
}
add_files("bindings", "bindings/*.*")
add_files("python", "python/*.py")
add_files("python/impl", "python/impl/**.py")
add_files("python/tests", "python/tests/**.py")

includedirs {
    "%{root}/source/deprecated/isaacsim.sensors.rtx/include",
}

add_ogn_dependencies(ogn)

repo_build.prebuild_link {
    { "docs", ext.target_dir .. "/docs" },
    { "data", ext.target_dir .. "/data" },
    { "python/impl", ogn.python_target_path .. "/impl" },
    { "python/tests", ogn.python_target_path .. "/tests" },
    { "python/generic_model_output", ogn.python_target_path .. "/generic_model_output" },
    { "python/sensor_checker", ogn.python_target_path .. "/sensor_checker" },
}

repo_build.prebuild_copy {
    { "python/__init__.py", ogn.python_target_path },
}
