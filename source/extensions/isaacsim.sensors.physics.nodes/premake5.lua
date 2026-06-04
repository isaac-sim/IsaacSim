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
local ogn = get_ogn_project_information(ext, "isaacsim/sensors/physics/nodes")

project_ext(ext)

-- C++ plugin for C++ OGN nodes
project_ext_plugin(ext, "isaacsim.sensors.physics.nodes.plugin")
add_files("impl", "plugins")
add_files("impl", "nodes")
add_files("iface", "%{root}/source/extensions/isaacsim.sensors.physics.nodes/include/**")
add_files("ogn", ogn.nodes_path)

add_ogn_dependencies(ogn, { "python/nodes" })

includedirs {
    "%{root}/source/extensions/isaacsim.sensors.physics.nodes/include",
    "%{root}/source/extensions/isaacsim.sensors.experimental.physics/include",
    "%{root}/source/extensions/isaacsim.core.includes/include",
    "%{root}/source/extensions/isaacsim.core.simulation_manager/include",
    "%{root}/_build/target-deps/usd/%{cfg.buildcfg}/include",
    "%{root}/_build/target-deps/usd_ext_physics/%{cfg.buildcfg}/include",
    extsbuild_dir .. "/usdrt.scenegraph/include",
    "%{kit_sdk_bin_dir}/dev/fabric/include/",
}

filter { "system:linux" }
includedirs {
    "%{root}/_build/target-deps/usd/%{cfg.buildcfg}/include/boost",
    "%{root}/_build/target-deps/python/include/python3.12",
}
filter {}

libdirs {
    "%{root}/_build/target-deps/usd/%{cfg.buildcfg}/lib",
    extsbuild_dir .. "/omni.usd.core/bin",
}

links { "omni.usd" }

extra_usd_libs = { "usdGeom", "ts" }
add_usd(extra_usd_libs)

filter { "configurations:debug" }
defines { "_DEBUG" }
filter { "configurations:release" }
defines { "NDEBUG" }
filter {}

-- OGN project (generates code from .ogn files)
project_ext_ogn(ext, ogn)

project_ext_bindings {
    ext = ext,
    project_name = ogn.python_project,
    module = "_physics_sensor_nodes",
    src = ogn.bindings_path,
    target_subdir = ogn.bindings_target_path,
}
add_files("bindings", "bindings/*.*")
includedirs {
    "%{root}/source/extensions/isaacsim.sensors.physics.nodes/include",
    "%{root}/source/extensions/isaacsim.core.includes/include",
}

-- Python files
add_files("python", "python/*.py")
add_files("python/impl", "python/impl/**.py")
add_files("python/nodes", "python/nodes/*.py")
add_files("python/tests", "python/tests/*.py")

repo_build.prebuild_link {
    { "python/impl", ogn.python_target_path .. "/impl" },
    { "python/tests", ogn.python_target_path .. "/tests" },
    { "python/nodes", ogn.python_target_path .. "/nodes" },
    { "data", ext.target_dir .. "/data" },
    { "docs", ext.target_dir .. "/docs" },
}

repo_build.prebuild_copy {
    { "python/*.py", ogn.python_target_path },
}
