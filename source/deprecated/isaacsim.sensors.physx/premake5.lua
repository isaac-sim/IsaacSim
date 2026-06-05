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
local ogn = get_ogn_project_information(ext, "isaacsim/sensors/physx")
project_ext(ext)

-- C++ Carbonite plugin
project_ext_plugin(ext, "isaacsim.sensors.physx.plugin")
add_files("impl", "plugins")
add_files("ogn", ogn.nodes_path)

add_ogn_dependencies(ogn)

include_physx()
includedirs {
    "%{root}/source/extensions/isaacsim.core.includes/include",
    "%{root}/source/extensions/isaacsim.robot.schema/include",
    "%{root}/_build/target-deps/gsl/include",
    "%{root}/_build/target-deps/usd/%{cfg.buildcfg}/include",
    "%{root}/_build/target-deps/usd_ext_physics/%{cfg.buildcfg}/include",
    "%{root}/_build/target-deps/omni_physics/%{config}/include",
    extsbuild_dir .. "/omni.syntheticdata/include",
    extsbuild_dir .. "/usdrt.scenegraph/include",
    "%{kit_sdk_bin_dir}/dev/fabric/include/",
    "%{root}/_build/target-deps/omni_client_library/include",
    "%{root}/_build/target-deps/python/include",
    "%{root}/source/deprecated/isaacsim.sensors.physx/include",
    isaac_sim_extra_extsbuild_dir .. "/isaacsim.util.debug_draw/include",
}
libdirs {
    "%{root}/_build/target-deps/usd/%{cfg.buildcfg}/lib",
    "%{root}/_build/target-deps/usd_ext_physics/%{cfg.buildcfg}/lib",
    extsbuild_dir .. "/omni.usd.core/bin",
    isaac_sim_extra_extsbuild_dir .. "/isaacsim.util.debug_draw/bin",
}

links {
    "omni.usd",
    "isaacsim.util.debug_draw.primitive_drawing",
}
extra_usd_libs = { "usdGeom", "usdPhysics", "ts" }

-- Begin OpenUSD
add_usd(extra_usd_libs)
-- End OpenUSD

filter { "system:linux" }
includedirs {
    "%{root}/_build/target-deps/usd/%{cfg.buildcfg}/include/boost",
    "%{root}/_build/target-deps/python/include/python3.12",
}
filter { "system:windows" }
libdirs {
    "%{root}/_build/target-deps/tbb/lib/intel64/vc14",
}
filter {}

filter { "configurations:debug" }
defines { "_DEBUG" }
filter { "configurations:release" }
defines { "NDEBUG" }
filter {}

project_ext_ogn(ext, ogn)

-- Python Bindings for Carbonite Plugin
project_ext_bindings {
    ext = ext,
    project_name = "isaacsim.sensors.physx.python",
    module = "_range_sensor",
    src = "bindings",
    target_subdir = "isaacsim/sensors/physx/bindings",
}

includedirs {
    "%{root}/source/deprecated/isaacsim.sensors.physx/include",
    "%{kit_sdk_bin_dir}/dev/fabric/include/",
}

repo_build.prebuild_link {
    { "docs", ext.target_dir .. "/docs" },
    { "data", ext.target_dir .. "/data" },
    { "include", ext.target_dir .. "/include" },
    { "python/tests", ext.target_dir .. "/isaacsim/sensors/physx/tests" },
    { "python/impl", ext.target_dir .. "/isaacsim/sensors/physx/impl" },
}

repo_build.prebuild_copy {
    { "python/*.py", ext.target_dir .. "/isaacsim/sensors/physx" },
}
