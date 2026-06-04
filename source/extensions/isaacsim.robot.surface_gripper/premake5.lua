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
local ogn = get_ogn_project_information(ext, "isaacsim/robot/surface_gripper")
local hostDepsDir = "%{root}/_build/host-deps"

-- Then define the main extension project
project_ext(ext)

-- C++ Carbonite plugin
project_ext_plugin(ext, "isaacsim.robot.surface_gripper.plugin")
cppdialect("C++17")

-- Specify clear dependencies without circular references
dependson {
    "prebuild",
    "carb.physics-usd.plugin",
    "omni.physx.plugin",
}

add_files("impl", "plugins")
add_files("ogn", ogn.nodes_path)

add_ogn_dependencies(ogn, { "python/nodes" })

include_physx()
add_cuda_dependencies()

includedirs {
    "%{root}/source/deprecated/isaacsim.core.utils/include",
    "%{root}/source/extensions/isaacsim.core.includes/include",
    "%{root}/source/extensions/isaacsim.robot.surface_gripper/include",
    "%{root}/source/extensions/isaacsim.core.nodes/include",
    "%{root}/source/extensions/isaacsim.robot.schema/include",
    extsbuild_dir .. "/usdrt.scenegraph/include",
    target_deps .. "/omni_client_library/include",
    target_deps .. "/usd/%{cfg.buildcfg}/include",
    target_deps .. "/usd/%{cfg.buildcfg}/include/boost",
    target_deps .. "/usd_ext_physics/%{cfg.buildcfg}/include",
    target_deps .. "/omni_physics/%{config}/include",
    target_deps .. "/python/include",
    "%{kit_sdk_bin_dir}/dev/fabric/include/",
}

libdirs {
    target_deps .. "/python/lib",
    target_deps .. "/usd/%{cfg.buildcfg}/lib",
    target_deps .. "/usd_ext_physics/%{cfg.buildcfg}/lib",
    extsbuild_dir .. "/omni.usd.core/bin",
}

links {
    "physxSchema",
}

extra_usd_libs = { "usdGeom", "usdPhysics", "usdUtils", "ts" }
add_usd(extra_usd_libs)

-- Platform specific configurations
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

-- First define the OGN project
project_ext_ogn(ext, ogn)

-- Python Bindings (make sure it depends on the plugin)
project_ext_bindings {
    ext = ext,
    project_name = "isaacsim.robot.surface_gripper.python",
    module = "_surface_gripper",
    src = "bindings",
    target_subdir = "isaacsim/robot/surface_gripper/bindings",
}

includedirs {
    target_deps .. "/usd/%{cfg.buildcfg}/include",
    target_deps .. "/usd/%{cfg.buildcfg}/include/boost",
    target_deps .. "/usd_ext_physics/%{cfg.buildcfg}/include",
    "%{kit_sdk_bin_dir}/dev/fabric/include/",
    "%{root}/source/deprecated/isaacsim.core.utils/include",
    "%{root}/source/extensions/isaacsim.robot.schema/include",
    "%{root}/source/extensions/isaacsim.robot.surface_gripper/include",
}

-- File operations at the end
repo_build.prebuild_link {
    { "python/impl", ext.target_dir .. "/isaacsim/robot/surface_gripper/impl" },
    { "python/tests", ext.target_dir .. "/isaacsim/robot/surface_gripper/tests" },
    { "data", ext.target_dir .. "/data" },
    { "docs", ext.target_dir .. "/docs" },
    { "include", ext.target_dir .. "/include" },
}

repo_build.prebuild_copy {
    { "python/*.py", ext.target_dir .. "/isaacsim/robot/surface_gripper" },
}
