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

project_ext(ext)

local python_target_path = "isaacsim/sensors/experimental/physics"

-- -------------------------------------
-- Build the C++ plugin
project_ext_plugin(ext, "isaacsim.sensors.experimental.physics.plugin")

add_files("include", "include/isaacsim/sensors/experimental/physics")
add_files("source", "plugins/isaacsim.sensors.experimental.physics")

add_cuda_dependencies()
include_physx()

includedirs {
    "include",
    "plugins/isaacsim.sensors.experimental.physics",
    "%{root}/source/extensions/isaacsim.core.experimental.prims/include",
    "%{root}/source/extensions/isaacsim.core.includes/include",
    "%{root}/source/extensions/isaacsim.robot.schema/include",
    "%{root}/source/extensions/isaacsim.core.simulation_manager/include",
    target_deps .. "/omni_physics/%{config}/include",
    target_deps .. "/usd/%{cfg.buildcfg}/include",
    target_deps .. "/usd_ext_physics/%{cfg.buildcfg}/include",
    extsbuild_dir .. "/usdrt.scenegraph/include",
    "%{kit_sdk_bin_dir}/dev/fabric/include/",
}

filter { "system:linux" }
includedirs {
    target_deps .. "/usd/%{cfg.buildcfg}/include/boost",
    target_deps .. "/python/include/python3.12",
}
filter {}

libdirs {
    target_deps .. "/usd/%{cfg.buildcfg}/lib",
    target_deps .. "/usd_ext_physics/%{cfg.buildcfg}/lib",
    extsbuild_dir .. "/omni.usd.core/bin",
}

links { "omni.usd", "physxSchema" }

extra_usd_libs = { "usdGeom", "usdPhysics", "usdUtils", "ts" }
add_usd(extra_usd_libs)

filter { "configurations:debug" }
defines { "_DEBUG" }
filter { "configurations:release" }
defines { "NDEBUG" }
filter {}

-- -------------------------------------
-- Build Python bindings (all sensors)
project_ext_bindings {
    ext = ext,
    project_name = "isaacsim.sensors.experimental.physics.python",
    module = "_physics_sensors",
    src = "bindings/isaacsim.sensors.experimental.physics",
    target_subdir = python_target_path .. "/bindings",
}
dependson { "isaacsim.sensors.experimental.physics.plugin" }

includedirs {
    "include",
    "%{root}/source/extensions/isaacsim.core.includes/include",
    "%{kit_sdk_bin_dir}/dev/fabric/include/",
}

filter { "configurations:debug" }
defines { "_DEBUG" }
filter { "configurations:release" }
defines { "NDEBUG" }
filter {}

-- -------------------------------------
-- Build C++ doctest tests
project_ext_tests(ext, "isaacsim.sensors.experimental.physics.tests")
add_files("source", "plugins/isaacsim.sensors.experimental.physics.tests")
includedirs {
    "include",
    "%{target_deps}/doctest/include",
    "%{root}/source/extensions/isaacsim.core.includes/include",
}
libdirs {
    extsbuild_dir .. "/omni.kit.test/bin",
}

filter { "configurations:debug" }
defines { "_DEBUG" }
filter { "configurations:release" }
defines { "NDEBUG" }
filter {}

-- -------------------------------------
-- Python files
add_files("python", "python/*.py")
add_files("python/impl", "python/impl/**.py")
add_files("python/tests", "python/tests/*.py")

repo_build.prebuild_link {
    { "python/impl", ext.target_dir .. "/" .. python_target_path .. "/impl" },
    { "python/tests", ext.target_dir .. "/" .. python_target_path .. "/tests" },
    { "data", ext.target_dir .. "/data" },
    { "docs", ext.target_dir .. "/docs" },
    { "include", ext.target_dir .. "/include" },
}

repo_build.prebuild_copy {
    { "python/*.py", ext.target_dir .. "/" .. python_target_path },
}
