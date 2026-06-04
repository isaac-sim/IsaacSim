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

-- Setup the basic extension variables
local ext = get_current_extension_info()
-- Set up the basic shared project information
project_ext(ext)

-- -------------------------------------
-- Build the C++ plugin that will be loaded by the extension
project_ext_plugin(ext, "isaacsim.core.experimental.primdata.plugin")

add_files("source", "plugins/isaacsim.core.experimental.primdata")
add_cuda_dependencies()
include_physx()
includedirs {
    "plugins/isaacsim.core.experimental.primdata",
    "%{kit_sdk_bin_dir}/dev/fabric/include/",
    "%{root}/source/extensions/isaacsim.core.experimental.prims/include",
    "%{root}/source/extensions/isaacsim.core.includes/include",
    "%{root}/source/extensions/isaacsim.core.simulation_manager/include",
    target_deps .. "/omni_physics/%{config}/include",
    "%{target_deps}/python/include/python3.12",
    "%{target_deps}/usd/%{cfg.buildcfg}/include",
    "%{target_deps}/usd/%{cfg.buildcfg}/include/boost",
    target_deps .. "/usd_ext_physics/%{cfg.buildcfg}/include",
    extsbuild_dir .. "/usdrt.scenegraph/include",
}

libdirs {
    target_deps .. "/usd/%{cfg.buildcfg}/lib",
    target_deps .. "/usd_ext_physics/%{cfg.buildcfg}/lib",
    extsbuild_dir .. "/omni.usd.core/bin",
}
links { "carb", "omni.usd", "physxSchema" }

extra_usd_libs = { "usdGeom", "usdUtils", "usdPhysics", "ts" }
add_usd(extra_usd_libs)

filter { "configurations:debug" }
defines { "_DEBUG" }
filter { "configurations:release" }
defines { "NDEBUG" }
filter {}

-- -------------------------------------
-- Build C++ doctest tests
project_ext_tests(ext, "isaacsim.core.experimental.primdata.tests")
add_files("source", "plugins/isaacsim.core.experimental.primdata.tests")
add_cuda_dependencies()
includedirs {
    "plugins/isaacsim.core.experimental.primdata",
    "plugins/",
    "%{target_deps}/doctest/include",
    "%{root}/source/extensions/isaacsim.core.includes/include",
    "%{root}/source/extensions/isaacsim.core.simulation_manager/include",
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
-- Link/copy folders and files to be packaged with the extension
repo_build.prebuild_link {
    { "data", ext.target_dir .. "/data" },
    { "docs", ext.target_dir .. "/docs" },
}
