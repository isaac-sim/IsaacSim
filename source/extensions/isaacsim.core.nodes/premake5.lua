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
local ogn = get_ogn_project_information(ext, "isaacsim/core/nodes")
project_ext(ext)
-- C++ Carbonite plugin
project_ext_plugin(ext, "isaacsim.core.nodes.plugin")
add_files("impl", "plugins")
add_files("impl", "cuda")
add_files("iface", "%{root}/source/extensions/isaacsim.core.nodes/include/**")
add_files("ogn", ogn.nodes_path)

add_cuda_dependencies()

filter { "system:linux", "platforms:x86_64" }
disablewarnings { "error=narrowing", "error=unused-but-set-variable", "error=unused-variable" }
filter { "system:windows" }
libdirs {
    "%{root}/_build/target-deps/tbb/lib/intel64/vc14",
}
filter {}

add_ogn_dependencies(ogn, { "python/nodes" })

include_physx()

includedirs {
    "%{root}/source/extensions/isaacsim.core.includes/include",
    "%{root}/source/extensions/isaacsim.core.experimental.prims/include",
    "%{root}/source/extensions/isaacsim.core.simulation_manager/include",
    "%{root}/_build/target-deps/usd/%{cfg.buildcfg}/include",
    "%{root}/_build/target-deps/usd/%{cfg.buildcfg}/include/boost",
    "%{root}/_build/target-deps/usd_ext_physics/%{cfg.buildcfg}/include",
    "%{root}/_build/target-deps/python/include/python3.12",
    "%{root}/_build/target-deps/omni_client_library/include",
    "%{root}/_build/target-deps/omni_physics/%{config}/include",
    extsbuild_dir .. "/omni.syntheticdata/include",
    extsbuild_dir .. "/usdrt.scenegraph/include",
    "%{root}/source/extensions/isaacsim.core.nodes/include",
    "%{kit_sdk_bin_dir}/dev/fabric/include/",
    "%{root}/_build/target-deps/python/include",
    "%{root}/source/extensions/isaacsim.robot.schema/include",
}
libdirs {
    "%{root}/_build/target-deps/usd/%{cfg.buildcfg}/lib",
    "%{root}/_build/target-deps/usd_ext_physics/%{cfg.buildcfg}/lib",
    extsbuild_dir .. "/omni.usd.core/bin",
}

links {
    "omni.usd",
}

extra_usd_libs = { "usdGeom", "usdPhysics", "ts" }


-- Begin OpenUSD
add_usd(extra_usd_libs)
-- End OpenUSD

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
add_files("python/tests", "python/tests/*.py")

includedirs {
    "%{root}/source/extensions/isaacsim.core.nodes/include",
    "%{kit_sdk_bin_dir}/dev/fabric/include/",
}

-- Add the standard dependencies all OGN projects have
repo_build.prebuild_copy {
    { "python/__init__.py", ogn.python_target_path },
}

repo_build.prebuild_link {
    { "python/scripts", ogn.python_target_path .. "/scripts" },
    { "docs", ext.target_dir .. "/docs" },
    { "data", ext.target_dir .. "/data" },
    { "include", ext.target_dir .. "/include" },
    { "python/impl", ogn.python_target_path .. "/impl" },
    { "python/tests", ogn.python_tests_target_path },
}
