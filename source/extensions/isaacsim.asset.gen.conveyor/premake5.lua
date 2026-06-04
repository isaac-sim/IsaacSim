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
local ogn = get_ogn_project_information(ext, "isaacsim/asset/gen/conveyor")


project_ext(ext)

project_ext_plugin(ext, "isaacsim.asset.gen.conveyor.plugin")
add_files("impl", "plugins")
add_files("ogn", ogn.nodes_path)

include_physx()
includedirs {
    "%{root}/source/extensions/isaacsim.core.includes/include",
    "%{root}/source/extensions/isaacsim.asset.gen.conveyor/include",
    target_deps .. "/usd/%{cfg.buildcfg}/include",
    target_deps .. "/usd/%{cfg.buildcfg}/include/boost",
    target_deps .. "/usd_ext_physics/%{cfg.buildcfg}/include",
    target_deps .. "/omni_physics/%{config}/include",
    target_deps .. "/omni_client_library/include",
    -- usdrt headers + Fabric headers are required so that compute() can write
    -- shader inputs:texture_translate into Fabric when Fabric Scene Delegate is enabled
    target_deps .. "/rtx_plugins/include",
    extsbuild_dir .. "/usdrt.scenegraph/include",
    "%{kit_sdk_bin_dir}/dev/fabric/include/",
}
libdirs {
    target_deps .. "/usd/%{cfg.buildcfg}/lib",
    target_deps .. "/usd_ext_physics/%{cfg.buildcfg}/lib",
    extsbuild_dir .. "/omni.usd.core/bin",
}

-- Linux-specific compile information
filter { "system:linux" }
exceptionhandling("On")
removeflags { "UndefinedIdentifiers" }
includedirs {
    target_deps .. "/usd/%{config}/include/boost",
    target_deps .. "/python/include/python3.12",
}
filter {}

add_ogn_dependencies(ogn)

links { "physxSchema", "omni.usd" }

-- Specifies the external libraries required by the nodes
extra_usd_libs = { "usdGeom", "usdShade", "usdPhysics", "ts" }

-- Begin OpenUSD
add_usd(extra_usd_libs)
-- End OpenUSD

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
    "%{root}/source/extensions/isaacsim.asset.gen.conveyor/include",
}

add_ogn_dependencies(ogn)

repo_build.prebuild_link {
    { "docs", ext.target_dir .. "/docs" },
    { "data", ext.target_dir .. "/data" },
    { "python/impl", ogn.python_target_path .. "/impl" },
    { "python/tests", ogn.python_target_path .. "/tests" },
    { "python/commands", ogn.python_target_path .. "/commands" },
}

repo_build.prebuild_copy {
    { "python/__init__.py", ogn.python_target_path },
}
