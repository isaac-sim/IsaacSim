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
project_ext(ext)

-- Python Bindings for Carobnite Plugin
project_ext_bindings {
    ext = ext,
    project_name = "isaacsim.core.utils.python",
    module = "_isaac_utils",
    src = "bindings",
    target_subdir = "isaacsim/core/utils/bindings",
}
staticruntime("Off")
add_files("impl", "plugins")
add_files("iface", "include")
defines { "ISAACSIM_CORE_UTILS_EXPORT" }

include_physx()
includedirs {
    "%{root}/source/extensions/isaacsim.core.includes/include",
    "%{root}/source/extensions/isaacsim.core.simulation_manager/include",
    "%{root}/_build/target-deps/usd/%{cfg.buildcfg}/include",
    "%{kit_sdk_bin_dir}/dev/fabric/include/",
    "%{root}/_build/target-deps/omni_client_library/include",
    extsbuild_dir .. "/usdrt.scenegraph/include",
    "%{root}/source/deprecated/isaacsim.core.utils/include",
    "%{root}/_build/target-deps/omni_physics/%{config}/include",
}

libdirs {
    "%{root}/_build/target-deps/usd/%{cfg.buildcfg}/lib",
    extsbuild_dir .. "/omni.usd.core/bin",
}
links { "omni.usd" }

extra_usd_libs = { "usdUtils", "usdGeom", "usdPhysics", "pcp", "ts" }

-- Begin OpenUSD
add_usd(extra_usd_libs)
-- End OpenUSD

filter { "system:linux", "platforms:x86_64", "configurations:release" }
links { "tbb" }
filter { "system:linux", "platforms:x86_64", "configurations:debug" }
links { "tbb_debug" }
filter {}

filter { "system:windows", "platforms:x86_64" }
-- link_boost_for_windows({"boost_python310"})
libdirs {
    "%{root}/_build/target-deps/tbb/lib/intel64/vc14",
}
filter {}

filter { "configurations:debug" }
defines { "_DEBUG" }
filter { "configurations:release" }
defines { "NDEBUG" }
filter {}

repo_build.prebuild_link {
    { "python/impl/numpy", ext.target_dir .. "/isaacsim/core/utils/numpy" },
    { "python/impl/torch", ext.target_dir .. "/isaacsim/core/utils/torch" },
    { "python/impl/warp", ext.target_dir .. "/isaacsim/core/utils/warp" },
    { "python/tests", ext.target_dir .. "/isaacsim/core/utils/tests" },
    { "docs", ext.target_dir .. "/docs" },
    { "data", ext.target_dir .. "/data" },
}

repo_build.prebuild_copy {
    { "python/impl/*.py", ext.target_dir .. "/isaacsim/core/utils" },
}
