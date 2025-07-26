-- SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

-- Python Bindings for Carbonite Plugin
project_ext_bindings {
    ext = ext,
    project_name = "isaacsim.core.cloner.python",
    module = "_isaac_cloner",
    src = "bindings",
    target_subdir = "isaacsim/core/cloner",
}
staticruntime("Off")
add_files("impl", "plugins")
add_files("iface", "include")
defines { "ISAACSIM_CORE_CLONER_EXPORT" }

includedirs {
    "%{root}/source/extensions/isaacsim.core.includes/include",
    "%{root}/_build/target-deps/usd/%{cfg.buildcfg}/include",
    "%{kit_sdk_bin_dir}/dev/fabric/include/",
    extsbuild_dir .. "/usdrt.scenegraph/include",
    "%{root}/source/extensions/isaacsim.core.cloner/include",
    "%{root}/source/extensions/isaacsim.core.cloner/plugins",
}

libdirs {
    "%{root}/_build/target-deps/usd/%{cfg.buildcfg}/lib",
}

extra_usd_libs = { "usdUtils" }

-- Begin OpenUSD
add_usd(extra_usd_libs)
-- End OpenUSD

filter { "configurations:debug" }
defines { "_DEBUG" }
filter { "configurations:release" }
defines { "NDEBUG" }
filter {}

repo_build.prebuild_link {
    { "python/impl", ext.target_dir .. "/isaacsim/core/cloner/impl" },
    { "python/tests", ext.target_dir .. "/isaacsim/core/cloner/tests" },
    { "docs", ext.target_dir .. "/docs" },
    { "data", ext.target_dir .. "/data" },
}

repo_build.prebuild_copy {
    { "python/*.py", ext.target_dir .. "/isaacsim/core/cloner" },
}
