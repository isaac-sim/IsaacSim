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

project_ext(ext)
-- C++ Carbonite plugin
project_ext_plugin(ext, "isaacsim.robot.wheeled_robots.plugin")
add_files("impl", "plugins")

filter { "system:linux", "platforms:x86_64" }
disablewarnings { "error=narrowing", "error=unused-but-set-variable", "error=unused-variable" }
filter { "system:windows" }
libdirs {
    "%{root}/_build/target-deps/tbb/lib/intel64/vc14",
}
filter {}

includedirs {
    "%{root}/source/extensions/isaacsim.core.includes/include",
    "%{root}/_build/target-deps/usd/%{cfg.buildcfg}/include",
    "%{root}/_build/target-deps/usd/%{cfg.buildcfg}/include/boost",
    "%{root}/_build/target-deps/usd_ext_physics/%{cfg.buildcfg}/include",
    "%{root}/_build/target-deps/python/include/python3.12",

    "%{root}/_build/target-deps/omni_physics/%{config}/include",
    "%{root}/source/deprecated/isaacsim.robot.wheeled_robots/include",
}

libdirs {
    "%{root}/_build/target-deps/usd/%{cfg.buildcfg}/lib",
    "%{root}/_build/target-deps/usd_ext_physics/%{cfg.buildcfg}/lib",
}

links {
    --    "usdGeom", "usdUtils", "omni.usd",
}

extra_usd_libs = { "ts" }

-- Begin OpenUSD
add_usd(extra_usd_libs)
-- End OpenUSD

filter { "configurations:debug" }
defines { "_DEBUG" }
filter { "configurations:release" }
defines { "NDEBUG" }
filter {}

project_ext_bindings {
    ext = ext,
    project_name = "isaacsim.robot.wheeled_robots.python",
    module = "_isaacsim_robot_wheeled_robots",
    src = "bindings",
    target_subdir = "isaacsim/robot/wheeled_robots/bindings",
}
add_files("bindings", "bindings/*.*")
add_files("python", "python/*.py")
add_files("python/controllers", "python/controllers/*.py")
add_files("python/tests", "python/tests/*.py")
add_files("python/robots", "python/robots/*.py")

includedirs {

    "%{root}/source/deprecated/isaacsim.robot.wheeled_robots/include",
}

repo_build.prebuild_copy {
    { "python/__init__.py", ext.target_dir .. "/isaacsim/robot/wheeled_robots" },
}

repo_build.prebuild_link {
    { "docs", ext.target_dir .. "/docs" },
    { "data", ext.target_dir .. "/data" },
    { "python/controllers", ext.target_dir .. "/isaacsim/robot/wheeled_robots/controllers" },
    { "python/robots", ext.target_dir .. "/isaacsim/robot/wheeled_robots/robots" },
    { "python/impl", ext.target_dir .. "/isaacsim/robot/wheeled_robots/impl" },
    { "python/tests", ext.target_dir .. "/isaacsim/robot/wheeled_robots/tests" },
}
