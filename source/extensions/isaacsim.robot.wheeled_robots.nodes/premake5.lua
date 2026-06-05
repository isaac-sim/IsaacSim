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
local ogn = get_ogn_project_information(ext, "isaacsim/robot/wheeled_robots/nodes")
project_ext(ext)

-- Build the C++ plugin that provides the OmniGraph nodes
project_ext_plugin(ext, "isaacsim.robot.wheeled_robots.nodes.plugin")

add_files("impl", "plugins")
add_files("ogn", ogn.nodes_path)

filter { "system:linux", "platforms:x86_64" }
disablewarnings { "error=narrowing", "error=unused-but-set-variable", "error=unused-variable" }
filter { "system:windows" }
libdirs {
    "%{root}/_build/target-deps/tbb/lib/intel64/vc14",
}
filter {}

-- Add OGN dependencies for the nodes
add_ogn_dependencies(ogn, { "python/nodes" })
add_ogn_dependencies(ogn, { "nodes" })

includedirs {
    "%{root}/source/extensions/isaacsim.core.includes/include",
    "%{root}/_build/target-deps/usd/%{cfg.buildcfg}/include",
    "%{root}/_build/target-deps/usd/%{cfg.buildcfg}/include/boost",
    "%{root}/_build/target-deps/usd_ext_physics/%{cfg.buildcfg}/include",
    "%{root}/_build/target-deps/python/include/python3.12",
    "%{root}/_build/target-deps/omni_physics/%{config}/include",
    "%{root}/source/extensions/isaacsim.robot.wheeled_robots.nodes/include",
}

libdirs {
    "%{root}/_build/target-deps/usd/%{cfg.buildcfg}/lib",
    "%{root}/_build/target-deps/usd_ext_physics/%{cfg.buildcfg}/lib",
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

-- Generate the OGN project (this handles all the .ogn files)
project_ext_ogn(ext, ogn)

-- Python Bindings for the plugin
project_ext_bindings {
    ext = ext,
    project_name = ogn.python_project,
    module = "_wheeled_robots_nodes",
    src = ogn.bindings_path,
    target_subdir = ogn.bindings_target_path,
}
add_files("bindings", "bindings/*.*")
add_files("python", "python/*.py")
add_files("python/nodes", "python/nodes/*.py")
add_files("python/tests", "python/tests/*.py")

includedirs {
    "%{root}/source/extensions/isaacsim.robot.wheeled_robots.nodes/include",
}

-- Copy/link necessary files for packaging
repo_build.prebuild_copy {
    { "python/__init__.py", ogn.python_target_path },
}

repo_build.prebuild_link {
    { "python/impl", ogn.python_target_path .. "/impl" },
    { "python/tests", ogn.python_tests_target_path },
    { "python/nodes", ogn.python_target_path .. "/nodes" },
    { "docs", ext.target_dir .. "/docs" },
    { "data", ext.target_dir .. "/data" },
}
