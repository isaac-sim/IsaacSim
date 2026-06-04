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
local ogn = get_ogn_project_information(ext, "isaacsim/replicator/mobility_gen")
ext.target_dir = deprecated_exts_path .. "/" .. ext.id

project_ext(ext)

project_ext_bindings {
    ext = ext,
    project_name = "isaacsim.replicator.mobility_gen",
    module = "_path_planner",
    src = ogn.bindings_path,
    target_subdir = ogn.bindings_target_path,
}

add_files("bindings", "bindings/*.*")

filter { "system:linux", "configurations:release" }
links { "tbb", "pthread" }
buildoptions { "-pthread" }
filter { "system:linux", "configurations:debug" }
links { "tbb_debug", "pthread" }
buildoptions { "-pthread" }
includedirs {
    "%{root}/_build/target-deps/python/include/python3.12",
}

filter { "system:windows" }
-- Warning C4099: 'omni::physx::IPhysx': type name first seen using 'class' now seen using 'struct'
disablewarnings { "4099" }
disablewarnings { "4251" }
-- link_boost_for_windows({"boost_python310"})

libdirs {
    "%{root}/_build/target-deps/tbb/lib/intel64/vc14",
}
filter {}

repo_build.prebuild_link {
    { "python/impl", ext.target_dir .. "/isaacsim/replicator/mobility_gen/impl" },
    { "python/tests", ext.target_dir .. "/isaacsim/replicator/mobility_gen/tests" },
    { "docs", ext.target_dir .. "/docs" },
    { "data", ext.target_dir .. "/data" },
}

repo_build.prebuild_copy {
    { "python/*.py", ext.target_dir .. "/isaacsim/replicator/mobility_gen" },
}
