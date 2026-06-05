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
local ogn = get_ogn_project_information(ext, "isaacsim/replicator/experimental/mobility_gen")

project_ext(ext)

project_ext_bindings {
    ext = ext,
    project_name = "isaacsim.replicator.experimental.mobility_gen",
    module = "_path_planner",
    src = ogn.bindings_path,
    target_subdir = ogn.bindings_target_path,
}

-- Windows MAX_PATH fix: default vcxproj location makes the linker's
-- pre-normalized output path 269 chars (> 260), causing LNK1104.
location ("%{root}/_compiler/".._ACTION)

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
    { "python/impl", ogn.python_target_path .. "/impl" },
    { "python/tests", ogn.python_target_path .. "/tests" },
    { "docs", ext.target_dir .. "/docs" },
    { "data", ext.target_dir .. "/data" },
}

repo_build.prebuild_copy {
    { "python/*.py", ogn.python_target_path },
}
