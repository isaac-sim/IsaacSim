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

local ext = get_current_extension_info()
dofile(root .. "/tools/isaac_build/extension_wheel.lua")

project_ext(ext)
stage_isaacsim_extension_wheel(ext, "isaacsim.common")

project_ext_plugin(ext, "isaacsim.common.plugin")
add_files("source", "plugins")
use_isaacsim_module_sdk("isaacsim.common", true)
links {
    "isaacsim-common-logging",
    "isaacsim-common-profiling",
}

filter { "configurations:debug" }
defines { "_DEBUG" }
filter { "configurations:release" }
defines { "NDEBUG" }
filter {}

repo_build.prebuild_link {
    { "data", ext.target_dir .. "/data" },
    { "docs", ext.target_dir .. "/docs" },
    { "python/tests", ext.target_dir .. "/isaacsim/common/tests" },
}
