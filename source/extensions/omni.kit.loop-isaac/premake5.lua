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

project_ext(ext)

-- C++ Carbonite plugin
project_ext_plugin(ext, "omni.kit.loop-isaac.plugin")
add_files("impl", "plugins/omni.kit.loop")
add_files("iface", "%{root}/include/omni/kit/**")

includedirs {
    "%{root}/source/extensions/omni.kit.loop-isaac/include",
}

-- Python Bindings for Carobnite Plugin
project_ext_bindings {
    ext = ext,
    project_name = "omni.kit.loop-isaac.python",
    module = "_loop",
    src = "bindings",
    target_subdir = "omni/kit/loop/bindings",
}

includedirs {
    "%{root}/source/extensions/omni.kit.loop-isaac/include",
}

repo_build.prebuild_link {
    { "docs", ext.target_dir .. "/docs" },
    { "data", ext.target_dir .. "/data" },
    { "include", ext.target_dir .. "/include" },
    { "python/tests", ext.target_dir .. "/omni/kit/loop/tests" },
}
repo_build.prebuild_copy {
    { "python/*.py", ext.target_dir .. "/omni/kit/loop" },
}
