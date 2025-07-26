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

-- -------------------------------------
-- Build the C++ plugin that will be loaded by the tests
project_ext_tests(ext, "isaacsim.core.includes.tests")
    add_files("source", "plugins/isaacsim.core.includes.tests")
    add_cuda_dependencies()
    includedirs {
        "include",
        "plugins/",
        "%{target_deps}/doctest/include",
    }
    -- link omni.kit.test (path or 'repo_precache_exts' config may need to be adjusted)
    libdirs {
        extsbuild_dir.."/omni.kit.test/bin",
    }

    filter { "configurations:debug" }
        defines { "_DEBUG" }
    filter { "configurations:release" }
        defines { "NDEBUG" }
    filter {}


repo_build.prebuild_link {
    { "data", ext.target_dir .. "/data" },
    { "docs", ext.target_dir .. "/docs" },
    { "include", ext.target_dir .. "/include" },
}
