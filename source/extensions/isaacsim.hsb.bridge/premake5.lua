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

if os.target() == "linux" then
    -- This is a pure Python umbrella extension.
    -- All C++ functionality has been moved to isaacsim.hsb.core and isaacsim.hsb.nodes.
    local ext = get_current_extension_info()
    project_ext(ext)

    -- Copy/link necessary files for packaging
    repo_build.prebuild_link {
        { "docs", ext.target_dir .. "/docs" },
        { "data", ext.target_dir .. "/data" },
    }

    repo_build.prebuild_copy {
        { "python/__init__.py", ext.target_dir .. "/isaacsim/hsb/bridge" },
        { "python/extension.py", ext.target_dir .. "/isaacsim/hsb/bridge" },
    }
else
    print("SKIPPING BUILD - Only supported on linux-x86_64")
end
