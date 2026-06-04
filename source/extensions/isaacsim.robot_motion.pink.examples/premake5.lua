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

-- PINK examples depend on isaacsim.robot_motion.pink which is linux-x86_64 only.
if platform_target == "linux-x86_64" then
    local ext = get_current_extension_info()
    project_ext (ext)

    repo_build.prebuild_link {
        { "data", ext.target_dir.."/data" },
        { "docs", ext.target_dir.."/docs" },
        { "isaacsim", ext.target_dir.."/isaacsim" },
    }
else
    print("SKIPPING isaacsim.robot_motion.pink.examples - only supported on linux-x86_64")
end
