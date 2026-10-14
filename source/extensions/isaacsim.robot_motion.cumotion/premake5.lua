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

-- Setup the basic extension variables
local ext = get_current_extension_info()
dofile(root .. "/tools/isaac_build/extension_wheel.lua")

-- Set up the basic shared project information
project_ext(ext)

-- -------------------------------------
-- Link/copy folders and files to be packaged with the extension
stage_isaacsim_extension_wheel_with_python_overlays(ext, "isaacsim.robot_motion.cumotion", {
    "isaacsim/robot_motion/cumotion/__init__.py",
    "isaacsim/robot_motion/cumotion/extension.py",
    "isaacsim/robot_motion/cumotion/impl/kit_cumotion_debug_visualizer.py",
})

repo_build.prebuild_link {
    { "data", ext.target_dir.."/data" },
    { "docs", ext.target_dir.."/docs" },
    { "python/tests", ext.target_dir.."/isaacsim/robot_motion/cumotion/tests" },
}
