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

repo_build.prebuild_link {
    { "config", ext.target_dir .. "/config" },
    { "docs", ext.target_dir .. "/docs" },
    { "data", ext.target_dir .. "/data" },
    { "include", ext.target_dir .. "/include" },
    { "robot_schema", ext.target_dir .. "/usd/schema/isaac/robot_schema" },
    { "sensor_schema", ext.target_dir .. "/usd/schema/isaac/sensor_schema" },
    { "range_sensor_schema", ext.target_dir .. "/usd/schema/isaac/range_sensor_schema" },
    { "python/tests", ext.target_dir .. "/isaacsim/robot/schema/tests" },
    { "python/compat", ext.target_dir .. "/compat" },
}
repo_build.prebuild_copy {
    { "python/__init__.py", ext.target_dir .. "/usd/schema/isaac" },
}
