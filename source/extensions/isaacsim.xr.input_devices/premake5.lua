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

-- Python Bindings for Carbonite Plugin
project_ext_bindings {
    ext = ext,
    project_name = "isaacsim.xr.input_devices.python",
    module = "_isaac_xr_input_devices",
    src = "bindings",
    target_subdir = "isaacsim/xr/input_devices",
}
staticruntime("Off")
add_files("impl", "plugins")
add_files("iface", "include")
defines { "ISAACSIM_XR_INPUT_DEVICES_EXPORT" }

-- Always include the basic headers
includedirs {
    "%{root}/source/extensions/isaacsim.core.includes/include",
    "%{root}/source/extensions/isaacsim.xr.input_devices/include",
    "%{root}/source/extensions/isaacsim.xr.input_devices/plugins",
}

-- Include ManusSDK from target-deps
includedirs {
    "%{root}/_build/target-deps/manus_sdk/include",
}
libdirs {
    "%{root}/_build/target-deps/manus_sdk/lib",
}
links {
    "ManusSDK_Integrated",
}

-- Include libsurvive from target-deps
includedirs {
    "%{root}/_build/target-deps/libsurvive/include",
}
libdirs {
    "%{root}/_build/target-deps/libsurvive/lib",
}
links {
    "survive",
}

repo_build.prebuild_link {
    { "python/impl", ext.target_dir .. "/isaacsim/xr/input_devices/impl" },
    { "python/tests", ext.target_dir .. "/isaacsim/xr/input_devices/tests" },
    { "docs", ext.target_dir .. "/docs" },
    { "data", ext.target_dir .. "/data" },
}

repo_build.prebuild_copy {
    { "python/*.py", ext.target_dir .. "/isaacsim/xr/input_devices" },
}
