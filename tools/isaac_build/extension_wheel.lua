-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
-- SPDX-License-Identifier: Apache-2.0

-- Stage independently built library wheels in Kit extension builds.

function stage_isaacsim_extension_wheel(ext, extension_name, stage_sdk_headers)
    local stage = "%{root}/_cmake_build/extension-wheel-stages/%{config}/" .. extension_name
    repo_build.prebuild_link {
        { stage .. "/pip_prebundle", ext.target_dir .. "/pip_prebundle" },
    }
    if stage_sdk_headers then
        repo_build.prebuild_link {
            { stage .. "/sdk/include", ext.target_dir .. "/include" },
        }
    end
end

-- Copy a staged wheel's prebundle into an extension and overlay extension-owned
-- Python adapters at their normal package paths. The copy keeps the shared
-- wheel stage immutable; writing through the ordinary pip_prebundle symlink would
-- modify the CMake-owned stage for every consumer.
function stage_isaacsim_extension_wheel_with_python_overlays(ext, extension_name, python_overlays)
    local stage = "%{root}/_cmake_build/extension-wheel-stages/%{config}/" .. extension_name
    local copies = {
        { stage .. "/pip_prebundle", ext.target_dir .. "/pip_prebundle" },
    }
    for _, path in ipairs(python_overlays) do
        table.insert(copies, { path, ext.target_dir .. "/pip_prebundle/" .. path })
    end
    repo_build.prebuild_copy(copies)
end

function use_isaacsim_module_sdk(extension_name, add_local_runtime_rpath)
    local sdk = "%{root}/_cmake_build/extension-wheel-stages/%{config}/" .. extension_name .. "/sdk"
    includedirs {
        sdk .. "/include",
    }
    libdirs {
        sdk .. "/lib",
    }
    if add_local_runtime_rpath then
        filter { "system:linux" }
        linkoptions { "-Wl,-rpath,'$$ORIGIN/../pip_prebundle/isaacsim/lib'" }
        filter {}
    end
end
