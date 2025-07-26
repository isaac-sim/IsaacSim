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

-- Shared build scripts from repo_build package
repo_build = require("omni/repo/build")

-- Repo root
root = repo_build.get_abs_path(".")

-- Deprecated Extension Path
deprecated_exts_path = "%{root}/_build/%{platform}/%{config}/extsDeprecated"
-- extensions needed to build isaac sim
isaac_sim_extsbuild_dir = "%{root}/_build/%{platform}/%{config}/extsbuild"
-- Shared build scripts from repo_build package
no_compile_commands_file = false

function isaacsim_build_settings()
    -- Default compilation settings
    exceptionhandling("On")
    rtti("On")
    defines { "__STDC_VERSION__=0" } -- Define this to zero to prevent errors

    filter { "system:windows" }
    defines {
        "HAVE_SNPRINTF",
        "HAVE_COPYSIGN",
        "_SILENCE_ALL_CXX17_DEPRECATION_WARNINGS",
        'BOOST_LIB_TOOLSET="vc142"',
        "_ALLOW_COMPILER_AND_STL_VERSION_MISMATCH",
        "_DISABLE_CONSTEXPR_MUTEX_CONSTRUCTOR",
    }
    disablewarnings { "4996" }
    -- Linux platform settings
    filter { "system:linux" }
    disablewarnings { "error=unused-function" }
    buildoptions { "-Wno-error", "-Wno-deprecated" }

    filter {}
end

function isaacsim_kit_settings()
    -- Setup include paths. Add kit SDK include paths too.
    -- TODO: cleanup once we confirm builds working on windows
    includedirs {
        -- targetDepsDir,
        -- targetDepsDir .. "/pybind11/include",
        -- carbSDKInclude,
        -- kit_sdk .. "/include",
        -- kit_sdk .. "/_build/target-deps/",
        -- kit_dev_dir .. "/include",
        "%{root}/_build/target-deps/gsl/include",
    }

    -- Carbonite carb lib
    libdirs {
        -- carbSDKLibs,
        -- carbSDKLibs .. "/scripting-python-3.10",
        -- "%{kit_sdk}/plugins",
        "%{root}/_build/target-deps/usd/%{cfg.buildcfg}/lib",
    }
end
-- Common folder links
function setup_isaacsim_folder_links()
    repo_build.prebuild_link {
        -- Link app configs in target dir for easier edit
        { "%{root}/source/apps", bin_dir .. "/apps" },
    }

    if not os.isdir(root .. "/_build/PACKAGE-LICENSES") then os.mkdir(root .. "/_build/PACKAGE-LICENSES") end
    

    repo_build.prebuild_link {
        { "source/python_packages", "_build/%{platform}/%{config}/python_packages" },
        { "source/standalone_examples", "_build/%{platform}/%{config}/standalone_examples" },
        { "source/tools", "_build/%{platform}/%{config}/tools" },
    }

    if os.target() == "linux" then
        repo_build.prebuild_link {
            { "source/scripts/python/linux-x86_64/icon", "_build/%{platform}/%{config}/data/icon" },
        }
        -- For docker tests
        repo_build.prebuild_copy {
            { "source/scripts/docker/tests/*", "_build/%{platform}/%{config}/dockertests" },
            -- {"source/scripts/docker/vulkan_check.sh",  "_build/%{platform}/%{config}"},
        }
    end

    repo_build.prebuild_copy {
        { "source/scripts/python/shared/*", "_build/%{platform}/%{config}" },
        { "source/scripts/python/%{platform}/*", "_build/%{platform}/%{config}" },
        { "source/scripts/jupyter_kernel", "_build/%{platform}/%{config}/jupyter_kernel" },
        { "source/scripts/run_tests.py", "_build/%{platform}/%{config}" },
        { "source/scripts/test_config.json", "_build/%{platform}/%{config}/tests" },
        { "source/scripts/warmup${shell_ext}", "_build/%{platform}/%{config}" },
        { "source/scripts/isaac-sim.docker*${shell_ext}", "_build/%{platform}/%{config}" },
        { "source/scripts/clear_caches*${shell_ext}", "_build/%{platform}/%{config}" },
        { "source/scripts/post_install*${shell_ext}", "_build/%{platform}/%{config}" },
        { "source/scripts/vscode/%{platform}", "_build/%{platform}/%{config}/.vscode" },
        { "source/scripts/telemetry/*", "_build/%{platform}/%{config}/config" },
        { "source/scripts/setup_ros_env${shell_ext}", "_build/%{platform}/%{config}" },
    }
end

function include_extensions()
    group("exts")
    for _, ext in ipairs(os.matchdirs(root .. "/source/deprecated/*")) do
        if os.isfile(ext .. "/premake5.lua") then include(ext) end
    end
    for _, ext in ipairs(os.matchdirs(root .. "/source/extensions/*")) do
        if os.isfile(ext .. "/premake5.lua") then include(ext) end
    end
end

function get_git_info(params, env_var)
    local val = os.getenv(env_var)
    if val ~= nil then return val end

    local str = nil
    local cmd = "git " .. params

    local proc = io.popen(cmd)
    if proc then
        str = proc:read("*a")
        local success, what, code = proc:close()
        if success then
            str = string.explode(str, "\n")[1]
        else
            str = nil
        end
    end

    if str == nil then
        str = "MISSING " .. env_var
    else
        print(env_var .. " " .. str)
    end
    return str
end

function generate_version_header()
    shortSha = get_git_info("rev-parse --short HEAD", "ISAACSIM_BUILD_SHA")
    commitDate = get_git_info('show -s --format="%ad"', "ISAACSIM_BUILD_DATE")
    branch = get_git_info("rev-parse --abbrev-ref HEAD", "ISAACSIM_BUILD_BRANCH")
    version = get_git_info("show HEAD:VERSION", "ISAACSIM_BUILD_VERSION")
    repo = get_git_info("config --get remote.origin.url", "ISAACSIM_BUILD_REPO")
    print(
        "Generating version header file: "
            .. branch
            .. " "
            .. shortSha
            .. " "
            .. version
            .. " "
            .. commitDate
            .. " "
            .. repo
    )

    os.mkdir("_build/generated/include/isaacSim")
    local new_text = '#pragma once\n#define ISAACSIM_BUILD_SHA "'
        .. shortSha
        .. '"\n#define ISAACSIM_BUILD_DATE "'
        .. commitDate
        .. '"\n#define ISAACSIM_BUILD_BRANCH "'
        .. branch
        .. '"\n#define ISAACSIM_BUILD_VERSION "'
        .. version
        .. '"\n#define ISAACSIM_BUILD_REPO "'
        .. repo
        .. '"\n'

    local file = io.open("_build/generated/include/isaacSim/Version.h", "r")
    local old_text = ""
    if file then
        old_text = file:read("*all")
        file:close()
    end

    -- if we overwrite Version.h with identical content, anything including that header will always get rebuilt
    -- let's try to avoid that
    if old_text and new_text == old_text then return end

    file = io.open("_build/generated/include/isaacSim/Version.h", "w")
    file:write(new_text)
    file:close(file)
end

-- Helper to create bat/sh files to run local kit files
function define_local_experience(app_name, kit_file, extra_args)
    local extra_args = extra_args or ""
    local kit_file = kit_file or app_name
    define_experience(app_name, {
        config_path = "apps/" .. kit_file .. ".kit",
        extra_args = extra_args,
    })
end

function group_apps(kit)
    group("apps")
    for _, config in ipairs(ALL_CONFIGS) do
        kit.write_version_file(config)
    end

    define_local_experience("isaac-sim", "isaacsim.exp.full")
    define_local_experience("isaac-sim.fabric", "isaacsim.exp.full.fabric")
    define_local_experience("isaac-sim.selector", "isaacsim.exp.selector")
    define_local_experience("isaac-sim.streaming", "isaacsim.exp.full.streaming", "--no-window ")
    define_local_experience("isaac-sim.xr.vr", "isaacsim.exp.base.xr.vr")
    define_local_experience(
        "isaac-sim.action_and_event_data_generation",
        "isaacsim.exp.action_and_event_data_generation.full"
    )
end

nvccPath = path.getabsolute("_build/target-deps/cuda/bin/nvcc")
filter { "system:windows" }
nvccHostCompilerVS = path.getabsolute("_build/host-deps/msvc/VC")
filter {}
-- -- Insert kit template premake configuration, it creates solution, finds extensions.. Look inside for more details.
-- dofile("_repo/deps/repo_kit_tools/kit-template/premake5.lua")
-- mostly so outside code knows we are building isaac-sim
building_for_isaac_sim = true
defines { "BUILDING_FOR_ISAAC_SIM" }

function setup_all(options)
    kit = require(root .. "/_repo/deps/repo_kit_tools/kit-template/premake5-kit")
    repo_build.setup_options()
    kit.include_kit()
    kit.random_hacks()
    kit.define_workspace()

    -- Workspace setup
    kit.workspace_basics()
    kit.workspace_build_settings(options)
    kit.workspace_kit_settings()
    kit.setup_toolchain(options)

    -- Skip this because isaac has to copy additional license files
    -- the below command links rather than copies like we need
    -- so we run setup_isaacsim_folder_links instead
    -- kit.setup_common_folder_links()
    kit.setup_kit_autopull()

    -- Isaac Sim Specific Setup
    include("premake5-isaacsim.lua") -- Shared build scripts from isaac sim
    include("premake5-tests.lua")
    isaacsim_build_settings()
    isaacsim_kit_settings()
    generate_version_header()
    setup_isaacsim_folder_links()
    include_extensions()
    group_apps(kit)
    create_tests()
end

setup_all { cppdialect = "C++17" }
