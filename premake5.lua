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

-- Translate the deprecated FatalCompileWarnings flag used by older fetched Kit
-- build helpers. This keeps project generation warning-free without modifying
-- Packman-managed files, and can be removed once those helpers use the current
-- Premake API directly.
local isaacsim_flags = flags
local isaacsim_removeflags = removeflags

local function translate_fatal_compile_warnings(values, handler, fatal_handler)
    if type(values) ~= "table" then
        handler(values)
        return
    end

    local translated_values = {}
    local has_fatal_compile_warnings = false
    for _, value in ipairs(values) do
        if value == "FatalCompileWarnings" then
            has_fatal_compile_warnings = true
        else
            table.insert(translated_values, value)
        end
    end
    if #translated_values > 0 then
        handler(translated_values)
    end
    if has_fatal_compile_warnings then
        fatal_handler { "All" }
    end
end

function flags(values)
    translate_fatal_compile_warnings(values, isaacsim_flags, fatalwarnings)
end

function removeflags(values)
    translate_fatal_compile_warnings(values, isaacsim_removeflags, removefatalwarnings)
end

-- Repo root
root = repo_build.get_abs_path(".")

isaac_sim_extra_extsbuild_dir = "%{root}/_build/%{platform}/%{config}/extsbuild"
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
    
    removefatalwarnings { "All" }

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
    includedirs {
        "%{root}/_build/target-deps/gsl/include",
        "%{root}/_build/target-deps/rtx_plugins/include",
    }

    -- Carbonite carb lib
    libdirs {
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


    repo_build.prebuild_copy {
        { "_build/generated/ThirdPartyNotices.txt", bin_dir .. "/ThirdPartyNotices.txt" },
    }

    repo_build.prebuild_link {
        { "source/python_packages", "_build/%{platform}/%{config}/python_packages" },
        { "source/standalone_examples", "_build/%{platform}/%{config}/standalone_examples" },
        { "source/tools", "_build/%{platform}/%{config}/tools" },
    }

    -- Agent skills and pointer guides. Assemble a public-only skills/ tree plus the AGENTS.md
    -- and CLAUDE.md guides in the build output so they ship in both the binary and pip packages
    -- without users cloning the repo. Dev-only skills under skills/_internal are skipped, and the
    -- public index/guide are shipped under their canonical names (the internal repo keeps
    -- SKILLS.public.md / AGENTS.public.md; the public repo already uses SKILLS.md / AGENTS.md).
    -- Per-directory entries keep the skill list current as skills are added.
    local cfg_dst = "_build/%{platform}/%{config}"
    local agent_stage = {}
    for _, skill_dir in ipairs(os.matchdirs(root .. "/skills/*")) do
        local skill_name = path.getname(skill_dir)
        if skill_name ~= "_internal" then
            table.insert(agent_stage, { "skills/" .. skill_name, cfg_dst .. "/skills/" .. skill_name })
        end
    end
    if os.isfile(root .. "/skills/SKILLS.public.md") then
        table.insert(agent_stage, { "skills/SKILLS.public.md", cfg_dst .. "/skills/SKILLS.md" })
    elseif os.isfile(root .. "/skills/SKILLS.md") then
        table.insert(agent_stage, { "skills/SKILLS.md", cfg_dst .. "/skills/SKILLS.md" })
    end
    if os.isfile(root .. "/AGENTS.public.md") then
        table.insert(agent_stage, { "AGENTS.public.md", cfg_dst .. "/AGENTS.md" })
    elseif os.isfile(root .. "/AGENTS.md") then
        table.insert(agent_stage, { "AGENTS.md", cfg_dst .. "/AGENTS.md" })
    end
    if os.isfile(root .. "/CLAUDE.md") then
        table.insert(agent_stage, { "CLAUDE.md", cfg_dst .. "/CLAUDE.md" })
    end
    repo_build.prebuild_copy(agent_stage)

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
        { "LICENSE", "_build/%{platform}/%{config}/APACHE-2.0-LICENSE.txt" },
        { "source/scripts/python/shared/*", "_build/%{platform}/%{config}" },
        { "source/scripts/python/%{platform}/*", "_build/%{platform}/%{config}" },
        { "source/scripts/jupyter_kernel", "_build/%{platform}/%{config}/jupyter_kernel" },
        { "source/scripts/run_tests.py", "_build/%{platform}/%{config}" },
        { "source/scripts/test_config.json", "_build/%{platform}/%{config}/tests" },
        { "source/scripts/warmup${shell_ext}", "_build/%{platform}/%{config}" },
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

    local branch
    branch = branch or get_git_info("rev-parse --abbrev-ref HEAD", "ISAACSIM_BUILD_BRANCH")

    -- Always print the final branch value
    print("ISAACSIM_BUILD_BRANCH " .. (branch or "UNKNOWN"))

    version = get_git_info("show HEAD:VERSION", "ISAACSIM_BUILD_VERSION")
    repo = get_git_info("config --get remote.origin.url", "ISAACSIM_BUILD_REPO")
    repo = string.gsub(repo, "(https://)([^@]+)@", "%1")
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
    define_local_experience("isaac-sim.newton", "isaacsim.exp.full.newton")
    define_local_experience("isaac-sim.newton.streaming", "isaacsim.exp.full.newton.streaming", "--no-window ")
    define_local_experience("isaac-sim.compatibility_check", "isaacsim.exp.compatibility_check")
    define_local_experience("isaac-sim.streaming", "isaacsim.exp.full.streaming", "--no-window ")
    if os.hostarch() == "x86_64" then
        define_local_experience("isaac-sim.xr.vr", "isaacsim.exp.base.xr.vr")
    end
    define_local_experience(
        "isaac-sim.action_and_event_data_generation",
        "isaacsim.exp.action_and_event_data_generation.full"
    )
end


-- Check for system NVCC first, but only on ARM64 (aarch64) where glibc compatibility issues exist
local osHostArch = os.hostarch()
local osTarget = os.target()
local systemNvccPath = "/usr/local/cuda/bin/nvcc"
nvccPath = path.getabsolute("_build/target-deps/cuda/bin/nvcc")
cudaIncludePath = path.getabsolute("_build/target-deps/cuda/include")
cudaLibPathLinux = path.getabsolute("_build/target-deps/cuda/lib64")
if osTarget == "linux" and os.isfile(systemNvccPath) and (osHostArch == "aarch64" or osHostArch == "arm64" or osHostArch == "ARM64") then
    nvccPath = systemNvccPath
    cudaIncludePath = "/usr/local/cuda/include"
    cudaLibPathLinux = "/usr/local/cuda/lib64"
end
print("Using NVCC binary: " .. nvccPath)
print("Using CUDA includes directory: " .. cudaIncludePath)
print("Using CUDA libs directory: " .. cudaLibPathLinux)

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
