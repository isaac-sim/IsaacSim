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

function include_physx()
    defines { "PX_PHYSX_STATIC_LIB" }

    filter { "system:windows" }
    libdirs { "%{root}/_build/target-deps/nvtx/lib/x64" }
    filter {}
    filter { "system:linux" }
    libdirs { "%{root}/_build/target-deps/nvtx/lib64" }
    filter {}

    filter { "configurations:debug" }
    defines { "_DEBUG" }
    filter { "configurations:release" }
    defines { "NDEBUG" }
    filter {}

    filter { "system:windows", "platforms:x86_64", "configurations:debug" }
    libdirs {
        "%{root}/_build/target-deps/physx/bin/win.x86_64.vc142.md/debug",
    }
    filter { "system:windows", "platforms:x86_64", "configurations:release" }
    libdirs {
        "%{root}/_build/target-deps/physx/bin/win.x86_64.vc142.md/checked",
    }
    filter {}

    filter { "system:linux", "platforms:x86_64", "configurations:debug" }
    libdirs {
        "%{root}/_build/target-deps/physx/bin/linux.x86_64/debug",
    }
    filter { "system:linux", "platforms:x86_64", "configurations:release" }
    libdirs {
        "%{root}/_build/target-deps/physx/bin/linux.x86_64/checked",
    }
    filter {}

    links {
        "PhysXExtensions_static_64",
        "PhysX_static_64",
        "PhysXPvdSDK_static_64",
        "PhysXCooking_static_64",
        "PhysXCommon_static_64",
        "PhysXFoundation_static_64",
    }

    includedirs {
        "%{root}/_build/target-deps/physx/include",
        "%{root}/_build/target-deps/usd_ext_physics/%{cfg.buildcfg}/include",
    }
end

-- This function binds the runtime and settings to be compatiable with kit. This means
-- that even though our debug builds
function setRuntimeToBeKitCompatible()
    if kit_sdk_config == "debug" then
        runtime("Debug")
        defines { "_DEBUG" }
    elseif kit_sdk_config == "release" then
        runtime("Release")
        defines { "NDEBUG" }
    else
        filter { "configurations:debug" }
        runtime("Debug")
        defines { "_DEBUG" }
        filter { "configurations:release" }
        runtime("Release")
        defines { "NDEBUG" }
        filter {}
    end
end

-- Helper function to implement a build step that preprocesses .cu files (CUDA code) for compilation
function make_nvcc_command(nvccPath, nvccHostCompilerVS, nvccHostCompilerFlags, nvccFlags, dependencies, isPtx)
    isPtx = isPtx or false
    local nvccXcompilerFlags = string.gsub(nvccHostCompilerFlags, "^%s*(.-)%s*$", "%1") -- trim leading and trailing spaces from the host compiler flags
    -- -- the flags in the following section precompiles for all DRIVESIM supported arch
    -- local sass = "-gencode=arch=compute_75,code=sm_75 ".. -- Turing
    -- "-gencode=arch=compute_80,code=sm_80 ".. -- Ampere
    -- "-gencode=arch=compute_86,code=sm_86 "..
    -- "-gencode=arch=compute_87,code=sm_87 "..
    -- "-gencode=arch=compute_89,code=sm_89 ".. -- Ada
    -- "-gencode=arch=compute_90,code=sm_90 "   -- Hopper

    -- For forward compatibility with future architectures we also include a Hopper-based PTX that can be JIT compiled at runtime
    -- local ptx = "-gencode=arch=compute_90,code=compute_90 "
    -- local nvccCompiler = nvcc114Path
    -- if isPtx then
    --     sass = ""
    --     ptx = ""
    --     nvccCompiler = nvcc114Path
    -- end
    if os.target() == "windows" then
        ext = ".obj"
        if isPtx then ext = ".ptx" end
        local compilerBindir = " --compiler-bindir " .. nvccHostCompilerVS
        local buildString = '"'
            .. nvccPath
            .. '" -std=c++17 '
            .. nvccFlags
            .. compilerBindir
            .. " -Xcompiler="
            .. iif(not nvccXcompilerFlags or #nvccXcompilerFlags == 0, "", nvccXcompilerFlags .. ",")
            .. '%{iif(not cfg.staticruntime or cfg.staticruntime ~= "On", "/MD", "/MT")..iif(not cfg.runtime or cfg.runtime == "Debug", "d", "")}'
            .. " -c -I "
            .. carbSDKInclude
            .. " -D_ALLOW_COMPILER_AND_STL_VERSION_MISMATCH -DBUILDING_FOR_ISAAC_SIM %{get_include_string(cfg.includedirs)} %{file.abspath} -o %{cfg.objdir}/%{file.basename}"
            .. ext
        buildmessage(buildString)
        buildcommands { buildString }
        buildoutputs { "%{cfg.objdir}/%{file.basename}" .. ext }
        buildinputs { dependencies }
    end
    if os.target() == "linux" then
        ext = ".o"
        if isPtx then ext = ".ptx" end
        local buildString = '"'
            .. nvccPath
            .. '" -std=c++17 '
            .. nvccFlags
            .. " -Xcompiler="
            .. commaficate(nvccXcompilerFlags)
            .. " -c -I "
            .. carbSDKInclude
            .. " -DBUILDING_FOR_ISAAC_SIM %{get_include_string(cfg.includedirs)} %{file.abspath} -o %{cfg.objdir}/%{file.basename}"
            .. ext
        buildcommands { "{MKDIR} %{cfg.objdir} ", buildString }
        buildoutputs { "%{cfg.objdir}/%{file.basename}" .. ext }
        buildinputs { dependencies }
    end
end

function make_ptx_header(nvccPath, nvccHostCompilerVS, nvccHostCompilerFlags, nvccFlags, dependencies)
    make_nvcc_command(nvccPath, nvccHostCompilerVS, nvccHostCompilerFlags, nvccFlags, dependencies, true)
    buildcommands {
        "{MKDIR} %{cfg.objdir} ",
        bin2cPath .. " -st -c -p 0 " .. "%{cfg.objdir}/%{file.basename}.ptx > %{cfg.objdir}/%{file.basename}.ptx.h",
    }
end

-- Helper function to call in your project definition when you have .cu files to process.
-- This sets up the CUDA compilation for all files within your project using the correct rules for each configuration.
function add_cuda_dependencies()
    -- First the build rules for CUDA files
    setRuntimeToBeKitCompatible()

    filter { "files:**.cu", "system:windows", "configurations:debug" }
    make_nvcc_command(nvccPath, nvccHostCompilerVS, "/Od", "-g -G -allow-unsupported-compiler")
    filter { "files:**.cu", "system:windows", "configurations:release" }
    make_nvcc_command(nvccPath, nvccHostCompilerVS, "", "-allow-unsupported-compiler")
    filter { "files:**.cu", "system:linux", "configurations:debug" }
    make_nvcc_command(nvccPath, nvccHostCompilerVS, "-fPIC -g", "-g -G -allow-unsupported-compiler")
    filter { "files:**.cu", "system:linux", "configurations:release" }
    make_nvcc_command(nvccPath, nvccHostCompilerVS, "-fPIC", "-allow-unsupported-compiler")
    filter {}

    -- link against CUDA runtime static library.
    links { "cudart_static" }

    -- Add in the library directories
    filter { "system:linux" }
    -- lib dir stubs in case you link against 'cuda'.
    libdirs { target_deps .. "/cuda/lib64/stubs" }
    -- lib dir in case you link against 'cudart_static'.
    libdirs { target_deps .. "/cuda/lib64/" }
    links { "cuda" }

    -- linking to cudart_static requires libpthread, libdl, and librt
    -- https://gitlab.kitware.com/cmake/cmake/-/issues/20249
    buildoptions { "-pthread" }
    links { "dl", "pthread", "rt" }
    filter { "system:windows" }
    libdirs { target_deps .. "/cuda/lib/x64" }
    links { "cuda" }
    filter {}

    -- CUDA-specific include directory
    includedirs { target_deps .. "/cuda/include" }
end

-- Define experience to test one particular extension.
-- @ext_name: Extension name.
-- @python_module: Python module name, if different from extension name. (optional)
function define_ext_test_experience(ext_name, args)
    local args = args or {}

    local python_module = get_value_or_default(args, "python_module", ext_name)
    local script_dir_token = (os.target() == "windows") and "%~dp0" or "$SCRIPT_DIR"
    local test_args = {
        "--empty", -- Start empty kit
        "--enable omni.kit.test", -- We always need omni.kit.test extension as testing framework
        "--/exts/omni.kit.test/testExtEnableProfiler=0",
        '--/exts/omni.kit.test/testExtArgs/0="--no-window"',
        '--/exts/omni.kit.test/testExtArgs/1="--allow-root"',
        "--/exts/omni.kit.test/runTestsAndQuit=true", -- Run tests and quit
        "--/exts/omni.kit.test/testExts/0='" .. python_module .. "'", -- Only include tests from the python module
        '--ext-folder "' .. script_dir_token .. '/../exts" ',
        '--ext-folder "' .. script_dir_token .. '/../extscache" ',
        '--ext-folder "' .. script_dir_token .. '/../extsDeprecated" ',
        '--ext-folder "' .. script_dir_token .. '/../apps" ',
        "--/app/enableStdoutOutput=0", -- this app just runs the test command, hide its output
        "--no-window",
        "--allow-root",
        "--/telemetry/mode=test",
        '--/crashreporter/data/testName="ext-test-' .. python_module .. '"',
    }

    -- Allow passing additional args
    local extra_test_args = get_value_or_default(args, "extra_test_args", {})
    test_args = concat_arrays(test_args, extra_test_args)

    local suite = get_value_or_default(args, "suite", EXT_TEST_TEST_SUITE_DEFAULT)

    local exp_args = {
        config_path = "",
        extra_args = table.concat(test_args, " "),
        define_project = false,
    }
    exp_args = merge_tables(exp_args, args)

    local test_name = suite and string.format("tests-%s-%s", suite, ext_name) or ("tests-" .. ext_name)
    define_test_experience(test_name, exp_args)
end

-- Define Kit experience. Different ways to run kit with particular config
function define_test_experience(name, args)
    local args = args or {}
    local experience = args.experience or name .. ".json"
    local config_path = get_value_or_default(args, "config_path", "experiences/" .. experience)
    local extra_args = args.extra_args or ""
    -- Write bat and sh files as another way to run them:
    for _, config in ipairs(ALL_CONFIGS) do
        local kit_sdk_config = get_value_or_default(args, "kit_sdk_config", kit_sdk_config)
        if kit_sdk_config == "%{config}" then kit_sdk_config = config end
        create_test_experience_runner(name, config_path, config, kit_sdk_config, extra_args)
    end
end
ROS2_EXTRA = {
    ["windows"] = [[
set RMW_IMPLEMENTATION=rmw_fastrtps_cpp
set ROS_DOMAIN_ID=93
pushd %~dp0\..\exts
set basedir=%cd%\isaacsim.ros2.bridge\humble\lib
popd
set PATH=%PATH%;%basedir%
]],
    ["linux"] = [[
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=$((($RANDOM % 18) + 80))
INTERNAL_LIBS=$(readlink -f $SCRIPT_DIR/../exts/isaacsim.ros2.bridge/humble/lib)
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$INTERNAL_LIBS
]],
}
-- Write experience running .bat/.sh file, like _build\windows-x86_64\release\example.helloext.app.bat
function create_test_experience_runner(name, config_path, config, kit_sdk_config, extra_args, executable)
    local os_target = os.target()
    if
        string.find(name, "ros2")
        or string.find(name, "omni.isaac.benchmarks")
        or string.find(name, "tf_viewer")
        or string.find(name, "isaacsim.app.setup")
        or string.find(name, "isaacsim.test.collection")
    then
        extra = ROS2_EXTRA[os_target]
    else
        extra = ""
    end
    if os_target == "windows" then
        local executable = executable or "kit.exe"
        local bat_file_dir = root .. "/_build/windows-x86_64/" .. config .. "/tests"
        local bat_file_path = bat_file_dir .. "/" .. name .. ".bat"
        local kit_bin_abs = string_fmt_vars_recursive(kit_sdk_bin_dir, {
            root = root,
            config = config,
            kit_sdk = kit_sdk,
            kit_sdk_config = kit_sdk_config,
            platform = "windows-x86_64",
        })
        local kit_bin_relative = path.normalize(path.getrelative(bat_file_dir, kit_bin_abs)):gsub("/", "\\")
        local config_path = (is_string_empty(config_path) and "") or '"%%~dp0' .. config_path .. '"'
        local f = io.open(bat_file_path, "w")
        f:write(
            string.format(
                KIT_TEST_SHELL_TEMPLATE[os_target],
                extra,
                kit_bin_relative,
                executable,
                config_path,
                extra_args
            )
        )
        f:close()
    else
        local executable = executable or "kit"
        local arch = io.popen("arch", "r"):read("*l")
        local platform_name = "linux"

        if os_target == "macosx" then
            arch = "universal"
            platform_name = "macos"
        end

        local sh_file_dir = root .. "/_build/" .. platform_name .. "-" .. arch .. "/" .. config .. "/tests"
        local sh_file_path = sh_file_dir .. "/" .. name .. ".sh"
        local kit_bin_abs = string_fmt_vars_recursive(kit_sdk_bin_dir, {
            root = root,
            config = config,
            kit_sdk = kit_sdk,
            kit_sdk_config = kit_sdk_config,
            platform = platform_name .. "-" .. arch,
        })
        local kit_bin_relative = path.normalize(path.getrelative(sh_file_dir, kit_bin_abs))
        local config_path = (is_string_empty(config_path) and "") or '"$SCRIPT_DIR/' .. config_path .. '"'
        local f = io.open(sh_file_path, "w")
        f:write(
            string.format(
                KIT_TEST_SHELL_TEMPLATE[os_target],
                extra,
                kit_bin_relative,
                executable,
                config_path,
                extra_args
            )
        )
        f:close()
        os.chmod(sh_file_path, 755)
    end
end

function python_sample_test(name, sample_path, args)
    local extra_args = args or ""
    for _, config in ipairs(ALL_CONFIGS) do
        create_python_sample_runner(name, sample_path, config, extra_args)
    end
end
function create_python_sample_runner(name, sample_path, config, extra_args)
    local os_target = os.target()
    if string.find(name, "ros2") then
        extra = ROS2_EXTRA[os_target]
    else
        extra = ""
    end
    if os.target() == "linux" then
        local sh_file_dir = root .. "/_build/linux-x86_64/" .. config .. "/tests"
        local sh_file_path = sh_file_dir .. "/" .. name .. ".sh"
        local f = io.open(sh_file_path, "w")
        print(sh_file_path)
        f:write(string.format(
            [[
#!/bin/bash
set -e
SCRIPT_DIR=$(dirname ${BASH_SOURCE})
SAMPLE_DIR=$SCRIPT_DIR/../
%s
"$SCRIPT_DIR/../python.sh" -m pip install -r $SCRIPT_DIR/../requirements.txt
"$SCRIPT_DIR/../python.sh" $SAMPLE_DIR/%s %s $@ --no-window
        ]],
            extra,
            sample_path,
            extra_args
        ))
        f:close()
        os.chmod(sh_file_path, 755)
    else
        local bat_file_dir = root .. "/_build/windows-x86_64/" .. config .. "/tests"
        local bat_file_path = bat_file_dir .. "/" .. name .. ".bat"

        local f = io.open(bat_file_path, "w")
        print(bat_file_path)
        f:write(string.format(
            [[
@echo off
setlocal
%s
call "%%~dp0..\python.bat" -m pip install -r "%%~dp0..\requirements.txt"
call "%%~dp0..\python.bat" "%%~dp0..\%s" %s %%*
        ]],
            extra,
            sample_path,
            extra_args
        ))
        f:close()
    end
end

function jupyter_sample_test(name, sample_path, args)
    local extra_args = args or ""
    for _, config in ipairs(ALL_CONFIGS) do
        jupyter_sample_runner(name, sample_path, config, extra_args)
    end
end
function jupyter_sample_runner(name, sample_path, config, extra_args)
    if os.target() == "linux" then
        local sh_file_dir = root .. "/_build/linux-x86_64/" .. config .. "/tests"
        local sh_file_path = sh_file_dir .. "/" .. name .. ".sh"
        local f = io.open(sh_file_path, "w")
        print(sh_file_path)
        f:write(string.format(
            [[
#!/bin/bash
set -e
SCRIPT_DIR=$(dirname ${BASH_SOURCE})
SAMPLE_DIR=$SCRIPT_DIR/../
"$SCRIPT_DIR/../jupyter_notebook.sh" test $SAMPLE_DIR/%s %s $@
        ]],
            sample_path,
            extra_args
        ))
        f:close()
        os.chmod(sh_file_path, 755)
    end
end

-- Template Used to generate all the kit.bat, test.bat and other batch/shell files
-- format are: bin_relative, executable, config_path, extra_args
KIT_RUNNER_SHELL_TEMPLATE = {
    ["windows"] = [[
@echo off
setlocal
set SCRIPT_DIR=%%~dp0
set NO_ROS_ENV=false

REM if this is the selector script, don't set up ros env
if /i "%%~n0"=="isaac-sim.selector" set NO_ROS_ENV=true

REM Check args for a flag to disable ROS environment setup
for %%%%a in (%%*) do (
    if "%%%%a"=="--no-ros-env" (
        set NO_ROS_ENV=true
        echo Skipping automatic ROS environment setup
        goto :continue
    )
)

:continue
REM Source ROS environment setup script if flag was not found
if "%%NO_ROS_ENV%%"=="false" if exist "%%SCRIPT_DIR%%setup_ros_env.bat" (
    call "%%SCRIPT_DIR%%setup_ros_env.bat"
)

call "%%~dp0%s\%s" %s %s %%*
]],
    ["linux"] = [[
#!/bin/bash
set -e
SCRIPT_DIR=$(dirname ${BASH_SOURCE})
export RESOURCE_NAME="IsaacSim"
export OLD_PYTHONPATH=$PYTHONPATH

# Check args for a flag to disable ROS environment setup
NO_ROS_ENV=false
for arg in "$@"; do
    if [ "$arg" == "--no-ros-env" ]; then
        NO_ROS_ENV=true
        echo "Skipping automatic ROS environment setup"
        break
    fi
done

# Source ROS environment setup script if flag was not found
if [ "$NO_ROS_ENV" == "false" ] && [ -f "$SCRIPT_DIR/setup_ros_env.sh" ]; then
    source "$SCRIPT_DIR/setup_ros_env.sh"
fi

exec "$SCRIPT_DIR/%s/%s" %s %s "$@"
]],
    ["macosx"] = [[
#!/bin/bash
set -e
SCRIPT_DIR=$(dirname ${BASH_SOURCE})
exec "$SCRIPT_DIR/%s/%s" %s %s "$@"
]],
}

KIT_TEST_SHELL_TEMPLATE = {
    ["windows"] = [[
@echo off
setlocal
%s
call "%%~dp0%s\%s" %s %s %%*
]],
    ["linux"] = [[
#!/bin/bash
set -e
SCRIPT_DIR=$(dirname ${BASH_SOURCE})
export RESOURCE_NAME="IsaacSim"
%s
exec "$SCRIPT_DIR/%s/%s" %s %s "$@"
]],
    ["macosx"] = [[
#!/bin/bash
set -e
SCRIPT_DIR=$(dirname ${BASH_SOURCE})
exec "$SCRIPT_DIR/%s/%s" %s %s "$@"
]],
}

function python_script_test(name, script)
    for _, config in ipairs(ALL_CONFIGS) do
        create_python_script_runner(name, script, config)
    end
end
function create_python_script_runner(name, script, config)
    if os.target() == "linux" then
        local sh_file_dir = root .. "/_build/linux-x86_64/" .. config .. "/tests"
        local sh_file_path = sh_file_dir .. "/" .. name .. ".sh"
        local f = io.open(sh_file_path, "w")
        print(sh_file_path)
        f:write(string.format(
            [[
#!/bin/bash
set -e
SCRIPT_DIR=$(dirname ${BASH_SOURCE})
SAMPLE_DIR=$SCRIPT_DIR/../
"$SCRIPT_DIR/../python.sh"  %s
        ]],
            script
        ))
        f:close()
        os.chmod(sh_file_path, 755)
    else
        local bat_file_dir = root .. "/_build/windows-x86_64/" .. config .. "/tests"
        local bat_file_path = bat_file_dir .. "/" .. name .. ".bat"

        local f = io.open(bat_file_path, "w")
        print(bat_file_path)
        f:write(string.format(
            [[
@echo off
setlocal
"%%~dp0..\python.bat" %s
        ]],
            script
        ))
        f:close()
    end
end

