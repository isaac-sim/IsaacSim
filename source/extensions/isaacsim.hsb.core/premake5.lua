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
    local ext = get_current_extension_info()
    project_ext(ext)

    -- Build the backend library (HSBSender + CUDA kernels)
    project_with_location("isaacsim.hsb.core")
    targetdir(ext.bin_dir)
    kind("SharedLib")
    language("C++")
    cppdialect("C++17")

    pic("On")
    staticruntime("Off")

    add_files("impl", "library/backend")
    add_files("iface", "include")

    files {
        "library/backend/HSBSender.cpp",
        "library/backend/RGBToVB1940Kernels.cu",
    }
    add_cuda_dependencies()

    includedirs {
        "%{root}/source/extensions/isaacsim.core.includes/include",
        "%{root}/source/extensions/isaacsim.hsb.core/include",
        "%{root}/_build/target-deps/hsb_emulator/include",
    }

    libdirs {
        "%{root}/_build/target-deps/hsb_emulator/lib",
        extsbuild_dir .. "/omni.usd.core/bin",
    }

    links {
        "emulation",
        "emulationcoe",
        "emulationroce",
        "emulation_sensors",
        "emulator_utils",
    }

    filter { "system:linux" }
    buildoptions("-fvisibility=default")
    linkoptions { "-Wl,--export-dynamic" }
    filter {}

    -- Build the Carbonite plugin (loads IHsbCore interface)
    project_ext_plugin(ext, "isaacsim.hsb.core.plugin")

    add_files("impl", "plugins")
    cppdialect("C++17")

    includedirs {
        "include",
        "%{root}/source/extensions/isaacsim.hsb.core/include",
    }

    libdirs {
        ext.bin_dir,
    }

    links {
        "isaacsim.hsb.core",
    }

    filter { "system:linux" }
    buildoptions("-fvisibility=default")
    linkoptions { "-Wl,--export-dynamic" }
    filter {}

    filter { "configurations:debug" }
    defines { "_DEBUG" }
    filter { "configurations:release" }
    defines { "NDEBUG" }
    filter {}

    -- Python bindings (triggers native plugin load)
    project_ext_bindings {
        ext = ext,
        project_name = "isaacsim.hsb.core.python",
        module = "_hsb_core",
        src = "bindings/isaacsim.hsb.core",
        target_subdir = "isaacsim/hsb/core/bindings",
    }
    add_files("bindings", "bindings/isaacsim.hsb.core/*.*")
    add_files("python", "python/*.py")

    includedirs {
        "include",
        "%{root}/source/extensions/isaacsim.hsb.core/include",
    }

    -- Build the C++ tests
    project_ext_tests(ext, "isaacsim.hsb.core.tests")
    cppdialect("C++17")
    add_files("source", "library/tests")
    add_cuda_dependencies()

    includedirs {
        "%{root}/source/extensions/isaacsim.core.includes/include",
        "%{root}/source/extensions/isaacsim.hsb.core/include",
        "%{root}/_build/target-deps/hsb_emulator/include",
        "%{target_deps}/doctest/include",
    }

    libdirs {
        extsbuild_dir .. "/omni.kit.test/bin",
        ext.bin_dir,
        "%{root}/_build/target-deps/hsb_emulator/lib",
    }

    links {
        "isaacsim.hsb.core",
        "emulation",
        "emulationcoe",
        "emulationroce",
        "emulation_sensors",
        "emulator_utils",
    }

    filter { "configurations:debug" }
    defines { "_DEBUG" }
    filter { "configurations:release" }
    defines { "NDEBUG" }
    filter {}

    -- Copy/link necessary files for packaging
    repo_build.prebuild_copy {
        { "python/__init__.py", ext.target_dir .. "/isaacsim/hsb/core" },
        { "python/extension.py", ext.target_dir .. "/isaacsim/hsb/core" },
        { "bindings/__init__.py", ext.target_dir .. "/isaacsim/hsb/core/bindings" },
    }

    repo_build.prebuild_link {
        { "docs", ext.target_dir .. "/docs" },
        { "data", ext.target_dir .. "/data" },
        { "include", ext.target_dir .. "/include" },
    }
else
    print("SKIPPING BUILD - Only supported on linux-x86_64")
end
