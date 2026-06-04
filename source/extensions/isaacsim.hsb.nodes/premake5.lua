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
    local ogn = get_ogn_project_information(ext, "isaacsim/hsb/nodes")
    project_ext(ext)

    -- Build OmniGraph nodes plugin (C++ OGN nodes)
    project_ext_plugin(ext, "isaacsim.hsb.nodes.plugin")

    add_files("impl", "plugins")
    add_files("ogn", ogn.nodes_path)

    add_ogn_dependencies(ogn, { "python/nodes" })
    add_ogn_dependencies(ogn, { "nodes" })

    cppdialect("C++17")
    add_cuda_dependencies()

    includedirs {
        "include",
        "%{root}/source/extensions/isaacsim.core.includes/include",
        "%{root}/source/extensions/isaacsim.hsb.core/include",
        "%{root}/source/extensions/isaacsim.hsb.nodes/include",
        "%{root}/_build/target-deps/hsb_emulator/include",
        "%{root}/_build/target-deps/nlohmann_json/include",
        "%{root}/_build/target-deps/omni_physics/%{config}/include",
        "%{root}/_build/target-deps/usd/%{cfg.buildcfg}/include",
        "%{kit_sdk_bin_dir}/dev/fabric/include/",
    }

    libdirs {
        extsbuild_dir .. "/isaacsim.hsb.core/bin",
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

    add_usd()

    filter { "system:linux" }
    disablewarnings { "error=narrowing", "error=unused-but-set-variable", "error=unused-variable" }
    buildoptions("-fvisibility=default")
    linkoptions { "-Wl,--export-dynamic" }
    filter {}

    filter { "configurations:debug" }
    defines { "_DEBUG" }
    filter { "configurations:release" }
    defines { "NDEBUG" }
    filter {}

    -- Generate the OGN project (handles .ogn files and generates database code)
    project_ext_ogn(ext, ogn)

    -- Python bindings for the plugin (triggers native plugin load and OGN registration)
    project_ext_bindings {
        ext = ext,
        project_name = ogn.python_project,
        module = "_hsb_nodes",
        src = ogn.bindings_path,
        target_subdir = ogn.bindings_target_path,
    }
    add_files("bindings", "bindings/isaacsim.hsb.nodes/*.*")
    add_files("python", "python/*.py")
    add_files("python/tests", "python/tests/*.py")

    includedirs {
        "include",
        "%{root}/source/extensions/isaacsim.hsb.nodes/include",
    }

    -- Copy/link necessary files for packaging
    repo_build.prebuild_copy {
        { "python/__init__.py", ogn.python_target_path },
        { "python/extension.py", ogn.python_target_path },
        { "python/tests/__init__.py", ogn.python_target_path .. "/tests" },
        { "python/tests/*.py", ogn.python_target_path .. "/tests" },
        { "bindings/__init__.py", ogn.python_target_path .. "/bindings" },
    }

    repo_build.prebuild_link {
        { "docs", ext.target_dir .. "/docs" },
        { "data", ext.target_dir .. "/data" },
        { "include", ext.target_dir .. "/include" },
        { "nodes", ext.target_dir .. "/nodes" },
        { "python/nodes", ogn.python_target_path .. "/nodes" },
    }
else
    print("SKIPPING BUILD - Only supported on linux-x86_64")
end
