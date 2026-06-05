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

if os.target() == "linux" then
    local ext = get_current_extension_info()
    project_ext(ext)

    -- Build the backend library
    project_with_location("isaacsim.ucx.core")
    targetdir(ext.bin_dir)
    kind("SharedLib")
    language("C++")
    cppdialect("C++17")

    pic("On")
    staticruntime("Off")

    add_files("impl", "library/backend")
    add_files("iface", "include")

    includedirs {
        "%{root}/source/extensions/isaacsim.core.includes/include",
        "%{root}/source/extensions/isaacsim.ucx.core/include",
        "%{root}/_build/target-deps/pip_ucx_prebundle/librmm/include",
        "%{root}/_build/target-deps/pip_ucx_prebundle/libucx/include",
        "%{root}/_build/target-deps/pip_ucx_prebundle/libucxx/include",
        "%{root}/_build/target-deps/pip_ucx_prebundle/rapids_logger/include",
        "%{kit_sdk_bin_dir}/dev/fabric/include/",
    }

    libdirs {
        "%{root}/_build/target-deps/pip_ucx_prebundle/librmm/lib64",
        "%{root}/_build/target-deps/pip_ucx_prebundle/libucx/lib",
        "%{root}/_build/target-deps/pip_ucx_prebundle/libucxx/lib64",
        "%{root}/_build/target-deps/pip_ucx_prebundle/rapids_logger/lib64",
        extsbuild_dir .. "/omni.usd.core/bin",
    }

    links {
        "ucxx",
        "ucp",
        "ucs",
        "uct",
        "ucm",
        "rmm",
        "rapids_logger",
    }

    filter { "system:linux" }
    disablewarnings { "error=pragmas" }
    buildoptions("-fvisibility=default")
    linkoptions { "-Wl,--export-dynamic" }
    filter { "system:windows" }
    buildoptions("-D_CRT_SECURE_NO_WARNINGS")
    filter {}

    filter { "configurations:debug" }
    defines { "_DEBUG" }
    filter { "configurations:release" }
    defines { "NDEBUG" }
    filter {}

    repo_build.prebuild_link {
        { "docs", ext.target_dir .. "/docs" },
        { "data", ext.target_dir .. "/data" },
        { "include", ext.target_dir .. "/include" },
        { "$root/_build/target-deps/pip_ucx_prebundle", ext.target_dir .. "/pip_prebundle" },
    }

    repo_build.prebuild_copy {
        { "python/*.py", ext.target_dir .. "/isaacsim/ucx/core" },
    }

    repo_build.prebuild_link {
        { "python/tests", ext.target_dir .. "/isaacsim/ucx/core/tests" },
    }

    repo_build.prebuild_copy {
        { "%{root}/_build/target-deps/pip_ucx_prebundle/librmm/lib64/lib**", ext.bin_dir .. "/" },
        { "%{root}/_build/target-deps/pip_ucx_prebundle/libucx/lib/lib**", ext.bin_dir .. "/" },
        { "%{root}/_build/target-deps/pip_ucx_prebundle/libucxx/lib64/lib**", ext.bin_dir .. "/" },
        { "%{root}/_build/target-deps/pip_ucx_prebundle/rapids_logger/lib64/lib**", ext.bin_dir .. "/" },
        -- UCX CUDA transport plugins and their CUDA runtime dep go into bin/ucx/ so that
        -- UCX_MODULE_DIR (set at runtime via dladdr) points to them, and libucm_cuda.so.0
        -- can resolve libcudart-256e6409.so.12.9.79 without needing bin/ucx/ in LD_LIBRARY_PATH.
        { "%{root}/_build/target-deps/pip_ucx_prebundle/libucx/lib/ucx/libuct_cuda**", ext.bin_dir .. "/ucx/" },
        { "%{root}/_build/target-deps/pip_ucx_prebundle/libucx/lib/ucx/libucm_cuda**", ext.bin_dir .. "/ucx/" },
        { "%{root}/_build/target-deps/pip_ucx_prebundle/libucx_cu12.libs/libcudart**", ext.bin_dir .. "/ucx/" },
    }

    -- Python Bindings
    project_ext_bindings {
        ext = ext,
        project_name = "isaacsim.ucx.core.python",
        module = "_ucx_core",
        src = "bindings",
        target_subdir = "isaacsim/ucx/core/bindings",
    }
    add_files("bindings", "bindings/*.*")

    includedirs {
        "%{root}/source/extensions/isaacsim.ucx.core/include",
        "%{root}/source/extensions/isaacsim.core.includes/include",
        "%{root}/_build/target-deps/pip_ucx_prebundle/librmm/include",
        "%{root}/_build/target-deps/pip_ucx_prebundle/libucx/include",
        "%{root}/_build/target-deps/pip_ucx_prebundle/libucxx/include",
        "%{root}/_build/target-deps/pip_ucx_prebundle/rapids_logger/include",
    }

    libdirs {
        ext.bin_dir,
        "%{root}/_build/target-deps/pip_ucx_prebundle/librmm/lib64",
        "%{root}/_build/target-deps/pip_ucx_prebundle/libucx/lib",
        "%{root}/_build/target-deps/pip_ucx_prebundle/libucxx/lib64",
        "%{root}/_build/target-deps/pip_ucx_prebundle/rapids_logger/lib64",
    }

    links {
        "isaacsim.ucx.core",
        "ucxx",
        "ucp",
        "ucs",
        "uct",
        "ucm",
        "rmm",
        "rapids_logger",
    }

    -- Build the C++ plugin that will be loaded by the tests
    project_ext_tests(ext, "isaacsim.ucx.core.tests")
    cppdialect("C++17")
    add_files("source", "library/tests")

    add_cuda_dependencies()

    includedirs {
        "include",
        "plugins/",
        "%{target_deps}/doctest/include",
        "%{root}/source/extensions/isaacsim.ucx.core/include",
        "%{root}/source/extensions/isaacsim.core.includes/include",
        "%{root}/_build/target-deps/pip_ucx_prebundle/librmm/include",
        "%{root}/_build/target-deps/pip_ucx_prebundle/libucx/include",
        "%{root}/_build/target-deps/pip_ucx_prebundle/libucxx/include",
        "%{root}/_build/target-deps/pip_ucx_prebundle/rapids_logger/include",
        "%{kit_sdk_bin_dir}/dev/fabric/include/",
    }

    libdirs {
        extsbuild_dir .. "/omni.kit.test/bin",
        ext.bin_dir,
        "%{root}/_build/target-deps/pip_ucx_prebundle/librmm/lib64",
        "%{root}/_build/target-deps/pip_ucx_prebundle/libucx/lib",
        "%{root}/_build/target-deps/pip_ucx_prebundle/libucxx/lib64",
        "%{root}/_build/target-deps/pip_ucx_prebundle/rapids_logger/lib64",
    }

    links {
        "isaacsim.ucx.core",
        "ucxx",
        "ucp",
        "ucs",
        "uct",
        "ucm",
        "rmm",
        "rapids_logger",
    }

    add_usd()

    filter { "system:linux" }
    disablewarnings { "error=narrowing", "error=unused-but-set-variable", "error=unused-variable" }
    filter { "system:windows" }
    filter {}

    filter { "configurations:debug" }
    defines { "_DEBUG" }
    filter { "configurations:release" }
    defines { "NDEBUG" }
    filter {}
else
    print("SKIPPING BUILD - Only supported on linux-x86_64")
end
