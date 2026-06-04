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
local ogn = get_ogn_project_information(ext, "isaacsim/asset/gen/omap")
project_ext(ext)

project_with_location("isaacsim.asset.gen.omap.generator")
targetdir(ext.bin_dir)
kind("SharedLib")
language("C++")
defines { "ISAACSIM_ASSET_GEN_OMAP_EXPORT" }

pic("On")
staticruntime("Off")
include_physx()
add_files("impl", "library")
includedirs {
    "%{root}/source/extensions/isaacsim.core.includes/include",
    "%{root}/_build/target-deps/usd/%{cfg.buildcfg}/include",
    "%{root}/_build/target-deps/omni_physics/%{config}/include",
    "%{root}/_build/target-deps/octomap/%{cfg.buildcfg}/include",
    "%{root}/source/extensions/isaacsim.asset.gen.omap/include",
}
libdirs {
    "%{root}/_build/target-deps/usd/%{cfg.buildcfg}/lib",
    "%{root}/_build/target-deps/usd_ext_physics/%{cfg.buildcfg}/lib",
}
links { "octomap", "octomath" }

extra_usd_libs = { "usdPhysics", "ts" }

-- Begin OpenUSD
add_usd(extra_usd_libs)
-- End OpenUSD

filter { "system:linux" }
disablewarnings { "error=pragmas" }
includedirs {
    "%{root}/_build/target-deps/python/include/python3.12",
}
buildoptions("-fvisibility=default")
libdirs {
    "%{root}/_build/target-deps/octomap/%{cfg.buildcfg}/lib64",
    "%{root}/_build/target-deps/octomap/%{cfg.buildcfg}/lib",
}
filter {}

filter { "system:windows" }
-- Warning C4099: 'omni::physx::IPhysx': type name first seen using 'class' now seen using 'struct'
disablewarnings { "4099", "4251" }
--  needed to static link against physx
-- linkoptions { "/ltcg" }
libdirs {
    "%{root}/_build/target-deps/tbb/lib/intel64/vc14",
    "%{root}/_build/target-deps/octomap/%{cfg.buildcfg}/lib",
}
filter {}

-- C++ Carbonite plugin
project_ext_plugin(ext, "isaacsim.asset.gen.omap.plugin")
dependson { "isaacsim.asset.gen.omap.generator" }

add_files("impl", "plugins")
include_physx()

includedirs {
    "%{root}/source/extensions/isaacsim.core.includes/include",
    "%{root}/_build/target-deps/usd/%{cfg.buildcfg}/include",
    "%{root}/_build/target-deps/omni_physics/%{config}/include",
    "%{root}/_build/target-deps/omni_client_library/include",
    "%{root}/source/extensions/isaacsim.asset.gen.omap/include",
    isaac_sim_extra_extsbuild_dir .. "/isaacsim.util.debug_draw/include",
    "%{kit_sdk_bin_dir}/dev/fabric/include/",
}
libdirs {
    "%{root}/_build/target-deps/usd/%{cfg.buildcfg}/lib",
    "%{root}/_build/target-deps/usd_ext_physics/%{cfg.buildcfg}/lib",
    extsbuild_dir .. "/omni.usd.core/bin",
    isaac_sim_extra_extsbuild_dir .. "/isaacsim.util.debug_draw/bin",
}
links { "isaacsim.util.debug_draw.primitive_drawing", "isaacsim.asset.gen.omap.generator", "omni.usd" }

extra_usd_libs = { "usdUtils", "ts" }

-- Begin OpenUSD
add_usd(extra_usd_libs)
-- End OpenUSD

includedirs {
    "%{root}/_build/target-deps/usd/%{cfg.buildcfg}/include/boost",
    "%{root}/_build/target-deps/python/include/python3.12",
}

filter { "system:linux" }
disablewarnings { "error=pragmas" }
libdirs {
    "%{root}/_build/target-deps/octomap/%{cfg.buildcfg}/lib64",
    "%{root}/_build/target-deps/octomap/%{cfg.buildcfg}/lib",
}
filter {}

filter { "system:windows" }
-- Warning C4099: 'omni::physx::IPhysx': type name first seen using 'class' now seen using 'struct'
disablewarnings { "4099", "4251" }
libdirs {
    "%{root}/_build/target-deps/tbb/lib/intel64/vc14",
    "%{root}/_build/target-deps/octomap/%{cfg.buildcfg}/lib",
}
filter {}

links { "octomap", "octomath" }

filter { "configurations:debug" }
defines { "_DEBUG" }
filter { "configurations:release" }
defines { "NDEBUG" }
filter {}

-- Python Bindings for Carobnite Plugin
project_ext_bindings {
    ext = ext,
    project_name = "isaacsim.asset.gen.omap.python",
    module = "_omap",
    src = ogn.bindings_path,
    target_subdir = ogn.bindings_target_path,
}

add_files("bindings", "bindings/*.*")

dependson { "isaacsim.asset.gen.omap.generator" }

include_physx()

includedirs {
    "%{root}/source/extensions/isaacsim.core.includes/include",
    "%{root}/_build/target-deps/omni_physics/%{config}/include",
    "%{root}/_build/target-deps/usd/%{cfg.buildcfg}/include",
    "%{root}/source/extensions/isaacsim.asset.gen.omap/include",
    "%{kit_sdk_bin_dir}/dev/fabric/include/",
}

libdirs {
    "%{root}/_build/target-deps/usd/%{cfg.buildcfg}/lib",
    "%{root}/_build/target-deps/usd_ext_physics/%{cfg.buildcfg}/lib",
}
links { "isaacsim.asset.gen.omap.generator" }

extra_usd_libs = { "usdUtils", "ts" }

-- Begin OpenUSD
add_usd(extra_usd_libs)
-- End OpenUSD

filter { "system:linux", "configurations:release" }
links { "tbb", "pthread" }
buildoptions { "-pthread" }
filter { "system:linux", "configurations:debug" }
links { "tbb_debug", "pthread" }
buildoptions { "-pthread" }
includedirs {
    "%{root}/_build/target-deps/usd/%{cfg.buildcfg}/include",
    "%{root}/_build/target-deps/python/include/python3.12",
}
-- libdirs {
--     "%{root}/_build/target-deps/octomap/%{cfg.buildcfg}/lib64",
-- }
filter { "system:windows" }
-- Warning C4099: 'omni::physx::IPhysx': type name first seen using 'class' now seen using 'struct'
disablewarnings { "4099" }
disablewarnings { "4251" }
-- link_boost_for_windows({"boost_python310"})

libdirs {
    "%{root}/_build/target-deps/tbb/lib/intel64/vc14",
}
filter {}

repo_build.prebuild_link {
    { "python/impl", ogn.python_target_path .. "/impl" },
    { "python/tests", ogn.python_target_path .. "/tests" },
    { "python/utils", ogn.python_target_path .. "/utils" },
    { "docs", ext.target_dir .. "/docs" },
    { "data", ext.target_dir .. "/data" },
    { "include", ext.target_dir .. "/include" },
}

repo_build.prebuild_copy {
    { "python/*.py", ogn.python_target_path },
}

if os.target() == "linux" then
    -- repo_build.prebuild_copy {
    --     { "%{root}/_build/target-deps/octomap/%{cfg.buildcfg}/lib64/*.so.*", ext.target_dir.."/bin" },
    -- }
else
    repo_build.prebuild_copy {
        { "%{root}/_build/target-deps/octomap/%{cfg.buildcfg}/bin/*.dll", ext.target_dir .. "/bin" },
    }
end
