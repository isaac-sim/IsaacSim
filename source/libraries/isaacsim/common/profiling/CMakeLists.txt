# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

find_package(Carbonite REQUIRED MODULE)

isaacsim_add_module(
    NAME isaacsim.common.profiling
    EXPORT_HEADER isaacsim/common/profiling/Export.h
    EXPORT_MACRO ISAACSIM_COMMON_PROFILING_API
    PUBLIC_C_HEADERS
        isaacsim/common/profiling/Profiling.h
        isaacsim/common/profiling/ProfilingHost.h
    PUBLIC_CPP_HEADERS
        isaacsim/common/profiling/Profiling.hpp
        isaacsim/common/profiling/details/ProfileMacros.hpp
    PUBLIC_C_SYMBOLS_FILE abi/c-api.symbols
    SOURCES
        src/Profiling.cpp
)
target_include_directories(isaacsim-common-profiling PRIVATE
    "${ISAACSIM_CARBONITE_ROOT}/include"
)

add_library(isaacsim-common-profiling-carbonite-host SHARED
    src/CarboniteProfilerHost.cpp
)
target_compile_features(isaacsim-common-profiling-carbonite-host PRIVATE cxx_std_17)
target_link_libraries(isaacsim-common-profiling-carbonite-host PRIVATE
    isaacsim::common-profiling
    carb::static
)
target_compile_definitions(isaacsim-common-profiling-carbonite-host PRIVATE
    CARB_STATIC_PLUGIN_BUILD=1
    CARB_PLUGIN_CLIENT_NAME=isaacsim_common_profiling_carbonite_host
    carb_ErrorApi=carb_ErrorApi_latest
    carb_Framework=carb_Framework_latest
    carb_filesystem_IFileSystem=carb_filesystem_IFileSystem_latest
    carb_profiler_IProfiler=carb_profiler_IProfiler_latest
)
_isaacsim_apply_build_options(isaacsim-common-profiling-carbonite-host)
_isaacsim_get_module_names(
    "isaacsim.common.profiling"
    profiling_target
    profiling_alias
    profiling_path
    profiling_output_dir
)
set_target_properties(isaacsim-common-profiling-carbonite-host PROPERTIES
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN YES
    OUTPUT_NAME "isaacsim-common-profiling-carbonite-host"
    LIBRARY_OUTPUT_DIRECTORY "${profiling_output_dir}/lib"
    RUNTIME_OUTPUT_DIRECTORY "${profiling_output_dir}/lib"
    ARCHIVE_OUTPUT_DIRECTORY "${profiling_output_dir}/lib"
    BUILD_RPATH "${ISAACSIM_LOADER_ORIGIN}"
    INSTALL_RPATH "${ISAACSIM_LOADER_ORIGIN}"
)
if(UNIX AND NOT APPLE)
    target_link_options(isaacsim-common-profiling-carbonite-host PRIVATE
        "LINKER:--exclude-libs,ALL"
        "LINKER:--version-script=${CMAKE_CURRENT_SOURCE_DIR}/abi/carbonite-host.symbols"
    )
    set_property(TARGET isaacsim-common-profiling-carbonite-host APPEND PROPERTY
        LINK_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/abi/carbonite-host.symbols"
    )
endif()

if(WIN32)
    set(carbonite_nvtx_plugin "carb.profiler-nvtx.plugin.dll")
elseif(APPLE)
    set(carbonite_nvtx_plugin "libcarb.profiler-nvtx.plugin.dylib")
else()
    set(carbonite_nvtx_plugin "libcarb.profiler-nvtx.plugin.so")
endif()
add_library(isaacsim-common-profiling-nvtx-plugin SHARED IMPORTED)
set_target_properties(isaacsim-common-profiling-nvtx-plugin PROPERTIES
    IMPORTED_CONFIGURATIONS "Debug;Release"
    IMPORTED_LOCATION_DEBUG "${ISAACSIM_CARBONITE_PLATFORM_DIR}/debug/${carbonite_nvtx_plugin}"
    IMPORTED_LOCATION_RELEASE "${ISAACSIM_CARBONITE_PLATFORM_DIR}/release/${carbonite_nvtx_plugin}"
    MAP_IMPORTED_CONFIG_MINSIZEREL Release
    MAP_IMPORTED_CONFIG_RELWITHDEBINFO Release
)
if(NOT WIN32)
    set_target_properties(isaacsim-common-profiling-nvtx-plugin PROPERTIES
        IMPORTED_SONAME_DEBUG "${carbonite_nvtx_plugin}"
        IMPORTED_SONAME_RELEASE "${carbonite_nvtx_plugin}"
    )
endif()

isaacsim_install_runtime_dependencies(
    MODULE isaacsim.common.profiling
    TARGETS
        isaacsim-common-profiling-carbonite-host
        isaacsim-common-profiling-nvtx-plugin
)

get_target_property(profiling_group isaacsim-common-profiling ISAACSIM_GROUP_NAME)
_isaacsim_get_group_component("${profiling_group}" runtime profiling_runtime_component)
install(FILES "${ISAACSIM_CARBONITE_ROOT}/PACKAGE-LICENSES/carbonite-LICENSES.txt"
    DESTINATION "${CMAKE_INSTALL_DATADIR}/licenses/${profiling_group}"
    COMPONENT "${profiling_runtime_component}"
)
if(ISAACSIM_ENABLE_PYTHON)
    _isaacsim_get_group_component("${profiling_group}" python profiling_python_component)
    install(FILES "${ISAACSIM_CARBONITE_ROOT}/PACKAGE-LICENSES/carbonite-LICENSES.txt"
        DESTINATION "${ISAACSIM_PYTHON_INSTALL_DIR}/isaacsim/common/profiling/_licenses"
        COMPONENT "${profiling_python_component}"
    )
endif()

isaacsim_add_cpp_tests(
    MODULE isaacsim.common.profiling
    SOURCES
        tests/cpp/Profiling.cpp
    LABELS
        abi
        threading
)
if(BUILD_TESTING)
    target_include_directories(tests-isaacsim-common-profiling PRIVATE
        "${ISAACSIM_CARBONITE_ROOT}/include"
    )
endif()

isaacsim_add_c_tests(
    MODULE isaacsim.common.profiling
    SOURCES
        tests/c/Profiling.cpp
    LABELS
        abi
)

isaacsim_add_nanobind(
    MODULE isaacsim.common.profiling
    SOURCE bindings/python/Bindings.cpp
)

isaacsim_add_python_tests(
    MODULE isaacsim.common.profiling
    REQUIRES_BINDINGS
    LABELS
        threading
)
