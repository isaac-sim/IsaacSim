# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

if(NOT TARGET ovstage::ovstage)
    set(
        ovstage_DIR
        "${ISAACSIM_NATIVE_RUNTIME_DEPS_DIR}/ovstage/lib/cmake/ovstage"
        CACHE PATH
        "Directory containing the selected ovstage package configuration"
        FORCE
    )
    find_package(ovstage REQUIRED
        PATHS "${ovstage_DIR}"
        NO_DEFAULT_PATH
    )
    set_property(TARGET ovstage::ovstage PROPERTY IMPORTED_GLOBAL TRUE)
endif()
get_target_property(isaacsim_ovgl_viewport_ovstage_library ovstage::ovstage IMPORTED_LOCATION)
if(NOT isaacsim_ovgl_viewport_ovstage_library OR isaacsim_ovgl_viewport_ovstage_library MATCHES "-NOTFOUND$")
    message(FATAL_ERROR "ovstage::ovstage does not declare its runtime library location")
endif()
get_filename_component(isaacsim_ovgl_viewport_ovstage_runtime_dir
    "${isaacsim_ovgl_viewport_ovstage_library}" DIRECTORY
)

set(isaacsim_ovgl_viewport_omni_client_root "${ISAACSIM_TARGET_DEPS_DIR}/omni_client_library")
set(isaacsim_ovgl_viewport_omni_client_header
    "${isaacsim_ovgl_viewport_omni_client_root}/include/OmniClientVersion.h"
)
if(NOT EXISTS "${isaacsim_ovgl_viewport_omni_client_header}")
    message(FATAL_ERROR
        "isaacsim.ovgl_viewport.debug requires OmniClient headers at "
        "${ISAACSIM_TARGET_DEPS_DIR}/omni_client_library. Run the library build without --no-pull."
    )
endif()
file(READ "${isaacsim_ovgl_viewport_omni_client_header}" isaacsim_ovgl_viewport_omni_client_version)
if(NOT isaacsim_ovgl_viewport_omni_client_version MATCHES
    "OMNICLIENT_BUILD_STRING \"2\\.72\\.3-release\\.7151\\+gl\\.5390bed9\""
)
    message(FATAL_ERROR "The pinned OmniClient package no longer matches the locked OVStage client ABI")
endif()

if(WIN32)
    set(isaacsim_ovgl_viewport_omni_client_files
        debug/omniclient.dll
        debug/omniclient.lib
        debug/omniverse_connection.dll
        release/omniclient.dll
        release/omniclient.lib
        release/omniverse_connection.dll
    )
elseif(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    set(isaacsim_ovgl_viewport_omni_client_files
        debug/libomniclient.so
        debug/libomniverse_connection.so
        release/libomniclient.so
        release/libomniverse_connection.so
    )
else()
    message(FATAL_ERROR "The OmniClient dependency is not configured for ${CMAKE_SYSTEM_NAME}")
endif()
foreach(isaacsim_ovgl_viewport_omni_client_file IN LISTS isaacsim_ovgl_viewport_omni_client_files)
    if(NOT EXISTS "${isaacsim_ovgl_viewport_omni_client_root}/${isaacsim_ovgl_viewport_omni_client_file}")
        message(FATAL_ERROR
            "The pinned OmniClient package is missing ${isaacsim_ovgl_viewport_omni_client_file}. "
            "Run the library build without --no-pull."
        )
    endif()
endforeach()

add_library(omni_client::connection SHARED IMPORTED)
add_library(omni_client::client SHARED IMPORTED)
if(WIN32)
    set_target_properties(omni_client::connection PROPERTIES
        IMPORTED_CONFIGURATIONS "DEBUG;RELEASE"
        IMPORTED_LOCATION "${isaacsim_ovgl_viewport_omni_client_root}/release/omniverse_connection.dll"
        IMPORTED_LOCATION_DEBUG "${isaacsim_ovgl_viewport_omni_client_root}/debug/omniverse_connection.dll"
        IMPORTED_LOCATION_RELEASE "${isaacsim_ovgl_viewport_omni_client_root}/release/omniverse_connection.dll"
        MAP_IMPORTED_CONFIG_MINSIZEREL Release
        MAP_IMPORTED_CONFIG_RELWITHDEBINFO Release
    )
    set_target_properties(omni_client::client PROPERTIES
        IMPORTED_CONFIGURATIONS "DEBUG;RELEASE"
        IMPORTED_IMPLIB "${isaacsim_ovgl_viewport_omni_client_root}/release/omniclient.lib"
        IMPORTED_IMPLIB_DEBUG "${isaacsim_ovgl_viewport_omni_client_root}/debug/omniclient.lib"
        IMPORTED_IMPLIB_RELEASE "${isaacsim_ovgl_viewport_omni_client_root}/release/omniclient.lib"
        IMPORTED_LOCATION "${isaacsim_ovgl_viewport_omni_client_root}/release/omniclient.dll"
        IMPORTED_LOCATION_DEBUG "${isaacsim_ovgl_viewport_omni_client_root}/debug/omniclient.dll"
        IMPORTED_LOCATION_RELEASE "${isaacsim_ovgl_viewport_omni_client_root}/release/omniclient.dll"
        INTERFACE_INCLUDE_DIRECTORIES "${isaacsim_ovgl_viewport_omni_client_root}/include"
        MAP_IMPORTED_CONFIG_MINSIZEREL Release
        MAP_IMPORTED_CONFIG_RELWITHDEBINFO Release
    )
else()
    set_target_properties(omni_client::connection PROPERTIES
        IMPORTED_CONFIGURATIONS "DEBUG;RELEASE"
        IMPORTED_LOCATION "${isaacsim_ovgl_viewport_omni_client_root}/release/libomniverse_connection.so"
        IMPORTED_LOCATION_DEBUG "${isaacsim_ovgl_viewport_omni_client_root}/debug/libomniverse_connection.so"
        IMPORTED_LOCATION_RELEASE "${isaacsim_ovgl_viewport_omni_client_root}/release/libomniverse_connection.so"
        IMPORTED_SONAME libomniverse_connection.so
        MAP_IMPORTED_CONFIG_MINSIZEREL Release
        MAP_IMPORTED_CONFIG_RELWITHDEBINFO Release
    )
    set_target_properties(omni_client::client PROPERTIES
        IMPORTED_CONFIGURATIONS "DEBUG;RELEASE"
        IMPORTED_LOCATION "${isaacsim_ovgl_viewport_omni_client_root}/release/libomniclient.so"
        IMPORTED_LOCATION_DEBUG "${isaacsim_ovgl_viewport_omni_client_root}/debug/libomniclient.so"
        IMPORTED_LOCATION_RELEASE "${isaacsim_ovgl_viewport_omni_client_root}/release/libomniclient.so"
        IMPORTED_SONAME libomniclient.so
        INTERFACE_INCLUDE_DIRECTORIES "${isaacsim_ovgl_viewport_omni_client_root}/include"
        INTERFACE_LINK_LIBRARIES omni_client::connection
        MAP_IMPORTED_CONFIG_MINSIZEREL Release
        MAP_IMPORTED_CONFIG_RELWITHDEBINFO Release
    )
endif()

if(WIN32)
    set(isaacsim_ovgl_viewport_sdl_cmake_dir "${ISAACSIM_PUBLIC_DEPS_ROOT}/Library/cmake")
    set(isaacsim_ovgl_viewport_sdl_license "${ISAACSIM_PUBLIC_DEPS_ROOT}/Library/licenses/SDL3/LICENSE.txt")
else()
    set(isaacsim_ovgl_viewport_sdl_cmake_dir "${ISAACSIM_PUBLIC_DEPS_ROOT}/lib/cmake/SDL3")
    set(isaacsim_ovgl_viewport_sdl_license "${ISAACSIM_PUBLIC_DEPS_ROOT}/share/licenses/SDL3/LICENSE.txt")
endif()
if(NOT TARGET SDL3::SDL3-shared)
    find_package(SDL3 3.4.14 EXACT CONFIG REQUIRED
        PATHS "${isaacsim_ovgl_viewport_sdl_cmake_dir}"
        NO_DEFAULT_PATH
    )
endif()

find_package(Threads REQUIRED)
if(NOT WIN32)
    set(isaacsim_ovgl_viewport_egl_library
        "${ISAACSIM_PUBLIC_DEPS_ROOT}/lib/libEGL${CMAKE_SHARED_LIBRARY_SUFFIX}"
    )
    set(isaacsim_ovgl_viewport_gles_library
        "${ISAACSIM_PUBLIC_DEPS_ROOT}/lib/libGLESv2${CMAKE_SHARED_LIBRARY_SUFFIX}"
    )
    foreach(isaacsim_ovgl_viewport_graphics_library IN ITEMS
        "${isaacsim_ovgl_viewport_egl_library}"
        "${isaacsim_ovgl_viewport_gles_library}"
    )
        if(NOT EXISTS "${isaacsim_ovgl_viewport_graphics_library}")
            message(FATAL_ERROR
                "isaacsim.ovgl_viewport.debug requires the Pixi-locked graphics library at "
                "${isaacsim_ovgl_viewport_graphics_library}. Run the library build without --no-pull."
            )
        endif()
    endforeach()
    if(NOT EXISTS "${ISAACSIM_PUBLIC_DEPS_INCLUDE_DIR}/EGL/egl.h" OR
       NOT EXISTS "${ISAACSIM_PUBLIC_DEPS_INCLUDE_DIR}/GLES3/gl32.h"
    )
        message(FATAL_ERROR
            "isaacsim.ovgl_viewport.debug requires the Pixi-locked EGL and GLES headers at "
            "${ISAACSIM_PUBLIC_DEPS_INCLUDE_DIR}. Run the library build without --no-pull."
        )
    endif()

    add_library(isaacsim_ovgl::egl SHARED IMPORTED)
    set_target_properties(isaacsim_ovgl::egl PROPERTIES
        IMPORTED_LOCATION "${isaacsim_ovgl_viewport_egl_library}"
        INTERFACE_INCLUDE_DIRECTORIES "${ISAACSIM_PUBLIC_DEPS_INCLUDE_DIR}"
    )
    add_library(isaacsim_ovgl::gles SHARED IMPORTED)
    set_target_properties(isaacsim_ovgl::gles PROPERTIES
        IMPORTED_LOCATION "${isaacsim_ovgl_viewport_gles_library}"
        INTERFACE_INCLUDE_DIRECTORIES "${ISAACSIM_PUBLIC_DEPS_INCLUDE_DIR}"
    )
endif()

set(isaacsim_ovgl_viewport_stb_include_dir "${ISAACSIM_PUBLIC_DEPS_INCLUDE_DIR}")
if(NOT EXISTS "${isaacsim_ovgl_viewport_stb_include_dir}/stb_image.h")
    message(FATAL_ERROR
        "isaacsim.ovgl_viewport.debug requires the Pixi-locked stb headers at "
        "${isaacsim_ovgl_viewport_stb_include_dir}."
    )
endif()
add_library(stb::image INTERFACE IMPORTED)
set_target_properties(stb::image PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${isaacsim_ovgl_viewport_stb_include_dir}"
)

if(NOT WIN32)
    set(isaacsim_ovgl_viewport_gl_dependencies
        isaacsim_ovgl::egl
        isaacsim_ovgl::gles
        ${CMAKE_DL_LIBS}
    )
endif()

set(ovgl_source_dir "${CMAKE_CURRENT_SOURCE_DIR}/src/details/ovgl")
set(ovgl_sources
    "${ovgl_source_dir}/AssetReader.cpp"
    "${ovgl_source_dir}/AssetReader.hpp"
    "${ovgl_source_dir}/BatchPaths.hpp"
    "${ovgl_source_dir}/MeshClassification.hpp"
    "${ovgl_source_dir}/MeshSoup.cpp"
    "${ovgl_source_dir}/MeshSoup.hpp"
    "${ovgl_source_dir}/Ovgl.cpp"
    "${ovgl_source_dir}/Ovgl.h"
    "${ovgl_source_dir}/PointInstancer.cpp"
    "${ovgl_source_dir}/PointInstancer.h"
    "${ovgl_source_dir}/Scene.cpp"
    "${ovgl_source_dir}/Scene.h"
    "${ovgl_source_dir}/Skinning.cpp"
    "${ovgl_source_dir}/Skinning.hpp"
    "${ovgl_source_dir}/UsdzArchive.cpp"
    "${ovgl_source_dir}/UsdzArchive.hpp"
    "${ovgl_source_dir}/VertexBuffer.cpp"
    "${ovgl_source_dir}/VertexBuffer.hpp"
    "${ovgl_source_dir}/gl/EglHeadless.h"
    "${ovgl_source_dir}/gl/Font8x16.h"
    "${ovgl_source_dir}/gl/GlApi.c"
    "${ovgl_source_dir}/gl/GlApi.h"
    "${ovgl_source_dir}/gl/GlLog.c"
    "${ovgl_source_dir}/gl/GlLog.h"
    "${ovgl_source_dir}/gl/Gpu.h"
    "${ovgl_source_dir}/gl/GpuOpenGles.c"
    "${ovgl_source_dir}/gl/Scene.h"
    "${ovgl_source_dir}/gl/ShadersGles.h"
    "${ovgl_source_dir}/gl/StbImageImplementation.c"
)
if(WIN32)
    list(APPEND ovgl_sources "${ovgl_source_dir}/gl/SdlHeadless.c")
else()
    list(APPEND ovgl_sources "${ovgl_source_dir}/gl/EglHeadless.c")
endif()

# GLSL programs are intentionally embedded as single C string constants. GCC
# supports their sizes but warns against the ISO C translation-limit minimum.
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    set_source_files_properties("${ovgl_source_dir}/gl/GpuOpenGles.c"
        PROPERTIES COMPILE_OPTIONS "-Wno-overlength-strings"
    )
endif()

isaacsim_add_module(
    NAME isaacsim.ovgl_viewport.debug
    EXPORT_HEADER isaacsim/ovgl_viewport/debug/Export.h
    EXPORT_MACRO ISAACSIM_OVGL_VIEWPORT_API
    PUBLIC_CPP_HEADERS
        isaacsim/ovgl_viewport/debug/Viewport.hpp
    SOURCES
        src/Renderer.cpp
        src/Renderer.hpp
        src/Viewport.cpp
        src/details/OvglBackend.cpp
        src/details/OvglBackend.hpp
        src/details/OvstageHelpers.hpp
        ${ovgl_sources}
    PRIVATE_DEPENDENCIES
        SDL3::SDL3-shared
        isaacsim::common-ovstage
        omni_client::client
        ovstage::ovstage
        stb::image
        Threads::Threads
        ${isaacsim_ovgl_viewport_gl_dependencies}
)
if(WIN32)
    target_compile_definitions(isaacsim-ovgl_viewport-debug PRIVATE NUSD_DESKTOP_GL=1 _USE_MATH_DEFINES)
endif()

get_target_property(isaacsim_ovgl_viewport_group isaacsim-ovgl_viewport-debug ISAACSIM_GROUP_NAME)
_isaacsim_get_group_component("${isaacsim_ovgl_viewport_group}" runtime isaacsim_ovgl_viewport_runtime_component)
install(FILES "${ISAACSIM_PUBLIC_DEPS_ROOT}/licenses/stb/LICENSE"
    DESTINATION "${CMAKE_INSTALL_DATADIR}/licenses/${isaacsim_ovgl_viewport_group}"
    RENAME "stb-LICENSE"
    COMPONENT "${isaacsim_ovgl_viewport_runtime_component}"
)
install(FILES "${isaacsim_ovgl_viewport_sdl_license}"
    DESTINATION "${CMAKE_INSTALL_DATADIR}/licenses/${isaacsim_ovgl_viewport_group}"
    RENAME "SDL3-LICENSE.txt"
    COMPONENT "${isaacsim_ovgl_viewport_runtime_component}"
)
install(FILES
    "${ISAACSIM_TARGET_DEPS_DIR}/omni_client_library/PACKAGE-LICENSES/omniverse client library-LICENSES.txt"
    DESTINATION "${CMAKE_INSTALL_DATADIR}/licenses/${isaacsim_ovgl_viewport_group}"
    RENAME "OmniClient-LICENSES.txt"
    COMPONENT "${isaacsim_ovgl_viewport_runtime_component}"
)

target_include_directories(isaacsim-ovgl_viewport-debug PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
    "${ovgl_source_dir}"
    "${CMAKE_CURRENT_SOURCE_DIR}/../.."
)
isaacsim_install_runtime_dependencies(
    MODULE isaacsim.ovgl_viewport.debug
    TARGETS
        SDL3::SDL3-shared
        omni_client::client
        omni_client::connection
        ovstage::ovstage
)
isaacsim_add_runtime_directory(
    MODULE isaacsim.ovgl_viewport.debug
    SOURCE "${isaacsim_ovgl_viewport_ovstage_runtime_dir}/plugins"
    DESTINATION plugins
    WINDOWS_INSTALL_DESTINATION "${CMAKE_INSTALL_BINDIR}/plugins"
)
isaacsim_add_runtime_directory(
    MODULE isaacsim.ovgl_viewport.debug
    SOURCE "${isaacsim_ovgl_viewport_ovstage_runtime_dir}/ovstage_usd_schemas"
    DESTINATION ovstage_usd_schemas
    WINDOWS_INSTALL_DESTINATION "${CMAKE_INSTALL_BINDIR}/ovstage_usd_schemas"
)

isaacsim_add_nanobind(
    MODULE isaacsim.ovgl_viewport.debug
    SOURCE bindings/python/Bindings.cpp
)

if(BUILD_TESTING)
    function(add_isaacsim_ovgl_test target source)
        add_executable(${target} "${CMAKE_CURRENT_SOURCE_DIR}/tests/cpp/${source}")
        target_include_directories(${target} PRIVATE
            "${ovgl_source_dir}"
        )
        add_test(NAME ${target} COMMAND ${target})
        set_tests_properties(${target} PROPERTIES LABELS "unit;ovgl")
    endfunction()

    add_isaacsim_ovgl_test(isaacsim-ovgl-test-mesh-classification TestMeshClassification.cpp)
    add_isaacsim_ovgl_test(isaacsim-ovgl-test-batch-path-identity TestBatchPathIdentity.cpp)

    add_isaacsim_ovgl_test(isaacsim-ovgl-test-asset-reader TestAssetReader.cpp)
    target_sources(isaacsim-ovgl-test-asset-reader PRIVATE "${ovgl_source_dir}/AssetReader.cpp")
    target_link_libraries(isaacsim-ovgl-test-asset-reader PRIVATE omni_client::client)
    if(WIN32)
        set_property(TEST isaacsim-ovgl-test-asset-reader APPEND PROPERTY ENVIRONMENT_MODIFICATION
            "PATH=path_list_prepend:$<TARGET_FILE_DIR:omni_client::client>"
        )
    endif()

    add_isaacsim_ovgl_test(isaacsim-ovgl-test-vertex-buffer TestVertexBuffer.cpp)
    target_sources(isaacsim-ovgl-test-vertex-buffer PRIVATE "${ovgl_source_dir}/VertexBuffer.cpp")

    add_isaacsim_ovgl_test(isaacsim-ovgl-test-skinning TestSkinning.cpp)
    target_sources(isaacsim-ovgl-test-skinning PRIVATE "${ovgl_source_dir}/Skinning.cpp")

    add_isaacsim_ovgl_test(isaacsim-ovgl-test-mesh-soup TestMeshSoup.cpp)
    target_sources(isaacsim-ovgl-test-mesh-soup PRIVATE "${ovgl_source_dir}/MeshSoup.cpp")

    add_isaacsim_ovgl_test(isaacsim-ovgl-test-usdz-cache TestUsdzArchiveCache.cpp)
    target_sources(isaacsim-ovgl-test-usdz-cache PRIVATE "${ovgl_source_dir}/UsdzArchive.cpp")
    target_link_libraries(isaacsim-ovgl-test-usdz-cache PRIVATE ${CMAKE_DL_LIBS})

    isaacsim_add_cpp_tests(
        MODULE isaacsim.ovgl_viewport.debug
        SOURCES tests/cpp/TestTransforms.cpp
        DEPENDENCIES
            isaacsim::common-ovstage
            ovstage::ovstage
        LABELS ovgl
    )
    target_include_directories(tests-isaacsim-ovgl_viewport-debug PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/../..")
    if(WIN32)
        add_custom_command(TARGET tests-isaacsim-ovgl_viewport-debug POST_BUILD
            COMMAND "${CMAKE_COMMAND}" -E copy_directory_if_different
                "${isaacsim_ovgl_viewport_ovstage_runtime_dir}/plugins"
                "$<TARGET_FILE_DIR:tests-isaacsim-ovgl_viewport-debug>/plugins"
            VERBATIM
        )
        set_property(TEST tests-isaacsim-ovgl_viewport-debug APPEND PROPERTY ENVIRONMENT_MODIFICATION
            "PATH=path_list_prepend:$<TARGET_FILE_DIR:tests-isaacsim-ovgl_viewport-debug>/plugins"
        )
    endif()
endif()

isaacsim_add_python_tests(
    MODULE isaacsim.ovgl_viewport.debug
    LABELS unit
    REQUIRES_BINDINGS
)
