# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

if(NOT DEFINED ISAACSIM_CMAKE_MINIMUM_VERSION OR NOT DEFINED ISAACSIM_CMAKE_POLICY_MAXIMUM_VERSION)
    message(FATAL_ERROR "Configure this example through the examples runner")
endif()
cmake_minimum_required(VERSION ${ISAACSIM_CMAKE_MINIMUM_VERSION}...${ISAACSIM_CMAKE_POLICY_MAXIMUM_VERSION})
project(IsaacSimOvglViewport LANGUAGES CXX)

if(NOT DEFINED ISAACSIM_CXX_STANDARD)
    message(FATAL_ERROR "ISAACSIM_CXX_STANDARD was not provided by the examples runner")
endif()

find_package(isaacsim_foundation CONFIG REQUIRED)
find_package(isaacsim_ovgl_viewport CONFIG REQUIRED)

set(isaacsim_openusd_preload_targets)
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    find_package(Python "${ISAACSIM_PYTHON_VERSION}" EXACT REQUIRED COMPONENTS Interpreter Development)
    execute_process(
        COMMAND "${Python_EXECUTABLE}" -c
            "import pathlib, pxr; print(pathlib.Path(pxr.__path__[0]).parent / 'usd_exchange.libs')"
        RESULT_VARIABLE isaacsim_usd_exchange_result
        OUTPUT_VARIABLE isaacsim_usd_exchange_library_directory
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
    )
endif()
if(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND isaacsim_usd_exchange_result EQUAL 0 AND
   IS_DIRECTORY "${isaacsim_usd_exchange_library_directory}"
)
    # Python-backed Foundation packages expect their Python entry point to preload OpenUSD. Retain the same runtime
    # libraries as direct dependencies when this example uses a native C++ entry point instead.
    list(APPEND isaacsim_openusd_preload_targets Python::Python)

    foreach(isaacsim_usd_component usd usdGeom usdPhysics usdUtils)
        file(GLOB isaacsim_usd_component_libraries
            LIST_DIRECTORIES FALSE
            "${isaacsim_usd_exchange_library_directory}/libusd_${isaacsim_usd_component}-*.so"
        )
        list(LENGTH isaacsim_usd_component_libraries isaacsim_usd_component_library_count)
        if(NOT isaacsim_usd_component_library_count EQUAL 1)
            message(FATAL_ERROR
                "Expected one usd-exchange library for ${isaacsim_usd_component}, found "
                "${isaacsim_usd_component_library_count} in ${isaacsim_usd_exchange_library_directory}"
            )
        endif()
        set(isaacsim_usd_component_target "isaacsim_usd_exchange_${isaacsim_usd_component}")
        add_library(${isaacsim_usd_component_target} SHARED IMPORTED)
        set_target_properties(${isaacsim_usd_component_target} PROPERTIES
            IMPORTED_LOCATION "${isaacsim_usd_component_libraries}"
        )
        list(APPEND isaacsim_openusd_preload_targets ${isaacsim_usd_component_target})
    endforeach()
endif()

add_executable(ovgl_viewport_cpp Main.cpp)
target_compile_features(ovgl_viewport_cpp PRIVATE "cxx_std_${ISAACSIM_CXX_STANDARD}")
target_link_libraries(ovgl_viewport_cpp PRIVATE
    isaacsim::foundation-objects
    isaacsim::ovgl_viewport-debug
)
if(isaacsim_openusd_preload_targets)
    target_link_libraries(ovgl_viewport_cpp PRIVATE
        "-Wl,--no-as-needed"
        ${isaacsim_openusd_preload_targets}
        "-Wl,--as-needed"
    )
endif()
set_target_properties(ovgl_viewport_cpp PROPERTIES
    CXX_EXTENSIONS OFF
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)
foreach(configuration DEBUG RELEASE RELWITHDEBINFO MINSIZEREL)
    set_target_properties(ovgl_viewport_cpp PROPERTIES
        "RUNTIME_OUTPUT_DIRECTORY_${configuration}" "${CMAKE_BINARY_DIR}/bin"
    )
endforeach()
