# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

if(NOT DEFINED ISAACSIM_CMAKE_MINIMUM_VERSION OR NOT DEFINED ISAACSIM_CMAKE_POLICY_MAXIMUM_VERSION)
    message(FATAL_ERROR "Configure this example through the examples runner")
endif()
cmake_minimum_required(VERSION ${ISAACSIM_CMAKE_MINIMUM_VERSION}...${ISAACSIM_CMAKE_POLICY_MAXIMUM_VERSION})
project(IsaacSimHelloWorldCppPython LANGUAGES CXX)

if(NOT DEFINED ISAACSIM_CXX_STANDARD OR NOT DEFINED ISAACSIM_PYTHON_VERSION)
    message(FATAL_ERROR "Toolchain requirements were not provided by the examples runner")
endif()

find_package(Python "${ISAACSIM_PYTHON_VERSION}" EXACT REQUIRED COMPONENTS Interpreter Development.Module)
find_package(isaacsim_common CONFIG REQUIRED)

Python_add_library(_hello_world_cpp MODULE WITH_SOABI Bindings.cpp)
target_compile_features(_hello_world_cpp PRIVATE "cxx_std_${ISAACSIM_CXX_STANDARD}")
set_target_properties(_hello_world_cpp PROPERTIES
    CXX_EXTENSIONS OFF
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/python"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/python"
)
foreach(configuration DEBUG RELEASE RELWITHDEBINFO MINSIZEREL)
    set_target_properties(_hello_world_cpp PROPERTIES
        "LIBRARY_OUTPUT_DIRECTORY_${configuration}" "${CMAKE_BINARY_DIR}/python"
        "RUNTIME_OUTPUT_DIRECTORY_${configuration}" "${CMAKE_BINARY_DIR}/python"
    )
endforeach()
target_link_libraries(_hello_world_cpp PRIVATE isaacsim::common-logging)

if(WIN32)
    add_custom_command(TARGET _hello_world_cpp POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_RUNTIME_DLLS:_hello_world_cpp>
            $<TARGET_FILE_DIR:_hello_world_cpp>
        COMMAND_EXPAND_LISTS
        VERBATIM
    )
endif()
