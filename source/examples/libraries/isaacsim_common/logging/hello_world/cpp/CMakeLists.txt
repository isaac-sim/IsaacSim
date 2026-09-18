# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

if(NOT DEFINED ISAACSIM_CMAKE_MINIMUM_VERSION OR NOT DEFINED ISAACSIM_CMAKE_POLICY_MAXIMUM_VERSION)
    message(FATAL_ERROR "Configure this example through the examples runner")
endif()
cmake_minimum_required(VERSION ${ISAACSIM_CMAKE_MINIMUM_VERSION}...${ISAACSIM_CMAKE_POLICY_MAXIMUM_VERSION})
project(IsaacSimHelloWorldCpp LANGUAGES CXX)

if(NOT DEFINED ISAACSIM_CXX_STANDARD)
    message(FATAL_ERROR "ISAACSIM_CXX_STANDARD was not provided by the examples runner")
endif()

find_package(isaacsim_common CONFIG REQUIRED)

add_executable(hello_world_cpp Main.cpp)
target_compile_features(hello_world_cpp PRIVATE "cxx_std_${ISAACSIM_CXX_STANDARD}")
set_target_properties(hello_world_cpp PROPERTIES
    CXX_EXTENSIONS OFF
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)
foreach(configuration DEBUG RELEASE RELWITHDEBINFO MINSIZEREL)
    set_target_properties(hello_world_cpp PROPERTIES
        "RUNTIME_OUTPUT_DIRECTORY_${configuration}" "${CMAKE_BINARY_DIR}/bin"
    )
endforeach()
target_link_libraries(hello_world_cpp PRIVATE isaacsim::common-logging)
