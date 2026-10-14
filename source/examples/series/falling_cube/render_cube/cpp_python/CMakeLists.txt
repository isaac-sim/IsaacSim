# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

if(NOT DEFINED ISAACSIM_CMAKE_MINIMUM_VERSION OR NOT DEFINED ISAACSIM_CMAKE_POLICY_MAXIMUM_VERSION)
    message(FATAL_ERROR "Configure this example through source/examples/examples.py")
endif()
cmake_minimum_required(VERSION ${ISAACSIM_CMAKE_MINIMUM_VERSION}...${ISAACSIM_CMAKE_POLICY_MAXIMUM_VERSION})
project(IsaacSimRenderCubeCppPython LANGUAGES CXX)

if(NOT DEFINED ISAACSIM_CXX_STANDARD OR NOT DEFINED ISAACSIM_PYTHON_VERSION)
    message(FATAL_ERROR "Toolchain requirements were not provided by source/examples/examples.py")
endif()

find_package(Python "${ISAACSIM_PYTHON_VERSION}" EXACT REQUIRED COMPONENTS Interpreter Development.Module)
find_package(isaacsim_common CONFIG REQUIRED)
find_package(isaacsim_foundation CONFIG REQUIRED)
find_package(isaacsim_physics CONFIG REQUIRED)
find_package(isaacsim_physics_engines CONFIG REQUIRED)
# The Isaac Sim package links OVPhysX privately, so consumers that need its
# runtime location must resolve the dependency package directly.
if(NOT TARGET ovphysx::ovphysx)
    find_package(ovphysx CONFIG REQUIRED)
endif()
# ovrtx ships its CMake package inside its Python distribution, so this resolves against the
# environment that installed it rather than against an Isaac Sim prefix. examples.py puts the
# runner's Python runtime dependencies on CMAKE_PREFIX_PATH for that reason.
find_package(ovrtx CONFIG REQUIRED)

Python_add_library(_render_cube_cpp MODULE WITH_SOABI Bindings.cpp)
target_compile_features(_render_cube_cpp PRIVATE "cxx_std_${ISAACSIM_CXX_STANDARD}")
set_target_properties(_render_cube_cpp PROPERTIES
    CXX_EXTENSIONS OFF
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/python"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/python"
)
foreach(configuration DEBUG RELEASE RELWITHDEBINFO MINSIZEREL)
    set_target_properties(_render_cube_cpp PROPERTIES
        "LIBRARY_OUTPUT_DIRECTORY_${configuration}" "${CMAKE_BINARY_DIR}/python"
        "RUNTIME_OUTPUT_DIRECTORY_${configuration}" "${CMAKE_BINARY_DIR}/python"
    )
endforeach()
target_link_libraries(_render_cube_cpp PRIVATE
    isaacsim::common-array
    isaacsim::foundation-objects
    isaacsim::foundation-prims
    isaacsim::physics-entities
    isaacsim::physics-manager
    ovrtx::ovrtx
)

if(WIN32)
    # Windows resolves an extension's dependencies while importing it, and reuses a library already
    # loaded under the same name. Copying the SDK beside the extension would therefore shadow the
    # libraries the Python distributions load and splice two runtimes into one process, so the SDK
    # is read where it was installed instead. That also keeps the libraries reaching their payloads
    # through a hop relative to themselves, such as the schema trees and the ../plugins ovphysx
    # hardcodes, beside the payloads they expect.
    #
    # main.py registers these directories before the import that resolves them, because nothing at
    # runtime reveals which SDK the example was linked against. ovrtx is absent here: it ships as a
    # Python distribution that main.py locates through the import system.
    #
    # They are recorded relative to this generated module, which main.py resolves them against. A
    # build tree that moves after being built carries the SDK installed beside it, so an absolute
    # path would name the directory the example was built in rather than the one it runs from.
    string(JOIN "\n" _render_cube_runtime_content
        "RUNTIME_DIRECTORIES = ["
        "    r\"$<PATH:RELATIVE_PATH,$<TARGET_FILE_DIR:isaacsim::physics_engines-ovphysx>,${CMAKE_BINARY_DIR}/python>\","
        "    r\"$<PATH:RELATIVE_PATH,$<TARGET_FILE_DIR:ovphysx::ovphysx>,${CMAKE_BINARY_DIR}/python>\","
        "]"
        ""
    )
    file(GENERATE
        OUTPUT "${CMAKE_BINARY_DIR}/python/_render_cube_runtime.py"
        CONTENT "${_render_cube_runtime_content}"
    )
endif()
