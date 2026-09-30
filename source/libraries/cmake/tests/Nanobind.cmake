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

cmake_minimum_required(VERSION 3.26...4.3)

set(test_source "${CMAKE_CURRENT_BINARY_DIR}/nanobind-preloads-source")
set(module_core "${CMAKE_CURRENT_LIST_DIR}/../IsaacSimModuleCore.cmake")
set(nanobind_helper "${CMAKE_CURRENT_LIST_DIR}/../IsaacSimNanobind.cmake")

file(REMOVE_RECURSE "${test_source}")
file(MAKE_DIRECTORY "${test_source}")
file(CONFIGURE
    OUTPUT "${test_source}/CMakeLists.txt"
    CONTENT [=[
cmake_minimum_required(VERSION 3.26...4.3)
project(nanobind_preloads NONE)
include("@module_core@")
include("@nanobind_helper@")

add_custom_target(isaacsim-dependency-module-python)
if(NOT TEST_CASE STREQUAL missing_stage)
    add_custom_target(stage-isaacsim-dependency-module-python)
endif()

if(TEST_CASE STREQUAL valid OR TEST_CASE STREQUAL missing_stage)
    _isaacsim_resolve_nanobind_stub_preloads(
        "isaacsim.consumer.module"
        preload_modules
        preload_dependencies
        "isaacsim.dependency.module"
    )
    if(NOT preload_modules STREQUAL "isaacsim.dependency.module.bindings._bindings")
        message(FATAL_ERROR "Unexpected preload modules: ${preload_modules}")
    endif()
    if(NOT preload_dependencies STREQUAL "stage-isaacsim-dependency-module-python")
        message(FATAL_ERROR "Unexpected preload dependencies: ${preload_dependencies}")
    endif()
elseif(TEST_CASE STREQUAL unknown)
    _isaacsim_resolve_nanobind_stub_preloads(
        "isaacsim.consumer.module" preload_modules preload_dependencies "isaacsim.unknown.module")
elseif(TEST_CASE STREQUAL invalid)
    _isaacsim_resolve_nanobind_stub_preloads(
        "isaacsim.consumer.module" preload_modules preload_dependencies "IsaacSim.Invalid")
else()
    message(FATAL_ERROR "Unknown test case: ${TEST_CASE}")
endif()
]=]
    @ONLY
)

function(configure_case test_case should_succeed expected_message)
    set(test_build "${CMAKE_CURRENT_BINARY_DIR}/nanobind-preloads-build-${test_case}")
    file(REMOVE_RECURSE "${test_build}")
    execute_process(
        COMMAND "${CMAKE_COMMAND}"
            -S "${test_source}"
            -B "${test_build}"
            "-DTEST_CASE=${test_case}"
        RESULT_VARIABLE result
        OUTPUT_VARIABLE output
        ERROR_VARIABLE error
    )
    string(CONCAT combined_output "${output}" "${error}")
    if(should_succeed AND NOT result EQUAL 0)
        message(FATAL_ERROR "${test_case} unexpectedly failed:\n${combined_output}")
    elseif(NOT should_succeed AND result EQUAL 0)
        message(FATAL_ERROR "${test_case} unexpectedly succeeded")
    elseif(NOT should_succeed AND NOT combined_output MATCHES "${expected_message}")
        message(FATAL_ERROR "${test_case} failed for the wrong reason:\n${combined_output}")
    endif()
    file(REMOVE_RECURSE "${test_build}")
endfunction()

configure_case(valid TRUE "")
configure_case(unknown FALSE "has no registered nanobind")
configure_case(missing_stage FALSE "has no staged nanobind")
configure_case(invalid FALSE "Invalid lowercase dotted module name")

file(REMOVE_RECURSE "${test_source}")
