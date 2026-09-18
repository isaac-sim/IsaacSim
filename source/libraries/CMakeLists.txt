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

include(cmake/IsaacSimSystemRequirements.cmake)
set(ISAACSIM_SYSTEM_REQUIREMENTS_FILE "${CMAKE_CURRENT_SOURCE_DIR}/system_requirements.toml")
isaacsim_load_system_requirements("${ISAACSIM_SYSTEM_REQUIREMENTS_FILE}")
if(NOT ISAACSIM_CMAKE_MINIMUM_VERSION STREQUAL "3.26"
   OR NOT ISAACSIM_CMAKE_POLICY_MAXIMUM_VERSION STREQUAL "4.3"
)
    message(FATAL_ERROR
        "The bootstrap cmake_minimum_required declaration must match "
        "${ISAACSIM_SYSTEM_REQUIREMENTS_FILE}"
    )
endif()

set(ISAACSIM_LIBRARIES_VERSION_FILE "${CMAKE_CURRENT_SOURCE_DIR}/VERSION")
file(STRINGS "${ISAACSIM_LIBRARIES_VERSION_FILE}" library_version_lines)
list(LENGTH library_version_lines library_version_line_count)
if(NOT library_version_line_count EQUAL 1)
    message(FATAL_ERROR "source/libraries/VERSION must contain exactly one version")
endif()
list(GET library_version_lines 0 ISAACSIM_LIBRARIES_VERSION)
set(pep440_integer "(0|[1-9][0-9]*)")
set(pep440_local_identifier "([0-9]*[a-z][a-z0-9]*|${pep440_integer})")
string(CONCAT pep440_pattern
    "^${pep440_integer}\\.${pep440_integer}\\.${pep440_integer}"
    "((a|b|rc)${pep440_integer})?(\\.post${pep440_integer})?(\\.dev${pep440_integer})?"
    "(\\+${pep440_local_identifier}(\\.${pep440_local_identifier})*)?$"
)
if(NOT ISAACSIM_LIBRARIES_VERSION MATCHES "${pep440_pattern}")
    message(FATAL_ERROR "Invalid canonical PEP 440 library version: ${ISAACSIM_LIBRARIES_VERSION}")
endif()
string(REGEX MATCH "^[0-9]+\\.[0-9]+\\.[0-9]+" ISAACSIM_LIBRARIES_RELEASE_VERSION "${ISAACSIM_LIBRARIES_VERSION}")

project(IsaacSim VERSION ${ISAACSIM_LIBRARIES_RELEASE_VERSION} LANGUAGES C CXX)

set(CMAKE_C_STANDARD "${ISAACSIM_C_STANDARD}")
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD "${ISAACSIM_CXX_STANDARD}")
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(CTest)
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

cmake_path(ABSOLUTE_PATH CMAKE_CURRENT_SOURCE_DIR NORMALIZE OUTPUT_VARIABLE ISAACSIM_LIBRARIES_DIR)
cmake_path(GET ISAACSIM_LIBRARIES_DIR PARENT_PATH ISAACSIM_SOURCE_DIR)
cmake_path(GET ISAACSIM_SOURCE_DIR PARENT_PATH ISAACSIM_REPO_ROOT)

set(
    ISAACSIM_TARGET_DEPS_DIR "${ISAACSIM_REPO_ROOT}/_cmake_build/isaacsim-libraries-target-deps"
    CACHE PATH "Library dependency root"
)
set(ISAACSIM_PUBLIC_DEPS_ROOT "" CACHE PATH "Pixi prefix containing pinned public build dependencies")
if(WIN32)
    set(ISAACSIM_PUBLIC_DEPS_INCLUDE_DIR "${ISAACSIM_PUBLIC_DEPS_ROOT}/Library/include")
else()
    set(ISAACSIM_PUBLIC_DEPS_INCLUDE_DIR "${ISAACSIM_PUBLIC_DEPS_ROOT}/include")
endif()
set(ISAACSIM_NATIVE_RUNTIME_DEPS_DIR "" CACHE PATH "Pixi native-support Python dependency root")
set(ISAACSIM_PYTHON_TEST_DEPS_DIR "" CACHE PATH "Pixi Python test dependency root")
set(ISAACSIM_PYTHON_RUNTIME_DEPS_DIR "" CACHE PATH "Pixi Python runtime dependency root")
set(ISAACSIM_PYTHON_SCHEMA_BUILD_DEPS_DIR
    ""
    CACHE PATH "Pixi USD schema generator Python dependency root"
)
set(ISAACSIM_EXTENSION_USD_ROOT "" CACHE PATH
    "Optional Kit OpenUSD root used to build extension-compatible schema wheels"
)
set(ISAACSIM_MODULE_OUTPUT_ROOT "${CMAKE_BINARY_DIR}/modules")
set(ISAACSIM_PYTHON_INSTALL_DIR "python" CACHE STRING "Python component install root")
set(ISAACSIM_PYTHON_STAGE_DIR "${CMAKE_BINARY_DIR}/python")
set(
    ISAACSIM_DOCUMENTATION_CATALOG
    "${CMAKE_BINARY_DIR}/documentation/library_catalog.json"
    CACHE FILEPATH
    "Build-only catalog consumed by the standalone Isaac Sim documentation"
)
option(ISAACSIM_ENABLE_PYTHON "Stage, install, and test Python modules" ON)
option(ISAACSIM_ENABLE_PYTHON_BINDINGS "Build Python bindings" ON)
option(ISAACSIM_WARNINGS_AS_ERRORS "Treat module compiler warnings as errors" OFF)
option(ISAACSIM_ENFORCE_COMPLETE_PACKAGES "Fail when any distribution group is incomplete" OFF)
option(ISAACSIM_ENABLE_COVERAGE "Instrument native module targets for coverage" OFF)
set(ISAACSIM_PACKAGE_GROUP "" CACHE STRING "Distribution group to validate for independent packaging")
set(ISAACSIM_COLLECTED_PYTHON_LICENSE_FILE "" CACHE FILEPATH
    "Locked Python dependency license aggregate selected by the Pixi launcher"
)
set(ISAACSIM_PYTHON_LICENSE_FILE
    "${CMAKE_BINARY_DIR}/PACKAGE-LICENSES/isaacsim-libraries-PIP-LICENSES.txt"
)

if(ISAACSIM_ENABLE_PYTHON_BINDINGS AND NOT ISAACSIM_ENABLE_PYTHON)
    message(FATAL_ERROR "ISAACSIM_ENABLE_PYTHON_BINDINGS requires ISAACSIM_ENABLE_PYTHON")
endif()
if(NOT EXISTS "${ISAACSIM_COLLECTED_PYTHON_LICENSE_FILE}")
    message(FATAL_ERROR
        "The locked Python dependency license aggregate is missing: ${ISAACSIM_COLLECTED_PYTHON_LICENSE_FILE}. "
        "Run the library build through source/libraries/build.sh."
    )
endif()
file(SIZE "${ISAACSIM_COLLECTED_PYTHON_LICENSE_FILE}" isaacsim_python_license_size)
if(isaacsim_python_license_size EQUAL 0)
    message(FATAL_ERROR
        "The locked Python dependency license aggregate is empty: ${ISAACSIM_COLLECTED_PYTHON_LICENSE_FILE}"
    )
endif()
get_filename_component(isaacsim_python_license_directory "${ISAACSIM_PYTHON_LICENSE_FILE}" DIRECTORY)
file(MAKE_DIRECTORY "${isaacsim_python_license_directory}")
configure_file("${ISAACSIM_COLLECTED_PYTHON_LICENSE_FILE}" "${ISAACSIM_PYTHON_LICENSE_FILE}" COPYONLY)
unset(isaacsim_python_license_directory)
unset(isaacsim_python_license_size)
if(ISAACSIM_ENABLE_COVERAGE)
    if(NOT CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        message(FATAL_ERROR "ISAACSIM_ENABLE_COVERAGE requires GCC or Clang")
    endif()
    add_compile_options(--coverage -O0 -g)
    add_link_options(--coverage)
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(IsaacSimModule)

if(BUILD_TESTING)
    if(NOT EXISTS "${ISAACSIM_PUBLIC_DEPS_INCLUDE_DIR}/doctest/doctest.h")
        message(FATAL_ERROR
            "Pinned doctest headers are missing from '${ISAACSIM_PUBLIC_DEPS_INCLUDE_DIR}'. "
            "Run the library build without --no-pull."
        )
    endif()

    add_library(doctest::doctest INTERFACE IMPORTED GLOBAL)
    set_target_properties(doctest::doctest PROPERTIES
        INTERFACE_COMPILE_FEATURES cxx_std_11
        INTERFACE_INCLUDE_DIRECTORIES "${ISAACSIM_PUBLIC_DEPS_INCLUDE_DIR}"
    )
endif()

if(ISAACSIM_ENABLE_PYTHON)
    set(
        Python_ROOT_DIR
        "${ISAACSIM_PUBLIC_DEPS_ROOT}"
        CACHE PATH
        "Root of the Python installation selected by the build launcher"
    )
    set(python_components Interpreter)
    if(ISAACSIM_ENABLE_PYTHON_BINDINGS)
        list(APPEND python_components Development.Module Development.SABIModule)
    endif()
    find_package(Python "${ISAACSIM_PYTHON_VERSION}" EXACT REQUIRED COMPONENTS ${python_components})
endif()

if(ISAACSIM_ENABLE_PYTHON_BINDINGS)
    if(NOT TARGET tsl::robin_map)
        if(NOT EXISTS "${ISAACSIM_PUBLIC_DEPS_INCLUDE_DIR}/tsl/robin_map.h")
            message(FATAL_ERROR
                "Pinned tsl-robin-map headers are missing from '${ISAACSIM_PUBLIC_DEPS_INCLUDE_DIR}'. "
                "Run the library build without --no-pull."
            )
        endif()
        add_library(tsl::robin_map INTERFACE IMPORTED)
        set_target_properties(tsl::robin_map PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES "${ISAACSIM_PUBLIC_DEPS_INCLUDE_DIR}"
        )
    endif()
    execute_process(
        COMMAND "${Python_EXECUTABLE}" -I -m nanobind --cmake_dir
        RESULT_VARIABLE nanobind_cmake_result
        OUTPUT_VARIABLE nanobind_cmake_dir
        ERROR_VARIABLE nanobind_cmake_error
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    if(NOT nanobind_cmake_result EQUAL 0)
        message(FATAL_ERROR
            "Could not query the pinned nanobind CMake package: ${nanobind_cmake_error}"
        )
    endif()
    if(NOT EXISTS "${nanobind_cmake_dir}/nanobind-config.cmake")
        message(FATAL_ERROR
            "Pinned nanobind CMake package is missing from '${nanobind_cmake_dir}'. "
            "Run the library build without --no-pull."
        )
    endif()
    find_package(nanobind CONFIG REQUIRED
        PATHS "${nanobind_cmake_dir}"
        NO_DEFAULT_PATH
    )
    unset(nanobind_cmake_error)
    unset(nanobind_cmake_result)
    unset(nanobind_cmake_dir)
endif()

add_subdirectory(isaacsim/common)
add_subdirectory(isaacsim/asset)
add_subdirectory(isaacsim/physics)
add_subdirectory(isaacsim/physics_engines)
add_subdirectory(isaacsim/foundation)
add_subdirectory(isaacsim_usd_schemas/robot)
add_subdirectory(isaacsim_usd_schemas/robot_motion)
add_subdirectory(isaacsim/ovgl_viewport)
add_subdirectory(isaacsim_usd_schemas/virtual_gantry)
add_subdirectory(isaacsim_deprecated)
add_subdirectory(isaacsim/robot_motion)
add_subdirectory(isaacsim/ovsim)
add_subdirectory(isaacsim/performance)
isaacsim_finalize_modules()

if(BUILD_TESTING)
    if(ISAACSIM_ENABLE_PYTHON)
        add_test(NAME tests-library-packaging
            COMMAND ${Python_EXECUTABLE} -m unittest discover
                -s "${CMAKE_CURRENT_SOURCE_DIR}/packaging/tests"
                -v
        )
        cmake_path(CONVERT "${ISAACSIM_PYTHON_TEST_DEPS_DIR}" TO_NATIVE_PATH_LIST packaging_test_python_path)
        set_tests_properties(tests-library-packaging PROPERTIES
            ENVIRONMENT "PYTHONDONTWRITEBYTECODE=1;PYTHONPATH=${packaging_test_python_path}"
            LABELS "unit;python;packaging"
        )
        add_test(NAME tests-library-build-tool
            COMMAND ${Python_EXECUTABLE} -m unittest discover
                -s "${CMAKE_CURRENT_SOURCE_DIR}/tools/tests"
                -v
        )
        set_tests_properties(tests-library-build-tool PROPERTIES
            ENVIRONMENT
                "PYTHONDONTWRITEBYTECODE=1;ISAACSIM_DOCUMENTATION_CATALOG=${ISAACSIM_DOCUMENTATION_CATALOG}"
            LABELS "unit;python;build"
        )
    endif()

    add_test(NAME tests-package-dependencies
        COMMAND ${CMAKE_COMMAND}
            -P "${CMAKE_CURRENT_SOURCE_DIR}/cmake/tests/PackageDependencies.cmake"
    )
    set_tests_properties(tests-package-dependencies PROPERTIES LABELS "unit;cmake;packaging")
    add_test(NAME tests-python-import-registration
        COMMAND ${CMAKE_COMMAND}
            -P "${CMAKE_CURRENT_SOURCE_DIR}/cmake/tests/PythonImports.cmake"
    )
    set_tests_properties(tests-python-import-registration PROPERTIES LABELS "unit;cmake;python;packaging")
    add_test(NAME tests-nanobind-stub-preloads
        COMMAND ${CMAKE_COMMAND}
            -P "${CMAKE_CURRENT_SOURCE_DIR}/cmake/tests/Nanobind.cmake"
    )
    set_tests_properties(tests-nanobind-stub-preloads PROPERTIES LABELS "unit;cmake;python;bindings")
    add_test(NAME tests-native-testing-helpers
        COMMAND ${CMAKE_COMMAND}
            -P "${CMAKE_CURRENT_SOURCE_DIR}/cmake/tests/Testing.cmake"
    )
    set_tests_properties(tests-native-testing-helpers PROPERTIES LABELS "unit;cmake;testing")
    add_test(NAME tests-copy-runtime-directory
        COMMAND ${CMAKE_COMMAND}
            -P "${CMAKE_CURRENT_SOURCE_DIR}/cmake/tests/CopyRuntimeDirectory.cmake"
    )
    set_tests_properties(tests-copy-runtime-directory PROPERTIES LABELS "unit;cmake;packaging")

    get_property(complete_package_groups GLOBAL PROPERTY ISAACSIM_COMPLETE_GROUP_NAMES)
    if(ISAACSIM_ENABLE_PYTHON_BINDINGS AND complete_package_groups)
        set(example_package_group_arguments)
        foreach(package_group IN LISTS complete_package_groups)
            list(APPEND example_package_group_arguments "--package-group=${package_group}")
        endforeach()
        add_test(NAME tests-examples
            COMMAND ${Python_EXECUTABLE}
                "${CMAKE_CURRENT_SOURCE_DIR}/testing/examples/run_examples_test.py"
                "--cmake=${CMAKE_COMMAND}"
                "--library-build-dir=${CMAKE_BINARY_DIR}"
                "--examples-dir=${ISAACSIM_SOURCE_DIR}/examples"
                "--system-requirements-file=${ISAACSIM_SYSTEM_REQUIREMENTS_FILE}"
                "--test-root=${CMAKE_BINARY_DIR}/examples-test"
                "--python=${Python_EXECUTABLE}"
                "--python-install-dir=${ISAACSIM_PYTHON_INSTALL_DIR}"
                "--python-runtime-deps-dir=${ISAACSIM_PYTHON_RUNTIME_DEPS_DIR}"
                "--target-deps-dir=${ISAACSIM_TARGET_DEPS_DIR}"
                "--readelf=${CMAKE_READELF}"
                "--generator=${CMAKE_GENERATOR}"
                "--generator-platform=${CMAKE_GENERATOR_PLATFORM}"
                "--generator-toolset=${CMAKE_GENERATOR_TOOLSET}"
                "--make-program=${CMAKE_MAKE_PROGRAM}"
                "--config=$<CONFIG>"
                ${example_package_group_arguments}
        )
        set_tests_properties(tests-examples PROPERTIES
            LABELS "install;consumer;examples"
            RUN_SERIAL TRUE
        )
        unset(example_package_group_arguments)
    endif()

    if(WIN32 AND ISAACSIM_ENABLE_PYTHON_BINDINGS AND complete_package_groups)
        set(public_entry_point_build_root "${CMAKE_BINARY_DIR}/examples-public-entry-point-test")
        add_test(NAME tests-examples-public-entry-point-clean
            COMMAND ${CMAKE_COMMAND} -E rm -rf "${public_entry_point_build_root}"
        )
        set_tests_properties(tests-examples-public-entry-point-clean PROPERTIES
            FIXTURES_SETUP examples-public-entry-point
            LABELS "consumer;examples;entry-point;setup"
        )
        add_test(NAME tests-examples-public-entry-point
            COMMAND "${ISAACSIM_SOURCE_DIR}/examples/example.bat"
                "--build-root=${public_entry_point_build_root}"
                "--config=$<CONFIG>"
                build
                hello_world.cpp
        )
        set_tests_properties(tests-examples-public-entry-point PROPERTIES
            FIXTURES_REQUIRED examples-public-entry-point
            LABELS "install;consumer;examples;entry-point"
        )
        unset(public_entry_point_build_root)
    endif()

    foreach(package_group IN LISTS complete_package_groups)
        _isaacsim_get_group_dependency_closure("${package_group}" package_group_closure)
        get_property(package_version GLOBAL PROPERTY "ISAACSIM_GROUP_${package_group}_VERSION")
        get_property(package_dependencies GLOBAL PROPERTY "ISAACSIM_GROUP_${package_group}_DEPENDENCIES")
        set(dependency_specs)
        foreach(dependency IN LISTS package_dependencies)
            get_property(dependency_specifier GLOBAL PROPERTY
                "ISAACSIM_GROUP_${package_group}_DEPENDENCY_${dependency}_SPECIFIER"
            )
            list(APPEND dependency_specs "${dependency}|${dependency_specifier}")
        endforeach()
        get_property(package_targets GLOBAL PROPERTY "ISAACSIM_GROUP_${package_group}_TARGETS")
        if(package_targets)
            set(has_cmake_package ON)
        else()
            set(has_cmake_package OFF)
        endif()
        set(install_contract_source_dir "${CMAKE_CURRENT_BINARY_DIR}/install-contract-consumer/${package_group}")
        _isaacsim_generate_install_contract("${install_contract_source_dir}" "${package_group}")
        set(test_name "tests-install-contract-${package_group}")
        add_test(NAME "${test_name}"
            COMMAND ${CMAKE_COMMAND}
                "-DLIBRARY_BUILD_DIR=${CMAKE_BINARY_DIR}"
                "-DCONSUMER_SOURCE_DIR=${install_contract_source_dir}"
                "-DTEST_ROOT=${CMAKE_BINARY_DIR}/install-contract-test/${package_group}"
                "-DPYTHON_EXECUTABLE=${Python_EXECUTABLE}"
                "-DPYTHON_ENABLED=${ISAACSIM_ENABLE_PYTHON}"
                "-DPYTHON_INSTALL_DIR=${ISAACSIM_PYTHON_INSTALL_DIR}"
                "-DPYTHON_TEST_SCRIPT=${install_contract_source_dir}/test_python.py"
                "-DPYTHON_RUNTIME_DEPS_DIR=${ISAACSIM_PYTHON_RUNTIME_DEPS_DIR}"
                "-DNATIVE_RUNTIME_DEPS_DIR=${ISAACSIM_NATIVE_RUNTIME_DEPS_DIR}"
                "-DPACKAGE_GROUP=${package_group}"
                "-DPACKAGE_VERSION=${package_version}"
                "-DPACKAGE_GROUPS=${package_group_closure}"
                "-DDEPENDENCY_SPECS=${dependency_specs}"
                "-DHAS_CMAKE_PACKAGE=${has_cmake_package}"
                "-DBINDINGS_ENABLED=${ISAACSIM_ENABLE_PYTHON_BINDINGS}"
                "-DTEST_GENERATOR=${CMAKE_GENERATOR}"
                "-DTEST_GENERATOR_PLATFORM=${CMAKE_GENERATOR_PLATFORM}"
                "-DTEST_GENERATOR_TOOLSET=${CMAKE_GENERATOR_TOOLSET}"
                "-DTEST_MAKE_PROGRAM=${CMAKE_MAKE_PROGRAM}"
                "-DCONFIG=$<CONFIG>"
                -P "${CMAKE_CURRENT_SOURCE_DIR}/cmake/RunInstallContractTest.cmake"
        )
        set_tests_properties("${test_name}" PROPERTIES LABELS "install;consumer;${package_group}" RUN_SERIAL TRUE)
    endforeach()
endif()
