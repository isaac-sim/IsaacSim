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

if(NOT LIBRARY_BUILD_DIR OR NOT CONSUMER_SOURCE_DIR OR NOT TEST_ROOT OR NOT PACKAGE_GROUP OR NOT PACKAGE_GROUPS)
    message(FATAL_ERROR "Installed consumer test paths were not provided")
endif()

if(NOT CONFIG)
    set(CONFIG Release)
endif()

if(NOT TEST_GENERATOR)
    message(FATAL_ERROR "Installed consumer test generator was not provided")
endif()

set(consumer_generator_arguments -G "${TEST_GENERATOR}")
if(TEST_GENERATOR_PLATFORM)
    list(APPEND consumer_generator_arguments -A "${TEST_GENERATOR_PLATFORM}")
endif()
if(TEST_GENERATOR_TOOLSET)
    list(APPEND consumer_generator_arguments -T "${TEST_GENERATOR_TOOLSET}")
endif()
if(TEST_MAKE_PROGRAM)
    list(APPEND consumer_generator_arguments "-DCMAKE_MAKE_PROGRAM=${TEST_MAKE_PROGRAM}")
endif()

set(cpp_prefix "${TEST_ROOT}/cpp")
set(python_prefix "${TEST_ROOT}/python-package")
set(consumer_build_dir "${TEST_ROOT}/consumer-build")
file(REMOVE_RECURSE "${TEST_ROOT}")
if(BINDINGS_ENABLED)
    set(load_installed_libraries OFF)
else()
    set(load_installed_libraries ON)
endif()

function(run_checked description)
    execute_process(
        COMMAND ${ARGN}
        RESULT_VARIABLE result
        OUTPUT_VARIABLE output
        ERROR_VARIABLE error
    )
    if(NOT result EQUAL 0)
        message(FATAL_ERROR "${description} failed (${result})\n${output}\n${error}")
    endif()
endfunction()

function(run_expect_failure description)
    execute_process(
        COMMAND ${ARGN}
        RESULT_VARIABLE result
        OUTPUT_VARIABLE output
        ERROR_VARIABLE error
    )
    if(result EQUAL 0)
        message(FATAL_ERROR "${description} unexpectedly succeeded\n${output}\n${error}")
    endif()
endfunction()

if(DEPENDENCY_SPECS AND HAS_CMAKE_PACKAGE)
    set(wrong_version_root "${TEST_ROOT}/wrong-version")
    set(wrong_version_package_prefix "${wrong_version_root}/package")
    set(wrong_version_dependency_prefix "${wrong_version_root}/dependency")
    set(wrong_version_consumer_build_dir "${wrong_version_root}/consumer-build")
    run_checked("${PACKAGE_GROUP} isolated runtime installation"
        "${CMAKE_COMMAND}" --install "${LIBRARY_BUILD_DIR}"
        --prefix "${wrong_version_package_prefix}" --component "${PACKAGE_GROUP}-runtime" --config "${CONFIG}"
    )
    run_checked("${PACKAGE_GROUP} isolated development installation"
        "${CMAKE_COMMAND}" --install "${LIBRARY_BUILD_DIR}"
        --prefix "${wrong_version_package_prefix}" --component "${PACKAGE_GROUP}-development" --config "${CONFIG}"
    )
    list(GET DEPENDENCY_SPECS 0 wrong_version_dependency)
    string(REGEX MATCH "^([^|]+)\\|(.+)$" unused "${wrong_version_dependency}")
    set(dependency_name "${CMAKE_MATCH_1}")
    file(MAKE_DIRECTORY "${wrong_version_dependency_prefix}")
    file(WRITE "${wrong_version_dependency_prefix}/${dependency_name}Config.cmake"
        "set(${dependency_name}_VERSION \"0.0.0\")\n"
    )
    set(wrong_version_prefix_path "${wrong_version_dependency_prefix};${wrong_version_package_prefix}")
    run_expect_failure("${PACKAGE_GROUP} exact dependency validation"
        "${CMAKE_COMMAND}" -S "${CONSUMER_SOURCE_DIR}" -B "${wrong_version_consumer_build_dir}"
        ${consumer_generator_arguments}
        "-DCMAKE_PREFIX_PATH=${wrong_version_prefix_path}" "-DCMAKE_BUILD_TYPE=${CONFIG}"
    )
endif()

foreach(package_group IN LISTS PACKAGE_GROUPS)
    run_checked("${package_group} runtime installation"
        "${CMAKE_COMMAND}" --install "${LIBRARY_BUILD_DIR}"
        --prefix "${cpp_prefix}" --component "${package_group}-runtime" --config "${CONFIG}"
    )
    run_checked("${package_group} development installation"
        "${CMAKE_COMMAND}" --install "${LIBRARY_BUILD_DIR}"
        --prefix "${cpp_prefix}" --component "${package_group}-development" --config "${CONFIG}"
    )
    run_checked("${package_group} documentation installation"
        "${CMAKE_COMMAND}" --install "${LIBRARY_BUILD_DIR}"
        --prefix "${cpp_prefix}" --component "${package_group}-documentation" --config "${CONFIG}"
    )
    set(license_relative_path
        "share/licenses/${package_group}/isaacsim-libraries-PIP-LICENSES.txt"
    )
    set(expected_license
        "${LIBRARY_BUILD_DIR}/PACKAGE-LICENSES/isaacsim-libraries-PIP-LICENSES.txt"
    )
    if(NOT EXISTS "${expected_license}")
        message(FATAL_ERROR "Build-owned dependency license snapshot is missing: ${expected_license}")
    endif()
    file(SHA256 "${expected_license}" expected_license_digest)
    set(license_prefixes "${cpp_prefix}")
    if(PYTHON_ENABLED)
        run_checked("${package_group} Python installation"
            "${CMAKE_COMMAND}" --install "${LIBRARY_BUILD_DIR}"
            --prefix "${python_prefix}" --component "${package_group}-python" --config "${CONFIG}"
        )
        list(APPEND license_prefixes "${python_prefix}")
    endif()
    foreach(prefix IN LISTS license_prefixes)
        set(installed_license "${prefix}/${license_relative_path}")
        if(NOT EXISTS "${installed_license}")
            message(FATAL_ERROR
                "${package_group} dependency license aggregate was not installed: ${installed_license}"
            )
        endif()
        file(SIZE "${installed_license}" installed_license_size)
        if(installed_license_size EQUAL 0)
            message(FATAL_ERROR
                "${package_group} dependency license aggregate is empty: ${installed_license}"
            )
        endif()
        file(SHA256 "${installed_license}" installed_license_digest)
        if(NOT installed_license_digest STREQUAL expected_license_digest)
            message(FATAL_ERROR
                "${package_group} dependency license aggregate does not match its build-owned snapshot: "
                "${installed_license}"
            )
        endif()
    endforeach()
endforeach()

set(package_manifest "${cpp_prefix}/share/isaacsim/packages/${PACKAGE_GROUP}/package.json")
if(NOT EXISTS "${package_manifest}")
    message(FATAL_ERROR "${PACKAGE_GROUP} package manifest was not installed: ${package_manifest}")
endif()
file(READ "${package_manifest}" package_manifest_json)
string(JSON installed_package_name GET "${package_manifest_json}" name)
string(JSON installed_package_version GET "${package_manifest_json}" version)
if(NOT installed_package_name STREQUAL PACKAGE_GROUP OR NOT installed_package_version STREQUAL PACKAGE_VERSION)
    message(FATAL_ERROR
        "Installed package identity mismatch: ${installed_package_name} ${installed_package_version}"
    )
endif()
get_filename_component(package_metadata_dir "${package_manifest}" DIRECTORY)
set(installed_version_file "${package_metadata_dir}/VERSION")
if(NOT EXISTS "${installed_version_file}")
    message(FATAL_ERROR "${PACKAGE_GROUP} shared VERSION was not installed")
endif()
file(READ "${installed_version_file}" installed_version)
string(STRIP "${installed_version}" installed_version)
if(NOT installed_version STREQUAL PACKAGE_VERSION)
    message(FATAL_ERROR "${PACKAGE_GROUP} installed VERSION mismatch: ${installed_version}")
endif()
string(JSON installed_module_count LENGTH "${package_manifest_json}" modules)
if(installed_module_count GREATER 0)
    math(EXPR last_module_index "${installed_module_count} - 1")
    foreach(index RANGE 0 ${last_module_index})
        string(JSON installed_module GET "${package_manifest_json}" modules ${index})
        set(installed_changelog "${package_metadata_dir}/modules/${installed_module}/CHANGELOG.md")
        if(NOT EXISTS "${installed_changelog}")
            message(FATAL_ERROR "${installed_module} changelog was not installed: ${installed_changelog}")
        endif()
    endforeach()
endif()
string(JSON installed_dependency_count LENGTH "${package_manifest_json}" dependencies)
list(LENGTH DEPENDENCY_SPECS expected_dependency_count)
if(NOT installed_dependency_count EQUAL expected_dependency_count)
    message(FATAL_ERROR
        "Installed dependency count mismatch: ${installed_dependency_count}, expected ${expected_dependency_count}"
    )
endif()
foreach(dependency_spec IN LISTS DEPENDENCY_SPECS)
    string(REGEX MATCH "^([^|]+)\\|(.+)$" unused "${dependency_spec}")
    set(dependency_name "${CMAKE_MATCH_1}")
    set(expected_dependency_specifier "${CMAKE_MATCH_2}")
    string(JSON installed_dependency_specifier GET "${package_manifest_json}" dependencies "${dependency_name}")
    if(NOT "${installed_dependency_specifier}" STREQUAL "${expected_dependency_specifier}")
        message(FATAL_ERROR
            "Installed exact dependency mismatch for ${dependency_name}: ${installed_dependency_specifier}, "
            "expected ${expected_dependency_specifier}"
        )
    endif()
endforeach()
# Inspect installed artifacts, not build-tree files: compiler-injected RPATHs
# and copied local runtime helpers can otherwise pass header/import checks while
# still depending on the build machine. Other platforms use different loaders.
if(CMAKE_HOST_SYSTEM_NAME STREQUAL "Linux")
    find_program(installed_readelf NAMES readelf llvm-readelf REQUIRED)
    set(installed_prefixes "${cpp_prefix}")
    if(PYTHON_ENABLED)
        list(APPEND installed_prefixes "${python_prefix}")
    endif()
    foreach(installed_prefix IN LISTS installed_prefixes)
        file(GLOB_RECURSE installed_libraries
            "${installed_prefix}/*.so" "${installed_prefix}/*.so.*" "${installed_prefix}/bin/isaacsim-*")
        foreach(installed_library IN LISTS installed_libraries)
            if(IS_DIRECTORY "${installed_library}" OR IS_SYMLINK "${installed_library}")
                continue()
            endif()
            # Generated module/runtime names and binding paths identify our binaries.
            # Imported vendor payloads retain their upstream loader policy.
            get_filename_component(installed_library_name "${installed_library}" NAME)
            if(NOT installed_library_name MATCHES "^libisaacsim[-_].*\\.so"
               AND NOT installed_library MATCHES "/bindings/_bindings[^/]*\\.so$"
               AND NOT installed_library MATCHES "/bin/isaacsim-[^/]+$")
                continue()
            endif()
            file(READ "${installed_library}" elf_magic LIMIT 4 HEX)
            if(NOT elf_magic STREQUAL "7f454c46")
                continue()
            endif()
            execute_process(
                COMMAND "${CMAKE_COMMAND}" -E env LC_ALL=C "${installed_readelf}" -d "${installed_library}"
                RESULT_VARIABLE readelf_result
                OUTPUT_VARIABLE dynamic_entries
                ERROR_VARIABLE readelf_error
            )
            if(NOT readelf_result EQUAL 0)
                message(FATAL_ERROR "Cannot inspect installed runtime ${installed_library}: ${readelf_error}")
            endif()
            string(REGEX MATCHALL "[^\n]*(RPATH|RUNPATH)[^\n]*" runtime_path_entries "${dynamic_entries}")
            foreach(runtime_path_entry IN LISTS runtime_path_entries)
                string(REGEX REPLACE ".*\\[([^]]*)\\].*" "\\1" runtime_paths "${runtime_path_entry}")
                string(REPLACE ":" ";" runtime_paths "${runtime_paths}")
                foreach(runtime_path IN LISTS runtime_paths)
                    if(IS_ABSOLUTE "${runtime_path}")
                        message(FATAL_ERROR
                            "Installed runtime ${installed_library} contains absolute RPATH: ${runtime_path}")
                    endif()
                endforeach()
            endforeach()
        endforeach()
    endforeach()
endif()

run_checked("Installed consumer configuration"
    "${CMAKE_COMMAND}" -S "${CONSUMER_SOURCE_DIR}" -B "${consumer_build_dir}"
    ${consumer_generator_arguments}
    "-DCMAKE_PREFIX_PATH=${cpp_prefix}" "-DCMAKE_BUILD_TYPE=${CONFIG}"
    "-DISAACSIM_INSTALL_CONTRACT_LOAD_LIBRARIES=${load_installed_libraries}"
)
run_checked("Installed consumer build"
    "${CMAKE_COMMAND}" --build "${consumer_build_dir}" --config "${CONFIG}" --parallel
)

if(WIN32)
    file(GLOB consumer_executables LIST_DIRECTORIES FALSE "${consumer_build_dir}/bin/isaacsim-consumer-*.exe")
    set(installed_runtime_paths "${cpp_prefix}/bin;$ENV{PATH}")
    cmake_path(CONVERT "${installed_runtime_paths}" TO_NATIVE_PATH_LIST installed_runtime_path)
    string(REPLACE ";" "\\;" installed_runtime_path "${installed_runtime_path}")
else()
    file(GLOB consumer_executables LIST_DIRECTORIES FALSE "${consumer_build_dir}/bin/isaacsim-consumer-*")
endif()
foreach(consumer_executable IN LISTS consumer_executables)
    if(WIN32)
        run_checked("Installed API consumer ${consumer_executable}"
            "${CMAKE_COMMAND}" -E env "PATH=${installed_runtime_path}" "${consumer_executable}"
        )
    else()
        run_checked("Installed API consumer ${consumer_executable}" "${consumer_executable}")
    endif()
endforeach()

if(PYTHON_ENABLED AND PYTHON_EXECUTABLE AND PYTHON_TEST_SCRIPT)
    set(installed_python_paths
        "${python_prefix}/${PYTHON_INSTALL_DIR};${NATIVE_RUNTIME_DEPS_DIR};${PYTHON_RUNTIME_DEPS_DIR}"
    )
    cmake_path(CONVERT "${installed_python_paths}" TO_NATIVE_PATH_LIST installed_python_path)
    string(REPLACE ";" "\\;" installed_python_path "${installed_python_path}")
    run_checked("Installed Python API consumer"
        "${CMAKE_COMMAND}" -E env
        "PYTHONDONTWRITEBYTECODE=1"
        "PYTHONPATH=${installed_python_path}"
        "${PYTHON_EXECUTABLE}" "${PYTHON_TEST_SCRIPT}"
    )
endif()

# Preserve the complete test installation on failure for diagnostics, but do not retain several gigabytes of duplicate
# runtime libraries after a successful contract check.
file(REMOVE_RECURSE "${TEST_ROOT}")
