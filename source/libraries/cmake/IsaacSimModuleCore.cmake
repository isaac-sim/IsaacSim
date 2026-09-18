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

include_guard(GLOBAL)

include(CMakeParseArguments)
include(CMakePackageConfigHelpers)
include(GenerateExportHeader)
include("${CMAKE_CURRENT_LIST_DIR}/IsaacSimPackagingFilters.cmake")

if(APPLE)
    set(ISAACSIM_LOADER_ORIGIN "@loader_path")
elseif(WIN32)
    set(ISAACSIM_LOADER_ORIGIN "")
else()
    set(ISAACSIM_LOADER_ORIGIN "$ORIGIN")
endif()

function(_isaacsim_validate_relative_path description path)
    cmake_path(NORMAL_PATH path OUTPUT_VARIABLE normalized_path)
    if(IS_ABSOLUTE "${normalized_path}" OR normalized_path MATCHES "(^|/)\.\.(/|$)")
        message(FATAL_ERROR "${description} must be relative and must not contain '..': ${path}")
    endif()
endfunction()

function(_isaacsim_count_path_components path output)
    _isaacsim_validate_relative_path("Install path" "${path}")
    cmake_path(NORMAL_PATH path OUTPUT_VARIABLE normalized_path)
    string(REPLACE "/" ";" components "${normalized_path}")
    list(FILTER components EXCLUDE REGEX "^(|\.)$")
    list(LENGTH components component_count)
    set(${output} "${component_count}" PARENT_SCOPE)
endfunction()

function(_isaacsim_module_name_to_c_symbol_prefix module_name output)
    string(REGEX REPLACE "[._]+" ";" module_name_parts "${module_name}")
    list(POP_FRONT module_name_parts symbol_prefix)
    foreach(module_name_part IN LISTS module_name_parts)
        string(SUBSTRING "${module_name_part}" 0 1 first_character)
        string(TOUPPER "${first_character}" first_character)
        string(SUBSTRING "${module_name_part}" 1 -1 remaining_characters)
        string(APPEND symbol_prefix "${first_character}${remaining_characters}")
    endforeach()
    set(${output} "${symbol_prefix}" PARENT_SCOPE)
endfunction()

function(_isaacsim_get_source_path_segments output_relative_path output_segments)
    file(RELATIVE_PATH relative_path "${ISAACSIM_LIBRARIES_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}")
    string(REPLACE "\\" "/" relative_path "${relative_path}")
    if(relative_path STREQUAL "" OR relative_path MATCHES "^\.\.(/|$)" OR IS_ABSOLUTE "${relative_path}")
        message(FATAL_ERROR
            "Source directory must be below ISAACSIM_LIBRARIES_DIR (${ISAACSIM_LIBRARIES_DIR}): "
            "${CMAKE_CURRENT_SOURCE_DIR}"
        )
    endif()

    string(REPLACE "/" ";" path_segments "${relative_path}")
    foreach(path_segment IN LISTS path_segments)
        if(NOT path_segment MATCHES "^[a-z][a-z0-9_]*$")
            message(FATAL_ERROR
                "Source path components must be lowercase identifiers: ${relative_path}"
            )
        endif()
    endforeach()
    set(${output_relative_path} "${relative_path}" PARENT_SCOPE)
    set(${output_segments} "${path_segments}" PARENT_SCOPE)
endfunction()

function(_isaacsim_validate_group_source_path group_name requested_module_prefix output_module_prefix)
    _isaacsim_get_source_path_segments(relative_path path_segments)
    list(LENGTH path_segments path_segment_count)

    if(path_segment_count EQUAL 1)
        list(GET path_segments 0 distribution_segment)
        if(NOT group_name STREQUAL distribution_segment)
            message(FATAL_ERROR
                "Distribution ${group_name} does not match its source path ${relative_path}; "
                "expected ${distribution_segment}"
            )
        endif()
        if(NOT requested_module_prefix)
            message(FATAL_ERROR
                "Flat distribution ${group_name} must declare MODULE_PREFIX"
            )
        endif()
        if(NOT requested_module_prefix MATCHES "^isaacsim(\\.[a-z][a-z0-9_]*)+$")
            message(FATAL_ERROR
                "Distribution ${group_name} has invalid MODULE_PREFIX: ${requested_module_prefix}"
            )
        endif()
        set(module_prefix "${requested_module_prefix}")
    elseif(path_segment_count EQUAL 2)
        list(GET path_segments 0 namespace_segment)
        list(GET path_segments 1 distribution_segment)
        if(namespace_segment STREQUAL "isaacsim")
            set(expected_group_name "${namespace_segment}_${distribution_segment}")
            if(NOT group_name STREQUAL expected_group_name)
                message(FATAL_ERROR
                    "Distribution ${group_name} does not match its source path ${relative_path}; "
                    "expected ${expected_group_name}"
                )
            endif()
            set(module_prefix "${namespace_segment}.${distribution_segment}")
            if(requested_module_prefix AND NOT requested_module_prefix STREQUAL module_prefix)
                message(FATAL_ERROR
                    "Distribution ${group_name} MODULE_PREFIX ${requested_module_prefix} does not match its source "
                    "path; "
                    "expected ${module_prefix}"
                )
            endif()
        else()
            # A namespace grouping directory outside `isaacsim/` (for example the
            # `isaacsim_usd_schemas/<distribution>` schema roots that mirror the
            # external `newton-usd-schemas`/`physx-usd-schemas` package boundary)
            # must declare an explicit MODULE_PREFIX because its distribution name
            # and public import namespace intentionally differ from the source
            # directory. The distribution name stays authoritative in pyproject.toml.
            if(NOT requested_module_prefix)
                message(FATAL_ERROR
                    "Distribution ${group_name} outside the isaacsim namespace directory must declare MODULE_PREFIX; "
                    "found ${relative_path}"
                )
            endif()
            if(NOT requested_module_prefix MATCHES "^[a-z][a-z0-9_]*(\\.[a-z][a-z0-9_]*)+$")
                message(FATAL_ERROR
                    "Distribution ${group_name} has invalid MODULE_PREFIX: ${requested_module_prefix}"
                )
            endif()
            set(module_prefix "${requested_module_prefix}")
        endif()
    else()
        message(FATAL_ERROR
            "Distribution ${group_name} must be registered one or two directories below ISAACSIM_LIBRARIES_DIR; "
            "found ${relative_path}"
        )
    endif()
    set(${output_module_prefix} "${module_prefix}" PARENT_SCOPE)
endfunction()

function(_isaacsim_validate_module_source_path module_name group_name)
    get_property(group_source_root GLOBAL PROPERTY "ISAACSIM_GROUP_${group_name}_SOURCE_ROOT")
    get_property(module_prefix GLOBAL PROPERTY "ISAACSIM_GROUP_${group_name}_MODULE_PREFIX")
    if(NOT group_source_root OR NOT module_prefix)
        message(FATAL_ERROR "Module ${module_name} belongs to unregistered distribution ${group_name}")
    endif()

    file(RELATIVE_PATH relative_path "${group_source_root}" "${CMAKE_CURRENT_SOURCE_DIR}")
    string(REPLACE "\\" "/" relative_path "${relative_path}")
    if(relative_path STREQUAL "" OR relative_path MATCHES "^\\.\\.(/|$)" OR IS_ABSOLUTE "${relative_path}")
        message(FATAL_ERROR
            "Module ${module_name} must be below distribution ${group_name} root ${group_source_root}; "
            "found ${CMAKE_CURRENT_SOURCE_DIR}"
        )
    endif()

    string(REPLACE "/" ";" path_segments "${relative_path}")
    foreach(path_segment IN LISTS path_segments)
        if(NOT path_segment MATCHES "^[a-z][a-z0-9_]*$")
            message(FATAL_ERROR
                "Module source path components must be lowercase identifiers: ${relative_path}"
            )
        endif()
    endforeach()

    string(REPLACE "/" "." relative_module_name "${relative_path}")
    set(expected_module_name "${module_prefix}.${relative_module_name}")
    if(NOT module_name STREQUAL expected_module_name)
        message(FATAL_ERROR
            "Module ${module_name} does not match its source path ${relative_path}; expected ${expected_module_name}"
        )
    endif()
endfunction()

function(_isaacsim_validate_public_headers module_name)
    set(multi_value_args PUBLIC_C_HEADERS PUBLIC_CPP_HEADERS)
    cmake_parse_arguments(ARG "" "" "${multi_value_args}" ${ARGN})
    if(NOT ARG_PUBLIC_C_HEADERS AND NOT ARG_PUBLIC_CPP_HEADERS)
        message(FATAL_ERROR "${module_name} must declare at least one public C or C++ header")
    endif()

    foreach(header IN LISTS ARG_PUBLIC_C_HEADERS)
        _isaacsim_validate_relative_path("${module_name} public C header" "${header}")
        if(NOT header MATCHES "\\.h$")
            message(FATAL_ERROR "${module_name} public C header must use the .h extension: ${header}")
        endif()
        if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/include/${header}")
            message(FATAL_ERROR "${module_name} public C header does not exist: include/${header}")
        endif()
        file(READ "${CMAKE_CURRENT_SOURCE_DIR}/include/${header}" header_contents)
        if(header_contents MATCHES "#[ \t]*include[ \t]*[<\"]carb/")
            message(FATAL_ERROR "${module_name} public C header exposes Carbonite: include/${header}")
        endif()
    endforeach()
    foreach(header IN LISTS ARG_PUBLIC_CPP_HEADERS)
        _isaacsim_validate_relative_path("${module_name} public C++ header" "${header}")
        if(NOT header MATCHES "\\.(hpp|tpp)$")
            message(FATAL_ERROR "${module_name} public C++ header must use the .hpp or .tpp extension: ${header}")
        endif()
        if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/include/${header}")
            message(FATAL_ERROR "${module_name} public C++ header does not exist: include/${header}")
        endif()
        file(READ "${CMAKE_CURRENT_SOURCE_DIR}/include/${header}" header_contents)
        if(header_contents MATCHES "#[ \t]*include[ \t]*[<\"]carb/")
            message(FATAL_ERROR "${module_name} public C++ header exposes Carbonite: include/${header}")
        endif()
    endforeach()

    file(GLOB_RECURSE source_public_headers
        RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/include"
        CONFIGURE_DEPENDS
        LIST_DIRECTORIES FALSE
        "${CMAKE_CURRENT_SOURCE_DIR}/include/*"
    )
    set(declared_public_headers ${ARG_PUBLIC_C_HEADERS} ${ARG_PUBLIC_CPP_HEADERS})
    list(SORT source_public_headers)
    list(SORT declared_public_headers)
    if(NOT "${source_public_headers}" STREQUAL "${declared_public_headers}")
        message(FATAL_ERROR
            "${module_name} include/ inventory does not match its declared public headers.\n"
            "Files: ${source_public_headers}\nDeclared: ${declared_public_headers}"
        )
    endif()
endfunction()

function(_isaacsim_install_public_headers group_name)
    _isaacsim_get_group_component("${group_name}" development development_component)
    foreach(header IN LISTS ARGN)
        get_filename_component(header_directory "${header}" DIRECTORY)
        install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/include/${header}"
            DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/${header_directory}"
            COMPONENT "${development_component}"
        )
    endforeach()
endfunction()

function(_isaacsim_validate_dependencies module_name)
    foreach(dependency IN LISTS ARGN)
        if(dependency MATCHES "(^|::)(carb|omni([-_.]|::)(kit|ext|graph))")
            message(FATAL_ERROR "${module_name} declares forbidden dependency ${dependency}")
        endif()
    endforeach()
endfunction()

function(_isaacsim_validate_private_dependencies module_name)
    foreach(dependency IN LISTS ARGN)
        if(dependency MATCHES "(^|::)omni([-_.]|::)(kit|ext|graph)")
            message(FATAL_ERROR "${module_name} declares forbidden private dependency ${dependency}")
        endif()
        if(dependency MATCHES "(^|::)carb")
            _isaacsim_resolve_target("${dependency}" dependency_target)
            if(NOT TARGET "${dependency_target}")
                message(FATAL_ERROR "${module_name} private Carbonite dependency is not a target: ${dependency}")
            endif()
            get_target_property(dependency_type "${dependency_target}" TYPE)
            if(NOT dependency_type STREQUAL "STATIC_LIBRARY")
                message(FATAL_ERROR
                    "${module_name} may only use Carbonite as a private static dependency: ${dependency}"
                )
            endif()
        endif()
    endforeach()
endfunction()

function(_isaacsim_get_current_group output_name output_version)
    if(NOT ISAACSIM_CURRENT_GROUP_NAME)
        message(FATAL_ERROR "Module registration must occur below an isaacsim_add_group() call")
    endif()
    get_property(group_version GLOBAL PROPERTY "ISAACSIM_GROUP_${ISAACSIM_CURRENT_GROUP_NAME}_VERSION")
    set(${output_name} "${ISAACSIM_CURRENT_GROUP_NAME}" PARENT_SCOPE)
    set(${output_version} "${group_version}" PARENT_SCOPE)
endfunction()

function(_isaacsim_get_group_component group_name kind output)
    set(${output} "${group_name}-${kind}" PARENT_SCOPE)
endfunction()

function(_isaacsim_validate_group_target_dependencies module_name group_name)
    get_property(declared_group_dependencies GLOBAL PROPERTY "ISAACSIM_GROUP_${group_name}_DEPENDENCIES")
    foreach(dependency IN LISTS ARGN)
        _isaacsim_resolve_target("${dependency}" dependency_target)
        if(NOT TARGET "${dependency_target}")
            continue()
        endif()
        get_target_property(dependency_group "${dependency_target}" ISAACSIM_GROUP_NAME)
        if(NOT dependency_group OR dependency_group MATCHES "-NOTFOUND$" OR dependency_group STREQUAL group_name)
            continue()
        endif()
        if(NOT dependency_group IN_LIST declared_group_dependencies)
            message(FATAL_ERROR
                "${module_name} links to ${dependency} from distribution ${dependency_group}, but ${group_name} "
                "does not declare that distribution with isaacsim_add_group_dependency(...)"
            )
        endif()
        set_property(GLOBAL APPEND PROPERTY "ISAACSIM_GROUP_${group_name}_USED_DEPENDENCIES" "${dependency_group}")
    endforeach()
endfunction()

function(_isaacsim_validate_external_package_dependencies module_name)
    get_property(group_names GLOBAL PROPERTY ISAACSIM_GROUP_NAMES)
    foreach(package_dependency IN LISTS ARGN)
        if(package_dependency IN_LIST group_names)
            message(FATAL_ERROR
                "${module_name} lists internal distribution ${package_dependency} in PACKAGE_DEPENDENCIES; "
                "declare it once with isaacsim_add_group_dependency(...)"
            )
        endif()
    endforeach()
endfunction()

function(_isaacsim_register_group_module module_name group_name)
    string(MAKE_C_IDENTIFIER "${module_name}" module_key)
    get_property(existing_group GLOBAL PROPERTY "ISAACSIM_MODULE_${module_key}_GROUP")
    get_property(existing_root GLOBAL PROPERTY "ISAACSIM_MODULE_${module_key}_ROOT")
    if(existing_group AND NOT existing_group STREQUAL group_name)
        message(FATAL_ERROR
            "Module ${module_name} is already registered to distribution ${existing_group}, not ${group_name}"
        )
    endif()
    if(existing_root AND NOT existing_root STREQUAL CMAKE_CURRENT_SOURCE_DIR)
        message(FATAL_ERROR "Module ${module_name} is already registered from ${existing_root}")
    endif()
    if(NOT existing_group)
        set(changelog "${CMAKE_CURRENT_SOURCE_DIR}/CHANGELOG.md")
        if(NOT EXISTS "${changelog}")
            message(FATAL_ERROR "Module ${module_name} must provide CHANGELOG.md at its module root")
        endif()
        set(module_docs_dir "")
        set(candidate_module_docs_dir "${CMAKE_CURRENT_SOURCE_DIR}/docs")
        get_property(group_docs_dir GLOBAL PROPERTY "ISAACSIM_GROUP_${group_name}_DOCS_DIR")
        if(EXISTS "${candidate_module_docs_dir}" AND NOT candidate_module_docs_dir STREQUAL group_docs_dir)
            if(NOT EXISTS "${candidate_module_docs_dir}/index.rst")
                message(FATAL_ERROR "${module_name} docs directory must provide docs/index.rst")
            endif()
            set(module_docs_dir "${candidate_module_docs_dir}")
        endif()
        set_property(GLOBAL PROPERTY "ISAACSIM_MODULE_${module_key}_GROUP" "${group_name}")
        set_property(GLOBAL PROPERTY "ISAACSIM_MODULE_${module_key}_ROOT" "${CMAKE_CURRENT_SOURCE_DIR}")
        set_property(GLOBAL PROPERTY "ISAACSIM_MODULE_${module_key}_CHANGELOG" "${changelog}")
        set_property(GLOBAL PROPERTY "ISAACSIM_MODULE_${module_key}_DOCS_DIR" "${module_docs_dir}")
        set_property(GLOBAL APPEND PROPERTY "ISAACSIM_GROUP_${group_name}_MODULES" "${module_name}")
    endif()
endfunction()

function(_isaacsim_record_module_public_api module_name)
    set(multi_value_args PUBLIC_C_HEADERS PUBLIC_CPP_HEADERS)
    cmake_parse_arguments(ARG "" "" "${multi_value_args}" ${ARGN})
    string(MAKE_C_IDENTIFIER "${module_name}" module_key)
    set_property(GLOBAL PROPERTY "ISAACSIM_MODULE_${module_key}_PUBLIC_C_HEADERS" "${ARG_PUBLIC_C_HEADERS}")
    set_property(GLOBAL PROPERTY "ISAACSIM_MODULE_${module_key}_PUBLIC_CPP_HEADERS" "${ARG_PUBLIC_CPP_HEADERS}")
endfunction()

function(_isaacsim_apply_build_options target)
    if(MSVC)
        target_compile_options(${target} PRIVATE
            /W4
            $<$<COMPILE_LANGUAGE:CXX>:/EHsc>
        )
        target_compile_definitions(${target} PRIVATE NOMINMAX)
        if(ISAACSIM_WARNINGS_AS_ERRORS)
            target_compile_options(${target} PRIVATE /WX)
        endif()
    else()
        target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
        if(ISAACSIM_WARNINGS_AS_ERRORS)
            target_compile_options(${target} PRIVATE -Werror)
        endif()
    endif()
endfunction()

function(_isaacsim_get_module_names module_name output_target output_alias output_path output_dir)
    if(NOT module_name MATCHES "^[a-z][a-z0-9_]*(\.[a-z][a-z0-9_]*)+$")
        message(FATAL_ERROR "Invalid lowercase dotted module name: ${module_name}")
    endif()

    string(REPLACE "." "-" module_target "${module_name}")
    string(REPLACE "." "/" module_path "${module_name}")
    string(REGEX REPLACE "^isaacsim\." "" module_alias_suffix "${module_name}")
    string(REPLACE "." "-" module_alias_suffix "${module_alias_suffix}")

    set(configuration_suffix "")
    if(CMAKE_CONFIGURATION_TYPES)
        set(configuration_suffix "/$<CONFIG>")
    endif()

    set(${output_target} "${module_target}" PARENT_SCOPE)
    set(${output_alias} "isaacsim::${module_alias_suffix}" PARENT_SCOPE)
    set(${output_path} "${module_path}" PARENT_SCOPE)
    set(${output_dir} "${ISAACSIM_MODULE_OUTPUT_ROOT}/${module_name}${configuration_suffix}" PARENT_SCOPE)
endfunction()

function(_isaacsim_prepare_module
    module_name output_target output_alias output_path output_dir output_group output_version
)
    _isaacsim_get_module_names("${module_name}" module_target module_alias module_path module_output_dir)
    _isaacsim_get_current_group(group_name group_version)
    _isaacsim_validate_module_source_path("${module_name}" "${group_name}")
    _isaacsim_register_group_module("${module_name}" "${group_name}")
    _isaacsim_enforce_module_boundary("${CMAKE_CURRENT_SOURCE_DIR}")

    set(${output_target} "${module_target}" PARENT_SCOPE)
    set(${output_alias} "${module_alias}" PARENT_SCOPE)
    set(${output_path} "${module_path}" PARENT_SCOPE)
    set(${output_dir} "${module_output_dir}" PARENT_SCOPE)
    set(${output_group} "${group_name}" PARENT_SCOPE)
    set(${output_version} "${group_version}" PARENT_SCOPE)
endfunction()

function(_isaacsim_get_python_stage_dir output)
    set(stage_dir "${ISAACSIM_PYTHON_STAGE_DIR}")
    if(CMAKE_CONFIGURATION_TYPES)
        string(APPEND stage_dir "/$<CONFIG>")
    endif()
    set(${output} "${stage_dir}" PARENT_SCOPE)
endfunction()

function(_isaacsim_enforce_module_boundary source_dir)
    file(GLOB_RECURSE module_sources CONFIGURE_DEPENDS
        "${source_dir}/*.c"
        "${source_dir}/*.cc"
        "${source_dir}/*.cpp"
        "${source_dir}/*.cxx"
        "${source_dir}/*.h"
        "${source_dir}/*.hh"
        "${source_dir}/*.hpp"
        "${source_dir}/*.hxx"
        "${source_dir}/*.py"
        "${source_dir}/*.pyi"
        "${source_dir}/*.cmake"
        "${source_dir}/CMakeLists.txt"
    )
    foreach(source_file IN LISTS module_sources)
        file(READ "${source_file}" source_contents)
        if(source_contents MATCHES "#[ \t]*include[ \t]*[<\"]omni/(kit|ext|graph)/")
            message(FATAL_ERROR "Module source includes a forbidden framework header: ${source_file}")
        endif()
        if(source_contents MATCHES "#[ \t]*include[ \t]*[<\"]pybind11/")
            message(FATAL_ERROR "Module source uses pybind11 instead of nanobind: ${source_file}")
        endif()
        if(source_contents MATCHES "(^|\n)[ \t]*(from|import)[ \t]+(carb|omni\.(kit|ext|graph))")
            message(FATAL_ERROR "Module Python imports a forbidden framework package: ${source_file}")
        endif()
        if(source_contents MATCHES "(find_package|target_link_libraries)[^\n]*omni[_.-](kit|ext|graph)")
            message(FATAL_ERROR "Module CMake declares a forbidden framework dependency: ${source_file}")
        endif()
    endforeach()
endfunction()

function(_isaacsim_resolve_target target output)
    if(TARGET "${target}")
        get_target_property(aliased_target "${target}" ALIASED_TARGET)
        if(aliased_target)
            set(target "${aliased_target}")
        endif()
    endif()
    set(${output} "${target}" PARENT_SCOPE)
endfunction()

function(_isaacsim_collect_runtime_targets target output)
    _isaacsim_resolve_target("${target}" resolved_target)
    set(pending "${resolved_target}")
    set(result)
    while(pending)
        list(POP_FRONT pending current)
        if(NOT TARGET "${current}" OR current IN_LIST result)
            continue()
        endif()
        get_target_property(is_module "${current}" ISAACSIM_MODULE_NAME)
        if(NOT is_module)
            continue()
        endif()
        get_target_property(dependencies "${current}" ISAACSIM_RUNTIME_DEPENDENCIES)
        foreach(dependency IN LISTS dependencies)
            _isaacsim_resolve_target("${dependency}" dependency_target)
            if(TARGET "${dependency_target}")
                list(APPEND pending "${dependency_target}")
            endif()
        endforeach()
        get_target_property(current_type "${current}" TYPE)
        if(NOT current_type STREQUAL "INTERFACE_LIBRARY")
            list(APPEND result "${current}")
        endif()
    endwhile()
    set(${output} "${result}" PARENT_SCOPE)
endfunction()

function(_isaacsim_collect_imported_runtime_targets output)
    set(pending ${ARGN})
    set(visited)
    set(result)
    while(pending)
        list(POP_FRONT pending current)
        _isaacsim_resolve_target("${current}" current_target)
        if(NOT TARGET "${current_target}" OR current_target IN_LIST visited)
            continue()
        endif()
        list(APPEND visited "${current_target}")

        get_target_property(link_dependencies "${current_target}" INTERFACE_LINK_LIBRARIES)
        foreach(link_dependency IN LISTS link_dependencies)
            if(link_dependency MATCHES "^\\$<LINK_ONLY:([^>]+)>$")
                set(link_dependency "${CMAKE_MATCH_1}")
            elseif(link_dependency MATCHES "^\\$<TARGET_NAME_IF_EXISTS:([^>]+)>$")
                set(link_dependency "${CMAKE_MATCH_1}")
            endif()
            _isaacsim_resolve_target("${link_dependency}" dependency_target)
            if(TARGET "${dependency_target}")
                list(APPEND pending "${dependency_target}")
            endif()
        endforeach()

        get_target_property(is_imported "${current_target}" IMPORTED)
        get_target_property(target_type "${current_target}" TYPE)
        if(is_imported AND target_type MATCHES "^(MODULE|SHARED)_LIBRARY$")
            list(APPEND result "${current_target}")
        endif()
    endwhile()
    set(${output} "${result}" PARENT_SCOPE)
endfunction()

function(isaacsim_add_module)
    set(one_value_args NAME EXPORT_HEADER EXPORT_MACRO PUBLIC_C_SYMBOLS_FILE)
    set(multi_value_args
        SOURCES PUBLIC_DEPENDENCIES PRIVATE_DEPENDENCIES PACKAGE_DEPENDENCIES PUBLIC_C_HEADERS PUBLIC_CPP_HEADERS
    )
    cmake_parse_arguments(ARG "" "${one_value_args}" "${multi_value_args}" ${ARGN})

    if(ARG_UNPARSED_ARGUMENTS)
        message(FATAL_ERROR "isaacsim_add_module received unknown arguments: ${ARG_UNPARSED_ARGUMENTS}")
    endif()

    foreach(required_arg NAME EXPORT_HEADER EXPORT_MACRO SOURCES)
        if(NOT ARG_${required_arg})
            message(FATAL_ERROR "isaacsim_add_module requires ${required_arg}")
        endif()
    endforeach()
    if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/include")
        message(FATAL_ERROR "${ARG_NAME} must provide an include directory")
    endif()
    _isaacsim_validate_relative_path("${ARG_NAME} export header" "${ARG_EXPORT_HEADER}")
    if(NOT ARG_EXPORT_HEADER MATCHES "\\.h$")
        message(FATAL_ERROR "${ARG_NAME} generated export header must use the .h extension: ${ARG_EXPORT_HEADER}")
    endif()
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/include/${ARG_EXPORT_HEADER}")
        message(FATAL_ERROR
            "${ARG_NAME} generated export header already exists in the source tree: ${ARG_EXPORT_HEADER}"
        )
    endif()

    _isaacsim_prepare_module("${ARG_NAME}"
        module_target module_alias module_path module_output_dir group_name group_version
    )
    get_property(group_release_version GLOBAL PROPERTY "ISAACSIM_GROUP_${group_name}_RELEASE_VERSION")
    string(REGEX REPLACE "^isaacsim\." "" module_alias_suffix "${ARG_NAME}")
    string(REPLACE "." "-" module_alias_suffix "${module_alias_suffix}")

    _isaacsim_validate_public_headers("${ARG_NAME}"
        PUBLIC_C_HEADERS ${ARG_PUBLIC_C_HEADERS}
        PUBLIC_CPP_HEADERS ${ARG_PUBLIC_CPP_HEADERS}
    )
    _isaacsim_record_module_public_api("${ARG_NAME}"
        PUBLIC_C_HEADERS ${ARG_PUBLIC_C_HEADERS}
        PUBLIC_CPP_HEADERS ${ARG_PUBLIC_CPP_HEADERS}
    )
    if(ARG_PUBLIC_C_SYMBOLS_FILE AND NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${ARG_PUBLIC_C_SYMBOLS_FILE}")
        message(FATAL_ERROR "${ARG_NAME} C ABI symbols file does not exist: ${ARG_PUBLIC_C_SYMBOLS_FILE}")
    endif()
    _isaacsim_validate_dependencies("${ARG_NAME}" ${ARG_PUBLIC_DEPENDENCIES} ${ARG_PACKAGE_DEPENDENCIES})
    _isaacsim_validate_private_dependencies("${ARG_NAME}" ${ARG_PRIVATE_DEPENDENCIES})
    _isaacsim_validate_external_package_dependencies("${ARG_NAME}" ${ARG_PACKAGE_DEPENDENCIES})
    _isaacsim_validate_group_target_dependencies("${ARG_NAME}" "${group_name}"
        ${ARG_PUBLIC_DEPENDENCIES} ${ARG_PRIVATE_DEPENDENCIES}
    )

    add_library(${module_target} SHARED ${ARG_SOURCES})
    add_library(${module_alias} ALIAS ${module_target})
    _isaacsim_apply_build_options(${module_target})
    target_compile_features(${module_target} PUBLIC cxx_std_17)
    target_include_directories(${module_target} PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/generated>"
        "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
    )
    target_link_libraries(${module_target}
        PUBLIC ${ARG_PUBLIC_DEPENDENCIES}
        PRIVATE ${ARG_PRIVATE_DEPENDENCIES}
    )
    generate_export_header(${module_target}
        BASE_NAME "${module_target}"
        EXPORT_FILE_NAME "${CMAKE_CURRENT_BINARY_DIR}/generated/${ARG_EXPORT_HEADER}"
        EXPORT_MACRO_NAME "${ARG_EXPORT_MACRO}"
    )

    set_target_properties(${module_target} PROPERTIES
        EXPORT_NAME "${module_alias_suffix}"
        VERSION "${group_release_version}"
        CXX_VISIBILITY_PRESET hidden
        VISIBILITY_INLINES_HIDDEN YES
        LIBRARY_OUTPUT_DIRECTORY "${module_output_dir}/lib"
        RUNTIME_OUTPUT_DIRECTORY "${module_output_dir}/lib"
        ARCHIVE_OUTPUT_DIRECTORY "${module_output_dir}/lib"
        BUILD_RPATH "${ISAACSIM_LOADER_ORIGIN}"
        INSTALL_RPATH "${ISAACSIM_LOADER_ORIGIN}"
        ISAACSIM_MODULE_NAME "${ARG_NAME}"
        ISAACSIM_GROUP_NAME "${group_name}"
        ISAACSIM_PACKAGE_VERSION "${group_version}"
        ISAACSIM_MODULE_ALIAS "${module_alias}"
        ISAACSIM_RUNTIME_DEPENDENCIES "${ARG_PUBLIC_DEPENDENCIES};${ARG_PRIVATE_DEPENDENCIES}"
        ISAACSIM_PUBLIC_C_HEADERS "${ARG_PUBLIC_C_HEADERS}"
        ISAACSIM_PUBLIC_CPP_HEADERS "${ARG_PUBLIC_CPP_HEADERS}"
        ISAACSIM_PUBLIC_C_SYMBOLS_FILE "${ARG_PUBLIC_C_SYMBOLS_FILE}"
    )

    if(ARG_PUBLIC_C_SYMBOLS_FILE)
        _isaacsim_module_name_to_c_symbol_prefix("${ARG_NAME}" c_symbol_prefix)
        set_target_properties(${module_target} PROPERTIES ISAACSIM_PUBLIC_C_SYMBOL_PREFIX "${c_symbol_prefix}")

        if(NOT TARGET check-abi)
            add_custom_target(check-abi)
        endif()
        if(NOT TARGET update-abi-baselines)
            add_custom_target(update-abi-baselines)
        endif()
        set(check_abi_target "check-abi-${module_target}")
        set(update_abi_target "update-abi-baseline-${module_target}")
        add_custom_target(${check_abi_target}
            COMMAND ${CMAKE_COMMAND}
                "-DBINARY=$<TARGET_FILE:${module_target}>"
                "-DSYMBOLS_FILE=${CMAKE_CURRENT_SOURCE_DIR}/${ARG_PUBLIC_C_SYMBOLS_FILE}"
                "-DSYMBOL_PREFIX=${c_symbol_prefix}"
                -P "${_ISAACSIM_CMAKE_HELPER_DIR}/CheckExportedSymbols.cmake"
            DEPENDS ${module_target}
            VERBATIM
        )
        add_custom_target(${update_abi_target}
            COMMAND ${CMAKE_COMMAND}
                "-DBINARY=$<TARGET_FILE:${module_target}>"
                "-DSYMBOLS_FILE=${CMAKE_CURRENT_SOURCE_DIR}/${ARG_PUBLIC_C_SYMBOLS_FILE}"
                "-DSYMBOL_PREFIX=${c_symbol_prefix}"
                -DUPDATE_BASELINE=ON
                -P "${_ISAACSIM_CMAKE_HELPER_DIR}/CheckExportedSymbols.cmake"
            DEPENDS ${module_target}
            VERBATIM
        )
        add_dependencies(check-abi ${check_abi_target})
        add_dependencies(update-abi-baselines ${update_abi_target})
    endif()

    _isaacsim_get_group_component("${group_name}" runtime runtime_component)
    _isaacsim_get_group_component("${group_name}" development development_component)
    install(TARGETS ${module_target}
        EXPORT "${group_name}Targets"
        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT "${runtime_component}"
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT "${runtime_component}"
        ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT "${development_component}"
        INCLUDES DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
    )
    _isaacsim_install_public_headers("${group_name}" ${ARG_PUBLIC_C_HEADERS} ${ARG_PUBLIC_CPP_HEADERS})
    get_filename_component(export_header_directory "${ARG_EXPORT_HEADER}" DIRECTORY)
    install(FILES "${CMAKE_CURRENT_BINARY_DIR}/generated/${ARG_EXPORT_HEADER}"
        DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/${export_header_directory}"
        COMPONENT "${development_component}"
    )

    set_property(GLOBAL APPEND PROPERTY ISAACSIM_MODULE_TARGETS "${module_target}")
    set_property(GLOBAL APPEND PROPERTY "ISAACSIM_GROUP_${group_name}_TARGETS" "${module_target}")
    set_property(GLOBAL APPEND PROPERTY
        "ISAACSIM_GROUP_${group_name}_CMAKE_DEPENDENCIES" ${ARG_PACKAGE_DEPENDENCIES}
    )
endfunction()

function(isaacsim_add_header_module)
    set(one_value_args NAME)
    set(multi_value_args PUBLIC_DEPENDENCIES PACKAGE_DEPENDENCIES PUBLIC_C_HEADERS PUBLIC_CPP_HEADERS)
    cmake_parse_arguments(ARG "" "${one_value_args}" "${multi_value_args}" ${ARGN})

    if(ARG_UNPARSED_ARGUMENTS)
        message(FATAL_ERROR "isaacsim_add_header_module received unknown arguments: ${ARG_UNPARSED_ARGUMENTS}")
    endif()

    foreach(required_arg NAME)
        if(NOT ARG_${required_arg})
            message(FATAL_ERROR "isaacsim_add_header_module requires ${required_arg}")
        endif()
    endforeach()
    if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/include")
        message(FATAL_ERROR "${ARG_NAME} must provide an include directory")
    endif()
    _isaacsim_prepare_module("${ARG_NAME}"
        module_target module_alias module_path module_output_dir group_name group_version
    )
    string(REGEX REPLACE "^isaacsim\." "" module_alias_suffix "${ARG_NAME}")
    string(REPLACE "." "-" module_alias_suffix "${module_alias_suffix}")

    _isaacsim_validate_public_headers("${ARG_NAME}"
        PUBLIC_C_HEADERS ${ARG_PUBLIC_C_HEADERS}
        PUBLIC_CPP_HEADERS ${ARG_PUBLIC_CPP_HEADERS}
    )
    _isaacsim_record_module_public_api("${ARG_NAME}"
        PUBLIC_C_HEADERS ${ARG_PUBLIC_C_HEADERS}
        PUBLIC_CPP_HEADERS ${ARG_PUBLIC_CPP_HEADERS}
    )
    _isaacsim_validate_dependencies("${ARG_NAME}" ${ARG_PUBLIC_DEPENDENCIES} ${ARG_PACKAGE_DEPENDENCIES})
    _isaacsim_validate_external_package_dependencies("${ARG_NAME}" ${ARG_PACKAGE_DEPENDENCIES})
    _isaacsim_validate_group_target_dependencies("${ARG_NAME}" "${group_name}" ${ARG_PUBLIC_DEPENDENCIES})

    add_library(${module_target} INTERFACE)
    add_library(${module_alias} ALIAS ${module_target})
    if(ARG_PUBLIC_CPP_HEADERS)
        target_compile_features(${module_target} INTERFACE cxx_std_17)
    endif()
    target_include_directories(${module_target} INTERFACE
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
    )
    target_link_libraries(${module_target} INTERFACE ${ARG_PUBLIC_DEPENDENCIES})
    set_target_properties(${module_target} PROPERTIES
        EXPORT_NAME "${module_alias_suffix}"
        ISAACSIM_MODULE_NAME "${ARG_NAME}"
        ISAACSIM_GROUP_NAME "${group_name}"
        ISAACSIM_PACKAGE_VERSION "${group_version}"
        ISAACSIM_MODULE_ALIAS "${module_alias}"
        ISAACSIM_RUNTIME_DEPENDENCIES ""
        ISAACSIM_PUBLIC_C_HEADERS "${ARG_PUBLIC_C_HEADERS}"
        ISAACSIM_PUBLIC_CPP_HEADERS "${ARG_PUBLIC_CPP_HEADERS}"
        ISAACSIM_PUBLIC_C_SYMBOLS_FILE ""
    )

    install(TARGETS ${module_target} EXPORT "${group_name}Targets")
    _isaacsim_install_public_headers("${group_name}" ${ARG_PUBLIC_C_HEADERS} ${ARG_PUBLIC_CPP_HEADERS})

    set_property(GLOBAL APPEND PROPERTY ISAACSIM_MODULE_TARGETS "${module_target}")
    set_property(GLOBAL APPEND PROPERTY "ISAACSIM_GROUP_${group_name}_TARGETS" "${module_target}")
    set_property(GLOBAL APPEND PROPERTY
        "ISAACSIM_GROUP_${group_name}_CMAKE_DEPENDENCIES" ${ARG_PACKAGE_DEPENDENCIES}
    )
endfunction()

function(isaacsim_add_runtime_directory)
    set(options NATIVE_ONLY WINDOWS_PYTHON_SHARED)
    set(one_value_args MODULE SOURCE DESTINATION WINDOWS_INSTALL_DESTINATION)
    cmake_parse_arguments(ARG "${options}" "${one_value_args}" "" ${ARGN})
    if(ARG_UNPARSED_ARGUMENTS)
        message(FATAL_ERROR "isaacsim_add_runtime_directory received unknown arguments: ${ARG_UNPARSED_ARGUMENTS}")
    endif()
    foreach(required_arg MODULE SOURCE DESTINATION)
        if(NOT ARG_${required_arg})
            message(FATAL_ERROR "isaacsim_add_runtime_directory requires ${required_arg}")
        endif()
    endforeach()
    if(NOT IS_ABSOLUTE "${ARG_SOURCE}")
        set(ARG_SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/${ARG_SOURCE}")
    endif()
    if(NOT IS_DIRECTORY "${ARG_SOURCE}")
        message(FATAL_ERROR "${ARG_MODULE} runtime directory does not exist: ${ARG_SOURCE}")
    endif()
    # Packman dependencies are commonly exposed as directory junctions on
    # Windows. CMake's install(DIRECTORY) otherwise attempts to recreate a
    # junction passed as the directory root, which requires symlink privileges.
    # Install from the resolved directory so its contents are copied instead.
    file(REAL_PATH "${ARG_SOURCE}" ARG_SOURCE)
    _isaacsim_validate_relative_path("${ARG_MODULE} runtime destination" "${ARG_DESTINATION}")
    if(ARG_WINDOWS_INSTALL_DESTINATION)
        _isaacsim_validate_relative_path(
            "${ARG_MODULE} Windows runtime install destination"
            "${ARG_WINDOWS_INSTALL_DESTINATION}"
        )
    endif()
    _isaacsim_get_module_names("${ARG_MODULE}" module_target module_alias module_path module_output_dir)
    if(NOT TARGET "${module_target}")
        message(FATAL_ERROR "isaacsim_add_runtime_directory requires a registered module: ${ARG_MODULE}")
    endif()
    get_target_property(group_name ${module_target} ISAACSIM_GROUP_NAME)
    _isaacsim_get_group_component("${group_name}" runtime runtime_component)

    file(GLOB_RECURSE runtime_files CONFIGURE_DEPENDS LIST_DIRECTORIES FALSE "${ARG_SOURCE}/*")
    list(FILTER runtime_files EXCLUDE REGEX "${ISAACSIM_PACKAGING_EXCLUDE_REGEX}")
    get_target_property(runtime_directory_count ${module_target} ISAACSIM_RUNTIME_DIRECTORY_COUNT)
    if(NOT runtime_directory_count OR runtime_directory_count MATCHES "-NOTFOUND$")
        set(runtime_directory_count 0)
    endif()
    math(EXPR runtime_directory_count "${runtime_directory_count} + 1")
    set_property(TARGET ${module_target} PROPERTY ISAACSIM_RUNTIME_DIRECTORY_COUNT "${runtime_directory_count}")
    set(runtime_stage_stamp "${module_output_dir}/runtime-directory-${runtime_directory_count}.stamp")
    set(runtime_stage_target "stage-${module_target}-runtime-directory-${runtime_directory_count}")
    add_custom_command(
        OUTPUT "${runtime_stage_stamp}"
        COMMAND ${CMAKE_COMMAND}
            "-DSOURCE=${ARG_SOURCE}"
            "-DDESTINATION=${module_output_dir}/lib/${ARG_DESTINATION}"
            -P "${_ISAACSIM_CMAKE_HELPER_DIR}/CopyRuntimeDirectory.cmake"
        COMMAND ${CMAKE_COMMAND} -E touch "${runtime_stage_stamp}"
        # Depend on the directory itself as well as its current files so adding
        # or removing runtime payload files invalidates an existing stage stamp.
        DEPENDS
            "${ARG_SOURCE}"
            ${runtime_files}
            "${_ISAACSIM_CMAKE_HELPER_DIR}/CopyRuntimeDirectory.cmake"
            "${_ISAACSIM_CMAKE_HELPER_DIR}/IsaacSimPackagingFilters.cmake"
        VERBATIM
    )
    add_custom_target(${runtime_stage_target} ALL DEPENDS "${runtime_stage_stamp}")
    add_dependencies(${module_target} ${runtime_stage_target})
    set(runtime_install_destination "${CMAKE_INSTALL_LIBDIR}/${ARG_DESTINATION}")
    if(WIN32 AND ARG_WINDOWS_INSTALL_DESTINATION)
        set(runtime_install_destination "${ARG_WINDOWS_INSTALL_DESTINATION}")
    endif()
    install(DIRECTORY "${ARG_SOURCE}/"
        DESTINATION "${runtime_install_destination}"
        COMPONENT "${runtime_component}"
        ${ISAACSIM_PACKAGING_EXCLUDE_ARGS}
    )
    if(ARG_WINDOWS_PYTHON_SHARED)
        if(ARG_NATIVE_ONLY)
            message(FATAL_ERROR
                "${ARG_MODULE} runtime directory cannot be both NATIVE_ONLY and WINDOWS_PYTHON_SHARED"
            )
        endif()
        set_property(TARGET ${module_target} APPEND PROPERTY
            ISAACSIM_WINDOWS_PYTHON_SHARED_RUNTIME_DESTINATIONS "${ARG_DESTINATION}"
        )
    endif()
    if(NOT ARG_NATIVE_ONLY)
        set_property(TARGET ${module_target} APPEND PROPERTY ISAACSIM_RUNTIME_DIRECTORIES
            "${ARG_SOURCE}" "${ARG_DESTINATION}"
        )
        set_property(TARGET ${module_target} APPEND PROPERTY ISAACSIM_RUNTIME_DIRECTORY_FILES ${runtime_files})
    endif()
endfunction()

function(isaacsim_install_runtime_dependencies)
    set(options NATIVE_ONLY)
    set(one_value_args MODULE DESTINATION)
    set(multi_value_args TARGETS)
    cmake_parse_arguments(ARG "${options}" "${one_value_args}" "${multi_value_args}" ${ARGN})
    if(ARG_UNPARSED_ARGUMENTS)
        message(FATAL_ERROR
            "isaacsim_install_runtime_dependencies received unknown arguments: ${ARG_UNPARSED_ARGUMENTS}"
        )
    endif()
    if(NOT ARG_MODULE OR NOT ARG_TARGETS)
        message(FATAL_ERROR "isaacsim_install_runtime_dependencies requires MODULE and TARGETS")
    endif()

    _isaacsim_get_module_names("${ARG_MODULE}" module_target module_alias module_path module_output_dir)
    if(NOT TARGET "${module_target}")
        message(FATAL_ERROR "isaacsim_install_runtime_dependencies requires a registered module: ${ARG_MODULE}")
    endif()
    get_target_property(group_name ${module_target} ISAACSIM_GROUP_NAME)
    _isaacsim_get_group_component("${group_name}" runtime runtime_component)
    if(WIN32)
        set(runtime_install_dir "${CMAKE_INSTALL_BINDIR}")
    else()
        set(runtime_install_dir "${CMAKE_INSTALL_LIBDIR}")
    endif()
    set(runtime_destination ".")
    if(ARG_DESTINATION)
        _isaacsim_validate_relative_path(
            "${ARG_MODULE} runtime dependency destination" "${ARG_DESTINATION}"
        )
        string(APPEND runtime_install_dir "/${ARG_DESTINATION}")
        set(runtime_destination "${ARG_DESTINATION}")
    endif()

    foreach(runtime_target IN LISTS ARG_TARGETS)
        _isaacsim_validate_dependencies("${ARG_MODULE} runtime" "${runtime_target}")
        _isaacsim_resolve_target("${runtime_target}" resolved_runtime_target)
        if(NOT TARGET "${resolved_runtime_target}")
            message(FATAL_ERROR "${ARG_MODULE} runtime dependency is not a target: ${runtime_target}")
        endif()
        get_target_property(runtime_target_imported "${resolved_runtime_target}" IMPORTED)
        if(runtime_target_imported)
            get_target_property(runtime_target_global "${resolved_runtime_target}" IMPORTED_GLOBAL)
            get_target_property(runtime_target_source_dir "${resolved_runtime_target}" SOURCE_DIR)
            if(NOT runtime_target_global
               AND "${runtime_target_source_dir}" STREQUAL "${CMAKE_CURRENT_SOURCE_DIR}")
                set_property(TARGET "${resolved_runtime_target}" PROPERTY IMPORTED_GLOBAL TRUE)
            endif()
        endif()
        install(FILES "$<TARGET_FILE:${resolved_runtime_target}>"
            DESTINATION "${runtime_install_dir}"
            COMPONENT "${runtime_component}"
        )
        if(NOT WIN32)
            # file(INSTALL) preserves a symlink without copying its target. Add
            # the resolved payload as well so imported linker-name symlinks do
            # not become dangling links after relocation.
            set(runtime_install_code
                "file(REAL_PATH \"$<TARGET_FILE:${resolved_runtime_target}>\" runtime_real_path)\n"
                "if(NOT runtime_real_path STREQUAL \"$<TARGET_FILE:${resolved_runtime_target}>\")\n"
                "  file(INSTALL DESTINATION \"\${CMAKE_INSTALL_PREFIX}/${runtime_install_dir}\" "
                "TYPE FILE FILES \"\${runtime_real_path}\")\n"
                "endif()"
            )
            string(JOIN "" runtime_install_code ${runtime_install_code})
            install(CODE "${runtime_install_code}" COMPONENT "${runtime_component}")
            # Imported shared-library targets commonly name a fully versioned
            # file while dependents load its SONAME. Install both spellings so
            # relocated packages do not rely on the dependency build prefix.
            install(FILES "$<TARGET_FILE:${resolved_runtime_target}>"
                DESTINATION "${runtime_install_dir}"
                RENAME "$<TARGET_SONAME_FILE_NAME:${resolved_runtime_target}>"
                COMPONENT "${runtime_component}"
            )
        endif()
        if(NOT ARG_NATIVE_ONLY)
            set_property(TARGET ${module_target} APPEND PROPERTY ISAACSIM_EXTERNAL_RUNTIME_TARGETS
                "${resolved_runtime_target}"
            )
            set_property(TARGET ${module_target} APPEND PROPERTY ISAACSIM_EXTERNAL_RUNTIME_DESTINATIONS
                "${runtime_destination}"
            )
        endif()
        if(BUILD_TESTING)
            string(MAKE_C_IDENTIFIER "${resolved_runtime_target}" runtime_test_suffix)
            set(runtime_test_name "tests-binary-boundary-runtime-${module_target}-${runtime_test_suffix}")
            add_test(NAME "${runtime_test_name}"
                COMMAND ${CMAKE_COMMAND}
                    "-DBINARY=$<TARGET_FILE:${resolved_runtime_target}>"
                    -P "${_ISAACSIM_CMAKE_HELPER_DIR}/CheckBinaryDependencies.cmake"
            )
            set_tests_properties("${runtime_test_name}" PROPERTIES LABELS "boundary;binary;runtime;${ARG_MODULE}")
        endif()
    endforeach()
endfunction()
