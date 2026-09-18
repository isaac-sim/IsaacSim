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

include("${CMAKE_CURRENT_LIST_DIR}/IsaacSimPackagingFilters.cmake")

function(_isaacsim_validate_python_import_name python_import_name)
    if(NOT python_import_name MATCHES "^isaacsim(\\.[a-z][a-z0-9_]*)+$")
        message(FATAL_ERROR
            "Invalid Python import name '${python_import_name}'. Expected isaacsim.<lowercase_segment>[...]."
        )
    endif()
endfunction()

function(_isaacsim_register_python_import module_name python_import_name group_name requires_bindings)
    _isaacsim_validate_python_import_name("${python_import_name}")
    get_property(python_imports GLOBAL PROPERTY ISAACSIM_PYTHON_IMPORTS)
    if(python_import_name IN_LIST python_imports)
        get_property(python_import_owners GLOBAL PROPERTY ISAACSIM_PYTHON_IMPORT_OWNERS)
        list(FIND python_imports "${python_import_name}" python_import_index)
        list(GET python_import_owners ${python_import_index} python_import_owner)
        message(FATAL_ERROR
            "Python import ${python_import_name} is already provided by ${python_import_owner}; "
            "${module_name} cannot provide it too"
        )
    endif()
    set_property(GLOBAL APPEND PROPERTY ISAACSIM_PYTHON_IMPORTS "${python_import_name}")
    set_property(GLOBAL APPEND PROPERTY ISAACSIM_PYTHON_IMPORT_OWNERS "${module_name}")
    set_property(GLOBAL PROPERTY
        "ISAACSIM_PYTHON_IMPORT_${python_import_name}_REQUIRES_BINDINGS" "${requires_bindings}"
    )
    set_property(GLOBAL APPEND PROPERTY "ISAACSIM_GROUP_${group_name}_PYTHON_IMPORTS" "${python_import_name}")
    string(MAKE_C_IDENTIFIER "${module_name}" module_key)
    set_property(GLOBAL APPEND PROPERTY "ISAACSIM_MODULE_${module_key}_PYTHON_IMPORTS" "${python_import_name}")
endfunction()

function(_isaacsim_add_python_module module_name python_import_name preserve_package_layout requires_bindings)
    _isaacsim_prepare_module("${module_name}"
        module_target module_alias module_path module_output_dir group_name group_version
    )
    _isaacsim_validate_python_import_name("${python_import_name}")
    string(REPLACE "." "/" python_import_path "${python_import_name}")
    set(module_python_source_dir "${CMAKE_CURRENT_SOURCE_DIR}/python")
    if(EXISTS "${module_python_source_dir}/tests")
        message(FATAL_ERROR "${module_name} Python tests must be placed under tests/python, not python/tests")
    endif()
    set(required_python_files __init__.py py.typed)
    if(NOT preserve_package_layout)
        list(APPEND required_python_files impl/__init__.py)
    endif()
    foreach(required_file IN LISTS required_python_files)
        if(NOT EXISTS "${module_python_source_dir}/${required_file}")
            message(FATAL_ERROR "${module_name} must provide python/${required_file}")
        endif()
    endforeach()
    if(NOT preserve_package_layout)
        file(STRINGS "${module_python_source_dir}/__init__.py" public_init_statements
            REGEX "^[ \t]*[^# \t].*$"
        )
        set(required_public_init_statements
            "from .impl import *"
            "from .impl import __all__ as __all__"
        )
        if(NOT public_init_statements STREQUAL required_public_init_statements)
            message(FATAL_ERROR
                "${module_name} python/__init__.py may contain only license comments, blank lines, and these "
                "statements: "
                "${required_public_init_statements}"
            )
        endif()
        file(READ "${module_python_source_dir}/impl/__init__.py" implementation_init)
        if(NOT implementation_init MATCHES "(^|\n)__all__[ \t]*=")
            message(FATAL_ERROR "${module_name} python/impl/__init__.py must define __all__")
        endif()
    endif()
    if(NOT ISAACSIM_ENABLE_PYTHON)
        return()
    endif()

    get_property(python_modules GLOBAL PROPERTY ISAACSIM_PYTHON_MODULE_NAMES)
    if(module_name IN_LIST python_modules)
        message(FATAL_ERROR "Python module ${module_name} is already registered")
    endif()
    _isaacsim_register_python_import(
        "${module_name}" "${python_import_name}" "${group_name}" "${requires_bindings}"
    )

    set(stage_target "stage-${module_target}-python-package")
    _isaacsim_get_python_stage_dir(python_stage_dir)
    if(preserve_package_layout)
        file(GLOB_RECURSE python_sources CONFIGURE_DEPENDS LIST_DIRECTORIES FALSE
            "${module_python_source_dir}/*"
        )
        list(FILTER python_sources EXCLUDE REGEX "${ISAACSIM_PACKAGING_EXCLUDE_REGEX}")
    else()
        set(implementation_source_dir "${module_python_source_dir}/impl")
        file(GLOB non_implementation_sources CONFIGURE_DEPENDS LIST_DIRECTORIES FALSE
            "${module_python_source_dir}/*.py"
        )
        file(GLOB_RECURSE implementation_sources CONFIGURE_DEPENDS LIST_DIRECTORIES FALSE
            "${implementation_source_dir}/*"
        )
        list(FILTER implementation_sources EXCLUDE REGEX "${ISAACSIM_PACKAGING_EXCLUDE_REGEX}")
    endif()
    set(stage_stamp "${module_output_dir}/python-package-stage.stamp")
    if(preserve_package_layout)
        add_custom_command(
            OUTPUT "${stage_stamp}"
            COMMAND ${CMAKE_COMMAND} -E make_directory "${module_output_dir}"
            COMMAND ${CMAKE_COMMAND} -E remove_directory "${python_stage_dir}/${python_import_path}"
            COMMAND ${CMAKE_COMMAND}
                "-DSOURCE=${module_python_source_dir}"
                "-DDESTINATION=${python_stage_dir}/${python_import_path}"
                -P "${_ISAACSIM_CMAKE_HELPER_DIR}/StageEditableDirectory.cmake"
            COMMAND ${CMAKE_COMMAND} -E touch "${stage_stamp}"
            DEPENDS
                ${python_sources}
                "${_ISAACSIM_CMAKE_HELPER_DIR}/StageEditableDirectory.cmake"
                "${_ISAACSIM_CMAKE_HELPER_DIR}/IsaacSimPackagingFilters.cmake"
            VERBATIM
        )
    else()
        add_custom_command(
            OUTPUT "${stage_stamp}"
            COMMAND ${CMAKE_COMMAND} -E make_directory "${module_output_dir}"
            COMMAND ${CMAKE_COMMAND} -E remove_directory "${python_stage_dir}/${python_import_path}"
            COMMAND ${CMAKE_COMMAND} -E make_directory "${python_stage_dir}/${python_import_path}"
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                ${non_implementation_sources} "${python_stage_dir}/${python_import_path}"
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${module_python_source_dir}/py.typed" "${python_stage_dir}/${python_import_path}/py.typed"
            COMMAND ${CMAKE_COMMAND}
                "-DSOURCE=${implementation_source_dir}"
                "-DDESTINATION=${python_stage_dir}/${python_import_path}/impl"
                -P "${_ISAACSIM_CMAKE_HELPER_DIR}/StageEditableDirectory.cmake"
            COMMAND ${CMAKE_COMMAND} -E touch "${stage_stamp}"
            DEPENDS
                ${non_implementation_sources}
                "${module_python_source_dir}/py.typed"
                ${implementation_sources}
                "${_ISAACSIM_CMAKE_HELPER_DIR}/StageEditableDirectory.cmake"
                "${_ISAACSIM_CMAKE_HELPER_DIR}/IsaacSimPackagingFilters.cmake"
            VERBATIM
        )
    endif()
    add_custom_target(${stage_target} ALL DEPENDS "${stage_stamp}")
    set_target_properties(${stage_target} PROPERTIES
        ISAACSIM_PYTHON_STAGE_STAMP "${stage_stamp}"
        ISAACSIM_GROUP_NAME "${group_name}"
        ISAACSIM_PACKAGE_VERSION "${group_version}"
    )
    add_dependencies("${group_name}-python-build" ${stage_target})

    _isaacsim_get_group_component("${group_name}" python python_component)
    install(DIRECTORY "${module_python_source_dir}/"
        DESTINATION "${ISAACSIM_PYTHON_INSTALL_DIR}/${python_import_path}"
        COMPONENT "${python_component}"
        ${ISAACSIM_PACKAGING_EXCLUDE_ARGS}
    )

    set_property(GLOBAL APPEND PROPERTY ISAACSIM_PYTHON_MODULE_NAMES "${module_name}")
endfunction()

function(isaacsim_add_python_module)
    cmake_parse_arguments(ARG "PRESERVE_PACKAGE_LAYOUT;REQUIRES_BINDINGS" "NAME" "" ${ARGN})

    if(ARG_UNPARSED_ARGUMENTS)
        message(FATAL_ERROR "isaacsim_add_python_module received unknown arguments: ${ARG_UNPARSED_ARGUMENTS}")
    endif()
    if(NOT ARG_NAME)
        message(FATAL_ERROR "isaacsim_add_python_module requires NAME")
    endif()

    _isaacsim_add_python_module(
        "${ARG_NAME}" "${ARG_NAME}" "${ARG_PRESERVE_PACKAGE_LAYOUT}" "${ARG_REQUIRES_BINDINGS}"
    )
endfunction()

function(isaacsim_add_compat_python_module)
    cmake_parse_arguments(ARG "REQUIRES_BINDINGS" "NAME;IMPORT_NAME" "" ${ARGN})

    if(ARG_UNPARSED_ARGUMENTS)
        message(FATAL_ERROR
            "isaacsim_add_compat_python_module received unknown arguments: ${ARG_UNPARSED_ARGUMENTS}"
        )
    endif()
    foreach(required_arg NAME IMPORT_NAME)
        if(NOT ARG_${required_arg})
            message(FATAL_ERROR "isaacsim_add_compat_python_module requires ${required_arg}")
        endif()
    endforeach()
    if(ARG_NAME STREQUAL ARG_IMPORT_NAME)
        message(FATAL_ERROR
            "isaacsim_add_compat_python_module requires different NAME and IMPORT_NAME values; "
            "use isaacsim_add_python_module for ${ARG_NAME}"
        )
    endif()

    _isaacsim_add_python_module("${ARG_NAME}" "${ARG_IMPORT_NAME}" FALSE "${ARG_REQUIRES_BINDINGS}")
endfunction()
