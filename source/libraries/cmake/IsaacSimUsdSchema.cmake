# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

include_guard(GLOBAL)

include("${CMAKE_CURRENT_LIST_DIR}/IsaacSimPackagingFilters.cmake")

# Register a codeless USD schema package whose generated resources are owned by
# the CMake build tree. PYTHON_IMPORTS are both the source-relative package
# roots and the public import roots recorded in the package manifest.
function(isaacsim_add_usd_schema_module)
    set(one_value_args NAME)
    set(multi_value_args PYTHON_IMPORTS SCHEMAS PUBLIC_CPP_HEADERS)
    cmake_parse_arguments(ARG "" "${one_value_args}" "${multi_value_args}" ${ARGN})
    if(ARG_UNPARSED_ARGUMENTS)
        message(FATAL_ERROR
            "isaacsim_add_usd_schema_module received unknown arguments: ${ARG_UNPARSED_ARGUMENTS}"
        )
    endif()
    if(NOT ARG_NAME OR NOT ARG_PYTHON_IMPORTS OR NOT ARG_SCHEMAS)
        message(FATAL_ERROR
            "isaacsim_add_usd_schema_module requires NAME, PYTHON_IMPORTS, and SCHEMAS"
        )
    endif()

    _isaacsim_get_current_group(group_name group_version)
    _isaacsim_register_group_module("${ARG_NAME}" "${group_name}")
    _isaacsim_enforce_module_boundary("${CMAKE_CURRENT_SOURCE_DIR}")
    if(ARG_PUBLIC_CPP_HEADERS)
        _isaacsim_validate_public_headers("${ARG_NAME}" PUBLIC_CPP_HEADERS ${ARG_PUBLIC_CPP_HEADERS})
        _isaacsim_install_public_headers("${group_name}" ${ARG_PUBLIC_CPP_HEADERS})
    elseif(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/include")
        message(FATAL_ERROR "${ARG_NAME} must declare its include/ inventory with PUBLIC_CPP_HEADERS")
    endif()
    _isaacsim_record_module_public_api("${ARG_NAME}"
        PUBLIC_CPP_HEADERS ${ARG_PUBLIC_CPP_HEADERS}
    )
    string(MAKE_C_IDENTIFIER "${ARG_NAME}" module_key)
    set_property(GLOBAL APPEND PROPERTY
        "ISAACSIM_MODULE_${module_key}_PYTHON_IMPORTS" ${ARG_PYTHON_IMPORTS}
    )

    set(import_paths)
    foreach(import_name IN LISTS ARG_PYTHON_IMPORTS)
        if(NOT import_name MATCHES "^[a-z][a-zA-Z0-9_]*(\\.[a-zA-Z][a-zA-Z0-9_]*)+$")
            message(FATAL_ERROR "${ARG_NAME} has invalid Python import root: ${import_name}")
        endif()
        string(REPLACE "." "/" import_path "${import_name}")
        set(import_source_dir "${CMAKE_CURRENT_SOURCE_DIR}/${import_path}")
        if(NOT EXISTS "${import_source_dir}/__init__.py")
            message(FATAL_ERROR "${ARG_NAME} import ${import_name} must provide ${import_path}/__init__.py")
        endif()
        if(NOT EXISTS "${import_source_dir}/py.typed")
            message(FATAL_ERROR "${ARG_NAME} import ${import_name} must provide ${import_path}/py.typed")
        endif()
        list(APPEND import_paths "${import_path}")
    endforeach()

    set(schema_sources)
    foreach(schema IN LISTS ARG_SCHEMAS)
        _isaacsim_validate_relative_path("${ARG_NAME} schema" "${schema}")
        set(schema_source "${CMAKE_CURRENT_SOURCE_DIR}/${schema}")
        if(NOT EXISTS "${schema_source}")
            message(FATAL_ERROR "${ARG_NAME} schema does not exist: ${schema}")
        endif()
        cmake_path(GET schema PARENT_PATH schema_directory)
        set(schema_has_import_root FALSE)
        foreach(import_path IN LISTS import_paths)
            string(FIND "${schema_directory}/" "${import_path}/" import_path_position)
            if(import_path_position EQUAL 0)
                set(schema_has_import_root TRUE)
                break()
            endif()
        endforeach()
        if(NOT schema_has_import_root)
            message(FATAL_ERROR
                "${ARG_NAME} schema ${schema} must be below one of its PYTHON_IMPORTS"
            )
        endif()
        if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${schema_directory}/plugInfo.json")
            message(FATAL_ERROR "${ARG_NAME} schema ${schema} must have a sibling plugInfo.json")
        endif()
        list(APPEND schema_sources "${schema_source}")
    endforeach()

    if(NOT ISAACSIM_ENABLE_PYTHON)
        return()
    endif()

    _isaacsim_get_module_names(
        "${ARG_NAME}" module_target module_alias module_path module_output_dir
    )
    _isaacsim_get_python_stage_dir(python_stage_dir)
    set(package_sources)
    foreach(import_name IN LISTS ARG_PYTHON_IMPORTS)
        string(REPLACE "." "/" import_path "${import_name}")
        set(import_source_dir "${CMAKE_CURRENT_SOURCE_DIR}/${import_path}")
        file(GLOB_RECURSE import_sources CONFIGURE_DEPENDS LIST_DIRECTORIES FALSE "${import_source_dir}/*")
        list(FILTER import_sources EXCLUDE REGEX "${ISAACSIM_PACKAGING_EXCLUDE_REGEX}")
        list(APPEND package_sources ${import_sources})
    endforeach()

    _isaacsim_get_group_component("${group_name}" python python_component)
    set(schema_variants usd2505)
    set(usd2505_root "${ISAACSIM_TARGET_DEPS_DIR}/openusd")
    set(usd2505_version "0.25.5")
    if(ISAACSIM_EXTENSION_USD_ROOT)
        list(APPEND schema_variants usd2511)
        set(usd2511_root "${ISAACSIM_EXTENSION_USD_ROOT}")
        set(usd2511_version "0.25.11")
    endif()

    foreach(schema_variant IN LISTS schema_variants)
        set(usd_root "${${schema_variant}_root}")
        set(usd_version "${${schema_variant}_version}")
        set(package_output_dir "${module_output_dir}/python-package-${schema_variant}")
        set(copy_commands)
        set(stage_commands)
        foreach(import_name IN LISTS ARG_PYTHON_IMPORTS)
            string(REPLACE "." "/" import_path "${import_name}")
            set(import_source_dir "${CMAKE_CURRENT_SOURCE_DIR}/${import_path}")
            list(APPEND copy_commands
                COMMAND ${CMAKE_COMMAND}
                    "-DSOURCE=${import_source_dir}"
                    "-DDESTINATION=${package_output_dir}/${import_path}"
                    -P "${_ISAACSIM_CMAKE_HELPER_DIR}/CopyPythonPackage.cmake"
            )
            if(schema_variant STREQUAL "usd2505")
                list(APPEND stage_commands
                    COMMAND ${CMAKE_COMMAND}
                        "-DSOURCE=${package_output_dir}/${import_path}"
                        "-DDESTINATION=${python_stage_dir}/${import_path}"
                        -P "${_ISAACSIM_CMAKE_HELPER_DIR}/StageEditableDirectory.cmake"
                )
            endif()
        endforeach()

        set(schema_commands)
        set(schema_outputs)
        foreach(schema IN LISTS ARG_SCHEMAS)
            set(schema_source "${CMAKE_CURRENT_SOURCE_DIR}/${schema}")
            cmake_path(GET schema PARENT_PATH schema_directory)
            set(schema_output_dir "${package_output_dir}/${schema_directory}")
            list(APPEND schema_commands
                COMMAND ${Python_EXECUTABLE} -s
                    "${_ISAACSIM_CMAKE_HELPER_DIR}/run_usd_gen_schema.py"
                    "--build-deps=${ISAACSIM_PYTHON_SCHEMA_BUILD_DEPS_DIR}"
                    "--usd-root=${usd_root}"
                    "--expected-usd-version=${usd_version}"
                    "--schema=${schema_source}"
                    "--output=${schema_output_dir}"
            )
            list(APPEND schema_outputs
                "${schema_output_dir}/generatedSchema.usda"
                "${schema_output_dir}/plugInfo.json"
            )
        endforeach()

        set(stage_stamp "${module_output_dir}/python-package-${schema_variant}-stage.stamp")
        add_custom_command(
            OUTPUT "${stage_stamp}" ${schema_outputs}
            COMMAND ${CMAKE_COMMAND} -E remove_directory "${package_output_dir}"
            COMMAND ${CMAKE_COMMAND} -E make_directory "${package_output_dir}"
            ${copy_commands}
            ${schema_commands}
            ${stage_commands}
            COMMAND ${CMAKE_COMMAND} -E touch "${stage_stamp}"
            DEPENDS
                ${package_sources}
                ${schema_sources}
                "${_ISAACSIM_CMAKE_HELPER_DIR}/CopyPythonPackage.cmake"
                "${_ISAACSIM_CMAKE_HELPER_DIR}/IsaacSimPackagingFilters.cmake"
                "${_ISAACSIM_CMAKE_HELPER_DIR}/run_usd_gen_schema.py"
                "${_ISAACSIM_CMAKE_HELPER_DIR}/StageEditableDirectory.cmake"
                "${usd_root}/lib/python/pxr/Usd/usdGenSchema.py"
            VERBATIM
        )
        if(schema_variant STREQUAL "usd2505")
            set(stage_target "stage-${module_target}-python-package")
        else()
            set(stage_target "stage-${module_target}-python-package-${schema_variant}")
        endif()
        add_custom_target(${stage_target} ALL DEPENDS "${stage_stamp}")
        set_target_properties(${stage_target} PROPERTIES
            ISAACSIM_PYTHON_STAGE_STAMP "${stage_stamp}"
            ISAACSIM_GROUP_NAME "${group_name}"
            ISAACSIM_PACKAGE_VERSION "${group_version}"
        )
        add_dependencies("${group_name}-python-build" ${stage_target})

        set(variant_component "${python_component}-${schema_variant}")
        set_property(GLOBAL APPEND PROPERTY
            "ISAACSIM_GROUP_${group_name}_PYTHON_VARIANT_COMPONENTS" "${variant_component}"
        )
        install(DIRECTORY "${package_output_dir}/"
            DESTINATION "${ISAACSIM_PYTHON_INSTALL_DIR}"
            COMPONENT "${variant_component}"
            ${ISAACSIM_PACKAGING_EXCLUDE_ARGS}
        )
        install(FILES
            "${CMAKE_BINARY_DIR}/packages/${group_name}/package.json"
            "${ISAACSIM_LIBRARIES_VERSION_FILE}"
            DESTINATION "${CMAKE_INSTALL_DATADIR}/isaacsim/packages/${group_name}"
            COMPONENT "${variant_component}"
        )
        if(schema_variant STREQUAL "usd2505")
            install(DIRECTORY "${package_output_dir}/"
                DESTINATION "${ISAACSIM_PYTHON_INSTALL_DIR}"
                COMPONENT "${python_component}"
                ${ISAACSIM_PACKAGING_EXCLUDE_ARGS}
            )
        endif()
    endforeach()

    set_property(GLOBAL APPEND PROPERTY ISAACSIM_PYTHON_MODULE_NAMES "${ARG_NAME}")
    set_property(GLOBAL APPEND PROPERTY ISAACSIM_PYTHON_IMPORTS ${ARG_PYTHON_IMPORTS})
    set_property(GLOBAL APPEND PROPERTY
        "ISAACSIM_GROUP_${group_name}_PYTHON_IMPORTS" ${ARG_PYTHON_IMPORTS}
    )
endfunction()
