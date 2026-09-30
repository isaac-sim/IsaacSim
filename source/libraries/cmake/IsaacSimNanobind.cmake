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

function(_isaacsim_make_relative_prefix output count suffix)
    if(WIN32)
        set(${output} "" PARENT_SCOPE)
        return()
    endif()
    set(relative_path "${ISAACSIM_LOADER_ORIGIN}")
    if(count GREATER 0)
        foreach(index RANGE 1 ${count})
            string(APPEND relative_path "/..")
        endforeach()
    endif()
    if(NOT suffix STREQUAL "")
        string(APPEND relative_path "/${suffix}")
    endif()
    set(${output} "${relative_path}" PARENT_SCOPE)
endfunction()

function(_isaacsim_apply_nanobind_msvc_debug_workaround target)
    if(NOT MSVC)
        return()
    endif()
    get_target_property(workaround_applied ${target} ISAACSIM_NANOBIND_MSVC_DEBUG_WORKAROUND)
    if(workaround_applied)
        return()
    endif()

    # Nanobind temporarily removes _DEBUG while including Python.h so release
    # Python headers do not select python312_d.lib. Include the MSVC runtime
    # declarations first so restoring _DEBUG does not leave the STL without
    # debug-only CRT declarations.
    target_compile_options(${target} PRIVATE
        "$<$<CONFIG:Debug>:/FIyvals.h>"
        "$<$<CONFIG:Debug>:/FIcrtdefs.h>"
    )
    set_property(TARGET ${target} PROPERTY ISAACSIM_NANOBIND_MSVC_DEBUG_WORKAROUND TRUE)
endfunction()

function(_isaacsim_register_python_runtime_stage module_target python_library_dir)
    if(WIN32)
        return()
    endif()

    get_property(runtime_stage_finalized GLOBAL PROPERTY ISAACSIM_PYTHON_RUNTIME_STAGE_FINALIZED)
    if(runtime_stage_finalized)
        message(FATAL_ERROR "Python runtime targets must be registered before module finalization")
    endif()
    get_property(runtime_stage_library_dir GLOBAL PROPERTY ISAACSIM_PYTHON_RUNTIME_STAGE_LIBRARY_DIR)
    if(runtime_stage_library_dir
       AND NOT "${runtime_stage_library_dir}" STREQUAL "${python_library_dir}")
        message(FATAL_ERROR
            "Python runtime targets use inconsistent staging directories: "
            "${runtime_stage_library_dir} and ${python_library_dir}"
        )
    endif()
    set_property(GLOBAL PROPERTY ISAACSIM_PYTHON_RUNTIME_STAGE_LIBRARY_DIR "${python_library_dir}")

    get_property(runtime_stage_targets GLOBAL PROPERTY ISAACSIM_PYTHON_RUNTIME_STAGE_TARGETS)
    if(module_target IN_LIST runtime_stage_targets)
        return()
    endif()
    set_property(GLOBAL APPEND PROPERTY ISAACSIM_PYTHON_RUNTIME_STAGE_TARGETS ${module_target})
    if(NOT TARGET stage-isaacsim-python-runtime)
        add_custom_target(stage-isaacsim-python-runtime)
    endif()
endfunction()

function(_isaacsim_get_external_runtime_entries module_target output_targets output_destinations)
    get_target_property(external_runtime_targets
        ${module_target} ISAACSIM_EXTERNAL_RUNTIME_TARGETS)
    if(NOT external_runtime_targets OR external_runtime_targets MATCHES "-NOTFOUND$")
        set(${output_targets} "" PARENT_SCOPE)
        set(${output_destinations} "" PARENT_SCOPE)
        return()
    endif()

    get_target_property(external_runtime_destinations
        ${module_target} ISAACSIM_EXTERNAL_RUNTIME_DESTINATIONS)
    if(NOT external_runtime_destinations
       OR external_runtime_destinations MATCHES "-NOTFOUND$")
        foreach(external_runtime_target IN LISTS external_runtime_targets)
            list(APPEND external_runtime_destinations ".")
        endforeach()
    endif()
    list(LENGTH external_runtime_targets target_count)
    list(LENGTH external_runtime_destinations destination_count)
    if(NOT target_count EQUAL destination_count)
        message(FATAL_ERROR
            "${module_target} declares ${target_count} external runtime targets but "
            "${destination_count} destinations"
        )
    endif()
    set(${output_targets} "${external_runtime_targets}" PARENT_SCOPE)
    set(${output_destinations} "${external_runtime_destinations}" PARENT_SCOPE)
endfunction()

function(_isaacsim_finalize_python_runtime_stage)
    if(WIN32 OR NOT TARGET stage-isaacsim-python-runtime)
        return()
    endif()
    get_property(runtime_stage_finalized GLOBAL PROPERTY ISAACSIM_PYTHON_RUNTIME_STAGE_FINALIZED)
    if(runtime_stage_finalized)
        return()
    endif()
    set_property(GLOBAL PROPERTY ISAACSIM_PYTHON_RUNTIME_STAGE_FINALIZED TRUE)

    # Compose the shared runtime once after every module is registered. Multiple
    # SDKs may contribute to the same destination, so independent remove/copy
    # commands would race and leave only the last payload that happened to run.
    get_property(python_library_dir GLOBAL PROPERTY ISAACSIM_PYTHON_RUNTIME_STAGE_LIBRARY_DIR)
    get_property(runtime_stage_targets GLOBAL PROPERTY ISAACSIM_PYTHON_RUNTIME_STAGE_TARGETS)
    set(runtime_stage_stamp "${CMAKE_BINARY_DIR}/isaacsim-python-runtime-stage.stamp")
    set(runtime_stage_commands
        COMMAND ${CMAKE_COMMAND} -E make_directory "${python_library_dir}"
    )
    set(runtime_stage_dependencies)
    set(initialized_runtime_destinations)

    foreach(module_target IN LISTS runtime_stage_targets)
        _isaacsim_get_external_runtime_entries(
            ${module_target} external_runtime_targets external_runtime_destinations)
        while(external_runtime_targets)
            list(POP_FRONT external_runtime_targets external_runtime_target)
            list(POP_FRONT external_runtime_destinations external_runtime_destination)
            set(external_runtime_stage_dir "${python_library_dir}")
            if(NOT external_runtime_destination STREQUAL ".")
                string(APPEND external_runtime_stage_dir "/${external_runtime_destination}")
            endif()
            list(APPEND runtime_stage_commands
                COMMAND ${CMAKE_COMMAND} -E make_directory
                    "${external_runtime_stage_dir}"
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    "$<TARGET_FILE:${external_runtime_target}>"
                    "${external_runtime_stage_dir}/$<TARGET_FILE_NAME:${external_runtime_target}>"
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    "$<TARGET_FILE:${external_runtime_target}>"
                    "${external_runtime_stage_dir}/$<TARGET_SONAME_FILE_NAME:${external_runtime_target}>"
            )
            list(APPEND runtime_stage_dependencies ${external_runtime_target})
        endwhile()

        get_target_property(runtime_directories
            ${module_target} ISAACSIM_RUNTIME_DIRECTORIES)
        if(runtime_directories AND NOT runtime_directories MATCHES "-NOTFOUND$")
            while(runtime_directories)
                list(POP_FRONT runtime_directories runtime_source runtime_destination)
                if(NOT runtime_destination IN_LIST initialized_runtime_destinations)
                    list(APPEND initialized_runtime_destinations "${runtime_destination}")
                    list(APPEND runtime_stage_commands
                        COMMAND ${CMAKE_COMMAND} -E remove_directory
                            "${python_library_dir}/${runtime_destination}"
                        COMMAND ${CMAKE_COMMAND} -E make_directory
                            "${python_library_dir}/${runtime_destination}"
                    )
                endif()
                list(APPEND runtime_stage_commands
                    COMMAND ${CMAKE_COMMAND} -E copy_directory
                        "${runtime_source}" "${python_library_dir}/${runtime_destination}"
                )
                list(APPEND runtime_stage_dependencies "${runtime_source}")
            endwhile()
        endif()
        get_target_property(runtime_directory_files
            ${module_target} ISAACSIM_RUNTIME_DIRECTORY_FILES)
        if(runtime_directory_files AND NOT runtime_directory_files MATCHES "-NOTFOUND$")
            list(APPEND runtime_stage_dependencies ${runtime_directory_files})
        endif()
    endforeach()
    list(REMOVE_DUPLICATES runtime_stage_dependencies)
    list(APPEND runtime_stage_commands
        COMMAND ${CMAKE_COMMAND} -E touch "${runtime_stage_stamp}"
    )

    add_custom_command(
        OUTPUT "${runtime_stage_stamp}"
        ${runtime_stage_commands}
        DEPENDS ${runtime_stage_dependencies}
        VERBATIM
    )
    add_custom_target(stage-isaacsim-python-runtime-worker DEPENDS "${runtime_stage_stamp}")
    add_dependencies(stage-isaacsim-python-runtime stage-isaacsim-python-runtime-worker)
endfunction()

function(_isaacsim_resolve_nanobind_stub_preloads module_name output_modules output_dependencies)
    set(preload_modules)
    set(preload_dependencies)
    foreach(preload IN LISTS ARGN)
        _isaacsim_get_module_names(
            "${preload}" preload_target preload_alias preload_path preload_output_dir)
        if(NOT TARGET "${preload_target}-python")
            message(FATAL_ERROR
                "${module_name} STUB_PRELOADS entry has no registered nanobind target: ${preload}")
        endif()
        if(NOT TARGET "stage-${preload_target}-python")
            message(FATAL_ERROR
                "${module_name} STUB_PRELOADS entry has no staged nanobind target: ${preload}")
        endif()
        list(APPEND preload_modules "${preload}.bindings._bindings")
        list(APPEND preload_dependencies "stage-${preload_target}-python")
    endforeach()
    set(${output_modules} "${preload_modules}" PARENT_SCOPE)
    set(${output_dependencies} "${preload_dependencies}" PARENT_SCOPE)
endfunction()

function(isaacsim_add_nanobind)
    set(options NO_STABLE_ABI)
    set(one_value_args MODULE SOURCE STABLE_ABI_EXCEPTION WINDOWS_RUNTIME_SIBLING_DIRECTORY)
    set(multi_value_args SOURCES DEPENDENCIES STUB_PRELOADS)
    cmake_parse_arguments(ARG "${options}" "${one_value_args}" "${multi_value_args}" ${ARGN})
    if(ARG_UNPARSED_ARGUMENTS)
        message(FATAL_ERROR "isaacsim_add_nanobind received unknown arguments: ${ARG_UNPARSED_ARGUMENTS}")
    endif()
    if(NOT ARG_MODULE)
        message(FATAL_ERROR "isaacsim_add_nanobind requires MODULE")
    endif()
    if(ARG_SOURCE AND ARG_SOURCES)
        message(FATAL_ERROR "${ARG_MODULE} must use either SOURCE or SOURCES, not both")
    endif()
    set(binding_sources ${ARG_SOURCES})
    if(ARG_SOURCE)
        list(APPEND binding_sources "${ARG_SOURCE}")
    endif()
    if(NOT binding_sources)
        message(FATAL_ERROR "isaacsim_add_nanobind requires SOURCE or SOURCES")
    endif()
    if(ARG_WINDOWS_RUNTIME_SIBLING_DIRECTORY
       AND NOT IS_DIRECTORY "${ARG_WINDOWS_RUNTIME_SIBLING_DIRECTORY}")
        message(FATAL_ERROR
            "${ARG_MODULE} WINDOWS_RUNTIME_SIBLING_DIRECTORY does not exist: "
            "${ARG_WINDOWS_RUNTIME_SIBLING_DIRECTORY}")
    endif()
    if(ARG_NO_STABLE_ABI AND NOT ARG_STABLE_ABI_EXCEPTION)
        message(FATAL_ERROR "${ARG_MODULE} must document why STABLE_ABI is disabled")
    endif()

    set(resolved_binding_sources)
    foreach(binding_source IN LISTS binding_sources)
        if(NOT IS_ABSOLUTE "${binding_source}")
            set(binding_source "${CMAKE_CURRENT_SOURCE_DIR}/${binding_source}")
        endif()
        if(NOT EXISTS "${binding_source}")
            message(FATAL_ERROR "${ARG_MODULE} binding source does not exist: ${binding_source}")
        endif()
        list(APPEND resolved_binding_sources "${binding_source}")
    endforeach()

    _isaacsim_get_module_names("${ARG_MODULE}" module_target module_alias module_path module_output_dir)
    if(NOT TARGET "${module_target}")
        message(FATAL_ERROR "isaacsim_add_nanobind must follow native module registration for ${ARG_MODULE}")
    endif()
    if(NOT ISAACSIM_ENABLE_PYTHON_BINDINGS)
        return()
    endif()
    get_target_property(group_name ${module_target} ISAACSIM_GROUP_NAME)
    get_target_property(package_version ${module_target} ISAACSIM_PACKAGE_VERSION)
    _isaacsim_validate_group_target_dependencies("${ARG_MODULE} Python binding" "${group_name}" ${ARG_DEPENDENCIES})
    _isaacsim_get_group_component("${group_name}" python python_component)
    isaacsim_add_python_module(NAME "${ARG_MODULE}")
    set(python_package_stage_target "stage-${module_target}-python-package")

    set(binding_target "${module_target}-python")
    set(nanobind_options NB_STATIC)
    if(NOT ARG_NO_STABLE_ABI)
        list(APPEND nanobind_options STABLE_ABI)
    endif()
    nanobind_add_module(${binding_target} ${nanobind_options} ${resolved_binding_sources})
    _isaacsim_apply_nanobind_msvc_debug_workaround(${binding_target})
    get_target_property(binding_link_libraries ${binding_target} LINK_LIBRARIES)
    foreach(nanobind_runtime_target IN LISTS binding_link_libraries)
        if(TARGET ${nanobind_runtime_target} AND nanobind_runtime_target MATCHES "^nanobind-")
            _isaacsim_apply_nanobind_msvc_debug_workaround(${nanobind_runtime_target})
        endif()
    endforeach()
    _isaacsim_apply_build_options(${binding_target})
    target_link_libraries(${binding_target} PRIVATE ${module_target} ${ARG_DEPENDENCIES})

    string(REPLACE "/" ";" module_components "${module_path}")
    list(LENGTH module_components module_depth)
    _isaacsim_count_path_components("${ISAACSIM_PYTHON_INSTALL_DIR}" python_install_depth)
    math(EXPR install_prefix_depth "${python_install_depth} + ${module_depth} + 1")
    _isaacsim_make_relative_prefix(prefix_library_rpath ${install_prefix_depth} "${CMAKE_INSTALL_LIBDIR}")
    _isaacsim_make_relative_prefix(package_library_rpath ${module_depth} "lib")

    set(binding_build_dir "${module_output_dir}/bindings")
    _isaacsim_get_python_stage_dir(python_stage_dir)
    set(python_binding_dir "${python_stage_dir}/${module_path}/bindings")
    set(binding_runtime_initializer "${_ISAACSIM_CMAKE_HELPER_DIR}/initialize_nanobind_runtime.py")
    set(python_library_dir "${python_stage_dir}/isaacsim/lib")
    set(python_runtime_stage_dir "${python_library_dir}")
    if(WIN32)
        set(python_runtime_stage_dir "${python_binding_dir}")
    endif()
    set_target_properties(${binding_target} PROPERTIES
        OUTPUT_NAME "_bindings"
        LIBRARY_OUTPUT_DIRECTORY "${binding_build_dir}"
        RUNTIME_OUTPUT_DIRECTORY "${binding_build_dir}"
        BUILD_RPATH "${package_library_rpath}"
        BUILD_RPATH_USE_ORIGIN YES
        INSTALL_RPATH "${package_library_rpath};${prefix_library_rpath}"
        ISAACSIM_GROUP_NAME "${group_name}"
        ISAACSIM_PACKAGE_VERSION "${package_version}"
    )

    _isaacsim_collect_runtime_targets(${module_target} runtime_targets)
    set(external_runtime_targets)
    set(external_runtime_destinations)
    foreach(runtime_target IN LISTS runtime_targets)
        _isaacsim_get_external_runtime_entries(
            ${runtime_target} runtime_external_targets runtime_external_destinations)
        while(runtime_external_targets)
            list(POP_FRONT runtime_external_targets runtime_external_target)
            list(POP_FRONT runtime_external_destinations runtime_external_destination)
            list(FIND external_runtime_targets "${runtime_external_target}" external_runtime_index)
            if(external_runtime_index EQUAL -1)
                list(APPEND external_runtime_targets "${runtime_external_target}")
                list(APPEND external_runtime_destinations "${runtime_external_destination}")
            else()
                list(GET external_runtime_destinations
                    ${external_runtime_index} existing_runtime_destination)
                if(NOT existing_runtime_destination STREQUAL runtime_external_destination)
                    message(FATAL_ERROR
                        "External runtime target ${runtime_external_target} is staged to both "
                        "${existing_runtime_destination} and ${runtime_external_destination}"
                    )
                endif()
            endif()
        endwhile()
    endforeach()
    set(stage_stamp "${module_output_dir}/python-binding-stage.stamp")
    set(stage_commands
        COMMAND ${CMAKE_COMMAND} -E make_directory "${python_binding_dir}"
        COMMAND ${CMAKE_COMMAND} -E make_directory "${python_library_dir}"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "$<TARGET_FILE:${binding_target}>" "${python_binding_dir}/$<TARGET_FILE_NAME:${binding_target}>"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${binding_runtime_initializer}" "${python_binding_dir}/__init__.py"
    )
    if(WIN32)
        # Preserve any package-provided POST_BUILD runtime payload (for example,
        # ovphysx's config and plugins) emitted beside the extension target.
        list(APPEND stage_commands
            COMMAND ${CMAKE_COMMAND} -E copy_directory
                "${binding_build_dir}" "${python_binding_dir}"
        )
        if(ARG_WINDOWS_RUNTIME_SIBLING_DIRECTORY)
            # Some Windows SDKs use a bin/plugins split. Keep the SDK-level
            # plugins beside bindings/ instead of merging them with a distinct
            # bindings/plugins tree.
            list(APPEND stage_commands
                COMMAND ${CMAKE_COMMAND}
                    "-DSOURCE=${ARG_WINDOWS_RUNTIME_SIBLING_DIRECTORY}"
                    "-DDESTINATION=${python_stage_dir}/${module_path}/plugins"
                    -P "${_ISAACSIM_CMAKE_HELPER_DIR}/CopyRuntimeDirectory.cmake"
            )
            install(DIRECTORY "${ARG_WINDOWS_RUNTIME_SIBLING_DIRECTORY}/"
                DESTINATION "${ISAACSIM_PYTHON_INSTALL_DIR}/${module_path}/plugins"
                COMPONENT "${python_component}"
                ${ISAACSIM_PACKAGING_EXCLUDE_ARGS}
            )
        endif()
    endif()
    set(stage_dependencies ${binding_target} ${module_target} ${python_package_stage_target})
    if(WIN32)
        list(APPEND stage_dependencies
            "${_ISAACSIM_CMAKE_HELPER_DIR}/CopyRuntimeDirectory.cmake"
            "${_ISAACSIM_CMAKE_HELPER_DIR}/IsaacSimPackagingFilters.cmake"
        )
    endif()
    get_target_property(python_package_stage_stamp
        ${python_package_stage_target} ISAACSIM_PYTHON_STAGE_STAMP
    )
    list(APPEND stage_dependencies "${python_package_stage_stamp}")
    foreach(runtime_target IN LISTS runtime_targets)
        if(NOT WIN32)
            _isaacsim_register_python_runtime_stage(
                ${runtime_target} "${python_library_dir}")
        endif()
        if(NOT runtime_target STREQUAL module_target)
            if(TARGET "stage-${runtime_target}-python-package")
                list(APPEND stage_dependencies "stage-${runtime_target}-python-package")
            endif()
            if(TARGET "stage-${runtime_target}-python")
                list(APPEND stage_dependencies "stage-${runtime_target}-python")
            endif()
        endif()
        # Windows resolves dependent DLLs from the importing extension's
        # directory. Copy the full transitive module closure beside each .pyd;
        # a sibling module's own staging directory is not on the DLL search path.
        if(WIN32 OR runtime_target STREQUAL module_target OR NOT TARGET "stage-${runtime_target}-python")
            list(APPEND stage_commands
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    "$<TARGET_FILE:${runtime_target}>"
                    "${python_runtime_stage_dir}/$<TARGET_FILE_NAME:${runtime_target}>"
            )
            list(APPEND stage_dependencies ${runtime_target})
            if(NOT WIN32)
                list(APPEND stage_commands
                    COMMAND ${CMAKE_COMMAND} -E copy_if_different
                        "$<TARGET_FILE:${runtime_target}>"
                        "${python_library_dir}/$<TARGET_SONAME_FILE_NAME:${runtime_target}>"
                )
            endif()
        endif()
    endforeach()
    if(NOT WIN32)
        list(APPEND stage_dependencies stage-isaacsim-python-runtime)
    endif()
    set(external_python_stage_dir "${python_library_dir}")
    if(WIN32)
        set(external_python_stage_dir "${python_binding_dir}")
    endif()
    foreach(runtime_target IN LISTS external_runtime_targets)
        if(WIN32)
            list(APPEND stage_commands
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    "$<TARGET_FILE:${runtime_target}>"
                    "${external_python_stage_dir}/$<TARGET_FILE_NAME:${runtime_target}>"
            )
            list(APPEND stage_dependencies ${runtime_target})
        endif()
    endforeach()
    set(runtime_directories)
    set(runtime_directory_files)
    foreach(runtime_target IN LISTS runtime_targets)
        get_target_property(runtime_target_directories
            ${runtime_target} ISAACSIM_RUNTIME_DIRECTORIES)
        if(runtime_target_directories
           AND NOT runtime_target_directories MATCHES "-NOTFOUND$")
            get_target_property(windows_python_shared_runtime_destinations
                ${runtime_target} ISAACSIM_WINDOWS_PYTHON_SHARED_RUNTIME_DESTINATIONS)
            if(NOT windows_python_shared_runtime_destinations
               OR windows_python_shared_runtime_destinations MATCHES "-NOTFOUND$")
                set(windows_python_shared_runtime_destinations)
            endif()
            while(runtime_target_directories)
                list(POP_FRONT runtime_target_directories runtime_source runtime_destination)
                if(runtime_destination IN_LIST windows_python_shared_runtime_destinations)
                    set(windows_python_shared TRUE)
                else()
                    set(windows_python_shared FALSE)
                endif()
                list(APPEND runtime_directories
                    "${runtime_source}" "${runtime_destination}" "${windows_python_shared}")
            endwhile()
        endif()
        get_target_property(runtime_target_directory_files
            ${runtime_target} ISAACSIM_RUNTIME_DIRECTORY_FILES)
        if(runtime_target_directory_files
           AND NOT runtime_target_directory_files MATCHES "-NOTFOUND$")
            list(APPEND runtime_directory_files ${runtime_target_directory_files})
        endif()
    endforeach()
    list(REMOVE_DUPLICATES runtime_directory_files)
    if(runtime_directories AND NOT runtime_directories MATCHES "-NOTFOUND$")
        while(runtime_directories)
            list(POP_FRONT runtime_directories
                runtime_source runtime_destination windows_python_shared)
            if(WIN32)
                list(APPEND stage_dependencies "${runtime_source}")
                if(windows_python_shared)
                    set(runtime_python_stage_dir "${python_library_dir}")
                    set(runtime_install_destination
                        "${ISAACSIM_PYTHON_INSTALL_DIR}/isaacsim/lib/${runtime_destination}")
                else()
                    set(runtime_python_stage_dir "${python_binding_dir}")
                    set(runtime_install_destination
                        "${ISAACSIM_PYTHON_INSTALL_DIR}/${module_path}/bindings/${runtime_destination}")
                endif()
                list(APPEND stage_commands
                    COMMAND ${CMAKE_COMMAND}
                        "-DSOURCE=${runtime_source}"
                        "-DDESTINATION=${runtime_python_stage_dir}/${runtime_destination}"
                        -P "${_ISAACSIM_CMAKE_HELPER_DIR}/CopyRuntimeDirectory.cmake"
                )
            else()
                set(runtime_install_destination
                    "${ISAACSIM_PYTHON_INSTALL_DIR}/isaacsim/lib/${runtime_destination}")
            endif()
            install(DIRECTORY "${runtime_source}/"
                DESTINATION "${runtime_install_destination}"
                COMPONENT "${python_component}"
                ${ISAACSIM_PACKAGING_EXCLUDE_ARGS}
            )
        endwhile()
    endif()
    if(runtime_directory_files)
        list(APPEND stage_dependencies ${runtime_directory_files})
    endif()
    list(APPEND stage_commands COMMAND ${CMAKE_COMMAND} -E touch "${stage_stamp}")

    add_custom_command(
        OUTPUT "${stage_stamp}"
        ${stage_commands}
        DEPENDS ${stage_dependencies} "${binding_runtime_initializer}"
        VERBATIM
    )
    add_custom_target(stage-${binding_target} ALL DEPENDS "${stage_stamp}")

    set(binding_stub "${python_binding_dir}/_bindings.pyi")
    if(EXISTS "${NB_DIR}/src/stubgen.py")
        set(nanobind_stubgen "${NB_DIR}/src/stubgen.py")
    elseif(EXISTS "${NB_DIR}/stubgen.py")
        set(nanobind_stubgen "${NB_DIR}/stubgen.py")
    else()
        message(FATAL_ERROR "Could not locate nanobind stubgen.py")
    endif()
    set(stub_python "${Python_EXECUTABLE}")
    set(stub_environment "PYTHONDONTWRITEBYTECODE=1")
    # stubgen imports the module cold, with none of the runtime setup a consumer gets.
    # A prebuilt dependency whose own DT_RPATH cannot reach a relocated sibling -- an
    # app-supplied OVStage, say -- is only resolvable through the loader search path.
    if(NOT WIN32 AND ISAACSIM_BINDING_EXTRA_LIBRARY_PATH)
        # Join only when there is something to join to: an empty entry, which a
        # trailing colon leaves behind when the variable is unset, is searched
        # relative to the working directory.
        set(_stub_library_path "${ISAACSIM_BINDING_EXTRA_LIBRARY_PATH}")
        if(NOT "$ENV{LD_LIBRARY_PATH}" STREQUAL "")
            string(APPEND _stub_library_path ":$ENV{LD_LIBRARY_PATH}")
        endif()
        list(APPEND stub_environment "LD_LIBRARY_PATH=${_stub_library_path}")
    endif()
    if(NOT WIN32)
        nanobind_sanitizer_preload_env(stub_sanitizer_environment ${binding_target} ${module_target})
        if(stub_sanitizer_environment)
            nanobind_resolve_python_path()
            set(stub_python "${NB_PY_PATH}")
            list(APPEND stub_environment "${stub_sanitizer_environment}")
            if(stub_sanitizer_environment MATCHES "asan")
                list(APPEND stub_environment "ASAN_OPTIONS=detect_leaks=0")
            endif()
        endif()
    endif()
    set(stub_launcher "${_ISAACSIM_CMAKE_HELPER_DIR}/run_nanobind_stubgen.py")
    set(stub_command "${stub_python}" -I "${stub_launcher}")
    if(WIN32)
        list(APPEND stub_command --binding-dir "${python_binding_dir}")
    endif()
    _isaacsim_resolve_nanobind_stub_preloads(
        "${ARG_MODULE}" stub_preload_modules stub_preload_dependencies ${ARG_STUB_PRELOADS})
    foreach(stub_preload_module IN LISTS stub_preload_modules)
        list(APPEND stub_command --preload-module "${stub_preload_module}")
    endforeach()
    list(APPEND stub_command "${nanobind_stubgen}")
    add_custom_command(
        OUTPUT "${binding_stub}"
        COMMAND ${CMAKE_COMMAND} -E env ${stub_environment}
            ${stub_command}
            -q
            -m "${ARG_MODULE}.bindings._bindings"
            -i "${python_stage_dir}"
            -i "${ISAACSIM_NATIVE_RUNTIME_DEPS_DIR}"
            -i "${ISAACSIM_PYTHON_RUNTIME_DEPS_DIR}"
            -o "${binding_stub}"
        DEPENDS
            ${binding_target}
            stage-${binding_target}
            "${python_package_stage_stamp}"
            "${nanobind_stubgen}"
            ${stub_launcher}
            ${stub_preload_dependencies}
        VERBATIM
    )
    add_custom_target(generate-${binding_target}-stub ALL DEPENDS "${binding_stub}")
    add_dependencies("${group_name}-python-build" generate-${binding_target}-stub)
    add_custom_target(build-${binding_target}-stub ALL)
    add_dependencies(build-${binding_target}-stub generate-${binding_target}-stub)
    if(NOT TARGET check-python-stubs)
        add_custom_target(check-python-stubs)
    endif()
    add_dependencies(check-python-stubs generate-${binding_target}-stub)

    install(TARGETS ${binding_target}
        LIBRARY DESTINATION "${ISAACSIM_PYTHON_INSTALL_DIR}/${module_path}/bindings" COMPONENT "${python_component}"
        RUNTIME DESTINATION "${ISAACSIM_PYTHON_INSTALL_DIR}/${module_path}/bindings" COMPONENT "${python_component}"
    )
    if(WIN32)
        install(TARGETS ${runtime_targets}
            RUNTIME DESTINATION "${ISAACSIM_PYTHON_INSTALL_DIR}/${module_path}/bindings"
            COMPONENT "${python_component}"
        )
    else()
        # A binding can depend on native modules owned by another distribution.
        # Register every runtime target with its owning group so dependency wheels
        # remain self-contained without duplicating their libraries in consumers.
        foreach(runtime_target IN LISTS runtime_targets)
            get_target_property(runtime_group ${runtime_target} ISAACSIM_GROUP_NAME)
            set_property(GLOBAL APPEND PROPERTY
                "ISAACSIM_GROUP_${runtime_group}_PYTHON_RUNTIME_TARGETS" ${runtime_target}
            )
        endforeach()
    endif()
    if(external_runtime_targets)
        set(pending_external_runtime_targets ${external_runtime_targets})
        set(pending_external_runtime_destinations ${external_runtime_destinations})
        while(pending_external_runtime_targets)
            list(POP_FRONT pending_external_runtime_targets runtime_target)
            list(POP_FRONT pending_external_runtime_destinations runtime_destination)
            if(WIN32)
                set(external_python_destination
                    "${ISAACSIM_PYTHON_INSTALL_DIR}/${module_path}/bindings")
            else()
                set(external_python_destination "${ISAACSIM_PYTHON_INSTALL_DIR}/isaacsim/lib")
                if(NOT runtime_destination STREQUAL ".")
                    string(APPEND external_python_destination "/${runtime_destination}")
                endif()
            endif()
            install(FILES "$<TARGET_FILE:${runtime_target}>"
                DESTINATION "${external_python_destination}"
                COMPONENT "${python_component}"
            )
            if(NOT WIN32)
                install(FILES "$<TARGET_FILE:${runtime_target}>"
                    DESTINATION "${external_python_destination}"
                    RENAME "$<TARGET_SONAME_FILE_NAME:${runtime_target}>"
                    COMPONENT "${python_component}"
                )
            endif()
        endwhile()
    endif()
    install(FILES "${binding_stub}"
        DESTINATION "${ISAACSIM_PYTHON_INSTALL_DIR}/${module_path}/bindings"
        COMPONENT "${python_component}"
    )
    install(FILES "${binding_runtime_initializer}"
        DESTINATION "${ISAACSIM_PYTHON_INSTALL_DIR}/${module_path}/bindings"
        RENAME "__init__.py"
        COMPONENT "${python_component}"
    )
endfunction()
