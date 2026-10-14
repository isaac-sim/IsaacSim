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
include("${CMAKE_CURRENT_LIST_DIR}/IsaacSimDocumentationCatalog.cmake")

# Register one independently installable distribution. All distributions use the
# repository-wide ISAACSIM_LIBRARIES_VERSION set by source/libraries/CMakeLists.txt.
function(isaacsim_add_group)
    cmake_parse_arguments(ARG "" "NAME;MODULE_PREFIX" "MODULES" ${ARGN})
    if(ARG_UNPARSED_ARGUMENTS)
        message(FATAL_ERROR "isaacsim_add_group received unknown arguments: ${ARG_UNPARSED_ARGUMENTS}")
    endif()
    if(NOT ARG_NAME MATCHES "^[a-z][a-z0-9_]*$")
        message(FATAL_ERROR "isaacsim_add_group requires a lowercase distribution NAME")
    endif()
    if(NOT ISAACSIM_LIBRARIES_VERSION OR NOT ISAACSIM_LIBRARIES_RELEASE_VERSION)
        message(FATAL_ERROR "isaacsim_add_group requires the repository-wide module version")
    endif()
    _isaacsim_validate_group_source_path("${ARG_NAME}" "${ARG_MODULE_PREFIX}" module_prefix)
    if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/pyproject.toml")
        message(FATAL_ERROR "Distribution group ${ARG_NAME} must provide pyproject.toml at its package root")
    endif()

    get_property(group_names GLOBAL PROPERTY ISAACSIM_GROUP_NAMES)
    if(ARG_NAME IN_LIST group_names)
        message(FATAL_ERROR "Distribution group ${ARG_NAME} is already registered")
    endif()
    if(NOT ARG_MODULES)
        message(FATAL_ERROR "Distribution group ${ARG_NAME} must declare MODULES")
    endif()

    list(REMOVE_DUPLICATES ARG_MODULES)
    foreach(module_name IN LISTS ARG_MODULES)
        if(NOT module_name MATCHES "^[a-z][a-z0-9_]*(\\.[a-z][a-z0-9_]*)+$")
            message(FATAL_ERROR "Distribution group ${ARG_NAME} has invalid module name: ${module_name}")
        endif()
        string(FIND "${module_name}" "${module_prefix}." module_prefix_position)
        if(NOT module_prefix_position EQUAL 0)
            message(FATAL_ERROR
                "Distribution ${ARG_NAME} may only declare modules below ${module_prefix}: ${module_name}"
            )
        endif()
        string(FIND "${module_name}" "${module_prefix}.docs." docs_descendant_position)
        if(module_name STREQUAL "${module_prefix}.docs" OR docs_descendant_position EQUAL 0)
            message(FATAL_ERROR
                "Distribution ${ARG_NAME} reserves ${module_prefix}.docs and its descendants for package documentation"
            )
        endif()
        foreach(other_module_name IN LISTS ARG_MODULES)
            string(FIND "${other_module_name}" "${module_name}." child_module_position)
            if(child_module_position EQUAL 0)
                message(FATAL_ERROR
                    "Distribution ${ARG_NAME} has overlapping module roots: ${module_name} and ${other_module_name}; "
                    "registered module roots must be leaves"
                )
            endif()
        endforeach()
    endforeach()
    if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/docs/index.rst")
        message(FATAL_ERROR "Distribution group ${ARG_NAME} must provide docs/index.rst")
    endif()
    set(group_docs_dir "${CMAKE_CURRENT_SOURCE_DIR}/docs")

    set(ISAACSIM_CURRENT_GROUP_NAME "${ARG_NAME}" PARENT_SCOPE)
    set_property(GLOBAL APPEND PROPERTY ISAACSIM_GROUP_NAMES "${ARG_NAME}")
    set_property(GLOBAL PROPERTY "ISAACSIM_GROUP_${ARG_NAME}_VERSION" "${ISAACSIM_LIBRARIES_VERSION}")
    set_property(GLOBAL PROPERTY "ISAACSIM_GROUP_${ARG_NAME}_RELEASE_VERSION" "${ISAACSIM_LIBRARIES_RELEASE_VERSION}")
    set_property(GLOBAL PROPERTY "ISAACSIM_GROUP_${ARG_NAME}_DOCS_DIR" "${group_docs_dir}")
    set_property(GLOBAL PROPERTY "ISAACSIM_GROUP_${ARG_NAME}_DEPENDENCIES" "")
    set_property(GLOBAL PROPERTY "ISAACSIM_GROUP_${ARG_NAME}_DECLARED_MODULES" "${ARG_MODULES}")
    set_property(GLOBAL PROPERTY "ISAACSIM_GROUP_${ARG_NAME}_SOURCE_ROOT" "${CMAKE_CURRENT_SOURCE_DIR}")
    set_property(GLOBAL PROPERTY "ISAACSIM_GROUP_${ARG_NAME}_MODULE_PREFIX" "${module_prefix}")
    add_custom_target("${ARG_NAME}-python-build")
endfunction()

# Declare one exact dependency between lockstep-versioned distribution packages.
# The dependency group must be registered first.
function(isaacsim_add_group_dependency)
    cmake_parse_arguments(ARG "" "GROUP;DEPENDS_ON" "" ${ARGN})
    if(ARG_UNPARSED_ARGUMENTS)
        message(FATAL_ERROR "isaacsim_add_group_dependency received unknown arguments: ${ARG_UNPARSED_ARGUMENTS}")
    endif()
    if(NOT ARG_GROUP OR NOT ARG_DEPENDS_ON)
        message(FATAL_ERROR "isaacsim_add_group_dependency requires GROUP and DEPENDS_ON")
    endif()

    get_property(group_names GLOBAL PROPERTY ISAACSIM_GROUP_NAMES)
    if(NOT ARG_GROUP IN_LIST group_names)
        message(FATAL_ERROR "Dependency owner group is not registered: ${ARG_GROUP}")
    endif()
    if(ARG_GROUP STREQUAL ARG_DEPENDS_ON)
        message(FATAL_ERROR "Distribution group ${ARG_GROUP} cannot depend on itself")
    endif()
    if(NOT ARG_DEPENDS_ON IN_LIST group_names)
        message(FATAL_ERROR
            "Distribution group ${ARG_GROUP} dependency ${ARG_DEPENDS_ON} must be registered before it is declared"
        )
    endif()

    get_property(group_dependencies GLOBAL PROPERTY "ISAACSIM_GROUP_${ARG_GROUP}_DEPENDENCIES")
    if(ARG_DEPENDS_ON IN_LIST group_dependencies)
        message(FATAL_ERROR "Distribution group ${ARG_GROUP} declares dependency ${ARG_DEPENDS_ON} more than once")
    endif()

    get_property(group_version GLOBAL PROPERTY "ISAACSIM_GROUP_${ARG_GROUP}_VERSION")
    get_property(dependency_version GLOBAL PROPERTY "ISAACSIM_GROUP_${ARG_DEPENDS_ON}_VERSION")
    if(NOT dependency_version STREQUAL group_version)
        message(FATAL_ERROR
            "Distribution packages ${ARG_GROUP} and ${ARG_DEPENDS_ON} must use the same source/libraries/VERSION: "
            "${group_version} != ${dependency_version}"
        )
    endif()

    set(dependency_key "${ARG_DEPENDS_ON}")
    set_property(GLOBAL APPEND PROPERTY "ISAACSIM_GROUP_${ARG_GROUP}_DEPENDENCIES" "${ARG_DEPENDS_ON}")
    set_property(GLOBAL PROPERTY
        "ISAACSIM_GROUP_${ARG_GROUP}_DEPENDENCY_${dependency_key}_VERSION" "${dependency_version}"
    )
    set_property(GLOBAL PROPERTY
        "ISAACSIM_GROUP_${ARG_GROUP}_DEPENDENCY_${dependency_key}_SPECIFIER" "==${dependency_version}"
    )
endfunction()

# Define the private libraries a distribution bundles but does not expose as part of its linked
# interface. The exported module targets already name them in IMPORTED_LINK_DEPENDENT_LIBRARIES,
# but nothing defined them, so the names dangled in a consumer's project; defining them lets a
# consumer resolve their locations, through $<TARGET_RUNTIME_DLLS> or $<TARGET_FILE_DIR>, without
# naming them itself. Only the location is exported, as these libraries are loaded, never linked,
# and so ship without an import library.
function(_isaacsim_install_runtime_exports group_name package_build_dir cmake_dir component)
    get_property(registered_modules GLOBAL PROPERTY "ISAACSIM_GROUP_${group_name}_MODULES")
    list(REMOVE_DUPLICATES registered_modules)

    set(runtime_targets "")
    set(runtime_destinations "")
    foreach(module_name IN LISTS registered_modules)
        _isaacsim_get_module_names("${module_name}" module_target module_alias module_path module_output_dir)
        if(NOT TARGET "${module_target}")
            continue()
        endif()
        get_target_property(module_targets ${module_target} ISAACSIM_EXTERNAL_RUNTIME_TARGETS)
        get_target_property(module_destinations ${module_target} ISAACSIM_EXTERNAL_RUNTIME_DESTINATIONS)
        if(module_targets AND NOT module_targets MATCHES "-NOTFOUND$")
            list(APPEND runtime_targets ${module_targets})
            list(APPEND runtime_destinations ${module_destinations})
        endif()
    endforeach()
    if(NOT runtime_targets)
        return()
    endif()

    # isaacsim_install_runtime_dependencies stages these libraries beside the module binaries,
    # which Windows keeps in the binary directory and other platforms in the library directory,
    # below the per-dependency destination it recorded.
    if(WIN32)
        set(runtime_root "${CMAKE_INSTALL_BINDIR}")
    else()
        set(runtime_root "${CMAKE_INSTALL_LIBDIR}")
    endif()

    set(runtime_content "")
    set(emitted_targets "")
    list(LENGTH runtime_targets runtime_target_count)
    math(EXPR last_runtime_index "${runtime_target_count} - 1")
    foreach(runtime_index RANGE ${last_runtime_index})
        list(GET runtime_targets ${runtime_index} runtime_target)
        list(GET runtime_destinations ${runtime_index} runtime_destination)
        if(runtime_target IN_LIST emitted_targets)
            continue()
        endif()
        list(APPEND emitted_targets "${runtime_target}")
        set(runtime_directory "${runtime_root}")
        if(NOT runtime_destination STREQUAL ".")
            string(APPEND runtime_directory "/${runtime_destination}")
        endif()
        string(APPEND runtime_content
            "if(NOT TARGET ${runtime_target})\n"
            "    add_library(${runtime_target} SHARED IMPORTED)\n"
            "    set_target_properties(${runtime_target} PROPERTIES\n"
            "        IMPORTED_LOCATION"
            " \"\${PACKAGE_PREFIX_DIR}/${runtime_directory}/$<TARGET_FILE_NAME:${runtime_target}>\"\n"
            "    )\n"
            "endif()\n"
        )
    endforeach()

    # PACKAGE_PREFIX_DIR is provided by the generated config file that includes this one. One
    # output serves every configuration: these are prebuilt dependency libraries carrying a single
    # IMPORTED_LOCATION, so the evaluated name does not vary by configuration, which is what
    # file(GENERATE) requires of a path that does not name one.
    set(runtime_file "${package_build_dir}/${group_name}Runtime.cmake")
    file(GENERATE OUTPUT "${runtime_file}" CONTENT "${runtime_content}")
    install(FILES "${runtime_file}" DESTINATION "${cmake_dir}" COMPONENT "${component}")
endfunction()

function(_isaacsim_make_json_array output)
    set(result "")
    set(separator "")
    foreach(value IN LISTS ARGN)
        string(REPLACE "\\" "\\\\" escaped "${value}")
        string(REPLACE "\"" "\\\"" escaped "${escaped}")
        string(APPEND result "${separator}\n    \"${escaped}\"")
        set(separator ",")
    endforeach()
    set(${output} "${result}" PARENT_SCOPE)
endfunction()

# Generate package metadata and install rules after all modules are registered.
function(isaacsim_finalize_modules)
    if(ARGN)
        message(FATAL_ERROR "isaacsim_finalize_modules received unknown arguments: ${ARGN}")
    endif()
    if(COMMAND _isaacsim_finalize_python_runtime_stage)
        _isaacsim_finalize_python_runtime_stage()
    endif()
    get_property(group_names GLOBAL PROPERTY ISAACSIM_GROUP_NAMES)
    if(ISAACSIM_PACKAGE_GROUP AND NOT ISAACSIM_PACKAGE_GROUP IN_LIST group_names)
        message(FATAL_ERROR "ISAACSIM_PACKAGE_GROUP is not registered: ${ISAACSIM_PACKAGE_GROUP}")
    endif()

    foreach(group_name IN LISTS group_names)
        get_property(group_version GLOBAL PROPERTY "ISAACSIM_GROUP_${group_name}_VERSION")
        get_property(group_release_version GLOBAL PROPERTY "ISAACSIM_GROUP_${group_name}_RELEASE_VERSION")
        get_property(declared_modules GLOBAL PROPERTY "ISAACSIM_GROUP_${group_name}_DECLARED_MODULES")
        get_property(registered_modules GLOBAL PROPERTY "ISAACSIM_GROUP_${group_name}_MODULES")
        list(REMOVE_DUPLICATES registered_modules)

        set(missing_modules ${declared_modules})
        set(undeclared_modules ${registered_modules})
        foreach(module_name IN LISTS registered_modules)
            list(REMOVE_ITEM missing_modules "${module_name}")
        endforeach()
        foreach(module_name IN LISTS declared_modules)
            list(REMOVE_ITEM undeclared_modules "${module_name}")
        endforeach()
        set(inventory_issues)
        if(missing_modules)
            string(JOIN ", " missing_module_message ${missing_modules})
            list(APPEND inventory_issues "missing modules: ${missing_module_message}")
        endif()
        if(undeclared_modules)
            string(JOIN ", " undeclared_module_message ${undeclared_modules})
            list(APPEND inventory_issues "undeclared modules: ${undeclared_module_message}")
        endif()
        if(inventory_issues)
            string(JOIN "; " inventory_issue_message ${inventory_issues})
            if(ISAACSIM_ENFORCE_COMPLETE_PACKAGES OR ISAACSIM_PACKAGE_GROUP STREQUAL group_name)
                message(FATAL_ERROR "Distribution group ${group_name} is incomplete; ${inventory_issue_message}")
            endif()
            message(STATUS "Skipping package ${group_name}; ${inventory_issue_message}")
            continue()
        endif()

        get_property(group_dependencies GLOBAL PROPERTY "ISAACSIM_GROUP_${group_name}_DEPENDENCIES")
        get_property(complete_groups GLOBAL PROPERTY ISAACSIM_COMPLETE_GROUP_NAMES)
        set(unavailable_dependencies)
        foreach(dependency IN LISTS group_dependencies)
            if(NOT dependency IN_LIST complete_groups)
                list(APPEND unavailable_dependencies "${dependency}")
            endif()
        endforeach()
        if(unavailable_dependencies)
            string(JOIN ", " unavailable_dependency_message ${unavailable_dependencies})
            if(ISAACSIM_ENFORCE_COMPLETE_PACKAGES OR ISAACSIM_PACKAGE_GROUP STREQUAL group_name)
                message(FATAL_ERROR
                    "Distribution group ${group_name} has unpackaged dependencies: ${unavailable_dependency_message}"
                )
            endif()
            message(STATUS "Skipping package ${group_name}; unpackaged dependencies: ${unavailable_dependency_message}")
            continue()
        endif()
        set_property(GLOBAL APPEND PROPERTY ISAACSIM_COMPLETE_GROUP_NAMES "${group_name}")

        get_property(cmake_dependencies GLOBAL PROPERTY "ISAACSIM_GROUP_${group_name}_CMAKE_DEPENDENCIES")
        list(REMOVE_DUPLICATES cmake_dependencies)
        set(config_find_dependencies "")
        set(manifest_dependencies "")
        set(dependency_separator "")
        foreach(dependency_name IN LISTS group_dependencies)
            get_property(dependency_version GLOBAL PROPERTY
                "ISAACSIM_GROUP_${group_name}_DEPENDENCY_${dependency_name}_VERSION"
            )
            get_property(dependency_specifier GLOBAL PROPERTY
                "ISAACSIM_GROUP_${group_name}_DEPENDENCY_${dependency_name}_SPECIFIER"
            )
            string(APPEND config_find_dependencies
                "find_dependency(${dependency_name} CONFIG)\n"
                "if(NOT DEFINED ${dependency_name}_VERSION OR NOT ${dependency_name}_VERSION STREQUAL "
                "\"${dependency_version}\")\n"
                "    set(${group_name}_FOUND FALSE)\n"
                "    set(${group_name}_NOT_FOUND_MESSAGE \"${group_name} requires ${dependency_name} exactly "
                "${dependency_version}\")\n"
                "    return()\n"
                "endif()\n"
            )
            string(APPEND manifest_dependencies
                "${dependency_separator}\n    \"${dependency_name}\": \"${dependency_specifier}\""
            )
            set(dependency_separator ",")
        endforeach()
        foreach(cmake_dependency IN LISTS cmake_dependencies)
            string(APPEND config_find_dependencies "find_dependency(${cmake_dependency})\n")
        endforeach()

        get_property(group_targets GLOBAL PROPERTY "ISAACSIM_GROUP_${group_name}_TARGETS")
        list(REMOVE_DUPLICATES group_targets)
        set(package_build_dir "${CMAKE_BINARY_DIR}/packages/${group_name}")
        file(MAKE_DIRECTORY "${package_build_dir}")
        set(package_cmake_dir "${CMAKE_INSTALL_LIBDIR}/cmake/${group_name}")
        if(group_targets)
            set(ISAACSIM_CONFIG_PACKAGE_NAME "${group_name}")
            set(ISAACSIM_CONFIG_PACKAGE_VERSION "${group_version}")
            set(ISAACSIM_CONFIG_FIND_DEPENDENCIES "${config_find_dependencies}")
            configure_package_config_file(
                "${_ISAACSIM_CMAKE_HELPER_DIR}/IsaacSimPackageConfig.cmake.in"
                "${package_build_dir}/${group_name}Config.cmake"
                INSTALL_DESTINATION "${package_cmake_dir}"
            )
            string(REGEX MATCH "^[0-9]+" ISAACSIM_CONFIG_PACKAGE_MAJOR "${group_release_version}")
            string(REGEX MATCH "^[0-9]+\\.([0-9]+)" unused "${group_release_version}")
            set(ISAACSIM_CONFIG_PACKAGE_MINOR "${CMAKE_MATCH_1}")
            set(ISAACSIM_CONFIG_PACKAGE_RELEASE_VERSION "${group_release_version}")
            configure_file(
                "${_ISAACSIM_CMAKE_HELPER_DIR}/IsaacSimPackageConfigVersion.cmake.in"
                "${package_build_dir}/${group_name}ConfigVersion.cmake"
                @ONLY
            )
            _isaacsim_get_group_component("${group_name}" development development_component)
            install(EXPORT "${group_name}Targets"
                FILE "${group_name}Targets.cmake"
                NAMESPACE isaacsim::
                DESTINATION "${package_cmake_dir}"
                COMPONENT "${development_component}"
            )
            install(FILES
                "${package_build_dir}/${group_name}Config.cmake"
                "${package_build_dir}/${group_name}ConfigVersion.cmake"
                DESTINATION "${package_cmake_dir}"
                COMPONENT "${development_component}"
            )
            _isaacsim_install_runtime_exports(
                "${group_name}" "${package_build_dir}" "${package_cmake_dir}" "${development_component}"
            )
        endif()

        get_property(python_imports GLOBAL PROPERTY "ISAACSIM_GROUP_${group_name}_PYTHON_IMPORTS")
        list(REMOVE_DUPLICATES python_imports)
        _isaacsim_make_json_array(manifest_modules ${registered_modules})
        _isaacsim_make_json_array(manifest_python_imports ${python_imports})
        _isaacsim_get_group_component("${group_name}" runtime runtime_component)
        _isaacsim_get_group_component("${group_name}" development development_component)
        _isaacsim_get_group_component("${group_name}" python python_component)
        _isaacsim_get_group_component("${group_name}" documentation documentation_component)
        set(documentation_destination "${CMAKE_INSTALL_DATADIR}/isaacsim/packages/${group_name}")

        install(FILES "${ISAACSIM_LIBRARIES_VERSION_FILE}"
            DESTINATION "${documentation_destination}"
            COMPONENT "${documentation_component}"
        )
        get_property(group_docs_dir GLOBAL PROPERTY "ISAACSIM_GROUP_${group_name}_DOCS_DIR")
        if(group_docs_dir)
            install(DIRECTORY "${group_docs_dir}/"
                DESTINATION "${documentation_destination}"
                COMPONENT "${documentation_component}"
            )
        endif()
        foreach(module_name IN LISTS registered_modules)
            string(MAKE_C_IDENTIFIER "${module_name}" module_key)
            get_property(module_changelog GLOBAL PROPERTY "ISAACSIM_MODULE_${module_key}_CHANGELOG")
            get_property(module_docs_dir GLOBAL PROPERTY "ISAACSIM_MODULE_${module_key}_DOCS_DIR")
            set(module_documentation_destination "${documentation_destination}/modules/${module_name}")
            install(FILES "${module_changelog}"
                DESTINATION "${module_documentation_destination}"
                COMPONENT "${documentation_component}"
            )
            if(module_docs_dir)
                install(DIRECTORY "${module_docs_dir}/"
                    DESTINATION "${module_documentation_destination}"
                    COMPONENT "${documentation_component}"
                )
            endif()
        endforeach()

        set(ISAACSIM_MANIFEST_NAME "${group_name}")
        set(ISAACSIM_MANIFEST_VERSION "${group_version}")
        set(ISAACSIM_MANIFEST_DEPENDENCIES "${manifest_dependencies}")
        set(ISAACSIM_MANIFEST_MODULES "${manifest_modules}")
        set(ISAACSIM_MANIFEST_PYTHON_IMPORTS "${manifest_python_imports}")
        set(ISAACSIM_MANIFEST_RUNTIME_COMPONENT "${runtime_component}")
        set(ISAACSIM_MANIFEST_DEVELOPMENT_COMPONENT "${development_component}")
        set(ISAACSIM_MANIFEST_PYTHON_COMPONENT "${python_component}")
        set(ISAACSIM_MANIFEST_DOCUMENTATION_COMPONENT "${documentation_component}")
        configure_file(
            "${_ISAACSIM_CMAKE_HELPER_DIR}/IsaacSimPackageManifest.json.in"
            "${package_build_dir}/package.json"
            @ONLY
        )
        foreach(component IN ITEMS "${runtime_component}" "${development_component}")
            install(FILES
                "${package_build_dir}/package.json"
                "${ISAACSIM_LIBRARIES_VERSION_FILE}"
                DESTINATION "${CMAKE_INSTALL_DATADIR}/isaacsim/packages/${group_name}"
                COMPONENT "${component}"
            )
        endforeach()
        install(FILES
            "${package_build_dir}/package.json"
            "${ISAACSIM_LIBRARIES_VERSION_FILE}"
            DESTINATION "${CMAKE_INSTALL_DATADIR}/isaacsim/packages/${group_name}"
            COMPONENT "${python_component}"
        )
        install(FILES "${package_build_dir}/package.json"
            DESTINATION "${documentation_destination}"
            COMPONENT "${documentation_component}"
        )

        set(license_components
            "${runtime_component}"
            "${development_component}"
        )
        if(ISAACSIM_ENABLE_PYTHON)
            get_property(python_variant_components GLOBAL PROPERTY
                "ISAACSIM_GROUP_${group_name}_PYTHON_VARIANT_COMPONENTS"
            )
            list(APPEND license_components "${python_component}" ${python_variant_components})
        endif()
        list(REMOVE_DUPLICATES license_components)
        foreach(component IN LISTS license_components)
            install(FILES "${ISAACSIM_PYTHON_LICENSE_FILE}"
                DESTINATION "${CMAKE_INSTALL_DATADIR}/licenses/${group_name}"
                COMPONENT "${component}"
            )
        endforeach()

        if(ISAACSIM_ENABLE_PYTHON_BINDINGS AND NOT WIN32)
            get_property(python_runtime_targets GLOBAL PROPERTY
                "ISAACSIM_GROUP_${group_name}_PYTHON_RUNTIME_TARGETS"
            )
            list(REMOVE_DUPLICATES python_runtime_targets)
            if(python_runtime_targets)
                foreach(python_runtime_target IN LISTS python_runtime_targets)
                    # Install targets instead of copying their build-tree files so
                    # CMake applies the target's install RPATH to Python packages.
                    install(TARGETS ${python_runtime_target}
                        LIBRARY
                            DESTINATION "${ISAACSIM_PYTHON_INSTALL_DIR}/isaacsim/lib"
                            COMPONENT "${python_component}"
                            NAMELINK_SKIP
                    )
                endforeach()
            endif()
        endif()
    endforeach()
    _isaacsim_generate_documentation_catalog()
endfunction()
