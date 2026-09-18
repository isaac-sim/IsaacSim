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

# ovphysx simulation backend module.
#
# Wraps the ovphysx C++ SDK behind isaacsim.physics.manager. ovstage comes either from
# the SDK (older packages bundled it) or from an app-supplied ovstage wheel, selected
# with ISAACSIM_OVSTAGE_ROOT.
# The SDK's private, namespaced USD and ovstage runtime is bundled under an
# OvPhysX-owned directory so the physics distribution remains independent from
# isaacsim.foundation and the two packages cannot overwrite one another.

set(ISAACSIM_OVPHYSX_ROOT "" CACHE PATH
    "ovphysx SDK root. Empty uses the Pixi-locked public release archive.")
set(ISAACSIM_OVSTAGE_ROOT "" CACHE PATH
    "Root of an app-supplied ovstage package (the `ovstage` directory of an\
 installed ovstage wheel). Empty uses the ovstage bundled in the ovphysx SDK, or\
 the Pixi-locked ovstage wheel when the SDK ships none.")

set(_ovphysx_dep "${ISAACSIM_PUBLIC_DEPS_ROOT}/ovphysx")

if(ISAACSIM_OVPHYSX_ROOT)
    set(_ovphysx_dep "${ISAACSIM_OVPHYSX_ROOT}")
elseif(DEFINED ENV{ISAACSIM_OVPHYSX_ROOT})
    set(_ovphysx_dep "$ENV{ISAACSIM_OVPHYSX_ROOT}")
endif()

set(_ovstage_root "${ISAACSIM_OVSTAGE_ROOT}")
if(NOT _ovstage_root AND DEFINED ENV{ISAACSIM_OVSTAGE_ROOT})
    set(_ovstage_root "$ENV{ISAACSIM_OVSTAGE_ROOT}")
endif()
# An SDK that no longer bundles OVStage falls back to the one the repository already
# fetches as a Pixi-locked wheel, which isaacsim.foundation.usd.ovstage also builds
# against -- so both reach the same library rather than two copies in one process.
if(WIN32)
    set(_sdk_ovstage_library "${_ovphysx_dep}/bin/ovstage.dll")
else()
    set(_sdk_ovstage_library "${_ovphysx_dep}/lib/libovstage.so")
endif()
if(NOT _ovstage_root AND NOT EXISTS "${_sdk_ovstage_library}")
    set(_ovstage_root "${ISAACSIM_NATIVE_RUNTIME_DEPS_DIR}/ovstage")
endif()

# Both roots are supplied by the caller, so on Windows they can arrive in native
# form. A backslash path reaches install(DIRECTORY) verbatim and is re-parsed as
# CMake code in the generated cmake_install.cmake, where "C:\Users\..." fails with
# "Invalid character escape '\U'". Only -D of a PATH cache entry gets normalized
# for free; the environment fallbacks above do not.
file(TO_CMAKE_PATH "${_ovphysx_dep}" _ovphysx_dep)
if(_ovstage_root)
    file(TO_CMAKE_PATH "${_ovstage_root}" _ovstage_root)
endif()

# ovstage is resolved BEFORE ovphysx: ovphysxConfig.cmake does find_dependency(ovstage),
# so an app-supplied package has to be locatable by the time find_package(ovphysx) runs.
# SDKs that still bundle ovstage export ovphysx::ovstage themselves; newer ones export an
# alias to ovstage::ovstage, so the same target name works either way.
if(_ovstage_root)
    if(NOT TARGET ovstage::ovstage)
        set(
            ovstage_DIR
            "${_ovstage_root}/lib/cmake/ovstage"
            CACHE PATH
            "Directory containing the selected ovstage package configuration"
            FORCE
        )
        find_package(ovstage REQUIRED
            PATHS "${ovstage_DIR}"
            NO_DEFAULT_PATH)
    endif()
    set(_ovstage_target ovstage::ovstage)
    # libovstage/ovstage.dll must keep its bin/ siblings: it RUNPATHs $ORIGIN/plugins,
    # and ovstage_create_instance loads omni.fabric.plugin from that same tree.
    set(_ovstage_runtime_dir "${_ovstage_root}/bin")
    # On Linux an SDK that bundled ovstage kept the library and its plugin tree under
    # lib/; an app-supplied package keeps both together in its own bin/.
    set(_ovstage_plugins_dir "${_ovstage_root}/bin/plugins")
    # libovphysx.so RPATHs $ORIGIN for its libovstage.so sibling, which an app-supplied
    # package no longer is; stubgen has to be able to find it at import time.
    set(ISAACSIM_BINDING_EXTRA_LIBRARY_PATH "${_ovstage_root}/bin")
else()
    set(_ovstage_target ovphysx::ovstage)
    set(_ovstage_runtime_dir "${_ovphysx_dep}/bin")
    set(_ovstage_plugins_dir "${_ovphysx_dep}/lib/plugins")
endif()

if(NOT TARGET ovphysx::ovphysx)
    find_package(ovphysx REQUIRED
        PATHS "${_ovphysx_dep}/lib/cmake/ovphysx"
        NO_DEFAULT_PATH)
endif()

# The SDK's imported target adds absolute SDK directories to every consumer's
# RUNPATH. Installed Isaac Sim libraries carry the required runtime beside
# themselves, so retain other interface options but remove those build-local paths.
if(UNIX AND NOT APPLE)
    get_target_property(_ovphysx_interface_link_options
        ovphysx::ovphysx INTERFACE_LINK_OPTIONS)
    if(_ovphysx_interface_link_options)
        list(FILTER _ovphysx_interface_link_options EXCLUDE REGEX "^-Wl,-rpath,")
        set_property(TARGET ovphysx::ovphysx PROPERTY
            INTERFACE_LINK_OPTIONS "${_ovphysx_interface_link_options}")
    endif()
endif()

set(_ovphysx_private_dependencies
    isaacsim::common-logging
    isaacsim::common-ovstage
    isaacsim::physics-manager
    ovphysx::ovphysx
    ${_ovstage_target})
set(_ovphysx_runtime_dependencies
    ovphysx::ovphysx
    ${_ovstage_target})
set(_ovphysx_system_includes
    "${ISAACSIM_PUBLIC_DEPS_INCLUDE_DIR}"
    "${_ovphysx_dep}/include"
    "${_ovphysx_dep}/include/ovphysx"
    "${_ovphysx_dep}/include/physx")

isaacsim_add_module(
    NAME isaacsim.physics_engines.ovphysx
    EXPORT_HEADER isaacsim/physics_engines/ovphysx/Export.h
    EXPORT_MACRO ISAACSIM_PHYSICS_ENGINES_OVPHYSX_API
    PUBLIC_CPP_HEADERS
        isaacsim/physics_engines/ovphysx/Backend.hpp
    SOURCES
        src/OvPhysxPlugin.cpp
        src/OvPhysxAdapter.cpp
        src/OvPhysxAdapter.hpp
        src/tensors/OvPhysxEntityViews.cpp
        src/tensors/OvPhysxEntityViews.hpp
        src/tensors/OvPhysxEntityFactories.cpp
        src/tensors/OvPhysxEntityFactories.hpp
        src/tensors/OvPhysxResultHelpers.hpp
        src/tensors/OvPhysxTensorHelpers.hpp
    PRIVATE_DEPENDENCIES
        ${_ovphysx_private_dependencies}
)

set(_ovphysx_target isaacsim-physics_engines-ovphysx)

# USD headers emit #warning directives (backward_warning.h); -Wno-cpp keeps them
# from tripping the module's warnings. DLPack comes from the locked Pixi build prefix.
if(NOT WIN32)
    target_compile_options(${_ovphysx_target} PRIVATE -Wno-cpp)
endif()
target_include_directories(${_ovphysx_target} SYSTEM PRIVATE ${_ovphysx_system_includes})

# Preserve the SDK's two-root loader layout below a physics-owned directory:
# lib/ovphysx/bin contains the SDK libraries and OVStage plugins, while
# lib/ovphysx/plugins contains the engine plugins resolved through ../plugins.
if(WIN32)
    isaacsim_install_runtime_dependencies(MODULE isaacsim.physics_engines.ovphysx
        TARGETS ${_ovphysx_runtime_dependencies})
    isaacsim_add_runtime_directory(MODULE isaacsim.physics_engines.ovphysx
        SOURCE "${_ovstage_plugins_dir}"
        DESTINATION plugins
        WINDOWS_INSTALL_DESTINATION "${CMAKE_INSTALL_BINDIR}/plugins")
    if(EXISTS "${_ovstage_runtime_dir}/ovstage_usd_schemas")
        isaacsim_add_runtime_directory(MODULE isaacsim.physics_engines.ovphysx
            SOURCE "${_ovstage_runtime_dir}/ovstage_usd_schemas"
            DESTINATION ovstage_usd_schemas
            WINDOWS_INSTALL_DESTINATION "${CMAKE_INSTALL_BINDIR}/ovstage_usd_schemas")
    endif()
    get_target_property(_ovphysx_group ${_ovphysx_target} ISAACSIM_GROUP_NAME)
    _isaacsim_get_group_component("${_ovphysx_group}" runtime _ovphysx_runtime_component)
    # Preserve the SDK's two-root Windows layout. OvPhysX requires config.toml
    # beside its DLL and derives engine plugins from the sibling ../plugins
    # directory, while OVStage keeps its private tree under bin/plugins. The
    # physics-engine distribution owns both so it remains usable without
    # isaacsim.foundation.
    install(FILES
        "${_ovphysx_dep}/bin/config.toml"
        "${_ovphysx_dep}/bin/ovphysx_internal.dll"
        DESTINATION "${CMAKE_INSTALL_BINDIR}"
        COMPONENT "${_ovphysx_runtime_component}")
    install(DIRECTORY "${_ovphysx_dep}/plugins/"
        DESTINATION plugins
        COMPONENT "${_ovphysx_runtime_component}")
else()
    set_property(TARGET ${_ovphysx_target} APPEND PROPERTY
        INSTALL_RPATH "${ISAACSIM_LOADER_ORIGIN}/ovphysx/bin")
    isaacsim_install_runtime_dependencies(MODULE isaacsim.physics_engines.ovphysx
        TARGETS ${_ovphysx_runtime_dependencies}
        DESTINATION "ovphysx/bin")
    isaacsim_add_runtime_directory(MODULE isaacsim.physics_engines.ovphysx
        SOURCE "${_ovstage_plugins_dir}"
        DESTINATION "ovphysx/bin/plugins")
    # ovstage resolves its USD schema tree beside the library it was loaded from, so it
    # follows the library rather than staying in the Python package. Only an
    # app-supplied ovstage ships one; an SDK that bundled ovstage carried its schemas
    # inside the plugin tree above.
    if(EXISTS "${_ovstage_runtime_dir}/ovstage_usd_schemas")
        isaacsim_add_runtime_directory(MODULE isaacsim.physics_engines.ovphysx
            SOURCE "${_ovstage_runtime_dir}/ovstage_usd_schemas"
            DESTINATION "ovphysx/bin/ovstage_usd_schemas")
    endif()
endif()
if(NOT WIN32)
    get_target_property(_ovphysx_group ${_ovphysx_target} ISAACSIM_GROUP_NAME)
    _isaacsim_get_group_component("${_ovphysx_group}" runtime _ovphysx_runtime_component)
    file(GLOB _ovphysx_api_libraries CONFIGURE_DEPENDS
        "${_ovphysx_dep}/lib/libovphysx.so*")
    file(GLOB _ovphysx_internal_libraries CONFIGURE_DEPENDS
        "${_ovphysx_dep}/lib/libovphysx_internal.so*")
    install(FILES
        "${_ovphysx_dep}/lib/config.toml"
        ${_ovphysx_api_libraries}
        ${_ovphysx_internal_libraries}
        DESTINATION "${CMAKE_INSTALL_LIBDIR}/ovphysx/bin"
        COMPONENT "${_ovphysx_runtime_component}")
    install(DIRECTORY "${_ovphysx_dep}/plugins/"
        DESTINATION "${CMAKE_INSTALL_LIBDIR}/ovphysx/plugins"
        COMPONENT "${_ovphysx_runtime_component}")

    if(ISAACSIM_ENABLE_PYTHON_BINDINGS)
        _isaacsim_get_group_component("${_ovphysx_group}" python _ovphysx_python_component)
        # libovphysx resolves its SDK plugin tree through $ORIGIN/../plugins,
        # while libovstage resolves the smaller tree through $ORIGIN/plugins.
        # Preserve both layouts in the standalone Python distribution.
        install(FILES
            "${_ovphysx_dep}/lib/config.toml"
            ${_ovphysx_api_libraries}
            ${_ovphysx_internal_libraries}
            DESTINATION "${ISAACSIM_PYTHON_INSTALL_DIR}/isaacsim/lib/ovphysx/bin"
            COMPONENT "${_ovphysx_python_component}")
        install(DIRECTORY "${_ovphysx_dep}/plugins/"
            DESTINATION "${ISAACSIM_PYTHON_INSTALL_DIR}/isaacsim/lib/ovphysx/plugins"
            COMPONENT "${_ovphysx_python_component}")
    endif()
endif()

if(ISAACSIM_ENABLE_PYTHON)
    # Bundle the ovstage Python data-plane package (used to build/attach USD stages).
    # isaacsim.physics_engines.ovstage.setup() adds ovstage_py/ to sys.path. The native
    # payload is dropped either way: ovstage is a runtime dependency of the backend, so
    # the library and its plugin tree are already staged beside libovphysx under
    # ovphysx/bin, which is where the loader hint points. Keeping a second copy here
    # would ship the same 5 MB library twice.
    set(_ovstage_py_curated "${CMAKE_CURRENT_BINARY_DIR}/ovstage_py")
    file(REMOVE_RECURSE "${_ovstage_py_curated}")
    if(_ovstage_root)
        set(_ovstage_py_source "${_ovstage_root}")
    else()
        set(_ovstage_py_source "${_ovphysx_dep}/python/ovstage")
    endif()
    file(COPY "${_ovstage_py_source}"
        DESTINATION "${_ovstage_py_curated}"
        PATTERN "bin" EXCLUDE
        PATTERN "include" EXCLUDE
        PATTERN "lib" EXCLUDE
        PATTERN "__pycache__" EXCLUDE)
    isaacsim_add_runtime_directory(MODULE isaacsim.physics_engines.ovphysx
        SOURCE "${_ovstage_py_curated}"
        DESTINATION "ovstage_py"
        WINDOWS_PYTHON_SHARED)
endif()

isaacsim_add_nanobind(
    MODULE isaacsim.physics_engines.ovphysx
    SOURCE bindings/python/BindingsOvPhysxPython.cpp
    WINDOWS_RUNTIME_SIBLING_DIRECTORY "${_ovphysx_dep}/plugins"
)
# Stubgen imports the binding through the public package, whose import activates the
# backend and calls `isaacsim.physics_engines.ovstage.setup()`. Ensure that package is
# staged first in a clean parallel build rather than relying on an existing build tree.
if(ISAACSIM_ENABLE_PYTHON_BINDINGS)
    add_dependencies(generate-${_ovphysx_target}-python-stub
        stage-isaacsim-physics_engines-ovstage-python-package)
endif()
if(WIN32 AND TARGET ${_ovphysx_target}-python)
    ovphysx_copy_runtime_dlls(${_ovphysx_target}-python)
    add_custom_command(TARGET ${_ovphysx_target}-python POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "$<TARGET_FILE:${_ovstage_target}>"
            "$<TARGET_FILE_DIR:${_ovphysx_target}-python>"
        COMMENT "ovphysx: copy binding-local ovstage runtime for standalone Python"
    )
    # Recreate the SDK's two-root layout exactly:
    #   bindings/         == ovstage runtime root (ovstage.dll and config)
    #   bindings/plugins/ == ovstage plugin tree
    #   plugins/          == SDK plugins (ovphysx; staged by isaacsim_add_nanobind)
    add_custom_command(TARGET ${_ovphysx_target}-python POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E remove_directory
            "$<TARGET_FILE_DIR:${_ovphysx_target}-python>/plugins"
        COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${_ovstage_runtime_dir}/plugins"
            "$<TARGET_FILE_DIR:${_ovphysx_target}-python>/plugins"
        COMMENT "ovphysx: restore distinct ovstage bin/plugins runtime"
    )
    if(_ovstage_root)
        # A bundled ovstage arrives with ovphysx_copy_runtime_dlls; an app-supplied one
        # does not, so put ovstage.dll beside ovphysx.dll (Windows resolves the import by
        # basename from the loading module's directory).
        add_custom_command(TARGET ${_ovphysx_target}-python POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${_ovstage_runtime_dir}/ovstage.dll"
                "$<TARGET_FILE_DIR:${_ovphysx_target}-python>"
            COMMENT "ovphysx: stage the app-supplied ovstage runtime beside the bindings"
        )
    endif()
    # The schema tree follows the library wherever it came from, and only some packages
    # ship one -- an SDK that bundled ovstage carried its schemas inside the plugin tree.
    if(EXISTS "${_ovstage_runtime_dir}/ovstage_usd_schemas")
        add_custom_command(TARGET ${_ovphysx_target}-python POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_directory
                "${_ovstage_runtime_dir}/ovstage_usd_schemas"
                "$<TARGET_FILE_DIR:${_ovphysx_target}-python>/ovstage_usd_schemas"
            COMMENT "ovphysx: stage the ovstage USD schema tree beside the bindings"
        )
    endif()
    get_target_property(_ovphysx_group ${_ovphysx_target} ISAACSIM_GROUP_NAME)
    _isaacsim_get_group_component("${_ovphysx_group}" python _ovphysx_python_component)
    # ovstage's plugin tree carries USD codegen templates, one of which is named
    # __init__.py. `pip install --target` byte-compiles it, so an app-supplied
    # package arrives with a __pycache__ inside the tree; copying that into the
    # Python package trips the install contract (no __pycache__ / *.pyc in an
    # installed package). Nothing here is importable, so drop the bytecode.
    install(DIRECTORY "${_ovstage_runtime_dir}/plugins/"
        DESTINATION "${ISAACSIM_PYTHON_INSTALL_DIR}/isaacsim/physics_engines/ovphysx/bindings/plugins"
        COMPONENT "${_ovphysx_python_component}"
        PATTERN "__pycache__" EXCLUDE
        PATTERN "*.pyc" EXCLUDE
        PATTERN "*.pyo" EXCLUDE)
    if(_ovstage_root)
        install(FILES "${_ovstage_runtime_dir}/ovstage.dll"
            DESTINATION "${ISAACSIM_PYTHON_INSTALL_DIR}/isaacsim/physics_engines/ovphysx/bindings"
            COMPONENT "${_ovphysx_python_component}")
    endif()
    if(EXISTS "${_ovstage_runtime_dir}/ovstage_usd_schemas")
        install(DIRECTORY "${_ovstage_runtime_dir}/ovstage_usd_schemas/"
            DESTINATION "${ISAACSIM_PYTHON_INSTALL_DIR}/isaacsim/physics_engines/ovphysx/bindings/ovstage_usd_schemas"
            COMPONENT "${_ovphysx_python_component}")
    endif()
    install(FILES
        "$<TARGET_FILE:${_ovstage_target}>"
        "${_ovphysx_dep}/bin/config.toml"
        "${_ovphysx_dep}/bin/ovphysx_internal.dll"
        DESTINATION "${ISAACSIM_PYTHON_INSTALL_DIR}/isaacsim/physics_engines/ovphysx/bindings"
        COMPONENT "${_ovphysx_python_component}")
endif()

isaacsim_add_python_tests(
    MODULE isaacsim.physics_engines.ovphysx
    REQUIRES_BINDINGS
    TEST_SUPPORT isaacsim.physics.manager
    RESOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../physics/manager/resources/tests"
)

# C++ tests: ovphysx SDK smoke + ovstage attach + tensor-helpers. They link the
# ovphysx SDK directly (the module lib's ovphysx dep is PRIVATE, non-transitive) and
# need the SDK headers, the private tensor-helpers header, and the USD/ovstage runtime.
set(_ovphysx_test_dependencies
    isaacsim::physics-manager
    ovphysx::ovphysx
    ${_ovstage_target})
if(NOT WIN32)
    # libov (monolithic USD, pulled in transitively) has NEEDED Python symbols;
    # link libpython so the test executable resolves them (as the umbrella did).
    list(APPEND _ovphysx_test_dependencies
        "${ISAACSIM_PUBLIC_DEPS_ROOT}/lib/libpython3.12.so")
endif()

isaacsim_add_cpp_tests(
    MODULE isaacsim.physics_engines.ovphysx
    SOURCES
        tests/cpp/TestOvPhysxCheckedStep.cpp
        tests/cpp/TestOvPhysxSmoke.cpp
        tests/cpp/TestOvPhysxOvstage.cpp
        tests/cpp/TestOvPhysxTensorHelpers.cpp
    DEPENDENCIES
        ${_ovphysx_test_dependencies}
    RESOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../physics/manager/resources/tests"
)
set(_ovphysx_test_target tests-isaacsim-physics_engines-ovphysx)
if(TARGET ${_ovphysx_test_target} AND NOT WIN32)
    target_compile_options(${_ovphysx_test_target} PRIVATE -Wno-cpp)
endif()
if(TARGET ${_ovphysx_test_target})
    target_include_directories(${_ovphysx_test_target} SYSTEM PRIVATE ${_ovphysx_system_includes})
    # TestOvPhysxTensorHelpers.cpp includes the module's private OvPhysxTensorHelpers.hpp.
    target_include_directories(${_ovphysx_test_target} PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src/tensors")
endif()
# TestOvPhysxOvstage.cpp loads CartPole.usda from the manager-owned shared test resources.
# The ovstage attach test loads the monolithic USD at runtime; the SDK libs self-locate
# their own plugins via RPATH, so add only the USD + ovphysx runtime dirs to the test.
if(TARGET ${_ovphysx_test_target} AND WIN32)
    ovphysx_copy_runtime_dlls(${_ovphysx_test_target})
    add_custom_command(TARGET ${_ovphysx_test_target} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "$<TARGET_FILE:${_ovstage_target}>"
            "$<TARGET_FILE_DIR:${_ovphysx_test_target}>"
        COMMAND ${CMAKE_COMMAND} -E remove_directory
            "$<TARGET_FILE_DIR:${_ovphysx_test_target}>/plugins"
        COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${_ovstage_runtime_dir}/plugins"
            "$<TARGET_FILE_DIR:${_ovphysx_test_target}>/plugins"
        COMMAND ${CMAKE_COMMAND} -E remove_directory
            "$<TARGET_FILE_DIR:${_ovphysx_test_target}>/../plugins"
        COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${_ovphysx_dep}/plugins"
            "$<TARGET_FILE_DIR:${_ovphysx_test_target}>/../plugins"
        COMMENT "ovphysx: stage distinct Windows ovstage and ovphysx plugin roots"
    )
    if(_ovstage_root)
        # The test links ovstage.lib, so the exe imports ovstage.dll by basename.
        # ovphysx_copy_runtime_dlls() deliberately skips ovstage.dll and an
        # app-supplied one is not in the SDK, so stage it beside the exe exactly as
        # the bindings do.
        add_custom_command(TARGET ${_ovphysx_test_target} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${_ovstage_runtime_dir}/ovstage.dll"
                "$<TARGET_FILE_DIR:${_ovphysx_test_target}>"
            COMMENT "ovphysx: stage the app-supplied ovstage runtime beside the tests"
        )
    endif()
    if(EXISTS "${_ovstage_runtime_dir}/ovstage_usd_schemas")
        add_custom_command(TARGET ${_ovphysx_test_target} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_directory
                "${_ovstage_runtime_dir}/ovstage_usd_schemas"
                "$<TARGET_FILE_DIR:${_ovphysx_test_target}>/ovstage_usd_schemas"
            COMMENT "ovphysx: stage the ovstage USD schema tree beside the tests"
        )
    endif()
    set_property(TEST ${_ovphysx_test_target} APPEND PROPERTY
        ENVIRONMENT_MODIFICATION
            "PATH=path_list_prepend:$<TARGET_FILE_DIR:${_ovphysx_test_target}>/plugins"
            # An app-supplied ovstage nests its runtime closure one level deeper
            # than the SDK's flat tree; omni_usd_resolver.dll imports omniclient.dll
            # from a sibling directory, so both need to be searchable.
            "PATH=path_list_prepend:$<TARGET_FILE_DIR:${_ovphysx_test_target}>/plugins/omni.client.lib"
            "PATH=path_list_prepend:$<TARGET_FILE_DIR:${_ovphysx_test_target}>/plugins/omni.usd_resolver"
            "PATH=path_list_prepend:$<TARGET_FILE_DIR:${_ovphysx_test_target}>/../plugins")
elseif(TARGET ${_ovphysx_test_target})
    set(_ovstage_rpath "")
    if(_ovstage_root)
        set(_ovstage_rpath ";${_ovstage_root}/bin")
    endif()
    set_property(TARGET ${_ovphysx_test_target} APPEND PROPERTY BUILD_RPATH
        "${_ovphysx_dep}/lib;${_ovphysx_dep}/plugins${_ovstage_rpath}")
    # libov uses DT_RUNPATH (non-transitive), so it can't see the test's rpath when it pulls
    # libpython; give ctest a global LD_LIBRARY_PATH covering the OvPhysX and Python
    # runtime directories so every transitive load resolves.
    set(_ovstage_ldpath "")
    if(_ovstage_root)
        set(_ovstage_ldpath ":${_ovstage_root}/bin")
    endif()
    set(_ovphysx_test_ld_library_path
        "${_ovphysx_dep}/lib:${_ovphysx_dep}/plugins:${ISAACSIM_PUBLIC_DEPS_ROOT}/lib${_ovstage_ldpath}")
    set_tests_properties(${_ovphysx_test_target} PROPERTIES
        ENVIRONMENT
            "LD_LIBRARY_PATH=${_ovphysx_test_ld_library_path}"
    )
endif()
