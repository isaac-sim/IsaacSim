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

# OpenUSD (with Python support)
if(ISAACSIM_ENABLE_PYTHON_BINDINGS)
    set(Python3_EXECUTABLE "${Python_EXECUTABLE}" CACHE FILEPATH "Python3 interpreter" FORCE)
    if(WIN32)
        set(Python3_LIBRARY "${Python_ROOT_DIR}/libs/python312.lib" CACHE FILEPATH "Python3 library" FORCE)
    else()
        set(
            Python3_LIBRARY
            "${Python_ROOT_DIR}/lib/libpython3.12.so"
            CACHE FILEPATH
            "Python3 library"
            FORCE
        )
    endif()
    set(Python3_INCLUDE_DIR "${Python_INCLUDE_DIRS}" CACHE PATH "Python3 include dir" FORCE)
    set(Python3_VERSION "${Python_VERSION}" CACHE STRING "Python3 version" FORCE)
    set(PXR_FIND_TBB_IN_CONFIG OFF CACHE BOOL "" FORCE)
    set(PXR_FIND_OPENSUBDIV_IN_CONFIG OFF CACHE BOOL "" FORCE)
    if(NOT TARGET TBB::tbb)
        add_library(TBB::tbb SHARED IMPORTED)
        if(WIN32)
            set_target_properties(TBB::tbb PROPERTIES
                IMPORTED_LOCATION "${ISAACSIM_TARGET_DEPS_DIR}/openusd/bin/tbb.dll"
                IMPORTED_IMPLIB "${ISAACSIM_TARGET_DEPS_DIR}/openusd/lib/tbb.lib"
                INTERFACE_INCLUDE_DIRECTORIES "${ISAACSIM_TARGET_DEPS_DIR}/openusd/include"
                # The OpenUSD package contains release TBB binaries. Explicitly
                # match their mode in Debug builds and avoid legacy TBB's broken
                # empty-_DEBUG detection with current MSVC preprocessors.
                INTERFACE_COMPILE_DEFINITIONS
                    "TBB_USE_DEBUG=0;TBB_USE_ASSERT=0;__TBB_NO_IMPLICIT_LINKAGE=1")
        else()
            set_target_properties(TBB::tbb PROPERTIES
                IMPORTED_LOCATION "${ISAACSIM_TARGET_DEPS_DIR}/openusd/lib/libtbb.so.2"
                IMPORTED_SONAME "libtbb.so.2"
                INTERFACE_INCLUDE_DIRECTORIES "${ISAACSIM_TARGET_DEPS_DIR}/openusd/include"
            )
        endif()
    endif()
    set(MaterialX_DIR "${ISAACSIM_TARGET_DEPS_DIR}/openusd/lib/cmake/MaterialX" CACHE PATH "" FORCE)
    file(WRITE "${CMAKE_BINARY_DIR}/cmake/imath-stub/ImathConfig.cmake" "# no-op")
    set(Imath_DIR "${CMAKE_BINARY_DIR}/cmake/imath-stub" CACHE PATH "" FORCE)
    find_package(pxr CONFIG REQUIRED
        PATHS "${ISAACSIM_TARGET_DEPS_DIR}/openusd"
        NO_DEFAULT_PATH
    )
    if(WIN32)
        # The Windows package exports its original build-machine Python include
        # path on several component targets. Replace that non-relocatable entry
        # with the Python dependency selected above.
        foreach(_pxr_target python tf gf vt ar sdf pcp usd)
            foreach(_include_property INTERFACE_INCLUDE_DIRECTORIES INTERFACE_SYSTEM_INCLUDE_DIRECTORIES)
                get_target_property(_pxr_includes ${_pxr_target} ${_include_property})
                if(_pxr_includes AND NOT _pxr_includes MATCHES "-NOTFOUND$")
                    list(FILTER _pxr_includes EXCLUDE REGEX "^[A-Za-z]:/g/")
                    list(APPEND _pxr_includes ${Python_INCLUDE_DIRS})
                    list(REMOVE_DUPLICATES _pxr_includes)
                    set_target_properties(${_pxr_target} PROPERTIES
                        ${_include_property} "${_pxr_includes}")
                endif()
            endforeach()
        endforeach()
    endif()
# OpenUSD (no Python)
else()
    set(PXR_FIND_TBB_IN_CONFIG OFF CACHE BOOL "" FORCE)
    if(NOT TARGET TBB::tbb)
        add_library(TBB::tbb SHARED IMPORTED)
        set_target_properties(TBB::tbb PROPERTIES
            IMPORTED_LOCATION             "${ISAACSIM_TARGET_DEPS_DIR}/openusd_no_python/lib/libtbb.so.2"
            IMPORTED_SONAME               "libtbb.so.2"
            INTERFACE_INCLUDE_DIRECTORIES "${ISAACSIM_TARGET_DEPS_DIR}/openusd_no_python/include"
        )
    endif()
    find_package(pxr CONFIG REQUIRED
        PATHS "${ISAACSIM_TARGET_DEPS_DIR}/openusd_no_python"
        NO_DEFAULT_PATH
    )
endif()

isaacsim_add_module(
    NAME isaacsim.foundation.usd.openusd
    EXPORT_HEADER isaacsim/foundation/usd/openusd/Export.h
    EXPORT_MACRO ISAACSIM_FOUNDATION_USD_OPENUSD_API
    PUBLIC_DEPENDENCIES
        isaacsim::common-array
        isaacsim::common-exceptions
    PRIVATE_DEPENDENCIES
        isaacsim::common-logging
    PUBLIC_CPP_HEADERS
        isaacsim/foundation/usd/openusd/Usd.hpp
    SOURCES
        src/details/UsdHelpers.hpp
        src/details/UsdHelpers.cpp
        src/Schemas.cpp
        src/Usd.cpp
)

if(ISAACSIM_ENABLE_PYTHON_BINDINGS)
    # Foundation Objects imports these packages to register their USD schemas.
    # Ship them with the Foundation Python component so installed applications
    # do not depend on the build's target-dependency directory.
    _isaacsim_get_group_component("isaacsim_foundation" python isaacsim_foundation_python_component)
    foreach(isaacsim_foundation_schema_package physx_usd_schemas newton_usd_schemas)
        install(DIRECTORY "${ISAACSIM_NATIVE_RUNTIME_DEPS_DIR}/${isaacsim_foundation_schema_package}"
            DESTINATION "${ISAACSIM_PYTHON_INSTALL_DIR}"
            COMPONENT "${isaacsim_foundation_python_component}"
            PATTERN "__pycache__" EXCLUDE
            PATTERN ".pytest_cache" EXCLUDE
            PATTERN ".mypy_cache" EXCLUDE
            PATTERN "*.pyc" EXCLUDE
            PATTERN "*.pyo" EXCLUDE
        )
    endforeach()
endif()

if(MSVC)
    # The Windows OpenUSD and TBB dependencies provide release-CRT binaries
    # only. Keep Debug symbols and optimization settings while matching their
    # runtime library to avoid requesting unavailable debug import libraries.
    set_property(TARGET isaacsim-foundation-usd-openusd PROPERTY
        MSVC_RUNTIME_LIBRARY MultiThreadedDLL)
endif()

target_compile_definitions(isaacsim-foundation-usd-openusd PRIVATE TBB_SUPPRESS_DEPRECATED_MESSAGES=1)
if(NOT APPLE AND NOT WIN32)
    # OpenUSD still includes libstdc++'s legacy ext/hash_set compatibility
    # header. Keep that dependency-local diagnostic out of library builds.
    target_compile_options(isaacsim-foundation-usd-openusd PRIVATE -Wno-deprecated)
endif()

target_include_directories(isaacsim-foundation-usd-openusd SYSTEM PRIVATE
    "${PXR_INCLUDE_DIRS}"
    $<$<BOOL:${ISAACSIM_ENABLE_PYTHON_BINDINGS}>:${Python_INCLUDE_DIRS}>
)

if(ISAACSIM_ENABLE_PYTHON_BINDINGS)
    set(_isaacsim_openusd_components
        usd
        usdGeom
        usdPhysics
        usdUtils
    )
    if(WIN32)
        # PE/COFF does not support unresolved shared-library symbols. The OpenUSD
        # package provides component import libraries. Native consumers do not
        # run through usd-core's Python DLL-directory initialization, so package
        # the imported shared-library closure needed by the linked components.
        target_link_libraries(isaacsim-foundation-usd-openusd PRIVATE
            ${_isaacsim_openusd_components}
        )
    else()
        # USD symbols are loaded by Python usd-core at runtime; no direct link needed.
        target_link_options(isaacsim-foundation-usd-openusd PRIVATE
            "LINKER:--allow-shlib-undefined"
        )
    endif()

    _isaacsim_collect_imported_runtime_targets(
        _isaacsim_openusd_runtime_targets
        ${_isaacsim_openusd_components}
        Python3::Python
    )
    isaacsim_install_runtime_dependencies(
        MODULE isaacsim.foundation.usd.openusd
        TARGETS ${_isaacsim_openusd_runtime_targets}
        NATIVE_ONLY
    )
    isaacsim_add_runtime_directory(
        MODULE isaacsim.foundation.usd.openusd
        SOURCE "${ISAACSIM_TARGET_DEPS_DIR}/openusd/lib/usd"
        DESTINATION usd
        WINDOWS_INSTALL_DESTINATION "${CMAKE_INSTALL_BINDIR}/usd"
        NATIVE_ONLY
    )

    if(UNIX AND NOT APPLE)
        # Terminal executables do not have a Python process to preload
        # usd-exchange. Keep their Linux USD link closure owned by Foundation.
        add_library(isaacsim-foundation-usd-openusd-standalone INTERFACE)
        add_library(isaacsim::foundation-usd-openusd-standalone ALIAS isaacsim-foundation-usd-openusd-standalone)
        target_link_libraries(isaacsim-foundation-usd-openusd-standalone INTERFACE
            "$<TARGET_FILE:usd>"
            "$<TARGET_FILE:usdGeom>"
            "$<TARGET_FILE:usdPhysics>"
            "$<TARGET_FILE:usdUtils>"
            "$<TARGET_FILE:Python3::Python>"
        )
        target_link_options(isaacsim-foundation-usd-openusd-standalone INTERFACE
            "LINKER:-rpath-link,${ISAACSIM_TARGET_DEPS_DIR}/openusd/lib"
        )
    endif()
else()
    target_link_directories(isaacsim-foundation-usd-openusd PRIVATE
        "$<TARGET_FILE_DIR:isaacsim-foundation-usd-openusd>"
    )
    target_link_libraries(isaacsim-foundation-usd-openusd PRIVATE
        tbb
        usd_ms
    )
    # Copy USD shared libraries and plugin descriptors alongside the module .so.
    file(GLOB isaacsim_openusd_shared_libs
        "${ISAACSIM_TARGET_DEPS_DIR}/openusd_no_python/lib/libusd*.so*"
        "${ISAACSIM_TARGET_DEPS_DIR}/openusd_no_python/lib/libtbb.so*"
    )
    if(NOT isaacsim_openusd_shared_libs)
        message(FATAL_ERROR
            "isaacsim.foundation.usd.openusd: OpenUSD libraries not found in "
            "'${ISAACSIM_TARGET_DEPS_DIR}/openusd_no_python/lib/'"
        )
    endif()
    set(isaacsim_openusd_copy_commands)
    foreach(isaacsim_openusd_lib ${isaacsim_openusd_shared_libs})
        get_filename_component(isaacsim_openusd_lib_name "${isaacsim_openusd_lib}" NAME)
        list(APPEND isaacsim_openusd_copy_commands
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${isaacsim_openusd_lib}"
                "$<TARGET_FILE_DIR:isaacsim-foundation-usd-openusd>/${isaacsim_openusd_lib_name}"
        )
    endforeach()
    add_custom_command(TARGET isaacsim-foundation-usd-openusd PRE_BUILD
        ${isaacsim_openusd_copy_commands}
    )
    install(FILES ${isaacsim_openusd_shared_libs}
        DESTINATION "${CMAKE_INSTALL_LIBDIR}"
        COMPONENT "isaacsim_foundation-runtime"
    )
    isaacsim_add_runtime_directory(
        MODULE isaacsim.foundation.usd.openusd
        SOURCE "${ISAACSIM_TARGET_DEPS_DIR}/openusd_no_python/lib/usd"
        DESTINATION usd
    )
endif()

# The native library registers these codeless schemas from a path beside itself in both Python-enabled and native-only
# builds. The Python component additionally installs the importable schema packages used by Python applications.
isaacsim_add_runtime_directory(
    MODULE isaacsim.foundation.usd.openusd
    SOURCE "${ISAACSIM_NATIVE_RUNTIME_DEPS_DIR}/physx_usd_schemas"
    DESTINATION third-party-schemas/physx_usd_schemas
    WINDOWS_INSTALL_DESTINATION "${CMAKE_INSTALL_BINDIR}/third-party-schemas/physx_usd_schemas"
)
isaacsim_add_runtime_directory(
    MODULE isaacsim.foundation.usd.openusd
    SOURCE "${ISAACSIM_NATIVE_RUNTIME_DEPS_DIR}/newton_usd_schemas"
    DESTINATION third-party-schemas/newton_usd_schemas
    WINDOWS_INSTALL_DESTINATION "${CMAKE_INSTALL_BINDIR}/third-party-schemas/newton_usd_schemas"
)
isaacsim_add_cpp_tests(
    MODULE isaacsim.foundation.usd.openusd
    REQUIRES_NO_BINDINGS
    RESOURCE_DIR resources/tests/isaacsim.foundation.usd.openusd
    SOURCES
        tests/cpp/Attribute.cpp
        tests/cpp/Prim.cpp
        tests/cpp/Schemas.cpp
        tests/cpp/SdfPath.cpp
        tests/cpp/Stage.cpp
        tests/cpp/Xform.cpp
    LABELS
        unit
)

if(BUILD_TESTING AND NOT ISAACSIM_ENABLE_PYTHON_BINDINGS)
    target_compile_definitions(tests-isaacsim-foundation-usd-openusd PRIVATE TBB_SUPPRESS_DEPRECATED_MESSAGES=1)
    if(NOT APPLE AND NOT WIN32)
        target_compile_options(tests-isaacsim-foundation-usd-openusd PRIVATE -Wno-deprecated)
    endif()
    # Schemas.cpp queries PXR_NS::UsdSchemaRegistry directly; the module links
    # usd_ms and tbb as PRIVATE so we must repeat them here for the test binary.
    target_link_directories(
        tests-isaacsim-foundation-usd-openusd
        PRIVATE "$<TARGET_FILE_DIR:isaacsim-foundation-usd-openusd>"
    )
    target_link_libraries(tests-isaacsim-foundation-usd-openusd PRIVATE tbb usd_ms)
endif()

isaacsim_add_python_module(
    NAME isaacsim.foundation.usd.openusd
)
