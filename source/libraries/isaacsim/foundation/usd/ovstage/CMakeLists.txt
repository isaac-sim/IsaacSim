# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

if(NOT TARGET ovstage::ovstage)
    set(ovstage_DIR
        "${ISAACSIM_NATIVE_RUNTIME_DEPS_DIR}/ovstage/lib/cmake/ovstage"
        CACHE PATH
        "Directory containing the selected ovstage package configuration"
        FORCE
    )
    find_package(ovstage REQUIRED
        PATHS "${ovstage_DIR}"
        NO_DEFAULT_PATH
    )
    set_property(TARGET ovstage::ovstage PROPERTY IMPORTED_GLOBAL TRUE)
endif()

isaacsim_add_module(
    NAME isaacsim.foundation.usd.ovstage
    EXPORT_HEADER isaacsim/foundation/usd/ovstage/Export.h
    EXPORT_MACRO ISAACSIM_FOUNDATION_USD_OVSTAGE_API
    PUBLIC_DEPENDENCIES
        isaacsim::common-array
        isaacsim::common-exceptions
    PRIVATE_DEPENDENCIES
        isaacsim::common-logging
        ovstage::ovstage
    PUBLIC_CPP_HEADERS
        isaacsim/foundation/usd/ovstage/Usd.hpp
    SOURCES
        src/details/UsdHelpers.hpp
        src/details/UsdHelpers.cpp
        src/Usd.cpp
)

get_target_property(isaacsim_ovstage_runtime ovstage::ovstage IMPORTED_LOCATION)
if(NOT isaacsim_ovstage_runtime OR isaacsim_ovstage_runtime MATCHES "-NOTFOUND$")
    message(FATAL_ERROR "ovstage::ovstage does not declare its runtime library location")
endif()
get_filename_component(isaacsim_ovstage_runtime_dir "${isaacsim_ovstage_runtime}" DIRECTORY)
isaacsim_install_runtime_dependencies(
    MODULE isaacsim.foundation.usd.ovstage
    TARGETS ovstage::ovstage
)
isaacsim_add_runtime_directory(
    MODULE isaacsim.foundation.usd.ovstage
    SOURCE "${isaacsim_ovstage_runtime_dir}/plugins"
    DESTINATION plugins
    WINDOWS_INSTALL_DESTINATION "${CMAKE_INSTALL_BINDIR}/plugins"
)
isaacsim_add_runtime_directory(
    MODULE isaacsim.foundation.usd.ovstage
    SOURCE "${isaacsim_ovstage_runtime_dir}/ovstage_usd_schemas"
    DESTINATION ovstage_usd_schemas
    WINDOWS_INSTALL_DESTINATION "${CMAKE_INSTALL_BINDIR}/ovstage_usd_schemas"
)

isaacsim_add_cpp_tests(
    MODULE isaacsim.foundation.usd.ovstage
    REQUIRES_NO_BINDINGS
    DEPENDENCIES
        ovstage::ovstage
    RESOURCE_DIR resources/tests/isaacsim.foundation.usd.ovstage
    SOURCES
        tests/cpp/Attribute.cpp
        tests/cpp/Prim.cpp
        tests/cpp/SdfPath.cpp
        tests/cpp/Stage.cpp
        tests/cpp/Xform.cpp
    LABELS
        unit
)

isaacsim_add_python_module(
    NAME isaacsim.foundation.usd.ovstage
)
