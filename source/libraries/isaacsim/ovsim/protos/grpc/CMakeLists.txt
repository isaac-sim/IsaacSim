# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

if(WIN32)
    set(_grpc_dependency_root "${ISAACSIM_PUBLIC_DEPS_ROOT}/Library")
else()
    set(_grpc_dependency_root "${ISAACSIM_PUBLIC_DEPS_ROOT}")
endif()
set(_grpc_cmake_root "${_grpc_dependency_root}/lib/cmake")

# gRPC's package config discovers these through CMake find modules. Point the
# modules at the same locked prefix as gRPC so the linked and shipped runtimes
# cannot come from different installations.
set(OPENSSL_ROOT_DIR "${_grpc_dependency_root}")
set(ZLIB_ROOT "${_grpc_dependency_root}")
if(UNIX AND NOT APPLE)
    # The Pixi zlib runtime intentionally has no unversioned linker symlink,
    # so FindZLIB would otherwise fall through to the system installation.
    set(ZLIB_INCLUDE_DIR "${ISAACSIM_PUBLIC_DEPS_ROOT}/include")
    set(ZLIB_LIBRARY "${ISAACSIM_PUBLIC_DEPS_ROOT}/lib/libz.so.1")
endif()
find_package(OpenSSL REQUIRED)
find_package(ZLIB REQUIRED)
find_package(Protobuf CONFIG REQUIRED
    PATHS "${_grpc_cmake_root}/protobuf"
    NO_DEFAULT_PATH)
find_package(gRPC CONFIG REQUIRED
    PATHS "${_grpc_cmake_root}/grpc"
    NO_DEFAULT_PATH)

function(_ovsim_grpc_add_runtime_target target relative_path)
    set(_location "${_grpc_dependency_root}/${relative_path}")
    if(NOT EXISTS "${_location}")
        message(FATAL_ERROR "The locked gRPC runtime dependency is missing: ${_location}")
    endif()
    file(REAL_PATH "${_location}" _location)
    add_library("${target}" SHARED IMPORTED)
    set_target_properties("${target}" PROPERTIES IMPORTED_LOCATION "${_location}")
    if(NOT WIN32)
        cmake_path(GET relative_path FILENAME soname)
        set_property(TARGET "${target}" PROPERTY IMPORTED_SONAME "${soname}")
    endif()
endfunction()

set(_proto_source_dir "${CMAKE_CURRENT_SOURCE_DIR}/src/grpc")
set(_proto_generated_dir "${CMAKE_CURRENT_BINARY_DIR}/generated-proto")
if(WIN32)
    set(_proto_cpp_out "dllexport_decl=ISAACSIM_OVSIM_PROTOS_GRPC_API:${_proto_generated_dir}")
else()
    set(_proto_cpp_out "${_proto_generated_dir}")
endif()
set(_proto_relative_sources
    simulation.proto
    worldstate.proto
    google/rpc/status.proto)
set(_proto_sources)
set(_generated_sources)
set(_generated_grpc_sources)
set(_generated_headers)

foreach(_proto_relative_source IN LISTS _proto_relative_sources)
    list(APPEND _proto_sources "${_proto_source_dir}/${_proto_relative_source}")
    string(REGEX REPLACE "\\.proto$" "" _proto_name "${_proto_relative_source}")
    list(APPEND _generated_sources
        "${_proto_generated_dir}/${_proto_name}.pb.cc"
        "${_proto_generated_dir}/${_proto_name}.grpc.pb.cc")
    list(APPEND _generated_grpc_sources
        "${_proto_generated_dir}/${_proto_name}.grpc.pb.cc")
    list(APPEND _generated_headers
        "${_proto_generated_dir}/${_proto_name}.pb.h"
        "${_proto_generated_dir}/${_proto_name}.grpc.pb.h")
endforeach()

if(UNIX AND NOT APPLE)
    # Conda's release-built Abseil shared library omits the debug-only
    # absl::Mutex::Dtor symbol. Keep only generated gRPC translation units on
    # their dependency's release-compatible header path in Debug builds.
    set_source_files_properties(${_generated_grpc_sources} PROPERTIES
        COMPILE_DEFINITIONS "$<$<CONFIG:Debug>:NDEBUG>"
    )
endif()

add_custom_command(
    OUTPUT ${_generated_sources} ${_generated_headers}
    COMMAND ${CMAKE_COMMAND} -E make_directory "${_proto_generated_dir}"
    COMMAND protobuf::protoc
        "--proto_path=${_proto_source_dir}"
        "--proto_path=${ISAACSIM_PUBLIC_DEPS_INCLUDE_DIR}"
        "--cpp_out=${_proto_cpp_out}"
        "--grpc_out=${_proto_generated_dir}"
        "--plugin=protoc-gen-grpc=$<TARGET_FILE:gRPC::grpc_cpp_plugin>"
        ${_proto_sources}
    DEPENDS
        ${_proto_sources}
        protobuf::protoc
        gRPC::grpc_cpp_plugin
    VERBATIM
)

isaacsim_add_module(
    NAME isaacsim.ovsim.protos.grpc
    EXPORT_HEADER isaacsim/ovsim/protos/grpc/Export.h
    EXPORT_MACRO ISAACSIM_OVSIM_PROTOS_GRPC_API
    PUBLIC_CPP_HEADERS
        isaacsim/ovsim/protos/grpc/Protocol.hpp
    SOURCES
        ${_generated_sources}
        src/ValueCodec.cpp
        src/ValueCodec.hpp
        src/StatusCodec.cpp
        src/StatusCodec.hpp
    PUBLIC_DEPENDENCIES
        isaacsim::common-array
    PRIVATE_DEPENDENCIES
        gRPC::grpc++
        protobuf::libprotobuf
)

target_include_directories(isaacsim-ovsim-protos-grpc PUBLIC
    "$<BUILD_INTERFACE:${_proto_generated_dir}>"
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>"
    "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/ovsim/interfaces/cpp>"
)

# Generated protobuf classes form the build-private ABI shared by the client and
# server modules, so their symbols must remain visible from this shared library.
set_target_properties(isaacsim-ovsim-protos-grpc PROPERTIES
    CXX_VISIBILITY_PRESET default
    VISIBILITY_INLINES_HIDDEN NO
    WINDOWS_EXPORT_ALL_SYMBOLS YES
)
if(WIN32)
    # Protobuf's generator can decorate messages and descriptor data with our
    # module export macro, but it cannot emit the corresponding include. Force
    # the generated export header into this target and its private consumers.
    target_compile_options(isaacsim-ovsim-protos-grpc
        PRIVATE "/FIisaacsim/ovsim/protos/grpc/Export.h"
        INTERFACE "$<BUILD_INTERFACE:/FIisaacsim/ovsim/protos/grpc/Export.h>"
    )
endif()
if(UNIX AND NOT APPLE)
    target_link_libraries(isaacsim-ovsim-protos-grpc PUBLIC
        "$<BUILD_INTERFACE:${ISAACSIM_PUBLIC_DEPS_ROOT}/lib/libstdc++.so.6>"
        "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_LIBDIR}/libstdc++.so.6>"
    )
endif()

set(_ovsim_grpc_runtime_roots
    gRPC::grpc++
    gRPC::grpc
    gRPC::gpr
    gRPC::address_sorting
    protobuf::libprotobuf
    utf8_range::utf8_validity
    utf8_range::utf8_range
    c-ares::cares
    re2::re2
)

if(WIN32)
    _ovsim_grpc_add_runtime_target(ovsim-grpc-runtime-ssl "bin/libssl-3-x64.dll")
    _ovsim_grpc_add_runtime_target(ovsim-grpc-runtime-crypto "bin/libcrypto-3-x64.dll")
    _ovsim_grpc_add_runtime_target(ovsim-grpc-runtime-zlib "bin/zlib.dll")
    list(APPEND _ovsim_grpc_runtime_roots
        ovsim-grpc-runtime-ssl
        ovsim-grpc-runtime-crypto
        ovsim-grpc-runtime-zlib
    )
elseif(UNIX AND NOT APPLE)
    _ovsim_grpc_add_runtime_target(ovsim-grpc-runtime-ssl "lib/libssl.so.3")
    _ovsim_grpc_add_runtime_target(ovsim-grpc-runtime-crypto "lib/libcrypto.so.3")
    _ovsim_grpc_add_runtime_target(ovsim-grpc-runtime-zlib "lib/libz.so.1")
    _ovsim_grpc_add_runtime_target(ovsim-grpc-runtime-stdcxx "lib/libstdc++.so.6")
    _ovsim_grpc_add_runtime_target(ovsim-grpc-runtime-gcc "lib/libgcc_s.so.1")
    list(APPEND _ovsim_grpc_runtime_roots
        ovsim-grpc-runtime-ssl
        ovsim-grpc-runtime-crypto
        ovsim-grpc-runtime-zlib
        ovsim-grpc-runtime-stdcxx
        ovsim-grpc-runtime-gcc
    )
    if(EXISTS "${ISAACSIM_PUBLIC_DEPS_ROOT}/lib/libatomic.so.1")
        _ovsim_grpc_add_runtime_target(ovsim-grpc-runtime-atomic "lib/libatomic.so.1")
        list(APPEND _ovsim_grpc_runtime_roots ovsim-grpc-runtime-atomic)
    endif()
endif()

# The public gRPC package is a shared-library family. Install the complete
# imported closure so Abseil and protocol dependencies do not leak from Pixi.
_isaacsim_collect_imported_runtime_targets(
    _ovsim_grpc_runtime_targets
    ${_ovsim_grpc_runtime_roots}
)
isaacsim_install_runtime_dependencies(
    MODULE isaacsim.ovsim.protos.grpc
    TARGETS ${_ovsim_grpc_runtime_targets}
)

isaacsim_add_cpp_tests(
    MODULE isaacsim.ovsim.protos.grpc
    SOURCES
        tests/cpp/ValueCodec.cpp
    DEPENDENCIES
        gRPC::grpc++
        protobuf::libprotobuf
)
