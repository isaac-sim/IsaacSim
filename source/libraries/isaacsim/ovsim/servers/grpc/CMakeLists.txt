# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

isaacsim_add_module(
    NAME isaacsim.ovsim.servers.grpc
    EXPORT_HEADER isaacsim/ovsim/servers/grpc/Export.h
    EXPORT_MACRO ISAACSIM_OVSIM_SERVERS_GRPC_API
    PUBLIC_CPP_HEADERS
        isaacsim/ovsim/servers/grpc/Server.hpp
    PRIVATE_DEPENDENCIES
        isaacsim::ovsim-clients-local
        isaacsim::ovsim-protos-grpc
        isaacsim::physics_engines-ovphysx
        gRPC::grpc++
    SOURCES
        src/HostedSimulation.cpp
        src/HostedSimulation.hpp
        src/Server.cpp
)

target_include_directories(isaacsim-ovsim-servers-grpc PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
)
if(UNIX AND NOT APPLE)
    target_link_libraries(isaacsim-ovsim-servers-grpc PUBLIC
        "$<BUILD_INTERFACE:${ISAACSIM_PUBLIC_DEPS_ROOT}/lib/libstdc++.so.6>"
        "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_LIBDIR}/libstdc++.so.6>"
    )
endif()

add_executable(isaacsim-ovsim-grpc-server src/Main.cpp)
target_compile_features(isaacsim-ovsim-grpc-server PRIVATE cxx_std_17)
target_link_libraries(isaacsim-ovsim-grpc-server PRIVATE
    isaacsim::ovsim-servers-grpc
    # Keep the local host implementation in the executable's direct load set.
    # Its OpenUSD/TBB runtime must initialize before the server constructs the
    # process-global local simulation used by HostedSimulation.
    isaacsim::ovsim-clients-local
)
if(TARGET isaacsim::foundation-usd-openusd-standalone)
    target_link_libraries(isaacsim-ovsim-grpc-server PRIVATE
        isaacsim::foundation-usd-openusd-standalone)
endif()
if(UNIX AND NOT APPLE)
    set(_ovsim_grpc_gl_libraries
        gl "${ISAACSIM_PUBLIC_DEPS_ROOT}/lib/libGL${CMAKE_SHARED_LIBRARY_SUFFIX}"
        glx "${ISAACSIM_PUBLIC_DEPS_ROOT}/lib/libGLX${CMAKE_SHARED_LIBRARY_SUFFIX}"
        opengl "${ISAACSIM_PUBLIC_DEPS_ROOT}/lib/libOpenGL${CMAKE_SHARED_LIBRARY_SUFFIX}"
        gl_dispatch "${ISAACSIM_PUBLIC_DEPS_ROOT}/lib/libGLdispatch${CMAKE_SHARED_LIBRARY_SUFFIX}.0"
    )
    while(_ovsim_grpc_gl_libraries)
        list(POP_FRONT _ovsim_grpc_gl_libraries _ovsim_grpc_gl_name _ovsim_grpc_gl_path)
        if(NOT EXISTS "${_ovsim_grpc_gl_path}")
            message(FATAL_ERROR
                "isaacsim.ovsim.servers.grpc requires the Pixi-locked GLVND library at "
                "${_ovsim_grpc_gl_path}. Run the library build without --no-pull."
            )
        endif()
        add_library(isaacsim_ovsim::${_ovsim_grpc_gl_name} SHARED IMPORTED)
        set_target_properties(isaacsim_ovsim::${_ovsim_grpc_gl_name} PROPERTIES
            IMPORTED_LOCATION "${_ovsim_grpc_gl_path}"
        )
    endwhile()
    target_link_options(isaacsim-ovsim-grpc-server PRIVATE "-Wl,--no-as-needed")
    target_link_libraries(isaacsim-ovsim-grpc-server PRIVATE
        isaacsim_ovsim::gl
        isaacsim_ovsim::glx
        isaacsim_ovsim::opengl
        isaacsim_ovsim::gl_dispatch
        "${ISAACSIM_PUBLIC_DEPS_ROOT}/lib/libstdc++.so.6"
    )
endif()
set_target_properties(isaacsim-ovsim-grpc-server PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${ISAACSIM_MODULE_OUTPUT_ROOT}/isaacsim_ovsim/bin"
    INSTALL_RPATH "${ISAACSIM_LOADER_ORIGIN}/../${CMAKE_INSTALL_LIBDIR}"
)

get_target_property(_server_group isaacsim-ovsim-servers-grpc ISAACSIM_GROUP_NAME)
_isaacsim_get_group_component("${_server_group}" runtime _server_runtime_component)
install(TARGETS isaacsim-ovsim-grpc-server
    RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
    COMPONENT "${_server_runtime_component}"
)

isaacsim_add_cpp_tests(
    MODULE isaacsim.ovsim.servers.grpc
    SOURCES
        tests/cpp/HostedSimulation.cpp
        tests/cpp/Server.cpp
    DEPENDENCIES
        isaacsim::ovsim-clients-grpc
        isaacsim::ovsim-clients-local
        isaacsim::ovsim-protos-grpc
        gRPC::grpc++
        protobuf::libprotobuf
    LABELS
        integration
)
set(_ovsim_grpc_server_test_target tests-isaacsim-ovsim-servers-grpc)
if(TARGET ${_ovsim_grpc_server_test_target})
    target_include_directories(${_ovsim_grpc_server_test_target} PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src"
    )
    if(TARGET isaacsim::foundation-usd-openusd-standalone)
        target_link_libraries(${_ovsim_grpc_server_test_target} PRIVATE
            isaacsim::foundation-usd-openusd-standalone)
    endif()

    if(UNIX AND NOT APPLE)
        target_link_options(${_ovsim_grpc_server_test_target} PRIVATE "-Wl,--no-as-needed")
        target_link_libraries(${_ovsim_grpc_server_test_target} PRIVATE
            isaacsim_ovsim::gl
            isaacsim_ovsim::glx
            isaacsim_ovsim::opengl
            isaacsim_ovsim::gl_dispatch
            "${ISAACSIM_PUBLIC_DEPS_ROOT}/lib/libstdc++.so.6"
        )
    endif()
    set_tests_properties(${_ovsim_grpc_server_test_target} PROPERTIES
        RUN_SERIAL TRUE
        TIMEOUT 300
    )
    if(WIN32)
        # The generic test helper copies linked DLLs beside the executable. OpenUSD and
        # OVPhysX consequently resolve their plugin metadata relative to this test-local
        # layout, so preserve the two runtimes' required directory structure here.
        add_custom_command(TARGET ${_ovsim_grpc_server_test_target} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${ISAACSIM_PUBLIC_DEPS_ROOT}/ovphysx/bin/config.toml"
                "${ISAACSIM_PUBLIC_DEPS_ROOT}/ovphysx/bin/ovphysx_internal.dll"
                "$<TARGET_FILE_DIR:${_ovsim_grpc_server_test_target}>"
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${ISAACSIM_NATIVE_RUNTIME_DEPS_DIR}/ovstage/bin/ovstage.dll"
                "$<TARGET_FILE_DIR:${_ovsim_grpc_server_test_target}>"
            COMMAND ${CMAKE_COMMAND} -E remove_directory
                "$<TARGET_FILE_DIR:${_ovsim_grpc_server_test_target}>/plugins"
            COMMAND ${CMAKE_COMMAND} -E copy_directory
                "${ISAACSIM_NATIVE_RUNTIME_DEPS_DIR}/ovstage/bin/plugins"
                "$<TARGET_FILE_DIR:${_ovsim_grpc_server_test_target}>/plugins"
            COMMAND ${CMAKE_COMMAND} -E remove_directory
                "$<TARGET_FILE_DIR:${_ovsim_grpc_server_test_target}>/../plugins"
            COMMAND ${CMAKE_COMMAND} -E copy_directory
                "${ISAACSIM_PUBLIC_DEPS_ROOT}/ovphysx/plugins"
                "$<TARGET_FILE_DIR:${_ovsim_grpc_server_test_target}>/../plugins"
            COMMAND ${CMAKE_COMMAND} -E copy_directory
                "${ISAACSIM_TARGET_DEPS_DIR}/openusd/lib/usd"
                "$<TARGET_FILE_DIR:${_ovsim_grpc_server_test_target}>/usd"
            COMMAND ${CMAKE_COMMAND} -E copy_directory
                "${ISAACSIM_NATIVE_RUNTIME_DEPS_DIR}/physx_usd_schemas"
                "$<TARGET_FILE_DIR:${_ovsim_grpc_server_test_target}>/third-party-schemas/physx_usd_schemas"
            COMMAND ${CMAKE_COMMAND} -E copy_directory
                "${ISAACSIM_NATIVE_RUNTIME_DEPS_DIR}/newton_usd_schemas"
                "$<TARGET_FILE_DIR:${_ovsim_grpc_server_test_target}>/third-party-schemas/newton_usd_schemas"
            COMMENT "ovsim gRPC server: stage Windows OpenUSD and OVPhysX test runtimes"
        )
        if(EXISTS "${ISAACSIM_NATIVE_RUNTIME_DEPS_DIR}/ovstage/bin/ovstage_usd_schemas")
            add_custom_command(TARGET ${_ovsim_grpc_server_test_target} POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_directory
                    "${ISAACSIM_NATIVE_RUNTIME_DEPS_DIR}/ovstage/bin/ovstage_usd_schemas"
                    "$<TARGET_FILE_DIR:${_ovsim_grpc_server_test_target}>/ovstage_usd_schemas"
                COMMENT "ovsim gRPC server: stage Windows OVStage schemas"
            )
        endif()
        set_property(TEST ${_ovsim_grpc_server_test_target} APPEND PROPERTY
            ENVIRONMENT_MODIFICATION
                "PATH=path_list_prepend:$<TARGET_FILE_DIR:${_ovsim_grpc_server_test_target}>/plugins"
                "PATH=path_list_prepend:$<TARGET_FILE_DIR:${_ovsim_grpc_server_test_target}>/plugins/omni.client.lib"
                "PATH=path_list_prepend:$<TARGET_FILE_DIR:${_ovsim_grpc_server_test_target}>/plugins/omni.usd_resolver"
                "PATH=path_list_prepend:$<TARGET_FILE_DIR:${_ovsim_grpc_server_test_target}>/../plugins"
        )
    else()
        string(JOIN ":" _ovsim_grpc_server_library_path
            "${ISAACSIM_PUBLIC_DEPS_ROOT}/ovphysx/lib"
            "${ISAACSIM_PUBLIC_DEPS_ROOT}/ovphysx/plugins"
            "${ISAACSIM_PUBLIC_DEPS_ROOT}/lib"
            "${ISAACSIM_NATIVE_RUNTIME_DEPS_DIR}/ovstage/bin"
        )
        set_tests_properties(${_ovsim_grpc_server_test_target} PROPERTIES
            ENVIRONMENT
                "LD_LIBRARY_PATH=${_ovsim_grpc_server_library_path}"
        )
    endif()
endif()
