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

isaacsim_add_module(
    NAME isaacsim.ovsim.clients.grpc
    EXPORT_HEADER isaacsim/ovsim/clients/grpc/Export.h
    EXPORT_MACRO ISAACSIM_OVSIM_CLIENTS_GRPC_API
    PUBLIC_DEPENDENCIES
        isaacsim::common-array
    PRIVATE_DEPENDENCIES
        isaacsim::ovsim-protos-grpc
        gRPC::grpc++
    PUBLIC_CPP_HEADERS
        isaacsim/ovsim/clients/grpc/GrpcSession.hpp
    SOURCES
        src/GrpcSession.cpp
)

target_include_directories(isaacsim-ovsim-clients-grpc PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/ovsim/interfaces/cpp>
)
if(UNIX AND NOT APPLE)
    target_link_libraries(isaacsim-ovsim-clients-grpc PUBLIC
        "$<BUILD_INTERFACE:${ISAACSIM_PUBLIC_DEPS_ROOT}/lib/libstdc++.so.6>"
        "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_LIBDIR}/libstdc++.so.6>"
    )
endif()

isaacsim_add_cpp_tests(
    MODULE isaacsim.ovsim.clients.grpc
    SOURCES
        tests/cpp/GrpcSession.cpp
    DEPENDENCIES
        isaacsim::ovsim-protos-grpc
        gRPC::grpc++
    LABELS
        unit
)
