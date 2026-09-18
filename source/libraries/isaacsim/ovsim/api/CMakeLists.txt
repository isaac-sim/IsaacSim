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
    NAME isaacsim.ovsim.api
    EXPORT_HEADER isaacsim/ovsim/api/Export.h
    EXPORT_MACRO ISAACSIM_OVSIM_API_API
    PUBLIC_DEPENDENCIES
        isaacsim::common-array
    PRIVATE_DEPENDENCIES
        isaacsim::ovsim-clients-local
        isaacsim::ovsim-clients-grpc
    PUBLIC_CPP_HEADERS
        isaacsim/ovsim/api/Factory.hpp
        isaacsim/ovsim/api/Types.hpp
    SOURCES
        src/Factory.cpp
)

target_include_directories(isaacsim-ovsim-api PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/ovsim/interfaces/cpp>
)
if(UNIX AND NOT APPLE)
    target_link_libraries(isaacsim-ovsim-api PUBLIC
        "$<BUILD_INTERFACE:${ISAACSIM_PUBLIC_DEPS_ROOT}/lib/libstdc++.so.6>"
        "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_LIBDIR}/libstdc++.so.6>"
    )
endif()

isaacsim_add_cpp_tests(
    MODULE isaacsim.ovsim.api
    REQUIRES_NO_BINDINGS
    RESOURCE_DIR resources/tests/isaacsim.ovsim.api
    SOURCES
        tests/cpp/Factory.cpp
    DEPENDENCIES
        isaacsim::foundation-ovsim
        isaacsim::physics-ovsim
    LABELS
        unit
)

isaacsim_add_nanobind(
    MODULE isaacsim.ovsim.api
    SOURCE bindings/python/Bindings.cpp
)

isaacsim_add_python_tests(
    MODULE isaacsim.ovsim.api
    REQUIRES_BINDINGS
    RESOURCE_DIR resources/tests/isaacsim.ovsim.api
    LABELS
        unit
)
