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

isaacsim_add_group(
    NAME isaacsim_ovsim
    MODULES
        isaacsim.ovsim.api
        isaacsim.ovsim.clients.grpc
        isaacsim.ovsim.clients.local
        isaacsim.ovsim.protos.grpc
        isaacsim.ovsim.servers.grpc
)

isaacsim_add_group_dependency(GROUP isaacsim_ovsim DEPENDS_ON isaacsim_common)
isaacsim_add_group_dependency(GROUP isaacsim_ovsim DEPENDS_ON isaacsim_foundation)
isaacsim_add_group_dependency(GROUP isaacsim_ovsim DEPENDS_ON isaacsim_physics)
isaacsim_add_group_dependency(GROUP isaacsim_ovsim DEPENDS_ON isaacsim_physics_engines)

add_subdirectory(protos/grpc)
add_subdirectory(clients/grpc)
add_subdirectory(clients/local)
add_subdirectory(servers/grpc)
add_subdirectory(api)

# Every public OV SIM wrapper includes the canonical interface contract. Install one
# shared copy for the package instead of repeating the same rule in each module.
_isaacsim_get_group_component("isaacsim_ovsim" development isaacsim_ovsim_development_component)
install(DIRECTORY "${CMAKE_SOURCE_DIR}/ovsim/interfaces/cpp/ovsim"
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
    COMPONENT "${isaacsim_ovsim_development_component}"
    FILES_MATCHING PATTERN "*.hpp"
)
