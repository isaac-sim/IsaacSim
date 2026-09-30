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
    NAME isaacsim.physics.ovsim
    EXPORT_HEADER isaacsim/physics/ovsim/Export.h
    EXPORT_MACRO ISAACSIM_PHYSICS_OVSIM_API
    PUBLIC_DEPENDENCIES
        isaacsim::common-array
    PRIVATE_DEPENDENCIES
        isaacsim::common-exceptions
        isaacsim::common-logging
        isaacsim::common-string
        isaacsim::physics-manager
        isaacsim::physics-entities
    PUBLIC_CPP_HEADERS
        isaacsim/physics/ovsim/control/authoring/Authoring.hpp
        isaacsim/physics/ovsim/control/simulation/Simulation.hpp
        isaacsim/physics/ovsim/data/Data.hpp
    SOURCES
        src/details/Registry.hpp
        src/details/Registry.cpp
        src/Control.cpp
        src/Data.cpp
        src/VerifyInterfaces.cpp
)

target_include_directories(isaacsim-physics-ovsim PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/ovsim/interfaces/cpp>
)

# The public physics wrappers include the canonical OV SIM interface headers,
# so ship those headers with the same development component.
get_target_property(isaacsim_physics_ovsim_group isaacsim-physics-ovsim ISAACSIM_GROUP_NAME)
_isaacsim_get_group_component(
    "${isaacsim_physics_ovsim_group}" development isaacsim_physics_ovsim_development_component
)
install(DIRECTORY "${CMAKE_SOURCE_DIR}/ovsim/interfaces/cpp/ovsim"
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
    COMPONENT "${isaacsim_physics_ovsim_development_component}"
    FILES_MATCHING PATTERN "*.hpp"
)

isaacsim_add_cpp_tests(
    MODULE isaacsim.physics.ovsim
    REQUIRES_NO_BINDINGS
    RESOURCE_DIR resources/tests/isaacsim.physics.ovsim
    SOURCES
        tests/cpp/Control.cpp
        tests/cpp/Data.cpp
    DEPENDENCIES
        isaacsim::common-exceptions
        isaacsim::physics-manager
        isaacsim::physics-entities
    LABELS
        unit
)

isaacsim_add_nanobind(
    MODULE isaacsim.physics.ovsim
    SOURCE bindings/python/Bindings.cpp
)

isaacsim_add_python_tests(
    MODULE isaacsim.physics.ovsim
    REQUIRES_BINDINGS
    RESOURCE_DIR resources/tests/isaacsim.physics.ovsim
    LABELS
        unit
)
