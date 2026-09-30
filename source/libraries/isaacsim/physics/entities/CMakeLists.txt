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
    NAME isaacsim.physics.entities
    EXPORT_HEADER isaacsim/physics/entities/Export.h
    EXPORT_MACRO ISAACSIM_PHYSICS_ENTITIES_API
    PUBLIC_DEPENDENCIES
        isaacsim::common-array
        isaacsim::physics-manager
        isaacsim::physics-registration
    PRIVATE_DEPENDENCIES
        isaacsim::common-logging
    PUBLIC_CPP_HEADERS
        isaacsim/physics/entities/ArticulationEntity.hpp
        isaacsim/physics/entities/PhysicsEntity.hpp
        isaacsim/physics/entities/RigidBodyEntity.hpp
    SOURCES
        src/details/EntityUtils.hpp
        src/details/EntityUtils.cpp
        src/ArticulationEntity.cpp
        src/PhysicsEntity.cpp
        src/RigidBodyEntity.cpp
)

isaacsim_add_cpp_tests(
    MODULE isaacsim.physics.entities
    SOURCES
        tests/cpp/ArticulationEntity.cpp
        tests/cpp/RigidBodyEntity.cpp
    DEPENDENCIES
        isaacsim::physics-manager
    LABELS
        unit
)

isaacsim_add_nanobind(
    MODULE isaacsim.physics.entities
    SOURCE bindings/python/Bindings.cpp
)

isaacsim_add_python_tests(
    MODULE isaacsim.physics.entities
    REQUIRES_BINDINGS
    LABELS
        unit
)
