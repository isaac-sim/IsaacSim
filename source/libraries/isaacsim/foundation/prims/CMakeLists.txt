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
    NAME isaacsim.foundation.prims
    EXPORT_HEADER isaacsim/foundation/prims/Export.h
    EXPORT_MACRO ISAACSIM_FOUNDATION_PRIMS_API
    PUBLIC_DEPENDENCIES
        isaacsim::common-array
        isaacsim::foundation-objects
        isaacsim::foundation-utils
    PRIVATE_DEPENDENCIES
        isaacsim::common-logging
    PUBLIC_CPP_HEADERS
        isaacsim/foundation/prims/physics/Articulation.hpp
        isaacsim/foundation/prims/physics/ColliderBody.hpp
        isaacsim/foundation/prims/physics/GroundPlane.hpp
        isaacsim/foundation/prims/physics/RigidBody.hpp
    SOURCES
        src/physics/Articulation.cpp
        src/physics/ColliderBody.cpp
        src/physics/GroundPlane.cpp
        src/physics/RigidBody.cpp
)

isaacsim_add_cpp_tests(
    MODULE isaacsim.foundation.prims
    REQUIRES_NO_BINDINGS
    RESOURCE_DIR resources/tests/isaacsim.foundation.prims
    SOURCES
        tests/cpp/physics/RigidBody.cpp
    DEPENDENCIES
        isaacsim::foundation-objects
    LABELS
        unit
)

isaacsim_add_nanobind(
    MODULE isaacsim.foundation.prims
    SOURCE bindings/python/Bindings.cpp
)

isaacsim_add_python_tests(
    MODULE isaacsim.foundation.prims
    REQUIRES_BINDINGS
    RESOURCE_DIR resources/tests/isaacsim.foundation.prims
    LABELS
        unit
)
