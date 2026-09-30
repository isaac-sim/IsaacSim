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
    NAME isaacsim.foundation.objects
    EXPORT_HEADER isaacsim/foundation/objects/Export.h
    EXPORT_MACRO ISAACSIM_FOUNDATION_OBJECTS_API
    PUBLIC_DEPENDENCIES
        isaacsim::common-array
        isaacsim::common-exceptions
    PRIVATE_DEPENDENCIES
        isaacsim::common-logging
        isaacsim::foundation-usd-openusd
        isaacsim::foundation-usd-ovstage
    PUBLIC_CPP_HEADERS
        isaacsim/foundation/objects/Camera.hpp
        isaacsim/foundation/objects/Mesh.hpp
        isaacsim/foundation/objects/Prim.hpp
        isaacsim/foundation/objects/Stage.hpp
        isaacsim/foundation/objects/Xform.hpp
        isaacsim/foundation/objects/lights/CylinderLight.hpp
        isaacsim/foundation/objects/lights/DiskLight.hpp
        isaacsim/foundation/objects/lights/DistantLight.hpp
        isaacsim/foundation/objects/lights/DomeLight.hpp
        isaacsim/foundation/objects/lights/Light.hpp
        isaacsim/foundation/objects/lights/RectLight.hpp
        isaacsim/foundation/objects/lights/SphereLight.hpp
        isaacsim/foundation/objects/physics_scenes/NewtonMjcScene.hpp
        isaacsim/foundation/objects/physics_scenes/PhysicsScene.hpp
        isaacsim/foundation/objects/physics_scenes/PhysxScene.hpp
        isaacsim/foundation/objects/shapes/Capsule.hpp
        isaacsim/foundation/objects/shapes/Cone.hpp
        isaacsim/foundation/objects/shapes/Cube.hpp
        isaacsim/foundation/objects/shapes/Cylinder.hpp
        isaacsim/foundation/objects/shapes/Plane.hpp
        isaacsim/foundation/objects/shapes/Shape.hpp
        isaacsim/foundation/objects/shapes/Sphere.hpp
    SOURCES
        src/Camera.cpp
        src/Mesh.cpp
        src/Prim.cpp
        src/Stage.cpp
        src/Xform.cpp
        src/lights/CylinderLight.cpp
        src/lights/DiskLight.cpp
        src/lights/DistantLight.cpp
        src/lights/DomeLight.cpp
        src/lights/Light.cpp
        src/lights/RectLight.cpp
        src/lights/SphereLight.cpp
        src/physics_scenes/NewtonMjcScene.cpp
        src/physics_scenes/PhysicsScene.cpp
        src/physics_scenes/PhysicsSceneUtils.hpp
        src/physics_scenes/PhysxScene.cpp
        src/shapes/Capsule.cpp
        src/shapes/Cone.cpp
        src/shapes/Cube.cpp
        src/shapes/Cylinder.cpp
        src/shapes/Plane.cpp
        src/shapes/Shape.cpp
        src/shapes/Sphere.cpp
)

isaacsim_add_cpp_tests(
    MODULE isaacsim.foundation.objects
    REQUIRES_NO_BINDINGS
    RESOURCE_DIR resources/tests/isaacsim.foundation.objects
    SOURCES
        tests/cpp/Prim.cpp
        tests/cpp/Stage.cpp
        tests/cpp/Xform.cpp
    DEPENDENCIES
        isaacsim::foundation-usd-openusd
        isaacsim::foundation-usd-ovstage
    LABELS
        unit
)

isaacsim_add_nanobind(
    MODULE isaacsim.foundation.objects
    SOURCE bindings/python/Bindings.cpp
)

isaacsim_add_python_tests(
    MODULE isaacsim.foundation.objects
    REQUIRES_BINDINGS
    RESOURCE_DIR resources/tests/isaacsim.foundation.objects
    LABELS
        unit
)
