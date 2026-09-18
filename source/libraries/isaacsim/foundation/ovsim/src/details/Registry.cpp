// SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "Registry.hpp"

#include <isaacsim/common/array/Array.hpp>
#include <isaacsim/common/exceptions/Exceptions.hpp>
#include <isaacsim/common/string/String.hpp>
#include <isaacsim/foundation/objects/Camera.hpp>
#include <isaacsim/foundation/objects/Mesh.hpp>
#include <isaacsim/foundation/objects/Prim.hpp>
#include <isaacsim/foundation/objects/Xform.hpp>
#include <isaacsim/foundation/objects/lights/CylinderLight.hpp>
#include <isaacsim/foundation/objects/lights/DiskLight.hpp>
#include <isaacsim/foundation/objects/lights/DistantLight.hpp>
#include <isaacsim/foundation/objects/lights/DomeLight.hpp>
#include <isaacsim/foundation/objects/lights/RectLight.hpp>
#include <isaacsim/foundation/objects/lights/SphereLight.hpp>
#include <isaacsim/foundation/objects/shapes/Capsule.hpp>
#include <isaacsim/foundation/objects/shapes/Cone.hpp>
#include <isaacsim/foundation/objects/shapes/Cube.hpp>
#include <isaacsim/foundation/objects/shapes/Cylinder.hpp>
#include <isaacsim/foundation/objects/shapes/Plane.hpp>
#include <isaacsim/foundation/objects/shapes/Sphere.hpp>
#include <isaacsim/foundation/prims/physics/Articulation.hpp>
#include <isaacsim/foundation/prims/physics/ColliderBody.hpp>
#include <isaacsim/foundation/prims/physics/GroundPlane.hpp>
#include <isaacsim/foundation/prims/physics/RigidBody.hpp>
#include <ovsim/interfaces/details/Exception.hpp>

#include <memory>
#include <unordered_map>

namespace isaacsim
{
namespace foundation
{
namespace ovsim
{
namespace details
{

namespace objects = isaacsim::foundation::objects;
namespace physics = isaacsim::foundation::prims::physics;

namespace
{

enum class InstanceType
{
    // physics
    eArticulation,
    eGroundPlane,
    eRigidBody,
    eColliderBody,
    // objects (shapes)
    eCapsule,
    eCone,
    eCube,
    eCylinder,
    ePlane,
    eSphere,
    // objects (lights)
    eCylinderLight,
    eDiskLight,
    eDistantLight,
    eDomeLight,
    eRectLight,
    eSphereLight,
    // objects (other)
    eCamera,
    eMesh,
    eXform,
    // object (base class)
    ePrim,
};

enum class InstanceMethodType
{
    eMeshDisplayColors,
    eShapeDisplayColors,
    eXformWorldPositions,
    eXformWorldOrientations,
};

std::unordered_map<std::vector<std::string>,
                   std::pair<InstanceType, std::shared_ptr<objects::Prim>>,
                   isaacsim::common::string::VectorStringHasher>
    g_instanceMap;
std::unordered_map<InstanceType, std::unordered_map<std::string, std::variant<std::string, InstanceMethodType>>>
    g_instanceMethodMap = {
        // physics
        { InstanceType::eArticulation,
          { { "position", InstanceMethodType::eXformWorldPositions },
            { "orientation", InstanceMethodType::eXformWorldOrientations } } },
        { InstanceType::eGroundPlane,
          { { "position", InstanceMethodType::eXformWorldPositions },
            { "orientation", InstanceMethodType::eXformWorldOrientations } } },
        { InstanceType::eRigidBody,
          { { "mass", "physics:mass" },
            { "position", InstanceMethodType::eXformWorldPositions },
            { "orientation", InstanceMethodType::eXformWorldOrientations } } },
        { InstanceType::eColliderBody,
          { { "position", InstanceMethodType::eXformWorldPositions },
            { "orientation", InstanceMethodType::eXformWorldOrientations } } },
        // objects (shapes)
        { InstanceType::eCapsule,
          { { "color", InstanceMethodType::eShapeDisplayColors },
            { "position", InstanceMethodType::eXformWorldPositions },
            { "orientation", InstanceMethodType::eXformWorldOrientations } } },
        { InstanceType::eCone,
          { { "color", InstanceMethodType::eShapeDisplayColors },
            { "position", InstanceMethodType::eXformWorldPositions },
            { "orientation", InstanceMethodType::eXformWorldOrientations } } },
        { InstanceType::eCube,
          { { "color", InstanceMethodType::eShapeDisplayColors },
            { "position", InstanceMethodType::eXformWorldPositions },
            { "orientation", InstanceMethodType::eXformWorldOrientations } } },
        { InstanceType::eCylinder,
          { { "color", InstanceMethodType::eShapeDisplayColors },
            { "position", InstanceMethodType::eXformWorldPositions },
            { "orientation", InstanceMethodType::eXformWorldOrientations } } },
        { InstanceType::ePlane,
          { { "color", InstanceMethodType::eShapeDisplayColors },
            { "position", InstanceMethodType::eXformWorldPositions },
            { "orientation", InstanceMethodType::eXformWorldOrientations } } },
        { InstanceType::eSphere,
          { { "color", InstanceMethodType::eShapeDisplayColors },
            { "position", InstanceMethodType::eXformWorldPositions },
            { "orientation", InstanceMethodType::eXformWorldOrientations } } },
        // objects (lights)
        { InstanceType::eCylinderLight,
          { { "color", "inputs:color" },
            { "exposure", "inputs:exposure" },
            { "intensity", "inputs:intensity" },
            { "radius", "inputs:radius" },
            { "length", "inputs:length" },
            { "position", InstanceMethodType::eXformWorldPositions },
            { "orientation", InstanceMethodType::eXformWorldOrientations } } },
        { InstanceType::eDiskLight,
          { { "color", "inputs:color" },
            { "exposure", "inputs:exposure" },
            { "intensity", "inputs:intensity" },
            { "radius", "inputs:radius" },
            { "position", InstanceMethodType::eXformWorldPositions },
            { "orientation", InstanceMethodType::eXformWorldOrientations } } },
        { InstanceType::eDistantLight,
          { { "color", "inputs:color" },
            { "exposure", "inputs:exposure" },
            { "intensity", "inputs:intensity" },
            { "angle", "inputs:angle" },
            { "position", InstanceMethodType::eXformWorldPositions },
            { "orientation", InstanceMethodType::eXformWorldOrientations } } },
        { InstanceType::eDomeLight,
          { { "color", "inputs:color" },
            { "exposure", "inputs:exposure" },
            { "intensity", "inputs:intensity" },
            { "guide-radius", "guideRadius" },
            { "texture-file", "inputs:texture:file" },
            { "texture-format", "inputs:texture:format" },
            { "position", InstanceMethodType::eXformWorldPositions },
            { "orientation", InstanceMethodType::eXformWorldOrientations } } },
        { InstanceType::eRectLight,
          { { "color", "inputs:color" },
            { "exposure", "inputs:exposure" },
            { "intensity", "inputs:intensity" },
            { "width", "inputs:width" },
            { "height", "inputs:height" },
            { "texture-file", "inputs:texture:file" },
            { "position", InstanceMethodType::eXformWorldPositions },
            { "orientation", InstanceMethodType::eXformWorldOrientations } } },
        { InstanceType::eSphereLight,
          { { "color", "inputs:color" },
            { "exposure", "inputs:exposure" },
            { "intensity", "inputs:intensity" },
            { "radius", "inputs:radius" },
            { "position", InstanceMethodType::eXformWorldPositions },
            { "orientation", InstanceMethodType::eXformWorldOrientations } } },
        // objects (other)
        { InstanceType::eCamera,
          { { "position", InstanceMethodType::eXformWorldPositions },
            { "orientation", InstanceMethodType::eXformWorldOrientations } } },
        { InstanceType::eMesh,
          { { "color", InstanceMethodType::eMeshDisplayColors },
            { "position", InstanceMethodType::eXformWorldPositions },
            { "orientation", InstanceMethodType::eXformWorldOrientations } } },
        { InstanceType::eXform,
          { { "position", InstanceMethodType::eXformWorldPositions },
            { "orientation", InstanceMethodType::eXformWorldOrientations } } },
    };

bool allOf(const array::Array& flags)
{
    return array::all(flags).item<bool>();
}

objects::ColorType toColorType(const InputValueType& values, const std::string& attributeName)
{
    return std::visit(
        [&attributeName](auto&& value) -> objects::ColorType
        {
            using T = std::decay_t<decltype(value)>;
            if constexpr (std::is_same_v<T, std::vector<std::vector<std::string>>>)
            {
                throw std::runtime_error("Invalid value type for attribute '" + attributeName + "'");
            }
            else
            {
                return value;
            }
        },
        values);
}

std::pair<InstanceType, std::shared_ptr<objects::Prim>> getOrCreateInstance(const std::vector<std::string>& paths)
{
    auto it = g_instanceMap.find(paths);
    if (it != g_instanceMap.end())
    {
        return it->second;
    }

    objects::Prim probe{ paths, /*resolvePaths=*/true }; // avoid re-expanding regular expressions
    auto resolvedPaths = probe.paths();
    InstanceType instanceType;
    std::shared_ptr<objects::Prim> instance;

    // Prims (physics)
    if (allOf(probe.hasApi("PhysicsArticulationRootAPI")))
    {
        // Check for articulation root before every other structural tests.
        instanceType = InstanceType::eArticulation;
        instance = std::make_shared<physics::Articulation>(resolvedPaths);
    }
    else if (allOf(physics::RigidBody::areOfType(resolvedPaths)))
    {
        instanceType = InstanceType::eRigidBody;
        instance =
            std::make_shared<physics::RigidBody>(resolvedPaths, std::nullopt, std::nullopt, /*applyPhysicsApis=*/false);
    }
    else if (allOf(physics::ColliderBody::areOfType(resolvedPaths)))
    {
        instanceType = InstanceType::eColliderBody;
        instance = std::make_shared<physics::ColliderBody>(resolvedPaths, std::nullopt, /*applyCollisionApis=*/false);
    }
    else if (allOf(physics::GroundPlane::areOfType(resolvedPaths)))
    {
        // Ground planes carry no physics API of their own and are matched by their child structure.
        instanceType = InstanceType::eGroundPlane;
        instance = std::make_shared<physics::GroundPlane>(resolvedPaths);
    }
    // Objects (shapes)
    else if (allOf(objects::shapes::Capsule::areOfType(resolvedPaths)))
    {
        instanceType = InstanceType::eCapsule;
        instance = std::make_shared<objects::shapes::Capsule>(resolvedPaths);
    }
    else if (allOf(objects::shapes::Cone::areOfType(resolvedPaths)))
    {
        instanceType = InstanceType::eCone;
        instance = std::make_shared<objects::shapes::Cone>(resolvedPaths);
    }
    else if (allOf(objects::shapes::Cube::areOfType(resolvedPaths)))
    {
        instanceType = InstanceType::eCube;
        instance = std::make_shared<objects::shapes::Cube>(resolvedPaths);
    }
    else if (allOf(objects::shapes::Cylinder::areOfType(resolvedPaths)))
    {
        instanceType = InstanceType::eCylinder;
        instance = std::make_shared<objects::shapes::Cylinder>(resolvedPaths);
    }
    else if (allOf(objects::shapes::Plane::areOfType(resolvedPaths)))
    {
        instanceType = InstanceType::ePlane;
        instance = std::make_shared<objects::shapes::Plane>(resolvedPaths);
    }
    else if (allOf(objects::shapes::Sphere::areOfType(resolvedPaths)))
    {
        instanceType = InstanceType::eSphere;
        instance = std::make_shared<objects::shapes::Sphere>(resolvedPaths);
    }
    // Objects (lights)
    else if (allOf(objects::lights::CylinderLight::areOfType(resolvedPaths)))
    {
        instanceType = InstanceType::eCylinderLight;
        instance = std::make_shared<objects::lights::CylinderLight>(resolvedPaths);
    }
    else if (allOf(objects::lights::DiskLight::areOfType(resolvedPaths)))
    {
        instanceType = InstanceType::eDiskLight;
        instance = std::make_shared<objects::lights::DiskLight>(resolvedPaths);
    }
    else if (allOf(objects::lights::DistantLight::areOfType(resolvedPaths)))
    {
        instanceType = InstanceType::eDistantLight;
        instance = std::make_shared<objects::lights::DistantLight>(resolvedPaths);
    }
    else if (allOf(objects::lights::DomeLight::areOfType(resolvedPaths)))
    {
        instanceType = InstanceType::eDomeLight;
        instance = std::make_shared<objects::lights::DomeLight>(resolvedPaths);
    }
    else if (allOf(objects::lights::RectLight::areOfType(resolvedPaths)))
    {
        instanceType = InstanceType::eRectLight;
        instance = std::make_shared<objects::lights::RectLight>(resolvedPaths);
    }
    else if (allOf(objects::lights::SphereLight::areOfType(resolvedPaths)))
    {
        instanceType = InstanceType::eSphereLight;
        instance = std::make_shared<objects::lights::SphereLight>(resolvedPaths);
    }
    // Objects (Camera)
    else if (allOf(objects::Camera::areOfType(resolvedPaths)))
    {
        instanceType = InstanceType::eCamera;
        instance = std::make_shared<objects::Camera>(resolvedPaths);
    }
    // Objects (Mesh)
    else if (allOf(objects::Mesh::areOfType(resolvedPaths)))
    {
        instanceType = InstanceType::eMesh;
        instance = std::make_shared<objects::Mesh>(resolvedPaths);
    }
    // Prims (physics::Articulation)
    else if (allOf(physics::Articulation::areOfType(resolvedPaths)))
    {
        // The prim is a wrapper above an articulation root rather than the root itself.
        // TODO: identify wrapper above articulation root properly.
        instanceType = InstanceType::eArticulation;
        instance = std::make_shared<physics::Articulation>(resolvedPaths);
    }
    // Objects (Xform)
    else if (allOf(objects::Xform::areOfType(resolvedPaths)))
    {
        instanceType = InstanceType::eXform;
        instance = std::make_shared<objects::Xform>(resolvedPaths);
    }
    // Objects (base class)
    else
    {
        instanceType = InstanceType::ePrim;
        instance = std::make_shared<objects::Prim>(resolvedPaths);
    }

    g_instanceMap.emplace(paths, std::make_pair(instanceType, instance));
    return { instanceType, instance };
}

} // namespace

void clearRegistry()
{
    g_instanceMap.clear();
}

std::vector<std::string> processPaths(const std::variant<std::string, std::vector<std::string>>& paths)
{
    return std::holds_alternative<std::string>(paths) ? std::vector<std::string>{ std::get<std::string>(paths) } :
                                                        std::get<std::vector<std::string>>(paths);
}

OutputValueType getAttributeValues(const std::vector<std::string>& paths,
                                   const std::string& attributeName,
                                   const std::optional<array::Array>& indices)
{
    auto [instanceType, instance] = getOrCreateInstance(paths);
    // Look up for instance-specific implementations.
    auto instanceTypeIterator = g_instanceMethodMap.find(instanceType);
    if (instanceTypeIterator != g_instanceMethodMap.end())
    {
        // Look up for attribute-specific implementations.
        auto methodIterator = instanceTypeIterator->second.find(attributeName);
        if (methodIterator != instanceTypeIterator->second.end())
        {
            // Attribute mapping to another attribute name
            if (std::holds_alternative<std::string>(methodIterator->second))
            {
                return instance->getAttributeValues(std::get<std::string>(methodIterator->second), indices);
            }
            // Attribute mapping to an instance method
            switch (std::get<InstanceMethodType>(methodIterator->second))
            {
            case InstanceMethodType::eMeshDisplayColors:
                return std::static_pointer_cast<objects::Mesh>(instance)->getDisplayColors(indices);
            case InstanceMethodType::eShapeDisplayColors:
                return instance->getAttributeValues("primvars:displayColor", indices);
                // TODO: return std::static_pointer_cast<objects::shapes::Shape>(instance)->getDisplayColors(indices);
            case InstanceMethodType::eXformWorldPositions:
                return std::get<0>(std::static_pointer_cast<objects::Xform>(instance)->getWorldPoses(indices));
            case InstanceMethodType::eXformWorldOrientations:
                return std::get<1>(std::static_pointer_cast<objects::Xform>(instance)->getWorldPoses(indices));
            default:
                throw std::runtime_error("Non-implemented instance method mapping for attribute '" + attributeName + "'");
            }
        }
    }
    // Default implementation: Prim::getAttributeValues.
    try
    {
        return instance->getAttributeValues(attributeName, indices);
    }
    catch (const isaacsim::common::exceptions::AttributeNameError& e)
    {
        std::vector<std::string> validAttributeNames = e.getValidAttributeNames();
        if (instanceTypeIterator != g_instanceMethodMap.end())
        {
            for (const auto& entry : instanceTypeIterator->second)
            {
                validAttributeNames.push_back(entry.first);
            }
        }
        throw ::ovsim::interfaces::details::AttributeError(
            attributeName, isaacsim::common::string::sort(validAttributeNames));
    }
}

void setAttributeValues(const std::vector<std::string>& paths,
                        const std::string& attributeName,
                        const InputValueType& values,
                        const std::optional<array::Array>& indices)
{
    auto [instanceType, instance] = getOrCreateInstance(paths);
    // Look up for instance-specific implementations.
    auto instanceTypeIterator = g_instanceMethodMap.find(instanceType);
    if (instanceTypeIterator != g_instanceMethodMap.end())
    {
        // Look up for attribute-specific implementations.
        auto methodIterator = instanceTypeIterator->second.find(attributeName);
        if (methodIterator != instanceTypeIterator->second.end())
        {
            // Attribute mapping to another attribute name
            if (std::holds_alternative<std::string>(methodIterator->second))
            {
                instance->setAttributeValues(std::get<std::string>(methodIterator->second), values, indices);
                return;
            }
            // Attribute mapping to an instance method
            switch (std::get<InstanceMethodType>(methodIterator->second))
            {
            case InstanceMethodType::eMeshDisplayColors:
                std::static_pointer_cast<objects::Mesh>(instance)->setDisplayColors(
                    toColorType(values, attributeName), indices);
                return;
            case InstanceMethodType::eShapeDisplayColors:
                std::static_pointer_cast<objects::shapes::Shape>(instance)->setDisplayColors(
                    toColorType(values, attributeName), indices);
                return;
            case InstanceMethodType::eXformWorldPositions:
                std::static_pointer_cast<objects::Xform>(instance)->setWorldPoses(
                    std::get<array::Array>(values), std::nullopt, indices);
                return;
            case InstanceMethodType::eXformWorldOrientations:
                std::static_pointer_cast<objects::Xform>(instance)->setWorldPoses(
                    std::nullopt, std::get<array::Array>(values), indices);
                return;
            default:
                throw std::runtime_error("Non-implemented instance method mapping for attribute '" + attributeName + "'");
            }
        }
    }
    // Default implementation: Prim::setAttributeValues.
    try
    {
        instance->setAttributeValues(attributeName, values, indices);
    }
    catch (const isaacsim::common::exceptions::AttributeNameError& e)
    {
        std::vector<std::string> validAttributeNames = e.getValidAttributeNames();
        if (instanceTypeIterator != g_instanceMethodMap.end())
        {
            for (const auto& entry : instanceTypeIterator->second)
            {
                validAttributeNames.push_back(entry.first);
            }
        }
        throw ::ovsim::interfaces::details::AttributeError(
            attributeName, isaacsim::common::string::sort(validAttributeNames));
    }
}

} // namespace details
} // namespace ovsim
} // namespace foundation
} // namespace isaacsim
