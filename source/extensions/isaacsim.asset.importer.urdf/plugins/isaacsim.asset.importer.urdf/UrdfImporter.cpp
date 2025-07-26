// SPDX-FileCopyrightText: Copyright (c) 2023-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <carb/logging/Log.h>

#include <isaacsim/asset/importer/urdf/ImportHelpers.h>
#include <isaacsim/asset/importer/urdf/UrdfImporter.h>
#include <isaacsim/core/includes/utils/Path.h>
#include <isaacsim/core/includes/utils/Usd.h>
#include <isaacsim/robot/schema/robot_schema.h>
#include <omni/ext/ExtensionsUtils.h>
#include <omni/ext/IExtensions.h>
#include <omni/kit/IApp.h>
#include <physicsSchemaTools/UsdTools.h>
#include <physxSchema/jointStateAPI.h>
#include <physxSchema/physxArticulationAPI.h>
#include <physxSchema/physxCollisionAPI.h>
#include <physxSchema/physxJointAPI.h>
#include <physxSchema/physxMeshMergeCollisionAPI.h>
#include <physxSchema/physxMimicJointAPI.h>
#include <physxSchema/physxSceneAPI.h>
#include <pxr/usd/sdf/layer.h>
#include <pxr/usd/sdf/primSpec.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usdPhysics/articulationRootAPI.h>
#include <pxr/usd/usdPhysics/collisionAPI.h>
#include <pxr/usd/usdPhysics/collisionGroup.h>
#include <pxr/usd/usdPhysics/driveAPI.h>
#include <pxr/usd/usdPhysics/fixedJoint.h>
#include <pxr/usd/usdPhysics/joint.h>
#include <pxr/usd/usdPhysics/limitAPI.h>
#include <pxr/usd/usdPhysics/massAPI.h>
#include <pxr/usd/usdPhysics/meshCollisionAPI.h>
#include <pxr/usd/usdPhysics/prismaticJoint.h>
#include <pxr/usd/usdPhysics/revoluteJoint.h>
#include <pxr/usd/usdPhysics/rigidBodyAPI.h>
#include <pxr/usd/usdPhysics/scene.h>
#include <pxr/usd/usdPhysics/sphericalJoint.h>
#include <pxr/usd/usdUtils/authoring.h>
#include <rapidjson/document.h>
#include <rapidjson/error/en.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>

#include <OmniClient.h>
#include <cmath>
#include <fstream>
#include <iostream>
namespace
{
constexpr float kNegligibleMass = 1.0e-5f;
constexpr float kSmallEps = 1.0e-5f;
}
namespace isaacsim
{
namespace asset
{
namespace importer
{
namespace urdf
{
using namespace isaacsim::core::includes::utils::path;
using namespace isaacsim::core::includes::utils::usd;

void saveJsonToFile(const rapidjson::Document& jsonDoc, const std::string& filename)
{
    rapidjson::StringBuffer buffer;
    rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
    jsonDoc.Accept(writer);

    std::ofstream outFile(filename);
    if (outFile.is_open())
    {
        outFile << buffer.GetString();
        outFile.close();
        CARB_LOG_INFO("JSON saved to %s", filename.c_str());
    }
    else
    {
        CARB_LOG_ERROR("Error saving JSON to file: %s", filename.c_str());
    }
}

UrdfRobot UrdfImporter::createAsset()
{
    UrdfRobot robot;
    if (!parseUrdf(assetRoot_, urdfPath_, robot))
    {
        CARB_LOG_ERROR("Failed to parse URDF file '%s'", urdfPath_.c_str());
        return robot;
    }
    robot.assetRoot = assetRoot_;
    robot.urdfPath = urdfPath_;

    if (config.mergeFixedJoints)
    {
        collapseFixedJoints(robot);
    }

    if (config.collisionFromVisuals)
    {
        addVisualMeshToCollision(robot);
    }

    return robot;
}


pxr::UsdPrim FindPrimByNameAndType(const pxr::UsdStageRefPtr& stage,
                                   const std::string& primName,
                                   const pxr::TfType& primType)
{
    // Define the root path for traversal
    pxr::SdfPath rootPath = pxr::SdfPath::AbsoluteRootPath();

    // Define the range for traversal with type filtering
    pxr::UsdPrimRange range(stage->GetPrimAtPath(rootPath));

    // Iterate over prims within the range
    for (const pxr::UsdPrim& prim : range)
    {
        // Check if the prim matches the given name
        if (prim.GetName() == primName && prim.IsA(primType))
        {
            return prim;
        }
    }
    // If the prim is not found, return an invalid prim
    return pxr::UsdPrim();
}
pxr::GfVec3d getScale(UrdfGeometry geometry)
{
    switch (geometry.type)
    {
    case UrdfGeometryType::MESH:
        return pxr::GfVec3d(geometry.scale_x, geometry.scale_y, geometry.scale_z);
    case UrdfGeometryType::BOX:
        return pxr::GfVec3d(geometry.size_x, geometry.size_y, geometry.size_z);
    default:
        return pxr::GfVec3d(1, 1, 1);
    }
}
pxr::UsdPrim addMeshReference(UrdfGeometry geometry,
                              pxr::UsdStageWeakPtr stage,
                              std::string assetRoot,
                              std::string urdfPath,
                              std::string meshName,
                              std::map<pxr::TfToken, pxr::SdfPath>& meshList,
                              std::map<pxr::TfToken, pxr::SdfPath>& materialList,
                              const pxr::SdfPath& robotRoot,
                              pxr::UsdGeomXform usdXform)
{
    std::string meshUri = geometry.meshFilePath;
    std::string meshPath = resolveXrefPath(assetRoot, urdfPath, meshUri);
    pxr::SdfPath path;
    if (meshPath.empty())
    {
        CARB_LOG_WARN("Failed to resolve mesh '%s'", meshUri.c_str());
        return pxr::UsdPrim();
    }
    else if (IsUsdFile(meshPath))
    {
        CARB_LOG_INFO("Adding Usd reference '%s'", meshPath.c_str());
        usdXform.GetPrim().GetReferences().AddReference(meshPath);
    }
    else
    {
        CARB_LOG_INFO("Found Mesh At: %s (%s)", meshPath.c_str(), meshName.c_str());
        std::string next_path = std::string("/meshes/") + makeValidUSDIdentifier(getPathStem(meshPath.c_str()));
        path = SimpleImport(stage, next_path, meshPath, meshList, materialList, robotRoot);
        usdXform.GetPrim().GetReferences().AddInternalReference(path);
    }
    return usdXform.GetPrim();
}

pxr::UsdPrim addSphere(pxr::UsdStageWeakPtr stage, std::string meshName, double radius, pxr::UsdGeomXform usdXform)
{
    pxr::UsdGeomSphere gprim = pxr::UsdGeomSphere::Define(stage, pxr::SdfPath(meshName + "/sphere"));
    pxr::VtVec3fArray extentArray(2);
    gprim.ComputeExtent(radius, &extentArray);
    gprim.GetExtentAttr().Set(extentArray);
    gprim.GetRadiusAttr().Set(double(radius));
    return usdXform.GetPrim();
}

pxr::UsdPrim addBox(pxr::UsdStageWeakPtr stage,
                    std::string meshName,
                    double size_x,
                    double size_y,
                    double size_z,
                    pxr::UsdGeomXform usdXform)
{
    pxr::UsdGeomCube gprim = pxr::UsdGeomCube::Define(stage, pxr::SdfPath(meshName + "/box"));
    pxr::VtVec3fArray extentArray(2);
    extentArray[1] = pxr::GfVec3f((float)size_x * 0.5f, (float)size_y * 0.5f, (float)size_z * 0.5f);
    extentArray[0] = -extentArray[1];
    gprim.GetExtentAttr().Set(extentArray);
    gprim.GetSizeAttr().Set(1.0);
    return usdXform.GetPrim();
}

pxr::UsdPrim addCylinder(
    pxr::UsdStageWeakPtr stage, std::string meshName, double length, double radius, pxr::UsdGeomXform usdXform)
{
    pxr::UsdGeomCylinder gprim = pxr::UsdGeomCylinder::Define(stage, pxr::SdfPath(meshName + "/cylinder"));
    pxr::VtVec3fArray extentArray(2);
    gprim.ComputeExtent(length, radius, pxr::UsdGeomTokens->z, &extentArray);
    gprim.GetAxisAttr().Set(pxr::UsdGeomTokens->z);
    gprim.GetExtentAttr().Set(extentArray);
    gprim.GetHeightAttr().Set(double(length));
    gprim.GetRadiusAttr().Set(double(radius));
    return usdXform.GetPrim();
}

pxr::UsdPrim addCapsule(
    pxr::UsdStageWeakPtr stage, std::string meshName, double length, double radius, pxr::UsdGeomXform usdXform)
{
    pxr::UsdGeomCapsule gprim = pxr::UsdGeomCapsule::Define(stage, pxr::SdfPath(meshName + "/capsule"));
    pxr::VtVec3fArray extentArray(2);
    gprim.ComputeExtent(length, radius, pxr::UsdGeomTokens->z, &extentArray);
    gprim.GetAxisAttr().Set(pxr::UsdGeomTokens->z);
    gprim.GetExtentAttr().Set(extentArray);
    gprim.GetHeightAttr().Set(double(length));
    gprim.GetRadiusAttr().Set(double(radius));
    return usdXform.GetPrim();
}

pxr::UsdPrim addMesh(pxr::UsdStageWeakPtr stage,
                     UrdfGeometry geometry,
                     std::string assetRoot,
                     std::string urdfPath,
                     std::string meshName,
                     std::map<pxr::TfToken, pxr::SdfPath>& meshList,
                     std::map<pxr::TfToken, pxr::SdfPath>& materialList,
                     const pxr::SdfPath& robotRoot,
                     Transform origin,
                     const double distanceScale,
                     const bool replaceCylindersWithCapsules = false)
{

    auto usdXform = pxr::UsdGeomXform::Define(
        stage, pxr::SdfPath(isaacsim::asset::importer::urdf::GetNewSdfPathString(stage, meshName)));
    Transform transform = origin;

    pxr::GfVec3d scale = getScale(geometry);

    usdXform.ClearXformOpOrder();
    usdXform.AddTranslateOp(pxr::UsdGeomXformOp::PrecisionDouble)
        .Set(distanceScale * pxr::GfVec3d(transform.p.x, transform.p.y, transform.p.z));
    usdXform.AddOrientOp(pxr::UsdGeomXformOp::PrecisionDouble)
        .Set(pxr::GfQuatd(transform.q.w, transform.q.x, transform.q.y, transform.q.z));
    usdXform.AddScaleOp(pxr::UsdGeomXformOp::PrecisionDouble).Set(distanceScale * scale);

    switch (geometry.type)
    {
    case UrdfGeometryType::MESH:
        return addMeshReference(
            geometry, stage, assetRoot, urdfPath, meshName, meshList, materialList, robotRoot, usdXform);
    case UrdfGeometryType::SPHERE:
        return addSphere(stage, meshName, geometry.radius, usdXform);
    case UrdfGeometryType::BOX:
        return addBox(stage, meshName, geometry.size_x, geometry.size_y, geometry.size_z, usdXform);
    case UrdfGeometryType::CYLINDER:
        if (!replaceCylindersWithCapsules)
            return addCylinder(stage, meshName, geometry.length, geometry.radius, usdXform);
        else
            return addCapsule(stage, meshName, geometry.length, geometry.radius, usdXform);
    case UrdfGeometryType::CAPSULE:
        return addCapsule(stage, meshName, geometry.length, geometry.radius, usdXform);
    default:
        CARB_LOG_WARN("Unsupported geometry type");
        return pxr::UsdPrim();
    }
}

void UrdfImporter::addMergedChildren(std::unordered_map<std::string, pxr::UsdStageRefPtr> stages,
                                     const UrdfLink& link,
                                     const pxr::UsdPrim& parentPrim,
                                     const UrdfRobot& robot)
{
    for (const auto& pair : link.mergedChildren)
    {
        CARB_LOG_INFO("Add Merged Child %s", pair.first.c_str());
        auto childXform =
            pxr::UsdGeomXform::Define(stages["stage"], parentPrim.GetPath().AppendPath(pxr::SdfPath(pair.first)));
        if (childXform)
        {
            childXform.ClearXformOpOrder();

            childXform.AddTranslateOp(pxr::UsdGeomXformOp::PrecisionDouble)
                .Set(config.distanceScale * pxr::GfVec3d(pair.second.p.x, pair.second.p.y, pair.second.p.z));
            childXform.AddOrientOp(pxr::UsdGeomXformOp::PrecisionDouble)
                .Set(pxr::GfQuatd(pair.second.q.w, pair.second.q.x, pair.second.q.y, pair.second.q.z));
            childXform.AddScaleOp(pxr::UsdGeomXformOp::PrecisionDouble).Set(pxr::GfVec3d(1, 1, 1));
            if (robot.links.find(pair.first) != robot.links.end())
            {
                const UrdfLink& child = robot.links.at(pair.first);
                addMergedChildren(stages, child, childXform.GetPrim(), robot);
            }
        }
    }
    // for (const auto& pair : link.mergedChildren)
    // {
    //     // const UrdfLink& child = robot.links.at(pair.first);
    //     Transform childTransform = pair.second;
    //     pxr::SdfPath childPath =
    //     parentPrim.GetPath().AppendPath(pxr::SdfPath(pxr::TfMakeValidIdentifier(pair.first))); pxr::UsdGeomXform
    //     childXform = pxr::UsdGeomXform::Define(stages["stage"], childPath); childXform.ClearXformOpOrder();
    //     childXform.AddTranslateOp(pxr::UsdGeomXformOp::PrecisionDouble)
    //         .Set(config.distanceScale * pxr::GfVec3d(childTransform.p.x, childTransform.p.y, childTransform.p.z));
    //     childXform.AddOrientOp(pxr::UsdGeomXformOp::PrecisionDouble)
    //         .Set(pxr::GfQuatd(childTransform.q.w, childTransform.q.x, childTransform.q.y, childTransform.q.z));
    //     childXform.AddScaleOp(pxr::UsdGeomXformOp::PrecisionDouble).Set(pxr::GfVec3d(1, 1, 1));

    // addMergedChildren(stages, child, childXform.GetPrim(), robot);
    // }
}

void UrdfImporter::addRigidBody(std::unordered_map<std::string, pxr::UsdStageRefPtr> stages,
                                const UrdfLink& link,
                                const Transform& poseBodyToWorld,
                                pxr::UsdGeomXform robotPrim,
                                const UrdfRobot& robot)
{
    setAuthoringLayer(stages["stage"], stages["base_stage"]->GetRootLayer()->GetIdentifier());
    std::string robotBasePath = robotPrim.GetPath().GetString() + "/";
    auto prim = stages["stage"]->GetPrimAtPath(pxr::SdfPath(robotBasePath + link.name));
    if (prim)
    {
        // it was already added, skip
        return;
    }
    CARB_LOG_INFO("Add Rigid Body: %s", link.name.c_str());
    // Create Link Prim
    auto linkPrim =
        pxr::UsdGeomXform::Define(stages["stage"], pxr::SdfPath(robotBasePath + pxr::TfMakeValidIdentifier(link.name)));
    if (linkPrim)
    {
        Transform transform = poseBodyToWorld; // urdfOriginToTransform(link.inertial.origin);

        linkPrim.ClearXformOpOrder();

        linkPrim.AddTranslateOp(pxr::UsdGeomXformOp::PrecisionDouble)
            .Set(config.distanceScale * pxr::GfVec3d(transform.p.x, transform.p.y, transform.p.z));
        linkPrim.AddOrientOp(pxr::UsdGeomXformOp::PrecisionDouble)
            .Set(pxr::GfQuatd(transform.q.w, transform.q.x, transform.q.y, transform.q.z));
        linkPrim.AddScaleOp(pxr::UsdGeomXformOp::PrecisionDouble).Set(pxr::GfVec3d(1, 1, 1));
        CARB_LOG_INFO("Add Merged Children %s", link.name.c_str());
        addMergedChildren(stages, link, linkPrim.GetPrim(), robot);

        setAuthoringLayer(stages["stage"], stages["physics_stage"]->GetRootLayer()->GetIdentifier());
        pxr::UsdPhysicsRigidBodyAPI physicsAPI = pxr::UsdPhysicsRigidBodyAPI::Apply(linkPrim.GetPrim());

        {
            setAuthoringLayer(stages["stage"], stages["robot_stage"]->GetRootLayer()->GetIdentifier());
            pxr::UsdPrim prim = stages["stage"]->GetPrimAtPath(linkPrim.GetPrim().GetPath());
            isaacsim::robot::schema::ApplyLinkAPI(prim);
            auto linksRel = robotPrim.GetPrim().GetRelationship(
                isaacsim::robot::schema::relationNames.at(isaacsim::robot::schema::Relations::ROBOT_LINKS));
            pxr::SdfPathVector targets;
            linksRel.AddTarget(prim.GetPath());
        }
        setAuthoringLayer(stages["stage"], stages["physics_stage"]->GetRootLayer()->GetIdentifier());
        pxr::UsdPhysicsMassAPI massAPI = pxr::UsdPhysicsMassAPI::Apply(linkPrim.GetPrim());
        if (link.inertial.hasMass)
        {
            massAPI.CreateMassAttr().Set(link.inertial.mass);
        }
        else if (config.density > 0 && link.collisions.size() > 0)
        {
            CARB_LOG_INFO("Applying default Density for link %s", link.name.c_str());
            // scale from kg/m^2 to specified units
            massAPI.CreateDensityAttr().Set(config.density);
        }
        else
        {
            // massAPI.CreateMassAttr().Set(kNegligibleMass); // Set mass significantly low that it won't impact
            //                                                // simulation, but not so low as to be treated as massless
            CARB_LOG_WARN("No mass specified for link %s", link.name.c_str());
        }

        if (link.inertial.hasInertia)
        {
            Matrix33 inertiaMatrix;
            inertiaMatrix.cols[0] = Vec3(link.inertial.inertia.ixx, link.inertial.inertia.ixy, link.inertial.inertia.ixz);
            inertiaMatrix.cols[1] = Vec3(link.inertial.inertia.ixy, link.inertial.inertia.iyy, link.inertial.inertia.iyz);
            inertiaMatrix.cols[2] = Vec3(link.inertial.inertia.ixz, link.inertial.inertia.iyz, link.inertial.inertia.izz);

            Quat principalAxes;
            Vec3 diaginertia = Diagonalize(inertiaMatrix, principalAxes);

            // input is meters, but convert to kit units
            massAPI.CreateDiagonalInertiaAttr().Set(config.distanceScale * config.distanceScale *
                                                    pxr::GfVec3f(diaginertia.x, diaginertia.y, diaginertia.z));
            massAPI.CreatePrincipalAxesAttr().Set(
                pxr::GfQuatf(principalAxes[3], principalAxes[0], principalAxes[1], principalAxes[2]));
        }

        if (link.inertial.hasOrigin)
        {
            massAPI.CreateCenterOfMassAttr().Set(pxr::GfVec3f(float(config.distanceScale * link.inertial.origin.p.x),
                                                              float(config.distanceScale * link.inertial.origin.p.y),
                                                              float(config.distanceScale * link.inertial.origin.p.z)));

            pxr::GfQuatf orientation = pxr::GfQuatf(
                link.inertial.origin.q.w, link.inertial.origin.q.x, link.inertial.origin.q.y, link.inertial.origin.q.z);
            if (link.inertial.hasInertia)
            {
                pxr::GfQuatf principalAxis;
                massAPI.GetPrincipalAxesAttr().Get(&principalAxis);
                orientation = orientation * principalAxis;
                massAPI.GetPrincipalAxesAttr().Set(orientation);
            }
            else
            {
                massAPI.CreatePrincipalAxesAttr().Set(orientation);
            }
        }
    }
    else
    {
        CARB_LOG_WARN("linkPrim %s not created", link.name.c_str());
        return;
    }
    CARB_LOG_INFO("Added Rigid Body. Adding Visuals (%s)", link.name.c_str());
    setAuthoringLayer(stages["stage"], stages["base_stage"]->GetRootLayer()->GetIdentifier());
    // Add Visuals
    auto meshes_base =
        pxr::UsdGeomXform::Define(stages["base_stage"], pxr::SdfPath(robotBasePath + link.name + "/visuals"));

    std::string source_name = "/visuals/" + link.name;
    auto source_prim = pxr::UsdGeomXform::Define(stages["base_stage"], pxr::SdfPath(source_name));
    for (size_t i = 0; i < link.visuals.size(); i++)
    {
        std::string meshName;
        {
            std::string name = "mesh_" + std::to_string(i);
            if (link.visuals[i].name.size() > 0)
            {
                name = link.visuals[i].name;
            }
            else if (link.visuals[i].geometry.type == UrdfGeometryType::MESH)
            {
                name = getPathStem(link.visuals[i].geometry.meshFilePath.c_str());
            }
            meshName = source_name + "/" + name;
        }
        {
            CARB_LOG_INFO("Creating Visual Prim %s", meshName.c_str());
        }
        bool loadMaterial = false;

        auto mat = link.visuals[i].material;
        auto urdfMatIter = robot.materials.find(link.visuals[i].material.name);
        if (urdfMatIter != robot.materials.end())
        {
            mat = urdfMatIter->second;
        }
        auto& color = mat.color;
        loadMaterial = (color.r >= 0 && color.g >= 0 && color.b >= 0);
        pxr::UsdPrim prim =
            addMesh(stages["base_stage"], link.visuals[i].geometry, assetRoot_, urdfPath_, meshName, meshPaths,
                    materialPaths, robotPrim.GetPath(), link.visuals[i].origin, config.distanceScale, false);
        if (!prim)
        {
            CARB_LOG_WARN("Prim %s not created", meshName.c_str());
        }
        else
        {
            if (loadMaterial)
            {
                // This Material was in the master list, reuse
                auto urdfMatIter = robot.materials.find(link.visuals[i].material.name);
                if (urdfMatIter != robot.materials.end())
                {
                    std::string path = matPrimPaths[link.visuals[i].material.name];

                    auto matPrim = stages["base_stage"]->GetPrimAtPath(pxr::SdfPath(path));

                    if (matPrim)
                    {
                        auto shadePrim = pxr::UsdShadeMaterial(matPrim);
                        if (shadePrim)
                        {
                            pxr::UsdShadeMaterialBindingAPI mbi(prim);
                            mbi.Bind(shadePrim);
                            pxr::UsdRelationship rel;
                            mbi.ComputeBoundMaterial(pxr::UsdShadeTokens->allPurpose, &rel);
                            pxr::UsdShadeMaterialBindingAPI::SetMaterialBindingStrength(
                                rel, pxr::UsdShadeTokens->strongerThanDescendants);
                        }
                    }
                }
                else
                {
                    auto& color = link.visuals[i].material.color;
                    std::stringstream ss;
                    ss << std::uppercase << std::hex << (int)(256 * color.r) << std::uppercase << std::hex
                       << (int)(256 * color.g) << std::uppercase << std::hex << (int)(256 * color.b);
                    std::pair<std::string, UrdfMaterial> mat_pair(ss.str(), link.visuals[i].material);

                    pxr::UsdShadeMaterial matPrim = addMaterial(stages["base_stage"], mat_pair, robotPrim.GetPath());
                    if (matPrim)
                    {
                        pxr::UsdShadeMaterialBindingAPI mbi(prim);
                        mbi.Bind(matPrim);
                        pxr::UsdRelationship rel;
                        mbi.ComputeBoundMaterial(pxr::UsdShadeTokens->allPurpose, &rel);
                        pxr::UsdShadeMaterialBindingAPI::SetMaterialBindingStrength(
                            rel, pxr::UsdShadeTokens->strongerThanDescendants);
                    }
                }
            }
        }
    }
    meshes_base.GetPrim().GetReferences().AddInternalReference(pxr::SdfPath(pxr::TfToken(source_name)));
    meshes_base.GetPrim().SetInstanceable(true);
    // Collisions
    meshes_base =
        pxr::UsdGeomXform::Define(stages["physics_stage"], pxr::SdfPath(robotBasePath + link.name + "/collisions"));
    source_name = "/colliders/" + link.name;
    source_prim = pxr::UsdGeomXform::Define(stages["base_stage"], pxr::SdfPath(source_name));
    CARB_LOG_INFO("Added Rigid Body. Adding Colliders (%s)", link.name.c_str());
    setAuthoringLayer(stages["stage"], stages["base_stage"]->GetRootLayer()->GetIdentifier());
    for (size_t i = 0; i < link.collisions.size(); i++)
    {

        std::string meshName;
        {
            std::string name = "mesh_" + std::to_string(i);
            if (link.collisions[i].name.size() > 0)
            {
                name = link.collisions[i].name;
            }
            else if (link.collisions[i].geometry.type == UrdfGeometryType::MESH)
            {
                name = getPathStem(link.collisions[i].geometry.meshFilePath.c_str());
            }
            meshName = source_name + "/" + name;
        }
        {
            CARB_LOG_INFO("Creating collider Prim %s", meshName.c_str());
        }
        pxr::UsdPrim prim = addMesh(stages["base_stage"], link.collisions[i].geometry, assetRoot_, urdfPath_, meshName,
                                    meshPaths, materialPaths, robotPrim.GetPath(), link.collisions[i].origin,
                                    config.distanceScale, config.replaceCylindersWithCapsules);
        // Enable collisions on prim
        if (prim)
        {
            setAuthoringLayer(stages["stage"], stages["physics_stage"]->GetRootLayer()->GetIdentifier());
            auto meshes_prim = stages["stage"]->GetPrimAtPath(prim.GetPath());
            // pxr::UsdPhysicsCollisionAPI::Apply(meshes_base.GetPrim());

            meshes_base.GetPrim().GetReferences().AddInternalReference(pxr::SdfPath(pxr::TfToken(source_name)));
            meshes_base.GetPrim().SetInstanceable(true);

            for (const auto& mesh_prim : meshes_prim.GetChildren())
            {
                pxr::UsdPhysicsCollisionAPI::Apply(mesh_prim.GetPrim());
                if (link.collisions[i].geometry.type == UrdfGeometryType::MESH)
                {
                    pxr::UsdPhysicsMeshCollisionAPI physicsMeshAPI =
                        pxr::UsdPhysicsMeshCollisionAPI::Apply(mesh_prim.GetPrim());
                    // Disable mesh merge Collision API for now
                    // pxr::PhysxSchemaPhysxMeshMergeCollisionAPI mergeAPI =
                    //     pxr::PhysxSchemaPhysxMeshMergeCollisionAPI::Apply(meshes_base.GetPrim());
                    // mergeAPI.GetCollisionMeshesCollectionAPI().GetIncludesRel().AddTarget(meshes_base.GetPrim().GetPath());
                    if (config.convexDecomp == true)
                    {
                        physicsMeshAPI.CreateApproximationAttr().Set(pxr::UsdPhysicsTokens.Get()->convexDecomposition);
                    }
                    else
                    {
                        physicsMeshAPI.CreateApproximationAttr().Set(pxr::UsdPhysicsTokens.Get()->convexHull);
                    }
                    break;
                }
            }
            setAuthoringLayer(stages["stage"], stages["base_stage"]->GetRootLayer()->GetIdentifier());
        }
        else
        {
            CARB_LOG_WARN("Prim %s not created", meshName.c_str());
        }
        pxr::UsdGeomMesh(prim).CreatePurposeAttr().Set(pxr::UsdGeomTokens->guide);
    }
    setAuthoringLayer(stages["stage"], stages["base_stage"]->GetRootLayer()->GetIdentifier());

    if (link.collisions.size() == 0)
    {
        if (!link.inertial.hasInertia || !config.importInertiaTensor)
        {
            CARB_LOG_WARN(
                "Link %s has no colliders, and no inertia was imported; assigning a small isotropic inertia matrix",
                link.name.c_str());

            setAuthoringLayer(stages["stage"], stages["physics_stage"]->GetRootLayer()->GetIdentifier());
            pxr::UsdPhysicsMassAPI massAPI = pxr::UsdPhysicsMassAPI(linkPrim.GetPrim());
            massAPI.CreateDiagonalInertiaAttr().Set(config.distanceScale * config.distanceScale * 10.0f *
                                                    pxr::GfVec3f(kNegligibleMass, kNegligibleMass, kNegligibleMass));
        }
    }
    CARB_LOG_INFO("Adding Cameras (%s)", link.name.c_str());
    setAuthoringLayer(stages["stage"], stages["sensor_stage"]->GetRootLayer()->GetIdentifier()); // Add Sensors
    for (auto& camera : link.cameras)
    {
        auto cameraPrim = pxr::UsdGeomCamera::Define(
            stages["sensor_stage"], pxr::SdfPath(linkPrim.GetPath().AppendChild(PXR_NS::TfToken(camera.name))));
        auto cameraXform = pxr::UsdGeomXformable(cameraPrim.GetPrim());
        cameraXform.ClearXformOpOrder();
        // TODO: not sure OV's default optical axis is the same as ROS, so we may need to transform the orientation from
        // URDF
        cameraXform.AddTranslateOp(pxr::UsdGeomXformOp::PrecisionDouble)
            .Set(config.distanceScale * pxr::GfVec3d(camera.origin.p.x, camera.origin.p.y, camera.origin.p.z));
        cameraXform.AddOrientOp(pxr::UsdGeomXformOp::PrecisionDouble)
            .Set(pxr::GfQuatd(camera.origin.q.w, camera.origin.q.x, camera.origin.q.y, camera.origin.q.z));
        cameraXform.AddScaleOp(pxr::UsdGeomXformOp::PrecisionDouble).Set(pxr::GfVec3d(1, 1, 1));
        cameraPrim.GetClippingRangeAttr().Set(config.distanceScale * pxr::GfVec2f(camera.clipNear, camera.clipFar));

        // Compute Focal Length Assuming a fixed horizontal aperture of 20.955mm (default in cameras created with
        // Omniverse, which is the equivalent of the standard 35mm spherical projector aperture)
        // And a non-distorted camera sensor (no default distortion model is available in official URDF spec)
        float aperture = 0;
        cameraPrim.GetHorizontalApertureAttr().Get(&aperture);
        float focal = aperture / (2 * tan(camera.hfov / 2));

        cameraPrim.GetFocalLengthAttr().Set(focal);
    }


    for (auto& lidar : link.lidars)
    {
        // Add a RTX Lidar camera and wire so it can load json config files
        auto lidarPrim = pxr::UsdGeomCamera::Define(
            stages["sensor_stage"], pxr::SdfPath(linkPrim.GetPath().AppendChild(PXR_NS::TfToken(lidar.name))));
        auto lidarXform = pxr::UsdGeomXformable(lidarPrim.GetPrim());
        lidarXform.ClearXformOpOrder();
        // TODO: not sure OV's default optical axis is the same as ROS, so we may need to transform the orientation from
        // URDF
        lidarXform.AddTranslateOp(pxr::UsdGeomXformOp::PrecisionDouble)
            .Set(config.distanceScale * pxr::GfVec3d(lidar.origin.p.x, lidar.origin.p.y, lidar.origin.p.z));
        lidarXform.AddOrientOp(pxr::UsdGeomXformOp::PrecisionDouble)
            .Set(pxr::GfQuatd(lidar.origin.q.w, lidar.origin.q.x, lidar.origin.q.y, lidar.origin.q.z));
        lidarXform.AddScaleOp(pxr::UsdGeomXformOp::PrecisionDouble).Set(pxr::GfVec3d(1, 1, 1));
        lidarPrim.GetClippingRangeAttr().Set(config.distanceScale * pxr::GfVec2f(0.001f, 1000.0f));

        // Lazy Apply the IsaacRtxLidarSensorAPI to the prim
        // Get the SdfPrimSpec for the prim
        pxr::SdfTokenListOp schemasListOp;
        pxr::SdfTokenListOp::ItemVector schemasList;
        schemasList.push_back(pxr::TfToken("IsaacRtxLidarSensorAPI"));
        schemasListOp.SetAddedItems(schemasList);
        lidarPrim.GetPrim().SetMetadata(pxr::UsdTokens->apiSchemas, pxr::VtValue(schemasListOp));

        pxr::UsdAttribute sensorTypeAttr =
            lidarPrim.GetPrim().CreateAttribute(pxr::TfToken("cameraSensorType"), pxr::SdfValueTypeNames->Token, false);
        sensorTypeAttr.Set(pxr::TfToken("lidar"));

        pxr::TfToken tokenValue1("camera");
        pxr::TfToken tokenValue2("radar");
        pxr::TfToken tokenValue3("lidar");

        pxr::VtArray<pxr::TfToken> validTokens{ tokenValue1, tokenValue2, tokenValue3 };
        // Set the valid tokens for the attribute
        sensorTypeAttr.SetMetadata(pxr::TfToken("allowedTokens"), validTokens);


        lidarPrim.GetPrim()
            .CreateAttribute(pxr::TfToken("sensorModelPluginName"), pxr::SdfValueTypeNames->String, false)
            .Set("omni.sensors.nv.lidar.lidar_core.plugin");
        omni::kit::IApp* app = carb::getCachedInterface<omni::kit::IApp>();
        omni::ext::ExtensionManager* extManager = app->getExtensionManager();
        pxr::SdfLayerRefPtr rootLayer = stages["sensor_stage"]->GetRootLayer();

        // Get the real path of the root layer, and a few extra useful paths
        std::string stagePath = getParent(rootLayer->GetRealPath());
        std::string sensorPath = omni::ext::getExtensionPath(
            extManager, omni::ext::getEnabledExtensionId(extManager, "isaacsim.sensors.rtx"));
        std::string importerPath = omni::ext::getExtensionPath(
            extManager, omni::ext::getEnabledExtensionId(extManager, "isaacsim.asset.importer.urdf"));
        if (lidar.isaacSimConfig != "")
        {
            // If it's a reference for a config file
            if (hasExtension(lidar.isaacSimConfig, "json"))
            {
                std::string configPath = resolveXrefPath(assetRoot_, urdfPath_, lidar.isaacSimConfig);
                std::string conf_name = getPathStem(configPath.c_str());
                if (stagePath != "")
                {
                    omniClientWait(
                        omniClientCopy(configPath.c_str(), pathJoin(stagePath, conf_name + ".json").c_str(), {}, {}));
                    if (sensorPath != "")
                    {
                        createSymbolicLink(
                            pathJoin(stagePath, conf_name + ".json"),
                            pathJoin(pathJoin(pathJoin(sensorPath, "data"), "lidar_configs"), conf_name + ".json"));
                    }
                }
                else
                {
                    CARB_LOG_WARN("Cannot copy/link over lidar configuration when importing into an in-memory stage.");
                }
                lidarPrim.GetPrim()
                    .CreateAttribute(pxr::TfToken("sensorModelConfig"), pxr::SdfValueTypeNames->String, false)
                    .Set(conf_name);
            }
            // Otherwise it's a preset config
            else
            {
                lidarPrim.GetPrim()
                    .CreateAttribute(pxr::TfToken("sensorModelConfig"), pxr::SdfValueTypeNames->String, false)
                    .Set(lidar.isaacSimConfig);
            }
        }
        // If no cofig file is provided, use the default urdf values to create a basic lidar sensor
        else if (stagePath != "")
        {

            std::string template_file =
                pathJoin(pathJoin(pathJoin(importerPath, "data"), "lidar_sensor_template"), "lidar_template.json");
            std::ifstream ifs(template_file);
            if (!ifs.is_open())
            {
                CARB_LOG_ERROR("Failed to open lidar template file: %s", template_file.c_str());
                lidarPrim.GetPrim()
                    .CreateAttribute(pxr::TfToken("sensorModelConfig"), pxr::SdfValueTypeNames->String, false)
                    .Set("");
            }
            else
            {

                std::string jsonStr((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());

                // Parse the JSON
                rapidjson::Document doc;
                doc.Parse(jsonStr.c_str());
                rapidjson::Document::AllocatorType& allocator = doc.GetAllocator();

                size_t totalRays = lidar.horizontal.samples * lidar.vertical.samples;

                doc["profile"]["scanRateBaseHz"] = lidar.updateRate;
                doc["profile"]["reportRateBaseHz"] = lidar.updateRate;

                doc["profile"]["numberOfEmitters"] = totalRays;
                doc["profile"]["numberOfChannels"] = totalRays;
                doc["profile"]["numLines"] = totalRays;
                float hMinDeg = (float)(lidar.horizontal.minAngle * (180.0 / M_PI));
                float vMinDeg = (float)(lidar.vertical.minAngle * (180.0 / M_PI));

                float hMaxDeg = (float)(lidar.horizontal.maxAngle * (180.0 / M_PI));
                float vMaxDeg = (float)(lidar.vertical.maxAngle * (180.0 / M_PI));

                doc["profile"]["startAzimuthDeg"] = hMinDeg;
                doc["profile"]["endAzimuthDeg"] = hMaxDeg;

                doc["profile"]["downElevationDeg"] = vMinDeg;
                doc["profile"]["upElevationDeg"] = vMaxDeg;

                float horizontal_step =
                    (float)(((lidar.horizontal.maxAngle - lidar.horizontal.minAngle) * (180.0 / M_PI)) /
                            (lidar.horizontal.samples - 1));
                float vertical_step = (float)(((lidar.vertical.maxAngle - lidar.vertical.minAngle) * (180.0 / M_PI)) /
                                              (lidar.horizontal.samples - 1));
                if (lidar.hasHorizontal || lidar.hasVertical)
                {
                    doc["profile"]["emitterStates"][0]["azimuthDeg"].Clear();
                    doc["profile"]["emitterStates"][0]["elevationDeg"].Clear();
                    doc["profile"]["emitterStates"][0]["fireTimeNs"].Clear();
                    doc["profile"]["emitterStates"][0]["channelId"].Clear();
                    doc["profile"]["emitterStates"][0]["rangeId"].Clear();
                    doc["profile"]["emitterStates"][0]["bank"].Clear();
                    doc["profile"]["numRaysPerLine"].Clear();
                    int count = 0;
                    for (size_t vs = 0; vs < lidar.vertical.samples; vs++)
                    {
                        doc["profile"]["numRaysPerLine"] =
                            doc["profile"]["numRaysPerLine"].PushBack(lidar.horizontal.samples, allocator);
                        for (size_t hs = 0; hs < lidar.horizontal.samples; hs++)
                        {
                            doc["profile"]["emitterStates"][0]["azimuthDeg"].PushBack(
                                hMinDeg + hs * horizontal_step, allocator);
                            doc["profile"]["emitterStates"][0]["elevationDeg"].PushBack(
                                vMinDeg + vs * vertical_step, allocator);
                            doc["profile"]["emitterStates"][0]["fireTimeNs"].PushBack(0, allocator);
                            doc["profile"]["emitterStates"][0]["channelId"].PushBack(count++, allocator);
                            doc["profile"]["emitterStates"][0]["rangeId"].PushBack(0, allocator);
                            doc["profile"]["emitterStates"][0]["bank"].PushBack(totalRays - count, allocator);
                        }
                    }
                }
                std::string conf_name = robot.name + "_" + lidar.name;
                saveJsonToFile(doc, pathJoin(stagePath, conf_name + ".json"));
                if (sensorPath != "")
                {
                    createSymbolicLink(
                        pathJoin(stagePath, conf_name + ".json"),
                        pathJoin(pathJoin(pathJoin(sensorPath, "data"), "lidar_configs"), conf_name + ".json"));
                }
                lidarPrim.GetPrim()
                    .CreateAttribute(pxr::TfToken("sensorModelConfig"), pxr::SdfValueTypeNames->String, false)
                    .Set(conf_name);
            }
        }
        else
        {
            CARB_LOG_WARN(
                "Cannot copy/link over lidar configuration when importing into an in-memory stage. Configuration file not generated.");
        }
    }
    setAuthoringLayer(stages["stage"], stages["stage"]->GetRootLayer()->GetIdentifier());
}

const char* getJointAxis(UrdfAxis axis, Quat& direction)
{
    Vec3 currentAxis(axis.x, axis.y, axis.z);
    direction = Quat(0, 0, 0, 1);

    static const Vec3 axes[] = { Vec3(1.0f, 0.0f, 0.0f), Vec3(0.0f, 1.0f, 0.0f), Vec3(0.0f, 0.0f, 1.0f) };
    static const char* axisNames[] = { "X", "Y", "Z" };

    for (int i = 0; i < 3; i++)
    {
        if (abs(Dot(currentAxis, axes[i])) > 1 - kSmallEps)
        {
            if (Dot(currentAxis, axes[i]) < 0)
            {
                direction = Quat(axes[i].y, axes[i].z, axes[i].x, 0); // rotate 180 degrees along any orthogonal axis
            }
            return axisNames[i];
        }
    }

    return "";
}


void configureDriveAPI(const UrdfJoint& joint,
                       pxr::UsdPhysicsJoint& jointPrim,
                       float distanceScale,
                       const ImportConfig& config)
{
    pxr::UsdPhysicsDriveAPI driveAPI = pxr::UsdPhysicsDriveAPI::Apply(
        jointPrim.GetPrim(), joint.type == UrdfJointType::PRISMATIC ? pxr::TfToken("linear") : pxr::TfToken("angular"));
    driveAPI.CreateMaxForceAttr().Set(
        joint.limit.effort > 0.0f ?
            joint.limit.effort * (joint.type == UrdfJointType::PRISMATIC ? distanceScale : distanceScale * distanceScale) :
            FLT_MAX);

    // configure drive type, target type, stiffness, and damping
    if (joint.drive.driveType == UrdfJointDriveType::FORCE)
    {
        driveAPI.CreateTypeAttr().Set(pxr::TfToken("force"));
    }
    else
    {
        driveAPI.CreateTypeAttr().Set(pxr::TfToken("acceleration"));
    }

    if (joint.drive.targetType == UrdfJointTargetType::POSITION)
    {
        driveAPI.CreateTargetPositionAttr().Set(joint.drive.target);
        driveAPI.CreateStiffnessAttr().Set(joint.drive.strength);
        driveAPI.CreateDampingAttr().Set(joint.drive.damping);
    }
    else if (joint.drive.targetType == UrdfJointTargetType::VELOCITY)
    {
        driveAPI.CreateTargetVelocityAttr().Set(joint.drive.target);
        driveAPI.CreateStiffnessAttr().Set(0.0f);
        driveAPI.CreateDampingAttr().Set(joint.drive.strength);
    }
    else if (joint.drive.targetType == UrdfJointTargetType::NONE)
    {
        driveAPI.CreateDampingAttr().Set(joint.dynamics.damping);
        driveAPI.CreateStiffnessAttr().Set(joint.dynamics.stiffness);
    }
}

template <class T>
void configureMimicAPI(const UrdfJoint& joint, T& jointPrim, pxr::UsdStageWeakPtr stage)
{
    auto axisToken = pxr::UsdPhysicsTokens.Get()->rotX;
    Quat direction;
    const char* axis = getJointAxis(joint.axis, direction);
    if (!strcmp(axis, "Y"))
    {
        axisToken = pxr::UsdPhysicsTokens.Get()->rotY;
    }
    else if (!strcmp(axis, "Z"))
    {
        axisToken = pxr::UsdPhysicsTokens.Get()->rotZ;
    }

    auto mimicAPI = pxr::PhysxSchemaPhysxMimicJointAPI::Apply(jointPrim.GetPrim(), axisToken);
    mimicAPI.GetGearingAttr().Set(-joint.mimic.multiplier);
    mimicAPI.GetOffsetAttr().Set(joint.mimic.offset);
    auto nat_freq = jointPrim.GetPrim().CreateAttribute(
        pxr::TfToken("physxMimicJoint:rot" + std::string(axis) + ":naturalFrequency"), pxr::SdfValueTypeNames->Float);
    auto damping_ratio = jointPrim.GetPrim().CreateAttribute(
        pxr::TfToken("physxMimicJoint:rot" + std::string(axis) + ":dampingRatio"), pxr::SdfValueTypeNames->Float);
    nat_freq.Set(joint.drive.naturalFrequency);
    damping_ratio.Set(joint.drive.dampingRatio);
    // Find source joint
    auto source_prim = FindPrimByNameAndType(stage, joint.mimic.joint, pxr::TfType::Find<pxr::UsdPhysicsJoint>());
    if (source_prim)
    {
        mimicAPI.GetReferenceJointRel().AddTarget(source_prim.GetPath());
        float source_min;
        float source_max;
        T(source_prim).GetLowerLimitAttr().Get(&source_min);
        T(source_prim).GetUpperLimitAttr().Get(&source_max);

        float lb = joint.mimic.multiplier * (source_min - 0.2f * (source_max - source_min));
        float ub = joint.mimic.multiplier * (source_max + 0.2f * (source_max - source_min));

        jointPrim.CreateLowerLimitAttr().Set(std::min(lb, ub));
        jointPrim.CreateUpperLimitAttr().Set(std::max(lb, ub));
    }
}

template <class T>
void AddSingleJoint(const UrdfJoint& joint,
                    std::unordered_map<std::string, pxr::UsdStageRefPtr> stages,
                    const pxr::SdfPath& jointPath,
                    pxr::UsdPhysicsJoint& jointPrimBase,
                    const float distanceScale,
                    const ImportConfig& config)
{
    setAuthoringLayer(stages["stage"], stages["physics_stage"]->GetRootLayer()->GetIdentifier());
    T jointPrim = T::Define(stages["physics_stage"], pxr::SdfPath(jointPath));
    jointPrimBase = jointPrim;

    Quat direction;
    const char* axis = getJointAxis(joint.axis, direction);
    jointPrim.CreateAxisAttr().Set(pxr::TfToken(axis ? axis : "X"));
    float scale = 180.0f / static_cast<float>(M_PI);
    if (joint.type != UrdfJointType::CONTINUOUS)
    {
        if (joint.type == UrdfJointType::PRISMATIC)
        {
            scale = distanceScale;
            pxr::PhysxSchemaJointStateAPI::Apply(jointPrim.GetPrim(), pxr::TfToken("linear"));
        }
        else
        {
            pxr::PhysxSchemaJointStateAPI::Apply(jointPrim.GetPrim(), pxr::TfToken("angular"));
        }
    }

    pxr::PhysxSchemaPhysxJointAPI physxJoint = pxr::PhysxSchemaPhysxJointAPI::Apply(jointPrim.GetPrim());
    if (joint.dynamics.friction > 0.0f)
    {
        physxJoint.CreateJointFrictionAttr().Set(joint.dynamics.friction);
    }

    if (config.parseMimic && joint.mimic.joint != "")
    {
        if (joint.limit.velocity > 0.0f)
        {
            CARB_LOG_WARN("Joint %s has a velocity limit defined but is set to mimic joint %s. Velocity limit ignored.",
                          joint.name.c_str(), joint.mimic.joint.c_str());
        }
        configureMimicAPI(joint, jointPrim, stages["stage"]);
    }
    else
    {
        jointPrim.CreateLowerLimitAttr().Set(scale * joint.limit.lower);
        jointPrim.CreateUpperLimitAttr().Set(scale * joint.limit.upper);

        configureDriveAPI(joint, jointPrim, distanceScale, config);
        physxJoint.CreateMaxJointVelocityAttr().Set(
            joint.limit.velocity > 0.0f ? static_cast<float>(joint.limit.velocity * scale) : FLT_MAX);
    }
}


void UrdfImporter::addJoint(std::unordered_map<std::string, pxr::UsdStageRefPtr> stages,
                            pxr::UsdGeomXform robotPrim,
                            const UrdfJoint& joint,
                            const Transform& poseJointToParentBody)
{

    std::string parentLinkPath = robotPrim.GetPath().GetString() + "/" + joint.parentLinkName;
    std::string childLinkPath = robotPrim.GetPath().GetString() + "/" + joint.childLinkName;
    std::string jointPath = GetNewSdfPathString(
        stages["stage"], robotPrim.GetPath().GetString() + "/joints/" + pxr::TfMakeValidIdentifier(joint.name));

    setAuthoringLayer(stages["stage"], stages["physics_stage"]->GetRootLayer()->GetIdentifier());
    pxr::UsdPhysicsJoint jointPrim;
    if (joint.type == UrdfJointType::FIXED)
    {
        if (config.mergeFixedJoints && !joint.dontCollapse)
        {
            return;
        }
        std::string jointPath = robotPrim.GetPath().GetString() + "/joints/" + pxr::TfMakeValidIdentifier(joint.name);

        if (!pxr::SdfPath::IsValidPathString(jointPath))
        {
            // jn->getName starts with a number which is not valid for usd path, so prefix it with "joint"
            jointPath = robotPrim.GetPath().GetString() + "/joints/" + "joint_" + joint.name;
        }
        jointPrim = pxr::UsdPhysicsFixedJoint::Define(stages["stage"], pxr::SdfPath(jointPath));
    }
    else if (joint.type == UrdfJointType::PRISMATIC)
    {
        std::string jointPath = robotPrim.GetPath().GetString() + "/joints/" + pxr::TfMakeValidIdentifier(joint.name);

        if (!pxr::SdfPath::IsValidPathString(jointPath))
        {
            // jn->getName starts with a number which is not valid for usd path, so prefix it with "joint"
            jointPath = robotPrim.GetPath().GetString() + "/joints/" + "joint_" + joint.name;
        }
        AddSingleJoint<pxr::UsdPhysicsPrismaticJoint>(
            joint, stages, pxr::SdfPath(jointPath), jointPrim, float(config.distanceScale), config);
    }
    // else if (joint.type == UrdfJointType::SPHERICAL)
    // {
    //     AddSingleJoint<UsdPhysicsSphericalJoint>(jn, stage, SdfPath(jointPath), jointPrim, skel,
    //     distanceScale);
    // }
    else if (joint.type == UrdfJointType::REVOLUTE || joint.type == UrdfJointType::CONTINUOUS)
    {

        std::string jointPath = robotPrim.GetPath().GetString() + "/joints/" + pxr::TfMakeValidIdentifier(joint.name);

        if (!pxr::SdfPath::IsValidPathString(jointPath))
        {
            // jn->getName starts with a number which is not valid for usd path, so prefix it with "joint"
            jointPath = robotPrim.GetPath().GetString() + "/joints/" + "joint_" + joint.name;
        }
        AddSingleJoint<pxr::UsdPhysicsRevoluteJoint>(
            joint, stages, pxr::SdfPath(jointPath), jointPrim, float(config.distanceScale), config);
    }
    else if (joint.type == UrdfJointType::FLOATING)
    {
        // There is no joint, skip
        return;
    }
    setAuthoringLayer(stages["stage"], stages["physics_stage"]->GetRootLayer()->GetIdentifier());

    pxr::SdfPathVector val0{ pxr::SdfPath(parentLinkPath) };
    pxr::SdfPathVector val1{ pxr::SdfPath(childLinkPath) };

    if (parentLinkPath != "")
    {
        jointPrim.CreateBody0Rel().SetTargets(val0);
    }

    pxr::GfVec3f localPos0 = config.distanceScale * pxr::GfVec3f(poseJointToParentBody.p.x, poseJointToParentBody.p.y,
                                                                 poseJointToParentBody.p.z);
    pxr::GfQuatf localRot0 = pxr::GfQuatf(
        poseJointToParentBody.q.w, poseJointToParentBody.q.x, poseJointToParentBody.q.y, poseJointToParentBody.q.z);
    pxr::GfVec3f localPos1 = config.distanceScale * pxr::GfVec3f(0, 0, 0);
    pxr::GfQuatf localRot1 = pxr::GfQuatf(1, 0, 0, 0);

    Quat jointAxisRotQuat(0.0f, 0.0f, 0.0f, 1.0f);
    const char* axis = getJointAxis(joint.axis, jointAxisRotQuat);
    if (!strcmp(axis, ""))
    {
        CARB_LOG_WARN(
            "%s: Joint Axis is not body aligned with X, Y or Z primary axis. Adjusting PhysX joint alignment to Axis X and reorienting bodies.",
            joint.name.c_str());
        // If the joint axis is not aligned with X, Y or Z we revert back to PhysX's default of aligning to X axis.
        // Need to rotate the joint frame to match the urdf defined axis
        // convert joint axis to angle-axis representation
        Vec3 jointAxisRotAxis = -Cross(urdfAxisToVec(joint.axis), Vec3(1.0f, 0.0f, 0.0f));
        float jointAxisRotAngle = acos(joint.axis.x); // this is equal to acos(Dot(joint.axis, Vec3(1.0f, 0.0f, 0.0f)))
        if (Dot(jointAxisRotAxis, jointAxisRotAxis) < kSmallEps)
        {
            // for axis along x we define an arbitrary perpendicular rotAxis (along y).
            // In that case the angle is 0 or 180deg
            jointAxisRotAxis = Vec3(0.0f, 1.0f, 0.0f);
        }
        // normalize jointAxisRotAxis
        jointAxisRotAxis /= sqrtf(Dot(jointAxisRotAxis, jointAxisRotAxis));
        // rotate the parent frame by the axis

        jointAxisRotQuat = QuatFromAxisAngle(jointAxisRotAxis, jointAxisRotAngle);
    }

    // apply transforms
    jointPrim.CreateLocalPos0Attr().Set(localPos0);
    jointPrim.CreateLocalRot0Attr().Set(
        localRot0 * pxr::GfQuatf(jointAxisRotQuat.w, jointAxisRotQuat.x, jointAxisRotQuat.y, jointAxisRotQuat.z));

    if (childLinkPath != "")
    {
        jointPrim.CreateBody1Rel().SetTargets(val1);
    }
    jointPrim.CreateLocalPos1Attr().Set(localPos1);
    jointPrim.CreateLocalRot1Attr().Set(
        localRot1 * pxr::GfQuatf(jointAxisRotQuat.w, jointAxisRotQuat.x, jointAxisRotQuat.y, jointAxisRotQuat.z));

    jointPrim.CreateBreakForceAttr().Set(FLT_MAX);
    jointPrim.CreateBreakTorqueAttr().Set(FLT_MAX);

    // Add Custom Attribute for Joint Equivalent Inertia
    jointPrim.GetPrim()
        .CreateAttribute(pxr::TfToken("physics:JointEquivalentInertia"), pxr::SdfValueTypeNames->Float, false)
        .Set(joint.jointInertia);

    if (jointPrim)
    {
        setAuthoringLayer(stages["stage"], stages["robot_stage"]->GetRootLayer()->GetIdentifier());
        pxr::UsdPrim prim = stages["stage"]->GetPrimAtPath(jointPrim.GetPrim().GetPath());
        isaacsim::robot::schema::ApplyJointAPI(prim);
        auto jointsRel = robotPrim.GetPrim().GetRelationship(
            isaacsim::robot::schema::relationNames.at(isaacsim::robot::schema::Relations::ROBOT_JOINTS));
        pxr::SdfPathVector targets;
        jointsRel.AddTarget(prim.GetPath());
    }
}
void UrdfImporter::addLinksAndJoints(std::unordered_map<std::string, pxr::UsdStageRefPtr> stages,
                                     const Transform& poseParentToWorld,
                                     const KinematicChain::Node* parentNode,
                                     const UrdfRobot& robot,
                                     pxr::UsdGeomXform robotPrim)
{

    if (parentNode->parentJointName_ == "")
    {
        if (config.fixBase)
        {
            // Define Root joint so it stays on top of asset
            setAuthoringLayer(stages["stage"], stages["physics_stage"]->GetRootLayer()->GetIdentifier());
            std::string rootJointPath = robotPrim.GetPath().GetString() + "/root_joint";
            pxr::UsdPhysicsFixedJoint rootJoint =
                pxr::UsdPhysicsFixedJoint::Define(stages["stage"], pxr::SdfPath(rootJointPath));
            setAuthoringLayer(stages["stage"], stages["stage"]->GetRootLayer()->GetIdentifier());
        }
        addRigidBody(stages, robot.links.at(parentNode->linkName_), poseParentToWorld, robotPrim, robot);
    }
    if (!parentNode->childNodes_.empty())
    {
        for (const auto& childNode : parentNode->childNodes_)
        {
            if (robot.joints.find(childNode->parentJointName_) != robot.joints.end())
            {
                if (robot.links.find(childNode->linkName_) != robot.links.end())
                {
                    const UrdfJoint urdfJoint = robot.joints.at(childNode->parentJointName_);
                    const UrdfLink& childLink = robot.links.at(childNode->linkName_);
                    Transform poseJointToLink = urdfJoint.origin;
                    // According to URDF spec, the frame of a link coincides with its parent joint frame
                    Transform poseLinkToWorld = poseParentToWorld * poseJointToLink;
                    {
                        // const UrdfLink& parentLink = robot.links.at(parentNode->linkName_);

                        // if (!parentLink.softs.size() && !childLink.softs.size()) // rigid parent, rigid child
                        {
                            if (urdfJoint.type != UrdfJointType::FIXED || !config.mergeFixedJoints ||
                                urdfJoint.dontCollapse)
                            {

                                addRigidBody(stages, childLink, poseLinkToWorld, robotPrim, robot);
                                addJoint(stages, robotPrim, urdfJoint, poseJointToLink);
                            }

                            // RigidBodyTopo bodyTopo;
                            // bodyTopo.bodyIndex = asset->bodyLookup.at(childNode->linkName_);
                            // bodyTopo.parentIndex = asset->bodyLookup.at(parentNode->linkName_);
                            // bodyTopo.jointIndex = asset->jointLookup.at(childNode->parentJointName_);
                            // bodyTopo.jointSpecStart = asset->jointLookup.at(childNode->parentJointName_);
                            // // URDF only has 1 DOF joints
                            // bodyTopo.jointSpecCount = 1;
                            // asset->rigidBodyHierarchy.push_back(bodyTopo);
                        }
                    }

                    // Recurse through the links children
                    addLinksAndJoints(stages, poseLinkToWorld, childNode.get(), robot, robotPrim);
                }
                else
                {
                    CARB_LOG_ERROR("Failed to Create Joint <%s>: Child link <%s> not found",
                                   childNode->parentJointName_.c_str(), childNode->linkName_.c_str());
                }
            }
            else
            {
                CARB_LOG_WARN("Joint <%s> is undefined", childNode->parentJointName_.c_str());
            }
        }
    }
    // Create root joint only once
    if (parentNode->parentJointName_ == "")
    {
        std::string link_name = parentNode->linkName_;
        if (robot.rootLink != "")
        {
            link_name = robot.rootLink;
        }
        const UrdfLink& urdfLink = robot.links.at(link_name);
        // Still import the original root link, but point the joint to the overriden rootLink
        addRigidBody(stages, robot.links.at(parentNode->linkName_), poseParentToWorld, robotPrim, robot);
        if (config.fixBase)
        {
            setAuthoringLayer(stages["stage"], stages["physics_stage"]->GetRootLayer()->GetIdentifier());
            std::string rootJointPath = robotPrim.GetPath().GetString() + "/root_joint";
            pxr::UsdPhysicsFixedJoint rootJoint =
                pxr::UsdPhysicsFixedJoint::Define(stages["stage"], pxr::SdfPath(rootJointPath));
            pxr::UsdPrim rootLinkPrim = rootJoint.GetPrim();
            // Apply articulation root schema
            pxr::UsdPhysicsArticulationRootAPI physicsSchema = pxr::UsdPhysicsArticulationRootAPI::Apply(rootLinkPrim);
            pxr::PhysxSchemaPhysxArticulationAPI physxSchema = pxr::PhysxSchemaPhysxArticulationAPI::Apply(rootLinkPrim);
            // Set schema attributes
            physxSchema.CreateEnabledSelfCollisionsAttr().Set(config.selfCollision);
            // These are reasonable defaults, might want to expose them via the import config in the future.
            physxSchema.CreateSolverPositionIterationCountAttr().Set(32);
            physxSchema.CreateSolverVelocityIterationCountAttr().Set(1);
            pxr::SdfPathVector val1{ pxr::SdfPath(robotPrim.GetPath().GetString() + "/" +
                                                  pxr::TfMakeValidIdentifier(link_name)) };
            if (!urdfLink.inertial.hasMass)
            {
                auto linkPrim =
                    stages["stage"]->GetPrimAtPath(pxr::SdfPath(robotPrim.GetPath().GetString() + "/" + link_name));
                pxr::UsdPhysicsMassAPI massAPI = pxr::UsdPhysicsMassAPI(linkPrim);
                // set base mass to zero and let physx deal with that to provide a mass that stabilizes
                // the base of the articulation
                massAPI.GetMassAttr().Set(0.0f);
            }
            rootJoint.CreateBody1Rel().SetTargets(val1);
        }
    }
    // return back to the main stage
    setAuthoringLayer(stages["stage"], stages["stage"]->GetRootLayer()->GetIdentifier());
}


void UrdfImporter::addLoopJoints(std::unordered_map<std::string, pxr::UsdStageRefPtr> stages,
                                 pxr::UsdGeomXform robotPrim,
                                 const UrdfRobot& robot,
                                 const ImportConfig& config)
{
    setAuthoringLayer(stages["stage"], stages["physics_stage"]->GetRootLayer()->GetIdentifier());
    for (const auto& loopJoint : robot.loopJoints)
    {
        std::string link[2] = { robotPrim.GetPath().GetString() + "/" + loopJoint.second.linkName[0],
                                robotPrim.GetPath().GetString() + "/" + loopJoint.second.linkName[1] };
        std::string jointPath =
            GetNewSdfPathString(stages["stage"], robotPrim.GetPath().GetString() + "/loop_joints/" +
                                                     pxr::TfMakeValidIdentifier(loopJoint.second.name));

        pxr::SdfPathVector val0{ pxr::SdfPath(link[0]) };
        pxr::SdfPathVector val1{ pxr::SdfPath(link[1]) };

        pxr::GfVec3f localPos[2];
        pxr::GfQuatf localRot[2];
        for (uint16_t i = 0; i < 2; i++)
        {

            localPos[i] =
                config.distanceScale * pxr::GfVec3f(loopJoint.second.linkPose[i].p.x, loopJoint.second.linkPose[i].p.y,
                                                    loopJoint.second.linkPose[i].p.z);
            localRot[i] = pxr::GfQuatf(loopJoint.second.linkPose[i].q.w, loopJoint.second.linkPose[i].q.x,
                                       loopJoint.second.linkPose[i].q.y, loopJoint.second.linkPose[i].q.z);
        }

        pxr::UsdPhysicsJoint jointPrim;
        if (loopJoint.second.type == UrdfJointType::SPHERICAL)
        {
            pxr::UsdPhysicsSphericalJoint::Define(stages["stage"], pxr::SdfPath(jointPath));
        }
        else
        {
            CARB_LOG_WARN("Loop joint %s is not of type spherical. Skipping.", loopJoint.second.name.c_str());
            continue;
        }

        jointPrim = pxr::UsdPhysicsJoint(stages["stage"]->GetPrimAtPath(pxr::SdfPath(jointPath)));
        if (jointPrim)
        {
            jointPrim.CreateBody0Rel().SetTargets(val0);
            jointPrim.CreateBody1Rel().SetTargets(val1);

            jointPrim.CreateLocalPos0Attr().Set(localPos[0]);
            jointPrim.CreateLocalPos1Attr().Set(localPos[1]);

            jointPrim.CreateLocalRot0Attr().Set(localRot[0]);
            jointPrim.CreateLocalRot1Attr().Set(localRot[1]);

            jointPrim.CreateExcludeFromArticulationAttr().Set(true);
        }
    }
    setAuthoringLayer(stages["stage"], stages["stage"]->GetRootLayer()->GetIdentifier());
}

void UrdfImporter::addMaterials(pxr::UsdStageWeakPtr stage, const UrdfRobot& robot, const pxr::SdfPath& prefixPath)
{
    stage->DefinePrim(pxr::SdfPath(prefixPath.GetString() + "/Looks"), pxr::TfToken("Scope"));
    for (auto& mat : robot.materials)
    {
        addMaterial(stage, mat, prefixPath);
    }
}

pxr::UsdShadeMaterial UrdfImporter::addMaterial(pxr::UsdStageWeakPtr stage,
                                                const std::pair<std::string, UrdfMaterial>& mat,
                                                const pxr::SdfPath& prefixPath)
{
    auto& color = mat.second.color;
    std::string name = mat.second.name;
    if (name == "")
    {
        name = mat.first;
    }
    if (color.r >= 0 && color.g >= 0 && color.b >= 0)
    {
        pxr::SdfPath shaderPath =
            prefixPath.AppendPath(pxr::SdfPath("Looks/" + makeValidUSDIdentifier("material_" + name)));
        pxr::UsdShadeMaterial matPrim = pxr::UsdShadeMaterial::Define(stage, shaderPath);
        if (matPrim)
        {
            pxr::UsdShadeShader pbrShader =
                pxr::UsdShadeShader::Define(stage, shaderPath.AppendPath(pxr::SdfPath("Shader")));
            if (pbrShader)
            {
                auto shader_out = pbrShader.CreateOutput(pxr::TfToken("out"), pxr::SdfValueTypeNames->Token);
                matPrim.CreateSurfaceOutput(pxr::TfToken("mdl")).ConnectToSource(shader_out);
                matPrim.CreateVolumeOutput(pxr::TfToken("mdl")).ConnectToSource(shader_out);
                matPrim.CreateDisplacementOutput(pxr::TfToken("mdl")).ConnectToSource(shader_out);
                pbrShader.GetImplementationSourceAttr().Set(pxr::UsdShadeTokens->sourceAsset);
                pbrShader.SetSourceAsset(pxr::SdfAssetPath("OmniPBR.mdl"), pxr::TfToken("mdl"));
                pbrShader.SetSourceAssetSubIdentifier(pxr::TfToken("OmniPBR"), pxr::TfToken("mdl"));

                pbrShader.CreateInput(pxr::TfToken("diffuse_color_constant"), pxr::SdfValueTypeNames->Color3f)
                    .Set(pxr::GfVec3f(color.r, color.g, color.b));

                matPrimPaths[name] = shaderPath.GetString();
                return matPrim;
            }
            else
            {
                CARB_LOG_WARN("Couldn't create shader at: %s", shaderPath.GetString().c_str());
            }
        }
        else
        {
            CARB_LOG_WARN("Couldn't create material at: %s", shaderPath.GetString().c_str());
        }
    }
    return pxr::UsdShadeMaterial();
}

std::string UrdfImporter::addToStage(std::unordered_map<std::string, pxr::UsdStageRefPtr> stages,
                                     const UrdfRobot& urdfRobot,
                                     bool getArticulationRoot = false)
{
    if (urdfRobot.links.size() == 0)
    {
        CARB_LOG_WARN("Cannot add robot to stage, number of links is zero");
        return "";
    }
    // The limit for links is now a 32bit index so this shouldn't be needed anymore
    // if (urdfRobot.links.size() >= 64)
    // {
    //     CARB_LOG_WARN(
    //         "URDF cannot have more than 63 links to be imported as a physx articulation. Try enabling the merge fixed
    //         joints option to reduce the number of links.");
    //     CARB_LOG_WARN("URDF has %d links", static_cast<int>(urdfRobot.links.size()));
    //     return "";
    // }

    if (config.createPhysicsScene)
    {
        setAuthoringLayer(stages["stage"], stages["physics_stage"]->GetRootLayer()->GetIdentifier());
        bool sceneExists = false;
        pxr::UsdPrimRange range = stages["stage"]->Traverse();
        for (pxr::UsdPrimRange::iterator iter = range.begin(); iter != range.end(); ++iter)
        {
            pxr::UsdPrim prim = *iter;

            if (prim.IsA<pxr::UsdPhysicsScene>())
            {
                sceneExists = true;
            }
        }
        if (!sceneExists)
        {
            // Create physics scene
            pxr::UsdPhysicsScene scene = pxr::UsdPhysicsScene::Define(stages["stage"], pxr::SdfPath("/physicsScene"));
            scene.CreateGravityDirectionAttr().Set(pxr::GfVec3f(0.0f, 0.0f, -1.0));
            scene.CreateGravityMagnitudeAttr().Set(9.81f * config.distanceScale);

            pxr::PhysxSchemaPhysxSceneAPI physxSceneAPI =
                pxr::PhysxSchemaPhysxSceneAPI::Apply(stages["stage"]->GetPrimAtPath(pxr::SdfPath("/physicsScene")));
            physxSceneAPI.CreateEnableCCDAttr().Set(true);
            physxSceneAPI.CreateEnableStabilizationAttr().Set(true);
            physxSceneAPI.CreateEnableGPUDynamicsAttr().Set(false);

            physxSceneAPI.CreateBroadphaseTypeAttr().Set(pxr::TfToken("MBP"));
            physxSceneAPI.CreateSolverTypeAttr().Set(pxr::TfToken("TGS"));
        }
    }
    setAuthoringLayer(stages["stage"], stages["stage"]->GetRootLayer()->GetIdentifier());
    pxr::SdfPath primPath =
        pxr::SdfPath(GetNewSdfPathString(stages["stage"], stages["stage"]->GetDefaultPrim().GetPath().GetString() + "/" +
                                                              makeValidUSDIdentifier(std::string(urdfRobot.name))));
    if (config.makeDefaultPrim)
        primPath =
            pxr::SdfPath(GetNewSdfPathString(stages["stage"], "/" + makeValidUSDIdentifier(std::string(urdfRobot.name))));
    // // Remove the prim we are about to add in case it exists
    // if (stage->GetPrimAtPath(primPath))
    // {
    //     stage->RemovePrim(primPath);
    // }

    setAuthoringLayer(stages["stage"], stages["stage"]->GetRootLayer()->GetIdentifier());

    if (stages["stage"]->GetRootLayer() != stages["base_stage"]->GetRootLayer()) // If the stage is not the base stage,
                                                                                 // we need to copy the root layer
    {
        primPath = pxr::SdfPath(
            GetNewSdfPathString(stages["base_stage"], "/" + makeValidUSDIdentifier(std::string(urdfRobot.name))));
    }
    pxr::SdfPath returnPath = primPath;

    pxr::UsdGeomXform robotPrim = pxr::UsdGeomXform::Define(stages["stage"], primPath);

    {
        setAuthoringLayer(stages["stage"], stages["robot_stage"]->GetRootLayer()->GetIdentifier());
        pxr::UsdPrim prim = stages["stage"]->GetPrimAtPath(robotPrim.GetPrim().GetPath());
        isaacsim::robot::schema::ApplyRobotAPI(prim);
    }
    setAuthoringLayer(stages["stage"], stages["base_stage"]->GetRootLayer()->GetIdentifier());

    pxr::UsdGeomXformable gprim = pxr::UsdGeomXformable(robotPrim);
    gprim.ClearXformOpOrder();
    gprim.AddTranslateOp(pxr::UsdGeomXformOp::PrecisionDouble).Set(pxr::GfVec3d(0, 0, 0));
    gprim.AddOrientOp(pxr::UsdGeomXformOp::PrecisionDouble).Set(pxr::GfQuatd(1, 0, 0, 0));
    gprim.AddScaleOp(pxr::UsdGeomXformOp::PrecisionDouble).Set(pxr::GfVec3d(1, 1, 1));

    setAuthoringLayer(stages["stage"], stages["stage"]->GetRootLayer()->GetIdentifier());
    if (config.makeDefaultPrim)
    {
        stages["stage"]->SetDefaultPrim(robotPrim.GetPrim());
    }
    for (auto& stage : stages)
    {
        if (stage.first != "stage")
        {
            auto prim = pxr::UsdGeomXform::Define(stage.second, primPath);
            stage.second->SetDefaultPrim(prim.GetPrim());
        }
    }


    KinematicChain chain;
    if (!chain.computeKinematicChain(urdfRobot))
    {
        return "";
    }
    robotPrim = pxr::UsdGeomXform(stages["stage"]->GetPrimAtPath(primPath));
    setAuthoringLayer(stages["stage"], stages["base_stage"]->GetRootLayer()->GetIdentifier());
    addMaterials(stages["stage"], urdfRobot, primPath);
    // Create Scope for visual and collider meshes
    if (urdfRobot.joints.size() > 0)
    {
        stages["stage"]->DefinePrim(pxr::SdfPath(primPath).AppendChild(pxr::TfToken("joints")), pxr::TfToken("Scope"));
    }
    if (urdfRobot.loopJoints.size() > 0)
    {
        stages["stage"]->DefinePrim(
            pxr::SdfPath(primPath).AppendChild(pxr::TfToken("loop_joints")), pxr::TfToken("Scope"));
    }
    stages["stage"]->DefinePrim(pxr::SdfPath("/visuals"), pxr::TfToken("Scope"));
    stages["stage"]->DefinePrim(pxr::SdfPath("/colliders"), pxr::TfToken("Scope"));
    stages["stage"]->DefinePrim(pxr::SdfPath("/meshes"), pxr::TfToken("Scope"));


    addLinksAndJoints(stages, Transform(), chain.baseNode.get(), urdfRobot, robotPrim);

    addLoopJoints(stages, robotPrim, urdfRobot, config);

    setAuthoringLayer(stages["stage"], stages["base_stage"]->GetRootLayer()->GetIdentifier());
    pxr::UsdGeomImageable(stages["stage"]->GetPrimAtPath(pxr::SdfPath("/visuals")))
        .CreateVisibilityAttr()
        .Set(pxr::UsdGeomTokens->invisible);
    pxr::UsdGeomImageable(stages["stage"]->GetPrimAtPath(pxr::SdfPath("/colliders")))
        .CreateVisibilityAttr()
        .Set(pxr::UsdGeomTokens->invisible);
    pxr::UsdGeomImageable(stages["stage"]->GetPrimAtPath(pxr::SdfPath("/meshes")))
        .CreateVisibilityAttr()
        .Set(pxr::UsdGeomTokens->invisible);


    setAuthoringLayer(stages["stage"], stages["physics_stage"]->GetRootLayer()->GetIdentifier());
    pxr::UsdPhysicsCollisionGroup collisionGroupRobot =
        pxr::UsdPhysicsCollisionGroup::Define(stages["stage"], pxr::SdfPath("/colliders/robotCollisionGroup"));
    collisionGroupRobot.GetCollidersCollectionAPI().CreateIncludesRel().AddTarget(robotPrim.GetPrim().GetPath());

    pxr::UsdPhysicsCollisionGroup collisionGroupColliders =
        pxr::UsdPhysicsCollisionGroup::Define(stages["stage"], pxr::SdfPath("/colliders/collidersCollisionGroup"));
    collisionGroupColliders.GetCollidersCollectionAPI().CreateIncludesRel().AddTarget(pxr::SdfPath("/colliders"));
    pxr::UsdRelationship collisionGroupCollidersRel = collisionGroupColliders.CreateFilteredGroupsRel();
    collisionGroupCollidersRel.AddTarget(collisionGroupRobot.GetPrim().GetPath());

    // Add articulation root prim on the actual root link
    pxr::SdfPath rootLinkPrimPath = pxr::SdfPath(primPath.GetString() + "/root_joint");
    if (!config.fixBase)
    {
        rootLinkPrimPath = pxr::SdfPath(primPath.GetString() + "/" + pxr::TfMakeValidIdentifier(urdfRobot.rootLink));
        pxr::UsdPrim rootLinkPrim = stages["stage"]->GetPrimAtPath(rootLinkPrimPath);
        // Apply articulation root schema, but only if there's any joint on the robot. (using the generated joints so we
        // properly ignore fixed joints)
        auto joints = stages["stage"]->GetPrimAtPath(pxr::SdfPath(primPath).AppendChild(pxr::TfToken("joints")));
        if (joints && !joints.GetChildren().empty())
        {
            pxr::UsdPhysicsArticulationRootAPI physicsSchema = pxr::UsdPhysicsArticulationRootAPI::Apply(rootLinkPrim);
            pxr::PhysxSchemaPhysxArticulationAPI physxSchema = pxr::PhysxSchemaPhysxArticulationAPI::Apply(rootLinkPrim);
            // Set schema attributes
            physxSchema.CreateEnabledSelfCollisionsAttr().Set(config.selfCollision);
            // These are reasonable defaults, might want to expose them via the import config in the future.
            physxSchema.CreateSolverPositionIterationCountAttr().Set(32);
            physxSchema.CreateSolverVelocityIterationCountAttr().Set(1);
        }
    }

    setAuthoringLayer(stages["stage"], stages["stage"]->GetRootLayer()->GetIdentifier());
    if (getArticulationRoot)
    {
        return rootLinkPrimPath.GetString();
    }
    else
    {
        return returnPath.GetString();
    }
}
}
}
}
}
