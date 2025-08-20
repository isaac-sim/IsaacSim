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


#include "isaacsim/robot/schema/robot_schema.h"

#include <isaacsim/asset/importer/mjcf/MjcfImporter.h>
#include <isaacsim/core/includes/math/core/Maths.h>
#include <isaacsim/core/includes/utils/Path.h>
#include <pxr/usd/usdGeom/plane.h>
#include <pxr/usd/usdPhysics/fixedJoint.h>
#include <pxr/usd/usdPhysics/sphericalJoint.h>

namespace isaacsim
{
namespace asset
{
namespace importer
{
namespace mjcf
{

struct JointDefinition
{
    std::string type;
    std::string body1;
    std::string body2;
    std::string axis;
    Vec3 position;
    Quat quaternion;
};

std::vector<JointDefinition> analyzeConstraints(const std::vector<MJCFEqualityConnect*>& constraints)
{
    // Group constraints by body pairs
    std::map<std::pair<std::string, std::string>, std::vector<MJCFEqualityConnect*>> constraintGroups;

    for (auto constraint : constraints)
    {
        std::pair<std::string, std::string> bodyPair;
        if (constraint->body1 < constraint->body2)
        {
            bodyPair = { constraint->body1, constraint->body2 };
        }
        else
        {
            bodyPair = { constraint->body2, constraint->body1 };
        }
        constraintGroups[bodyPair].push_back(constraint);
    }

    std::vector<JointDefinition> jointDefinitions;

    for (const auto& [bodyPair, constraints] : constraintGroups)
    {
        JointDefinition jointDef;
        jointDef.body1 = constraints[0]->body1;
        jointDef.body2 = constraints[0]->body2;

        switch (constraints.size())
        {
        case 1:
        {
            // Spherical joint
            jointDef.type = "spherical";
            jointDef.position = constraints[0]->anchor;
            break;
        }
        case 2:
        {
            // Revolute joint
            jointDef.type = "revolute";
            Vec3 axis = constraints[0]->anchor - constraints[1]->anchor;
            axis = Normalize(axis);
            for (int i = 0; i < 3; ++i)
            {
                axis[i] = fabs(axis[i]); // Direction doesn't matter
            }

            int principalAxisIdx = getClosestAxis(axis);
            const char* principalAxes[] = { "X", "Y", "Z" };

            jointDef.axis = principalAxes[principalAxisIdx];
            jointDef.position = (constraints[0]->anchor + constraints[1]->anchor) * 0.5f;

            Vec3 principalAxis;
            principalAxis[principalAxisIdx] = 1.0f;

            if (Length(axis - principalAxis) > 1e-6f)
            {
                jointDef.quaternion = quaternionFromVectors(axis, principalAxis);
            }
            else
            {
                jointDef.quaternion = Quat(1, 0, 0, 0);
            }
            break;
        }
        case 3:
        {
            // Fixed joint
            jointDef.type = "fixed";
            jointDef.position = Vec3(0, 0, 0);
            for (const auto& constraint : constraints)
            {
                jointDef.position = jointDef.position + constraint->anchor;
            }
            jointDef.position = jointDef.position * (1.0f / 3.0f);
            break;
        }
        }
        jointDefinitions.push_back(jointDef);
    }

    return jointDefinitions;
}

MJCFImporter::MJCFImporter(const std::string fullPath, ImportConfig& config)
{
    defaultClassName = "main";

    std::string filePath = fullPath;
    baseDirPath = isaacsim::core::includes::utils::path::MakeRelativePath(filePath, "");

    tinyxml2::XMLDocument doc;
    tinyxml2::XMLElement* root = LoadFile(doc, filePath);
    if (!root)
    {
        return;
    }

    LoadCompiler(root->FirstChildElement("compiler"), compiler);
    // if the mjcf file contains <include file="....">, load the included file as
    // well
    {
        tinyxml2::XMLDocument includeDoc;
        tinyxml2::XMLElement* includeElement = root->FirstChildElement("include");
        tinyxml2::XMLElement* includeRoot = LoadInclude(includeDoc, includeElement, baseDirPath);
        while (includeRoot)
        {
            LoadGlobals(includeRoot, defaultClassName, baseDirPath, worldBody, bodies, actuators, tendons, contacts,
                        equalityConnects, simulationMeshCache, meshes, materials, textures, compiler, classes,
                        jointToActuatorIdx, config);

            includeElement = includeElement->NextSiblingElement("include");
            includeRoot = LoadInclude(includeDoc, includeElement, baseDirPath);
        }
    }

    LoadGlobals(root, defaultClassName, baseDirPath, worldBody, bodies, actuators, tendons, contacts, equalityConnects,
                simulationMeshCache, meshes, materials, textures, compiler, classes, jointToActuatorIdx, config);

    for (int i = 0; i < int(bodies.size()); ++i)
    {
        populateBodyLookup(bodies[i]);
    }

    computeKinematicHierarchy();

    if (!createContactGraph())
    {
        CARB_LOG_ERROR(
            "*** Could not create contact graph to compute collision "
            "groups! Are contacts specified properly?\n");
    }

    // loading is complete if it reaches here
    this->isLoaded = true;
}

MJCFImporter::~MJCFImporter()
{
    for (int i = 0; i < (int)bodies.size(); i++)
    {
        delete bodies[i];
    }
    for (int i = 0; i < (int)actuators.size(); i++)
    {
        delete actuators[i];
    }
    for (int i = 0; i < (int)tendons.size(); i++)
    {
        delete tendons[i];
    }
    for (int i = 0; i < (int)contacts.size(); i++)
    {
        delete contacts[i];
    }
}

void MJCFImporter::applyMaterial(pxr::UsdStageWeakPtr stage, pxr::UsdPrim& prim, MJCFVisualElement* element)
{
    if (element->material != "")
    {
        if (materials.find(element->material) != materials.end())
        {
            MJCFMaterial material = materials.find(element->material)->second;
            MJCFTexture* texture = nullptr;
            if (material.texture != "")
            {
                if (textures.find(material.texture) == textures.end())
                {
                    CARB_LOG_ERROR("Cannot find texture with name %s.\n", material.texture.c_str());
                }
                else
                {
                    texture = &(textures.find(material.texture)->second);
                    // Only MESH type has UV mapping
                    if (element->type == MJCFVisualElement::MESH)
                    {
                        material.project_uvw = false;
                    }
                }
            }
            Vec4 color = Vec4();
            createAndBindMaterial(stage, prim, &material, texture, color, false, materialPaths);
        }
        else
        {
            CARB_LOG_ERROR("Cannot find material with name %s.\n", element->material.c_str());
        }
    }
    else if (element->rgba.x != 0.2f || element->rgba.y != 0.2f || element->rgba.z != 0.2f)
    {
        // Create material with color only
        createAndBindMaterial(stage, prim, nullptr, nullptr, element->rgba, true, materialPaths);
    }
}

void MJCFImporter::populateBodyLookup(MJCFBody* body)
{
    nameToBody[body->name] = body;
    for (MJCFGeom* geom : body->geoms)
    {
        // not a visual geom
        if (!(geom->contype == 0 && geom->conaffinity == 0))
        {
            geomNameToIdx[geom->name] = int(collisionGeoms.size());
            collisionGeoms.push_back(geom);
        }
    }

    for (MJCFBody* childBody : body->bodies)
    {
        populateBodyLookup(childBody);
    }
}

bool MJCFImporter::AddPhysicsEntities(std::unordered_map<std::string, pxr::UsdStageRefPtr> stages,
                                      const Transform trans,
                                      const std::string& rootPrimPath,
                                      const ImportConfig& config)
{

    this->createBodyForFixedJoint = config.createBodyForFixedJoint;

    for (auto const& stage : stages)
    {
        CARB_LOG_INFO("set stage meta data: %s", stage.first.c_str());
        pxr::UsdEditContext context(stages["stage"], stage.second->GetRootLayer());
        setStageMetadata(stage.second, config);
    }

    createRoot(stages, trans, rootPrimPath, config);
    std::string instanceableUSDPath = config.instanceableMeshUsdPath;
    for (int i = 0; i < (int)bodies.size(); i++)
    {
        std::string rootArtPrimPath = rootPrimPath + "/" + SanitizeUsdName(bodies[i]->name);
        pxr::UsdGeomXform rootArtPrim = pxr::UsdGeomXform::Define(stages["base_stage"], pxr::SdfPath(rootArtPrimPath));
        {
            pxr::UsdPrim robotPrim = stages["stage"]->GetPrimAtPath(pxr::SdfPath(rootArtPrimPath));
            pxr::UsdEditContext context(stages["stage"], stages["robot_stage"]->GetRootLayer());
            isaacsim::robot::schema::ApplyRobotAPI(robotPrim);
        }
        pxr::UsdPrim robotPrim = rootArtPrim.GetPrim();
        CreatePhysicsBodyAndJoint(stages, bodies[i], rootPrimPath, rootArtPrimPath, trans, true, rootPrimPath, config,
                                  instanceableUSDPath, robotPrim);
    }
    {
        pxr::UsdEditContext context(stages["stage"], stages["physics_stage"]->GetRootLayer());
        AddContactFilters(stages["stage"]);
        AddTendons(stages["stage"], rootPrimPath);

        addWorldGeomsAndSites(stages, rootPrimPath, config, instanceableUSDPath);
    }
    std::vector<JointDefinition> jointDefinitions = analyzeConstraints(equalityConnects);
    for (const auto& jointDef : jointDefinitions)
    {
        pxr::UsdEditContext context(stages["stage"], stages["physics_stage"]->GetRootLayer());
        std::string jointPath = rootPrimPath + "/loop_joints/" + SanitizeUsdName(jointDef.body1 + "_" + jointDef.body2);
        // Get the body prim paths
        auto body1Path = bodyNameToPrim[jointDef.body1].GetPath();
        auto body2Path = bodyNameToPrim[jointDef.body2].GetPath();
        // Get the body prims and their world poses
        pxr::UsdPrim body1Prim = stages["stage"]->GetPrimAtPath(pxr::SdfPath(body1Path));
        pxr::UsdPrim body2Prim = stages["stage"]->GetPrimAtPath(pxr::SdfPath(body2Path));
        if (!body1Prim || !body2Prim)
        {
            // CARB_LOG_ERROR("Body not found: %s", body1Path.c_str());
            // CARB_LOG_ERROR("Body not found: %s", body2Path.c_str());
            continue;
        }
        pxr::GfMatrix4f body1Transform =
            pxr::GfMatrix4f(isaacsim::core::includes::utils::usd::GetGlobalTransform(body1Prim));
        pxr::GfMatrix4f body2Transform =
            pxr::GfMatrix4f(isaacsim::core::includes::utils::usd::GetGlobalTransform(body2Prim));

        // Joint position in world space
        pxr::GfMatrix4f jointWorldTransform =
            pxr::GfMatrix4f(pxr::GfRotation(pxr::GfQuatf(jointDef.quaternion.x, jointDef.quaternion.y,
                                                         jointDef.quaternion.z, jointDef.quaternion.w)),
                            pxr::GfVec3f(jointDef.position.x, jointDef.position.y, jointDef.position.z)) *
            body1Transform;

        pxr::GfMatrix4f poseJointToChildBody = jointWorldTransform * body2Transform.GetInverse();

        if (jointDef.type == "revolute")
        {
            pxr::UsdPhysicsRevoluteJoint jointPrim =
                pxr::UsdPhysicsRevoluteJoint::Define(stages["stage"], pxr::SdfPath(jointPath));

            // Set the rotation axis
            jointPrim.CreateAxisAttr().Set(pxr::TfToken(jointDef.axis));
        }
        else if (jointDef.type == "spherical")
        {
            pxr::UsdPhysicsSphericalJoint jointPrim =
                pxr::UsdPhysicsSphericalJoint::Define(stages["stage"], pxr::SdfPath(jointPath));
        }
        else if (jointDef.type == "fixed")
        {
            pxr::UsdPhysicsFixedJoint jointPrim =
                pxr::UsdPhysicsFixedJoint::Define(stages["stage"], pxr::SdfPath(jointPath));
        }
        pxr::UsdPhysicsJoint jointPrim = pxr::UsdPhysicsJoint(stages["stage"]->GetPrimAtPath(pxr::SdfPath(jointPath)));

        pxr::SdfPathVector val0{ pxr::SdfPath(body1Path) };
        pxr::SdfPathVector val1{ pxr::SdfPath(body2Path) };

        jointPrim.CreateBody0Rel().SetTargets(val0);
        jointPrim.CreateLocalPos0Attr().Set(pxr::GfVec3f(jointDef.position.x, jointDef.position.y, jointDef.position.z));
        jointPrim.CreateLocalRot0Attr().Set(
            pxr::GfQuatf(jointDef.quaternion.x, jointDef.quaternion.y, jointDef.quaternion.z, jointDef.quaternion.w));

        jointPrim.CreateBody1Rel().SetTargets(val1);
        jointPrim.CreateLocalPos1Attr().Set(poseJointToChildBody.ExtractTranslation());
        jointPrim.CreateLocalRot1Attr().Set(poseJointToChildBody.ExtractRotationQuat());

        jointPrim.CreateExcludeFromArticulationAttr().Set(true);
    }
    return true;
}

void moveVisualGeom(pxr::UsdPrim prim,
                    MJCFGeom* geom,
                    const ImportConfig& config,
                    std::map<std::string, MeshInfo>& simulationMeshCache)
{
    if (prim)
    {
        CARB_LOG_INFO("Prim Exists");
        // set the transformations first
        pxr::GfMatrix4d mat;
        mat.SetIdentity();
        mat.SetTranslateOnly(pxr::GfVec3d(geom->pos.x, geom->pos.y, geom->pos.z));
        mat.SetRotateOnly(pxr::GfQuatd(geom->quat.w, geom->quat.x, geom->quat.y, geom->quat.z));

        pxr::GfMatrix4d scale;
        scale.SetIdentity();
        scale.SetScale(pxr::GfVec3d(config.distanceScale, config.distanceScale, config.distanceScale));
        if (geom->type == MJCFVisualElement::ELLIPSOID)
        {
            scale.SetScale(config.distanceScale * pxr::GfVec3d(geom->size.x, geom->size.y, geom->size.z));
        }
        else if (geom->type == MJCFVisualElement::SPHERE)
        {
            Vec3 cen = geom->pos;
            Quat q = geom->quat;
            // scale.SetIdentity();
            mat.SetTranslateOnly(config.distanceScale * pxr::GfVec3d(cen.x, cen.y, cen.z));
            mat.SetRotateOnly(pxr::GfQuatd(q.w, q.x, q.y, q.z));
        }
        else if (geom->type == MJCFVisualElement::CAPSULE)
        {
            Vec3 cen;
            Quat q;

            if (geom->hasFromTo)
            {
                Vec3 diff = geom->to - geom->from;
                diff = Normalize(diff);
                Vec3 rotVec = Cross(Vec3(1.0f, 0.0f, 0.0f), diff);
                if (Length(rotVec) < 1e-5)
                {
                    rotVec = Vec3(0.0f, 1.0f, 0.0f); // default rotation about y-axis
                }
                else
                {
                    rotVec = Normalize(rotVec); // z axis
                }

                float angle = acos(diff.x);
                cen = 0.5f * (geom->from + geom->to);
                q = QuatFromAxisAngle(rotVec, angle);
            }
            else
            {
                cen = geom->pos;
                q = geom->quat * QuatFromAxisAngle(Vec3(0.0f, 1.0f, 0.0f), -kPi * 0.5f);
            }

            mat.SetTranslateOnly(config.distanceScale * pxr::GfVec3d(cen.x, cen.y, cen.z));
            mat.SetRotateOnly(pxr::GfQuatd(q.w, q.x, q.y, q.z));
        }
        else if (geom->type == MJCFVisualElement::CYLINDER)
        {
            Vec3 cen;
            Quat q;
            if (geom->hasFromTo)
            {
                cen = 0.5f * (geom->from + geom->to);
                Vec3 axis = geom->to - geom->from;
                q = GetRotationQuat(Vec3(0.0f, 0.0f, 1.0f), Normalize(axis));
            }
            else
            {
                cen = geom->pos;
                q = geom->quat;
            }

            mat.SetRotateOnly(pxr::GfQuatd(q.w, q.x, q.y, q.z));
            mat.SetTranslateOnly(config.distanceScale * pxr::GfVec3d(cen.x, cen.y, cen.z));
        }
        else if (geom->type == MJCFVisualElement::BOX)
        {
            Vec3 s = geom->size;
            Vec3 cen = geom->pos;
            Quat q = geom->quat;
            scale.SetScale(config.distanceScale * pxr::GfVec3d(s.x, s.y, s.z));
            mat.SetTranslateOnly(config.distanceScale * pxr::GfVec3d(cen.x, cen.y, cen.z));
            mat.SetRotateOnly(pxr::GfQuatd(q.w, q.x, q.y, q.z));
        }
        else if (geom->type == MJCFVisualElement::MESH)
        {
            Vec3 cen = geom->pos;
            Quat q = geom->quat;

            MeshInfo meshInfo = simulationMeshCache.find(geom->mesh)->second;
            scale.SetScale(config.distanceScale *
                           pxr::GfVec3d(meshInfo.mesh->scale.x, meshInfo.mesh->scale.y, meshInfo.mesh->scale.z));

            mat.SetTranslateOnly(config.distanceScale * pxr::GfVec3d(cen.x, cen.y, cen.z));
            mat.SetRotateOnly(pxr::GfQuatd(q.w, q.x, q.y, q.z));
        }
        else if (geom->type == MJCFVisualElement::PLANE)
        {
            Vec3 cen = geom->pos;
            Quat q = geom->quat;
            scale.SetIdentity();
            mat.SetTranslateOnly(config.distanceScale * pxr::GfVec3d(cen.x, cen.y, cen.z));
            mat.SetRotateOnly(pxr::GfQuatd(q.w, q.x, q.y, q.z));
        }

        pxr::UsdGeomXformable gprim = pxr::UsdGeomXformable(prim);
        gprim.ClearXformOpOrder();
        gprim.AddTranslateOp(pxr::UsdGeomXformOp::PrecisionDouble).Set(mat.ExtractTranslation());
        gprim.AddOrientOp(pxr::UsdGeomXformOp::PrecisionDouble).Set(mat.ExtractRotationQuat());
        gprim.AddScaleOp(pxr::UsdGeomXformOp::PrecisionDouble)
            .Set(pxr::GfVec3d(scale.GetRow3(0)[0], scale.GetRow3(1)[1], scale.GetRow3(2)[2]));
        CARB_LOG_INFO("Done Adding Mesh");
    }
}

bool MJCFImporter::addVisualGeom(pxr::UsdStageWeakPtr stage,
                                 pxr::UsdPrim bodyPrim,
                                 MJCFBody* body,
                                 std::string bodyPath,
                                 const ImportConfig& config,
                                 bool createGeoms,
                                 const std::string rootPrimPath)
{
    bool hasVisualGeoms = false;
    std::string baseVisualsPath = "/visuals/" + bodyPrim.GetName().GetString();
    pxr::UsdPrim visualsPrim = stage->DefinePrim(pxr::SdfPath(bodyPath + "/visuals"), pxr::TfToken("Xform"));
    pxr::UsdPrim basePrim = stage->DefinePrim(pxr::SdfPath(baseVisualsPath), pxr::TfToken("Xform"));

    for (int i = 0; i < (int)body->geoms.size(); i++)
    {
        bool isVisual = body->geoms[i]->contype == 0 && body->geoms[i]->conaffinity == 0;
        if (config.visualizeCollisionGeoms || !body->hasVisual || isVisual)
        {
            // Path where the mesh will be referenced
            pxr::UsdPrim prim;
            std::string geomPath = baseVisualsPath + "/" + SanitizeUsdName(body->geoms[i]->name);

            std::string meshName = body->geoms[i]->name;
            if (body->geoms[i]->type == MJCFVisualElement::MESH)
            {
                meshName = body->geoms[i]->mesh;
            }
            if (body->geoms[i]->type != MJCFVisualElement::MESH ||
                convertedMeshes.find(meshName) == convertedMeshes.end())
            {
                std::string meshPath = "/meshes/" + SanitizeUsdName(meshName);
                pxr::UsdPrim mesh_prim = createPrimitiveGeom(stage, meshPath, body->geoms[i], simulationMeshCache,
                                                             config, materialPaths, true, rootPrimPath, false);
                convertedMeshes[meshName] = mesh_prim.GetPath();
            }
            pxr::SdfPath baseMeshPrimPath = getNextFreePath(
                stage, pxr::SdfPath(baseVisualsPath).AppendChild(pxr::TfToken(SanitizeUsdName(meshName))));
            pxr::UsdPrim baseMeshPrim = stage->DefinePrim(baseMeshPrimPath, pxr::TfToken("Xform"));
            moveVisualGeom(baseMeshPrim, body->geoms[i], config, simulationMeshCache);
            baseMeshPrim.GetReferences().AddInternalReference(convertedMeshes[meshName]);
            // parse material and texture using helper function
            applyMaterial(stage, baseMeshPrim, body->geoms[i]);
            geomPrimMap[body->geoms[i]->name] = prim;
        }
        geomToBodyPrim[body->geoms[i]->name] = bodyPrim;
        hasVisualGeoms = true;
    }
    visualsPrim.GetReferences().AddInternalReference(basePrim.GetPath());
    visualsPrim.SetInstanceable(true);
    return hasVisualGeoms;
}

void MJCFImporter::addVisualSites(
    pxr::UsdStageWeakPtr stage, pxr::UsdPrim bodyPrim, MJCFBody* body, std::string bodyPath, const ImportConfig& config)
{
    for (int i = 0; i < (int)body->sites.size(); i++)
    {
        std::string name = SanitizeUsdName(body->sites[i]->name);
        std::string sitePath = bodyPath + "/sites/" + name;
        pxr::UsdPrim prim;
        if (body->sites[i]->hasGeom)
        {
            stage->DefinePrim(pxr::SdfPath(bodyPath + "/sites"), pxr::TfToken("Xform"));
            prim = stage->DefinePrim(pxr::SdfPath(sitePath), pxr::TfToken("Xform"));
            // prim = createPrimitiveGeom(stage, sitePath, body->sites[i], config, true);
            if (body->sites[i]->type != MJCFVisualElement::MESH || convertedMeshes.find(name) == convertedMeshes.end())
            {
                std::string meshPath = "/meshes/" + SanitizeUsdName(name);
                pxr::UsdPrim mesh_prim = stage->GetPrimAtPath(pxr::SdfPath(meshPath));
                int counter = 0;
                while (mesh_prim)
                {
                    meshPath = std::string("/meshes/" + SanitizeUsdName(name) + "_" + std::to_string(++counter));
                    mesh_prim = stage->GetPrimAtPath(pxr::SdfPath(meshPath));
                }
                mesh_prim = createPrimitiveGeom(stage, meshPath, body->sites[i], config, true);
                convertedMeshes[name] = mesh_prim.GetPath();
            }
            // moveVisualGeom(prim, body->sites[i], config, simulationMeshCache);
            prim.GetReferences().AddInternalReference(convertedMeshes[name]);
            prim.SetInstanceable(true);
            // parse material and texture using helper function
            applyMaterial(stage, prim, body->sites[i]);
        }
        else
        {
            prim = pxr::UsdGeomXform::Define(stage, pxr::SdfPath(sitePath)).GetPrim();
        }
        sitePrimMap[body->sites[i]->name] = prim;
        siteToBodyPrim[body->sites[i]->name] = bodyPrim;
    }
}

void MJCFImporter::addWorldGeomsAndSites(std::unordered_map<std::string, pxr::UsdStageRefPtr> stages,
                                         std::string rootPath,
                                         const ImportConfig& config,
                                         const std::string instanceableUsdPath)
{
    std::string dummyPath = rootPath + "/worldBody";
    pxr::UsdPrim dummyLink;
    {
        pxr::UsdEditContext context(stages["stage"], stages["base_stage"]->GetRootLayer());
        dummyLink = pxr::UsdGeomXform::Define(stages["stage"], pxr::SdfPath(dummyPath)).GetPrim();
    }
    // we have to create a dummy link to place the sites/geoms defined in the
    // world body
    {
        pxr::UsdEditContext context(stages["stage"], stages["physics_stage"]->GetRootLayer());
        pxr::UsdPhysicsArticulationRootAPI physicsSchema = pxr::UsdPhysicsArticulationRootAPI::Apply(dummyLink);
        pxr::PhysxSchemaPhysxArticulationAPI physxSchema = pxr::PhysxSchemaPhysxArticulationAPI::Apply(dummyLink);
        physxSchema.CreateEnabledSelfCollisionsAttr().Set(config.selfCollision);
    }

    for (int i = 0; i < (int)worldBody.geoms.size(); i++)
    {
        std::string bodyPath = dummyPath + "/" + SanitizeUsdName(worldBody.geoms[i]->name);
        auto bodyPathSdf = getNextFreePath(stages["stage"], pxr::SdfPath(bodyPath));
        bodyPath = bodyPathSdf.GetString();
        std::string uniqueName = bodyPathSdf.GetName();
        pxr::UsdPrim bodyLink;
        {
            pxr::UsdEditContext context(stages["stage"], stages["base_stage"]->GetRootLayer());
            bodyLink = pxr::UsdGeomXform::Define(stages["stage"], bodyPathSdf).GetPrim();
        }
        {
            pxr::UsdEditContext context(stages["stage"], stages["physics_stage"]->GetRootLayer());
            pxr::UsdPhysicsRigidBodyAPI physicsAPI = pxr::UsdPhysicsRigidBodyAPI::Apply(bodyLink);
            pxr::PhysxSchemaPhysxRigidBodyAPI::Apply(bodyLink);
            physicsAPI.GetKinematicEnabledAttr().Set(true);
        }

        // createFixedRoot(stage, dummyPath + "/joints/rootJoint_" + uniqueName,
        // dummyPath + "/" + uniqueName);

        bool isVisual = worldBody.geoms[i]->contype == 0 && worldBody.geoms[i]->conaffinity == 0;
        if (isVisual)
        {
            worldBody.hasVisual = true;
        }
        else
        {
            if (worldBody.geoms[i]->type != MJCFVisualElement::PLANE)
            {

                {
                    pxr::UsdEditContext context(stages["stage"], stages["physics_stage"]->GetRootLayer());
                    std::string baseCollisionPath = "/collisions/" + uniqueName;
                    std::string geomPath = bodyPath + baseCollisionPath;
                    stages["physics_stage"]->DefinePrim(pxr::SdfPath(bodyPath + "/collisions"), pxr::TfToken("Xform"));
                    pxr::UsdPrim basePrim =
                        stages["base_stage"]->DefinePrim(pxr::SdfPath(baseCollisionPath), pxr::TfToken("Xform"));
                    pxr::UsdPrim baseMeshPrim = stages["base_stage"]->DefinePrim(
                        getNextFreePath(
                            stages["base_stage"],
                            pxr::SdfPath(baseCollisionPath).AppendChild(pxr::TfToken(SanitizeUsdName(uniqueName)))),
                        pxr::TfToken("Xform"));
                    pxr::UsdPrim prim =
                        stages["physics_stage"]->DefinePrim(pxr::SdfPath(geomPath), pxr::TfToken("Xform"));
                    ;

                    std::string meshName = worldBody.geoms[i]->name;
                    if (worldBody.geoms[i]->type == MJCFVisualElement::MESH)
                    {
                        meshName = worldBody.geoms[i]->mesh;
                    }
                    if (worldBody.geoms[i]->type != MJCFVisualElement::MESH ||
                        convertedMeshes.find(meshName) == convertedMeshes.end())
                    {
                        std::string meshPath = "/meshes/" + SanitizeUsdName(meshName);
                        pxr::UsdPrim mesh_prim =
                            createPrimitiveGeom(stages["base_stage"], meshPath, worldBody.geoms[i], simulationMeshCache,
                                                config, materialPaths, false, rootPath, true);
                        convertedMeshes[meshName] = mesh_prim.GetPath();
                    }
                    moveVisualGeom(baseMeshPrim, worldBody.geoms[i], config, simulationMeshCache);
                    baseMeshPrim.GetReferences().AddInternalReference(convertedMeshes[meshName]);
                    prim.GetReferences().AddInternalReference(basePrim.GetPath());
                    for (auto collisionMeshPrim : pxr::UsdPrimRange(basePrim))
                    {
                        if (collisionMeshPrim.IsA<pxr::UsdGeomGprim>())
                        {
                            applyCollisionGeom(stages["stage"], collisionMeshPrim, config.convexDecomp);
                        }
                    }

                    if (prim)
                    {
                        // applyCollisionGeom(stages["stage"], prim, config.convexDecomp);
                        nameToUsdCollisionPrim[worldBody.geoms[i]->name] = bodyPath;
                    }
                    else
                    {
                        CARB_LOG_ERROR("Collision geom %s could not created", worldBody.geoms[i]->name.c_str());
                    }
                    prim.SetInstanceable(true);
                    // enable collisions on prim
                }
            }
            else
            {
                pxr::UsdEditContext context(stages["stage"], stages["physics_stage"]->GetRootLayer());
                stages["physics_stage"]->DefinePrim(pxr::SdfPath(bodyPath + "/collisions"), pxr::TfToken("Xform"));
                // add collision plane (do not support instanceable asset for the plane)
                pxr::SdfPath collisionPlanePath = pxr::SdfPath(bodyPath + "/collisions/CollisionPlane");
                pxr::UsdGeomPlane collisionPlane = pxr::UsdGeomPlane::Define(stages["physics_stage"], collisionPlanePath);
                collisionPlane.CreatePurposeAttr().Set(pxr::UsdGeomTokens->guide);
                collisionPlane.CreateAxisAttr().Set(pxr::UsdGeomTokens->z);
                pxr::UsdPrim collisionPlanePrim = collisionPlane.GetPrim();
                pxr::UsdPhysicsCollisionAPI::Apply(collisionPlanePrim);

                MJCFGeom* geom = worldBody.geoms[i];
                // set transformations for collision plane
                pxr::GfMatrix4d mat;
                mat.SetIdentity();
                mat.SetTranslateOnly(pxr::GfVec3d(geom->pos.x, geom->pos.y, geom->pos.z));
                mat.SetRotateOnly(pxr::GfQuatd(geom->quat.w, geom->quat.x, geom->quat.y, geom->quat.z));
                pxr::GfMatrix4d scale;
                scale.SetIdentity();
                scale.SetScale(pxr::GfVec3d(config.distanceScale, config.distanceScale, config.distanceScale));
                Vec3 cen = geom->pos;
                Quat q = geom->quat;
                scale.SetIdentity();
                mat.SetTranslateOnly(config.distanceScale * pxr::GfVec3d(cen.x, cen.y, cen.z));
                mat.SetRotateOnly(pxr::GfQuatd(q.w, q.x, q.y, q.z));
                pxr::UsdGeomXformable gprim = pxr::UsdGeomXformable(collisionPlanePrim);
                gprim.ClearXformOpOrder();
                gprim.AddTranslateOp(pxr::UsdGeomXformOp::PrecisionDouble).Set(mat.ExtractTranslation());
                gprim.AddOrientOp(pxr::UsdGeomXformOp::PrecisionDouble).Set(mat.ExtractRotationQuat());
                gprim.AddScaleOp(pxr::UsdGeomXformOp::PrecisionDouble)
                    .Set(pxr::GfVec3d(config.distanceScale, config.distanceScale, config.distanceScale));
            }
        }


        pxr::UsdEditContext context(stages["stage"], stages["base_stage"]->GetRootLayer());
        std::string geomPath = bodyPath + "/visuals/" + uniqueName;
        stages["base_stage"]->DefinePrim(pxr::SdfPath(bodyPath + "/visuals"), pxr::TfToken("Xform"));
        pxr::UsdPrim prim = stages["base_stage"]->DefinePrim(pxr::SdfPath(geomPath), pxr::TfToken("Xform"));
        pxr::SdfPath baseMeshPrimPath = getNextFreePath(
            stages["base_stage"], pxr::SdfPath(geomPath).AppendChild(pxr::TfToken(SanitizeUsdName(uniqueName))));
        pxr::UsdPrim baseMeshPrim = stages["base_stage"]->DefinePrim(baseMeshPrimPath, pxr::TfToken("Xform"));
        std::string meshName = worldBody.geoms[i]->name;
        if (worldBody.geoms[i]->type == MJCFVisualElement::MESH)
        {
            meshName = worldBody.geoms[i]->mesh;
        }
        if (worldBody.geoms[i]->type != MJCFVisualElement::MESH ||
            convertedMeshes.find(meshName) == convertedMeshes.end())
        {
            std::string meshPath = "/meshes/" + SanitizeUsdName(meshName);
            pxr::UsdPrim mesh_prim =
                createPrimitiveGeom(stages["base_stage"], meshPath, worldBody.geoms[i], simulationMeshCache, config,
                                    materialPaths, true, rootPath, false);
            convertedMeshes[meshName] = mesh_prim.GetPath();
        }
        moveVisualGeom(baseMeshPrim, worldBody.geoms[i], config, simulationMeshCache);
        baseMeshPrim.GetReferences().AddInternalReference(convertedMeshes[meshName]);
        baseMeshPrim.SetInstanceable(true);
        // pxr::UsdPrim prim = createPrimitiveGeom(stages["stage"], geomPath, worldBody.geoms[i],
        // simulationMeshCache,
        //                                         config, materialPaths, true, rootPath, false);

        if (!config.visualizeCollisionGeoms && worldBody.hasVisual &&
            !stages["base_stage"]->GetPrimAtPath(pxr::SdfPath(bodyPath + "/visuals")).GetChildren().empty())
        {
            // turn off visibility for collision prim
            pxr::UsdGeomImageable imageable(prim);
            imageable.MakeInvisible();
        }

        // parse material and texture using helper function
        applyMaterial(stages["base_stage"], baseMeshPrim, worldBody.geoms[i]);
        geomPrimMap[worldBody.geoms[i]->name] = prim;

        geomToBodyPrim[worldBody.geoms[i]->name] = bodyLink;
    }

    addVisualSites(stages["base_stage"], dummyLink, &worldBody, dummyPath, config);
}

void MJCFImporter::AddContactFilters(pxr::UsdStageWeakPtr stage)
{
    // adding collision filtering pairs
    for (int i = 0; i < (int)contactGraph.size(); i++)
    {
        std::string& primPath = nameToUsdCollisionPrim[contactGraph[i]->name];
        pxr::UsdPhysicsFilteredPairsAPI filteredPairsAPI =
            pxr::UsdPhysicsFilteredPairsAPI::Apply(stage->GetPrimAtPath(pxr::SdfPath(primPath)));
        std::set<int> neighborhood = contactGraph[i]->adjacentNodes;
        neighborhood.insert(i);
        for (int j = 0; j < (int)contactGraph.size(); j++)
        {
            if (neighborhood.find(j) == neighborhood.end())
            {
                std::string& filteredPrimPath = nameToUsdCollisionPrim[contactGraph[j]->name];
                filteredPairsAPI.CreateFilteredPairsRel().AddTarget(pxr::SdfPath(filteredPrimPath));
            }
        }
    }
}

void createTendonAxisRootAPI(const pxr::UsdPhysicsJoint& rootJointPrim,
                             const pxr::TfToken& name,
                             const pxr::VtArray<float>& coef,
                             const MJCFTendon* t)
{
    pxr::PhysxSchemaPhysxTendonAxisRootAPI rootAPI =
        pxr::PhysxSchemaPhysxTendonAxisRootAPI::Apply(rootJointPrim.GetPrim(), name);

    float Stiffness = t->stiffness;
    if (Stiffness == 0)
    {
        CARB_LOG_WARN(
            "Tendon %s has no stiffness, setting to 1.0 - Please check your model once imported to validate the behavior",
            name.GetText());
        Stiffness = 1.0;
    }
    if (t->limited)
    {
        rootAPI.CreateLowerLimitAttr().Set(t->range[0]);
        rootAPI.CreateUpperLimitAttr().Set(t->range[1]);
        // If the tendon is limited, we need to set the limit stiffness instead of the Stiffness.
        rootAPI.CreateLimitStiffnessAttr().Set(Stiffness);
        rootAPI.CreateStiffnessAttr().Set(0.0f);
    }
    else
    {
        rootAPI.CreateLimitStiffnessAttr().Set(0.0f);
        rootAPI.CreateStiffnessAttr().Set(Stiffness);
    }

    if (t->springlength >= 0)
    {
        rootAPI.CreateRestLengthAttr().Set(t->springlength);
    }

    rootAPI.CreateDampingAttr().Set(t->damping);
    pxr::VtArray<float> forcecoeffs;
    for (int i = 0; i < (int)coef.size(); i++)
    {
        auto forcecoeff = coef[i];
        if (abs(forcecoeff) < 1e-6)
        {
            forcecoeff = 1.0f * sign(forcecoeff);
        }
        else
        {
            forcecoeff = 1.0f / forcecoeff;
        }
        forcecoeffs.push_back(forcecoeff);
    }
    pxr::PhysxSchemaPhysxTendonAxisAPI(rootAPI, rootAPI.GetName()).CreateForceCoefficientAttr().Set(forcecoeffs);

    pxr::PhysxSchemaPhysxTendonAttachmentAPI(rootAPI, rootAPI.GetName()).CreateGearingAttr().Set(coef);
}

void MJCFImporter::AddTendons(pxr::UsdStageWeakPtr stage, std::string rootPath)
{
    // adding tendons
    size_t tendonIndex = 0;
    for (const MJCFTendon* t : tendons)
    {
        if (t->type == MJCFTendon::FIXED)
        {
            // setting the joint with the lowest kinematic hierarchy number as the
            // TendonAxisRoot
            std::string tendonName = t->name;
            if (tendonName.empty())
            {
                tendonName = SanitizeUsdName("tendon_" + std::to_string(tendonIndex++));
            }
            if (t->fixedJoints.size() != 0)
            {
                MJCFTendon::FixedJoint* rootJoint = t->fixedJoints[0];
                for (int i = 0; i < (int)t->fixedJoints.size(); i++)
                {
                    if (jointToKinematicHierarchy[t->fixedJoints[i]->joint] < jointToKinematicHierarchy[rootJoint->joint])
                    {
                        rootJoint = t->fixedJoints[i];
                    }
                }


                // adding the TendonAxisRoot api to the root joint
                pxr::VtArray<float> coef = { rootJoint->coef };

                if (revoluteJointsMap.find(rootJoint->joint) != revoluteJointsMap.end())
                {
                    createTendonAxisRootAPI(revoluteJointsMap[rootJoint->joint], pxr::TfToken(tendonName), coef, t);
                }
                else if (prismaticJointsMap.find(rootJoint->joint) != prismaticJointsMap.end())
                {
                    createTendonAxisRootAPI(prismaticJointsMap[rootJoint->joint], pxr::TfToken(tendonName), coef, t);
                }
                else if (d6JointsMap.find(rootJoint->joint) != d6JointsMap.end())
                {
                    createTendonAxisRootAPI(d6JointsMap[rootJoint->joint], pxr::TfToken(tendonName), coef, t);
                }

                else
                {
                    CARB_LOG_ERROR(
                        "Joint %s required for tendon %s cannot be found", rootJoint->joint.c_str(), t->name.c_str());
                }

                // adding TendonAxis api to the other joints in the tendon
                for (int i = 0; i < (int)t->fixedJoints.size(); i++)
                {
                    if (t->fixedJoints[i]->joint != rootJoint->joint)
                    {
                        MJCFTendon::FixedJoint* childJoint = t->fixedJoints[i];
                        pxr::VtArray<float> coef = { childJoint->coef };
                        if (revoluteJointsMap.find(childJoint->joint) != revoluteJointsMap.end())
                        {
                            pxr::UsdPhysicsRevoluteJoint childJointPrim = revoluteJointsMap[childJoint->joint];
                            pxr::PhysxSchemaPhysxTendonAxisAPI axisAPI = pxr::PhysxSchemaPhysxTendonAxisAPI::Apply(
                                childJointPrim.GetPrim(), pxr::TfToken(tendonName));
                            axisAPI.CreateGearingAttr().Set(coef);
                            pxr::VtArray<float> forcecoeffs;
                            for (int i = 0; i < (int)coef.size(); i++)
                            {
                                auto forcecoeff = coef[i];
                                if (abs(forcecoeff) < 1e-6)
                                {
                                    forcecoeff = 1.0f * sign(forcecoeff);
                                }
                                else
                                {
                                    forcecoeff = 1.0f / forcecoeff;
                                }
                                forcecoeffs.push_back(forcecoeff);
                            }
                            axisAPI.CreateForceCoefficientAttr().Set(forcecoeffs);
                        }
                        else if (prismaticJointsMap.find(childJoint->joint) != prismaticJointsMap.end())
                        {
                            pxr::UsdPhysicsPrismaticJoint childJointPrim = prismaticJointsMap[childJoint->joint];
                            pxr::PhysxSchemaPhysxTendonAxisAPI axisAPI = pxr::PhysxSchemaPhysxTendonAxisAPI::Apply(
                                childJointPrim.GetPrim(), pxr::TfToken(tendonName));
                            axisAPI.CreateGearingAttr().Set(coef);
                            pxr::VtArray<float> forcecoeffs;
                            for (int i = 0; i < (int)coef.size(); i++)
                            {
                                auto forcecoeff = coef[i];
                                if (abs(forcecoeff) < 1e-6)
                                {
                                    forcecoeff = 1.0f * sign(forcecoeff);
                                }
                                else
                                {
                                    forcecoeff = 1.0f / forcecoeff;
                                }
                                forcecoeffs.push_back(forcecoeff);
                            }
                            axisAPI.CreateForceCoefficientAttr().Set(forcecoeffs);
                        }
                        else if (d6JointsMap.find(childJoint->joint) != d6JointsMap.end())
                        {
                            pxr::UsdPhysicsJoint childJointPrim = d6JointsMap[childJoint->joint];
                            pxr::PhysxSchemaPhysxTendonAxisAPI axisAPI = pxr::PhysxSchemaPhysxTendonAxisAPI::Apply(
                                childJointPrim.GetPrim(), pxr::TfToken(tendonName));
                            axisAPI.CreateGearingAttr().Set(coef);
                            pxr::VtArray<float> forcecoeffs;
                            for (int i = 0; i < (int)coef.size(); i++)
                            {
                                auto forcecoeff = coef[i];
                                if (abs(forcecoeff) < 1e-6)
                                {
                                    forcecoeff = 1.0f * sign(forcecoeff);
                                }
                                else
                                {
                                    forcecoeff = 1.0f / forcecoeff;
                                }
                                forcecoeffs.push_back(forcecoeff);
                            }
                            axisAPI.CreateForceCoefficientAttr().Set(forcecoeffs);
                        }
                        else
                        {
                            CARB_LOG_ERROR("Joint %s required for tendon %s cannot be found", childJoint->joint.c_str(),
                                           t->name.c_str());
                        }
                    }
                }
            }
            else
            {
                CARB_LOG_ERROR("%s cannot be added since it has no specified joints to attach to.", t->name.c_str());
            }
        }
        else if (t->type == MJCFTendon::SPATIAL)
        {
            std::map<std::string, int> attachmentNames;
            bool isFirstAttachment = true;

            if (t->spatialAttachments.size() > 0)
            {
                for (auto it = t->spatialBranches.begin(); it != t->spatialBranches.end(); it++)
                {
                    std::vector<MJCFTendon::SpatialAttachment*> attachments = it->second;
                    pxr::UsdPrim parentPrim;
                    std::string parentName;
                    for (int i = (int)attachments.size() - 1; i >= 0; --i)
                    {
                        MJCFTendon::SpatialAttachment* attachment = attachments[i];
                        std::string name;
                        pxr::UsdPrim linkPrim;
                        bool hasLink = false;
                        if (attachment->type == MJCFTendon::SpatialAttachment::GEOM)
                        {
                            name = attachment->geom;
                            if (geomToBodyPrim.find(name) != geomToBodyPrim.end())
                            {
                                linkPrim = geomToBodyPrim[name];
                                hasLink = true;
                            }
                        }
                        else if (attachment->type == MJCFTendon::SpatialAttachment::SITE)
                        {
                            name = attachment->site;
                            if (siteToBodyPrim.find(name) != siteToBodyPrim.end())
                            {
                                linkPrim = siteToBodyPrim[name];
                                hasLink = true;
                            }
                        }
                        if (!hasLink)
                        {
                            pxr::UsdPrim dummyLink = stage->GetPrimAtPath(pxr::SdfPath(rootPath + "/worldBody"));

                            // check if they are part of the world sites/geoms
                            if (attachment->type == MJCFTendon::SpatialAttachment::GEOM)
                            {
                                name = attachment->geom;
                                linkPrim = dummyLink;
                                geomToBodyPrim[name] = dummyLink;
                                hasLink = true;
                            }
                            else if (attachment->type == MJCFTendon::SpatialAttachment::SITE)
                            {
                                name = attachment->site;
                                linkPrim = dummyLink;
                                hasLink = true;
                            }

                            if (!hasLink)
                            {
                                // we shouldn't be here...
                                CARB_LOG_ERROR(
                                    "Error adding attachment %s. Failed to find "
                                    "attached link.\n",
                                    name.c_str());
                            }
                        }

                        // create additional attachments if duplicates are found
                        if (attachmentNames.find(name) != attachmentNames.end())
                        {
                            attachmentNames[name]++;
                            name = name + "_" + std::to_string(attachmentNames[name] - 1);
                        }
                        else
                        {
                            attachmentNames[name] = 0;
                        }

                        // setting the first attachment link as the AttachmentRoot
                        if (isFirstAttachment)
                        {
                            isFirstAttachment = false;
                            parentPrim = linkPrim;
                            parentName = name;
                            auto rootApi =
                                pxr::PhysxSchemaPhysxTendonAttachmentRootAPI::Apply(linkPrim, pxr::TfToken(name));
                            pxr::GfVec3f localPos = GetLocalPos(*attachment);
                            pxr::PhysxSchemaPhysxTendonAttachmentAPI(rootApi, pxr::TfToken(name))
                                .CreateLocalPosAttr()
                                .Set(localPos);
                            rootApi.CreateStiffnessAttr().Set(t->stiffness);
                            rootApi.CreateDampingAttr().Set(t->damping);
                        }
                        // last attachment point
                        else if (i == 0)
                        {
                            auto leafApi =
                                pxr::PhysxSchemaPhysxTendonAttachmentLeafAPI::Apply(linkPrim, pxr::TfToken(name));
                            pxr::PhysxSchemaPhysxTendonAttachmentAPI(leafApi, pxr::TfToken(name))
                                .CreateParentLinkRel()
                                .AddTarget(parentPrim.GetPath());
                            pxr::PhysxSchemaPhysxTendonAttachmentAPI(leafApi, pxr::TfToken(name))
                                .CreateParentAttachmentAttr()
                                .Set(pxr::TfToken(parentName));
                            pxr::GfVec3f localPos = GetLocalPos(*attachment);
                            pxr::PhysxSchemaPhysxTendonAttachmentAPI(leafApi, pxr::TfToken(name))
                                .CreateLocalPosAttr()
                                .Set(localPos);
                        }
                        // intermediate attachment point
                        else
                        {
                            auto attachmentApi =
                                pxr::PhysxSchemaPhysxTendonAttachmentAPI::Apply(linkPrim, pxr::TfToken(name));
                            attachmentApi.CreateParentLinkRel().AddTarget(parentPrim.GetPath());
                            attachmentApi.CreateParentAttachmentAttr().Set(pxr::TfToken(parentName));
                            pxr::GfVec3f localPos = GetLocalPos(*attachment);
                            attachmentApi.CreateLocalPosAttr().Set(localPos);
                        }

                        // set current body as parent
                        parentName = name;
                        parentPrim = linkPrim;
                    }
                }
            }
            else
            {
                CARB_LOG_ERROR(
                    "Spatial tendon %s cannot be added since it has no "
                    "attachments specified.",
                    t->name.c_str());
            }
        }
        else
        {
            CARB_LOG_ERROR(
                "Tendon %s is not a fixed or spatial tendon. Only fixed "
                "and spatial tendons are currently supported.",
                t->name.c_str());
        }
    }
}

pxr::GfVec3f MJCFImporter::GetLocalPos(MJCFTendon::SpatialAttachment attachment)
{
    pxr::GfVec3f localPos;
    if (attachment.type == MJCFTendon::SpatialAttachment::GEOM)
    {
        if (geomToBodyPrim.find(attachment.geom) != geomToBodyPrim.end())
        {
            const pxr::UsdPrim rootPrim = geomToBodyPrim[attachment.geom];
            const pxr::UsdPrim geomPrim = geomPrimMap[attachment.geom];
            pxr::GfVec3d geomTranslate = pxr::UsdGeomXformable(geomPrim)
                                             .ComputeLocalToWorldTransform(pxr::UsdTimeCode::Default())
                                             .ExtractTranslation();
            pxr::GfVec3d linkTranslate = pxr::UsdGeomXformable(rootPrim)
                                             .ComputeLocalToWorldTransform(pxr::UsdTimeCode::Default())
                                             .ExtractTranslation();
            localPos = pxr::GfVec3f(geomTranslate - linkTranslate);
        }
    }
    else if (attachment.type == MJCFTendon::SpatialAttachment::SITE)
    {
        if (siteToBodyPrim.find(attachment.site) != siteToBodyPrim.end())
        {
            pxr::UsdPrim rootPrim = siteToBodyPrim[attachment.site];
            pxr::UsdPrim sitePrim = sitePrimMap[attachment.site];
            pxr::GfVec3d siteTranslate = pxr::UsdGeomXformable(sitePrim)
                                             .ComputeLocalToWorldTransform(pxr::UsdTimeCode::Default())
                                             .ExtractTranslation();
            pxr::GfVec3d linkTranslate = pxr::UsdGeomXformable(rootPrim)
                                             .ComputeLocalToWorldTransform(pxr::UsdTimeCode::Default())
                                             .ExtractTranslation();
            localPos = pxr::GfVec3f(siteTranslate - linkTranslate);
        }
    }
    return localPos;
}

void MJCFImporter::CreatePhysicsBodyAndJoint(std::unordered_map<std::string, pxr::UsdStageRefPtr> stages,
                                             MJCFBody* body,
                                             const std::string& rootPath,
                                             const std::string& rootPrimPath,
                                             const Transform& trans,
                                             const bool isRoot,
                                             const std::string& parentBodyPath,
                                             const ImportConfig& config,
                                             const std::string& instanceableUsdPath,
                                             pxr::UsdPrim& robotPrim)
{
    pxr::UsdEditContext context(stages["stage"], stages["base_stage"]->GetRootLayer());

    Transform myTrans;
    myTrans = trans * Transform(body->pos, body->quat);
    int numJoints = (int)body->joints.size();
    if ((!createBodyForFixedJoint) && ((body->joints.size() == 0) && (!isRoot)))
    {
        CARB_LOG_WARN(
            "RigidBodySpec with no joint will have no geometry for now, "
            "to avoid instability of fixed joint!");
    }
    else
    {
        pxr::UsdEditContext context(stages["stage"], stages["base_stage"]->GetRootLayer());
        if (!body->inertial && body->geoms.size() == 0)
        {
            CARB_LOG_WARN("*** Neither inertial nor geometries where specified for %s", body->name.c_str());
            // return;
        }
        std::string bodyPath = rootPrimPath + "/" + SanitizeUsdName(body->name);
        pxr::UsdGeomXformable bodyPrim = createBody(stages["stage"], bodyPath, myTrans, config);
        {
            pxr::UsdEditContext context(stages["stage"], stages["robot_stage"]->GetRootLayer());
            pxr::UsdPrim linkPrim = stages["stage"]->GetPrimAtPath(pxr::SdfPath(bodyPath));
            isaacsim::robot::schema::ApplyLinkAPI(linkPrim);
            auto linksRel = stages["stage"]
                                ->GetPrimAtPath(robotPrim.GetPath())
                                .GetRelationship(isaacsim::robot::schema::relationNames.at(
                                    isaacsim::robot::schema::Relations::ROBOT_LINKS));
            pxr::SdfPathVector targets;
            linksRel.GetTargets(&targets);
            targets.push_back(linkPrim.GetPath());
            linksRel.SetTargets(targets);
        }

        // add Rigid Body
        if (bodyPrim)
        {
            bodyNameToPrim[body->name] = bodyPrim.GetPrim();
            applyRigidBody(stages, bodyPrim, body, config);
        }
        else
        {
            CARB_LOG_ERROR("Body prim %s could not created", body->name.c_str());
            // return;
        }
        if (isRoot)
        {
            pxr::UsdGeomXform rootPrim = pxr::UsdGeomXform::Define(stages["stage"], pxr::SdfPath(bodyPath));
            {
                pxr::UsdEditContext context(stages["stage"], stages["physics_stage"]->GetRootLayer());
                applyArticulationAPI(stages, rootPrim, config);
                if (config.fixBase || numJoints == 0)
                {
                    // enable multiple root joints
                    createFixedRoot(stages, rootPath + "/joints/rootJoint_" + SanitizeUsdName(body->name), bodyPath);
                }
            }
        }

        pxr::UsdPrim collisionPrim =
            stages["physics_stage"]->DefinePrim(pxr::SdfPath(bodyPath + "/collisions"), pxr::TfToken("Xform"));
        // bool hasVisualGeoms = false;
        // add collision geoms first to detect whether visuals are available
        {

            pxr::UsdEditContext context(stages["stage"], stages["physics_stage"]->GetRootLayer());
            std::string uniqueName = bodyPrim.GetPrim().GetName().GetString();
            std::string baseCollisionsPath = "/collisions/" + uniqueName;
            pxr::UsdPrim basePrim =
                stages["physics_stage"]->DefinePrim(pxr::SdfPath(baseCollisionsPath), pxr::TfToken("Xform"));
            for (int i = 0; i < (int)body->geoms.size(); i++)
            {
                bool isVisual = body->geoms[i]->contype == 0 && body->geoms[i]->conaffinity == 0;

                if (isVisual)
                {
                    body->hasVisual = true;
                }
                else
                {
                    pxr::UsdEditContext context(stages["stage"], stages["physics_stage"]->GetRootLayer());
                    std::string meshName = body->geoms[i]->name;
                    if (body->geoms[i]->type == MJCFVisualElement::MESH)
                    {
                        meshName = body->geoms[i]->mesh;
                    }
                    if (body->geoms[i]->type != MJCFVisualElement::MESH ||
                        convertedMeshes.find(meshName) == convertedMeshes.end())
                    {
                        std::string meshPath = "/meshes/" + SanitizeUsdName(meshName);
                        pxr::UsdPrim mesh_prim =
                            createPrimitiveGeom(stages["base_stage"], meshPath, body->geoms[i], simulationMeshCache,
                                                config, materialPaths, false, rootPrimPath, true);
                        convertedMeshes[meshName] = mesh_prim.GetPath();
                    }
                    pxr::SdfPath baseMeshPrimPath = getNextFreePath(
                        stages["base_stage"],
                        pxr::SdfPath(baseCollisionsPath).AppendChild(pxr::TfToken(SanitizeUsdName(meshName))));
                    pxr::UsdPrim baseMeshPrim = stages["base_stage"]->DefinePrim(baseMeshPrimPath, pxr::TfToken("Xform"));
                    moveVisualGeom(baseMeshPrim, body->geoms[i], config, simulationMeshCache);
                    baseMeshPrim.GetReferences().AddInternalReference(convertedMeshes[meshName]);
                    nameToUsdCollisionPrim[body->geoms[i]->name] = bodyPath;
                }
            }
            collisionPrim.GetReferences().AddInternalReference(basePrim.GetPath());
            for (auto collisionMeshPrim : pxr::UsdPrimRange(basePrim))
            {
                if (collisionMeshPrim.IsA<pxr::UsdGeomGprim>())
                {
                    applyCollisionGeom(stages["stage"], collisionMeshPrim, config.convexDecomp);
                }
            }
            // applyCollisionGeom(stages["stage"], basePrim, config.convexDecomp);
            // applyCollisionGeom(stages["stage"], collisionPrim, config.convexDecomp);
            collisionPrim.SetInstanceable(true);
        }
        // add visual geoms
        addVisualGeom(stages["base_stage"], bodyPrim.GetPrim(), body, bodyPath, config, false, rootPrimPath);
        // add sites
        if (config.importSites)
        {
            addVisualSites(stages["base_stage"], bodyPrim.GetPrim(), body, bodyPath, config);
        }
        if (!config.visualizeCollisionGeoms)
        {
            // turn off visibility for collision prim
            pxr::UsdGeomImageable imageable(collisionPrim);
            imageable.MakeInvisible();
        }
        // add joints
        // create joint linked to parent

        // if the body is a root and there is no joint for the root, do not need to
        // go into this if statement to create any joints. However, if the root body
        // has joints but the import config has fixBase set to true, also no need to
        // create any additional joints.
        {
            addJoints(
                stages, rootPath, parentBodyPath, bodyPath, body, config, trans, myTrans, isRoot, numJoints, robotPrim);
            // recursively create children's bodies
            for (int i = 0; i < (int)body->bodies.size(); i++)
            {
                CreatePhysicsBodyAndJoint(stages, body->bodies[i], rootPath, rootPrimPath, myTrans, false, bodyPath,
                                          config, instanceableUsdPath, robotPrim);
            }
        }
    }
}

void MJCFImporter::addJoints(std::unordered_map<std::string, pxr::UsdStageRefPtr> stages,
                             const std::string& rootPath,
                             const std::string& parentBodyPath,
                             const std::string& bodyPath,
                             MJCFBody* body,
                             const ImportConfig& config,
                             const Transform& trans0,
                             const Transform& trans1,
                             const bool isRoot,
                             const int numJoints,
                             pxr::UsdPrim& robotPrim)
{
    pxr::UsdEditContext context(stages["stage"], stages["physics_stage"]->GetRootLayer());
    if (!(isRoot && (numJoints == 0 || config.fixBase)))
    {
        // jointSpec transform
        Transform origin;
        if (body->joints.size() > 0)
        {
            // origin at last joint (deepest)
            origin.p = body->joints[0]->pos;
        }
        else
        {
            origin.p = Vec3(0.0f, 0.0f, 0.0f);
        }

        // compute joint frame and map joint axes to D6 axes
        int axisMap[3] = { 0, 1, 2 };
        computeJointFrame(origin, axisMap, body);

        origin = trans1 * origin;

        Transform ptran = trans0;
        Transform mtran = trans1;

        Transform ppose = (Inverse(ptran)) * origin;
        Transform cpose = (Inverse(mtran)) * origin;

        std::string jointPath = rootPath + "/joints/" + SanitizeUsdName(body->name);

        int numJoints = (int)body->joints.size();
        if (numJoints == 0)
        {
            Transform poseJointToParentBody = Transform(body->pos, body->quat);
            Transform poseJointToChildBody = Transform();
            pxr::UsdPhysicsJoint jointPrim = createFixedJoint(stages["stage"], jointPath, poseJointToParentBody,
                                                              poseJointToChildBody, parentBodyPath, bodyPath, config);
        }
        else if (numJoints == 1)
        {
            Transform poseJointToParentBody = Transform(ppose.p, ppose.q);
            Transform poseJointToChildBody = Transform(cpose.p, cpose.q);
            MJCFJoint* joint = body->joints.front();


            auto actuatorIterator = jointToActuatorIdx.find(joint->name);
            int actuatorIdx = actuatorIterator != jointToActuatorIdx.end() ? actuatorIterator->second : -1;
            MJCFActuator* actuator = nullptr;
            if (actuatorIdx != -1)
            {
                actuatorIdx = actuatorIterator->second;
                actuator = actuators[actuatorIdx];
            }
            std::string jointPath = rootPath + "/joints/" + SanitizeUsdName(joint->name);

            if (joint->type == MJCFJoint::HINGE)
            {
                pxr::UsdPhysicsRevoluteJoint jointPrim =
                    pxr::UsdPhysicsRevoluteJoint::Define(stages["stage"], pxr::SdfPath(jointPath));
                initPhysicsJoint(jointPrim, poseJointToParentBody, poseJointToChildBody, parentBodyPath, bodyPath,
                                 config.distanceScale);
                applyPhysxJoint(jointPrim, joint);

                // joint was aligned such that its hinge axis is aligned with local
                // x-axis.
                jointPrim.CreateAxisAttr().Set(pxr::UsdPhysicsTokens->x);
                if (joint->limited)
                {
                    jointPrim.CreateLowerLimitAttr().Set(joint->range.x * 180 / kPi);
                    jointPrim.CreateUpperLimitAttr().Set(joint->range.y * 180 / kPi);
                }

                pxr::PhysxSchemaPhysxLimitAPI physxLimitAPI =
                    pxr::PhysxSchemaPhysxLimitAPI::Apply(jointPrim.GetPrim(), pxr::TfToken(pxr::UsdPhysicsTokens->x));
                physxLimitAPI.CreateStiffnessAttr().Set(joint->stiffness);
                physxLimitAPI.CreateDampingAttr().Set(joint->damping);

                revoluteJointsMap[joint->name] = jointPrim;

                createJointDrives(jointPrim, joint, actuator, "angular", config);
            }
            else if (joint->type == MJCFJoint::SLIDE)
            {
                pxr::UsdPhysicsPrismaticJoint jointPrim =
                    pxr::UsdPhysicsPrismaticJoint::Define(stages["stage"], pxr::SdfPath(jointPath));
                initPhysicsJoint(jointPrim, poseJointToParentBody, poseJointToChildBody, parentBodyPath, bodyPath,
                                 config.distanceScale);
                applyPhysxJoint(jointPrim, joint);

                // joint was aligned such that its hinge axis is aligned with local
                // x-axis.
                jointPrim.CreateAxisAttr().Set(pxr::UsdPhysicsTokens->x);

                if (joint->limited)
                {
                    jointPrim.CreateLowerLimitAttr().Set(config.distanceScale * joint->range.x);
                    jointPrim.CreateUpperLimitAttr().Set(config.distanceScale * joint->range.y);
                }

                pxr::PhysxSchemaPhysxLimitAPI physxLimitAPI =
                    pxr::PhysxSchemaPhysxLimitAPI::Apply(jointPrim.GetPrim(), pxr::TfToken(pxr::UsdPhysicsTokens->x));
                physxLimitAPI.CreateStiffnessAttr().Set(joint->stiffness);
                physxLimitAPI.CreateDampingAttr().Set(joint->damping);

                prismaticJointsMap[joint->name] = jointPrim;

                createJointDrives(jointPrim, joint, actuator, "linear", config);
            }
            else if (joint->type == MJCFJoint::BALL)
            {
                pxr::UsdPhysicsJoint jointPrim = createD6Joint(stages["stage"], jointPath, poseJointToParentBody,
                                                               poseJointToChildBody, parentBodyPath, bodyPath, config);
                applyPhysxJoint(jointPrim, body->joints[0]);

                // lock all translational axes to create a D6 joint.
                std::string translationAxes[6] = { "transX", "transY", "transZ" };
                for (int i = 0; i < 3; ++i)
                {
                    pxr::UsdPhysicsLimitAPI limitAPI =
                        pxr::UsdPhysicsLimitAPI::Apply(jointPrim.GetPrim(), pxr::TfToken(translationAxes[i]));
                    limitAPI.CreateLowAttr().Set(1.0f);
                    limitAPI.CreateHighAttr().Set(-1.0f);
                }

                d6JointsMap[joint->name] = jointPrim;
            }
            else if (joint->type == MJCFJoint::FREE)
            {
            }
            else
            {
                CARB_LOG_WARN(
                    "*** Only hinge, slide, ball, and free joints are "
                    "supported by MJCF importer");
            }
        }
        else
        {
            Transform poseJointToParentBody = Transform(ppose.p, ppose.q);
            Transform poseJointToChildBody = Transform(cpose.p, cpose.q);
            pxr::UsdPhysicsJoint jointPrim = createD6Joint(stages["stage"], jointPath, poseJointToParentBody,
                                                           poseJointToChildBody, parentBodyPath, bodyPath, config);
            applyPhysxJoint(jointPrim, body->joints[0]);

            // TODO: this needs to be updated to support all joint types and
            // combinations set joint limits
            for (int jid = 0; jid < (int)body->joints.size(); jid++)
            {
                // all locked
                for (int k = 0; k < 6; ++k)
                {
                    body->joints[jid]->velocityLimits[k] = 100.f;
                }

                if (body->joints[jid]->type != MJCFJoint::HINGE && body->joints[jid]->type != MJCFJoint::SLIDE)
                {
                    CARB_LOG_WARN(
                        "*** Only hinge and slide joints are supported by "
                        "MJCF importer");
                    continue;
                }

                if (body->joints[jid]->ref != 0.0f)
                {
                    CARB_LOG_WARN("Don't know how to deal with joint with ref != 0 yet");
                }

                // actuators - TODO: how do we set this part? what do we need to set?
                auto actuatorIterator = jointToActuatorIdx.find(body->joints[jid]->name);
                int actuatorIdx = actuatorIterator != jointToActuatorIdx.end() ? actuatorIterator->second : -1;
                MJCFActuator* actuator = nullptr;
                if (actuatorIdx != -1)
                {
                    actuatorIdx = actuatorIterator->second;
                    actuator = actuators[actuatorIdx];
                }

                applyJointLimits(jointPrim, body->joints[jid], actuator, axisMap, jid, numJoints, config);

                d6JointsMap[body->joints[jid]->name] = jointPrim;
            }
        }
        {
            pxr::UsdEditContext context(stages["stage"], stages["robot_stage"]->GetRootLayer());
            pxr::UsdPrim jointPrim = stages["stage"]->GetPrimAtPath(pxr::SdfPath(jointPath));
            if (jointPrim)
            {
                isaacsim::robot::schema::ApplyJointAPI(jointPrim);
                auto jointsRel = stages["stage"]
                                     ->GetPrimAtPath(robotPrim.GetPath())
                                     .GetRelationship(isaacsim::robot::schema::relationNames.at(
                                         isaacsim::robot::schema::Relations::ROBOT_JOINTS));
                pxr::SdfPathVector targets;
                jointsRel.GetTargets(&targets);
                targets.push_back(jointPrim.GetPath());
                jointsRel.SetTargets(targets);
            }
        }
    }
}

void MJCFImporter::computeJointFrame(Transform& origin, int* axisMap, const MJCFBody* body)
{
    if (body->joints.size() == 0)
    {
        origin.q = Quat();
    }
    else
    {
        if (body->joints.size() == 1)
        {
            // align D6 x-axis with the given axis
            origin.q = GetRotationQuat({ 1.0f, 0.0f, 0.0f }, body->joints[0]->axis);
        }
        else if (body->joints.size() == 2)
        {
            Quat Q = GetRotationQuat(body->joints[0]->axis, { 1.0f, 0.0f, 0.0f });
            Vec3 a = { 1.0f, 0.0f, 0.0f };
            Vec3 b = Normalize(Rotate(Q, body->joints[1]->axis));

            if (fabs(Dot(a, b)) > 1e-4f)
            {
                CARB_LOG_WARN("*** Non-othogonal joint axes are not supported");
                // exit(0);
            }

            // map third axis to D6 y- or z-axis and compute third axis accordingly
            Vec3 c;
            if (std::fabs(Dot(b, { 0.0f, 1.0f, 0.0f })) > std::fabs(Dot(b, { 0.0f, 0.0f, 1.0f })))
            {
                axisMap[1] = 1;
                c = Normalize(Cross(body->joints[0]->axis, body->joints[1]->axis));
                Matrix33 M(Normalize(body->joints[0]->axis), Normalize(body->joints[1]->axis), c);
                origin.q = Quat(M);
            }
            else
            {
                axisMap[1] = 2;
                axisMap[2] = 1;
                c = Normalize(Cross(body->joints[1]->axis, body->joints[0]->axis));
                Matrix33 M(Normalize(body->joints[0]->axis), c, Normalize(body->joints[1]->axis));
                origin.q = Quat(M);
            }
        }
        else if (body->joints.size() == 3)
        {
            Quat Q = GetRotationQuat(body->joints[0]->axis, { 1.0f, 0.0f, 0.0f });
            Vec3 a = { 1.0f, 0.0f, 0.0f };
            Vec3 b = Normalize(Rotate(Q, body->joints[1]->axis));
            Vec3 c = Normalize(Rotate(Q, body->joints[2]->axis));

            if (fabs(Dot(a, b)) > 1e-4f || fabs(Dot(a, c)) > 1e-4f || fabs(Dot(b, c)) > 1e-4f)
            {
                CARB_LOG_WARN("*** Non-othogonal joint axes are not supported");
                // exit(0);
            }

            if (std::fabs(Dot(b, { 0.0f, 1.0f, 0.0f })) > std::fabs(Dot(b, { 0.0f, 0.0f, 1.0f })))
            {
                axisMap[1] = 1;
                axisMap[2] = 2;
                Matrix33 M(Normalize(body->joints[0]->axis), Normalize(body->joints[1]->axis),
                           Normalize(body->joints[2]->axis));
                origin.q = Quat(M);
            }
            else
            {
                axisMap[1] = 2;
                axisMap[2] = 1;
                Matrix33 M(Normalize(body->joints[0]->axis), Normalize(body->joints[2]->axis),
                           Normalize(body->joints[1]->axis));
                origin.q = Quat(M);
            }
        }
        else
        {
            CARB_LOG_ERROR("*** Don't know how to handle >3 joints per body pair");
            exit(0);
        }
    }
}

bool MJCFImporter::contactBodyExclusion(MJCFBody* body1, MJCFBody* body2)
{
    // Assumes that contact graph is already set up
    // handle current geoms first
    for (MJCFGeom* geom1 : body1->geoms)
    {
        if (geom1->conaffinity & geom1->contype)
        {
            for (MJCFGeom* geom2 : body2->geoms)
            {
                if (geom2->conaffinity & geom2->contype)
                {
                    auto index1 = geomNameToIdx.find(geom1->name);
                    auto index2 = geomNameToIdx.find(geom2->name);
                    if (index1 == geomNameToIdx.end() || index2 == geomNameToIdx.end())
                    {
                        return false;
                    }

                    int geomIndex1 = index1->second;
                    int geomIndex2 = index2->second;

                    contactGraph[geomIndex1]->adjacentNodes.erase(geomIndex2);
                    contactGraph[geomIndex2]->adjacentNodes.erase(geomIndex1);
                }
            }
        }
    }
    return true;
}

bool MJCFImporter::createContactGraph()
{
    contactGraph = std::vector<ContactNode*>();

    // initialize nodes with no contacts
    for (int i = 0; i < int(collisionGeoms.size()); ++i)
    {
        ContactNode* node = new ContactNode();
        node->name = collisionGeoms[i]->name;
        contactGraph.push_back(node);
    }

    // First check pairwise compatability with contype/conaffinity
    for (int i = 0; i < int(collisionGeoms.size()) - 1; ++i)
    {
        for (int j = i + 1; j < int(collisionGeoms.size()); ++j)
        {
            MJCFGeom* geom1 = collisionGeoms[i];
            MJCFGeom* geom2 = collisionGeoms[j];
            if ((geom1->contype & geom2->conaffinity) || (geom2->contype && geom1->conaffinity))
            {
                contactGraph[i]->adjacentNodes.insert(j);
                contactGraph[j]->adjacentNodes.insert(i);
            }
        }
    }

    // Handle contact specifications
    for (auto& contact : contacts)
    {
        if (contact->type == MJCFContact::PAIR)
        {
            auto index1 = geomNameToIdx.find(contact->geom1);
            auto index2 = geomNameToIdx.find(contact->geom2);
            if (index1 == geomNameToIdx.end() || index2 == geomNameToIdx.end())
            {
                return false;
            }

            int geomIndex1 = index1->second;
            int geomIndex2 = index2->second;

            contactGraph[geomIndex1]->adjacentNodes.insert(geomIndex2);
            contactGraph[geomIndex2]->adjacentNodes.insert(geomIndex1);
        }
        else if (contact->type == MJCFContact::EXCLUDE)
        {
            // this is on the level of bodies, not geoms
            auto body1 = nameToBody.find(contact->body1);
            auto body2 = nameToBody.find(contact->body2);
            if (body1 == nameToBody.end() || body2 == nameToBody.end())
            {
                return false;
            }
            if (!contactBodyExclusion(body1->second, body2->second))
            {
                return false;
            }
        }
    }

    return true;
}

void MJCFImporter::computeKinematicHierarchy()
{
    // prepare bodyQueue for breadth-first search
    for (int i = 0; i < int(bodies.size()); i++)
    {
        bodyQueue.push(bodies[i]);
    }

    int level_num = 0;
    int num_bodies_at_level;

    while (bodyQueue.size() != 0)
    {
        num_bodies_at_level = (int)bodyQueue.size();

        for (int i = 0; i < num_bodies_at_level; i++)
        {

            MJCFBody* body = bodyQueue.front();
            bodyQueue.pop();
            for (MJCFBody* childBody : body->bodies)
            {
                bodyQueue.push(childBody);
            }

            for (MJCFJoint* joint : body->joints)
            {
                jointToKinematicHierarchy[joint->name] = level_num;
            }
        }
        level_num += 1;
    }
}


} // namespace mjcf
} // namespace importer
} // namespace asset
} // namespace isaacsim
