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

#define CARB_EXPORTS

// clang-format off
#include <pch/UsdPCH.h>
// clang-format on

#include <carb/PluginUtils.h>
#include <carb/logging/Log.h>

#include <isaacsim/asset/importer/mjcf/IMjcf.h>
#include <isaacsim/asset/importer/mjcf/MjcfImporter.h>
#include <isaacsim/core/includes/utils/Path.h>
#include <isaacsim/robot/schema/robot_schema.h>
#include <omni/ext/IExt.h>
#include <omni/kit/IApp.h>
#include <omni/kit/IStageUpdate.h>
#include <pxr/usd/sdf/layer.h>
#include <pxr/usd/sdf/payload.h>
#include <pxr/usd/usd/payloads.h>
#include <pxr/usd/usdPhysics/collisionGroup.h>

#include <stdio.h>

#define EXTENSION_NAME "isaacsim.asset.importer.mjcf.plugin"

using namespace carb;
using namespace isaacsim::core::includes::utils::path;
const struct carb::PluginImplDesc kPluginImpl = { EXTENSION_NAME, "MJCF Utilities", "NVIDIA",
                                                  carb::PluginHotReload::eEnabled, "dev" };

CARB_PLUGIN_IMPL(kPluginImpl, isaacsim::asset::importer::mjcf::Mjcf)
CARB_PLUGIN_IMPL_DEPS(omni::kit::IApp, carb::logging::ILogging)

namespace
{
bool FileExists(const std::string& path)
{
    std::ifstream file(path);
    return file.good();
}

void OpenOrCreateNew(pxr::UsdStageRefPtr& stage, const std::string& stage_identifier)
{
    if (pxr::UsdStage::IsSupportedFile(stage_identifier))
    {
        if (FileExists(stage_identifier))
        {
            stage = pxr::UsdStage::Open(stage_identifier);
        }
        if (!stage)
        {
            CARB_LOG_INFO("Creating Stage: %s", stage_identifier.c_str());
            stage = pxr::UsdStage::CreateNew(stage_identifier);
        }
        else
        {
            stage->GetRootLayer()->Clear();
            stage->Save();
        }
    }
    else
    {
        CARB_LOG_ERROR("Stage identifier %s is not supported", stage_identifier.c_str());
    }
}


// passed in from python
void createAssetFromMJCF(const char* fileName,
                         const char* primName,
                         isaacsim::asset::importer::mjcf::ImportConfig& config,
                         const std::string& stage_identifier = "")
{
    isaacsim::asset::importer::mjcf::MJCFImporter mjcf(fileName, config);
    if (!mjcf.isLoaded)
    {
        printf("cannot load mjcf xml file\n");
    }
    Transform trans = Transform();

    bool save_stage = true;
    std::unordered_map<std::string, pxr::UsdStageRefPtr> stages;
    stages["stage"] = pxr::UsdStageRefPtr();
    stages["sensor_stage"] = pxr::UsdStageRefPtr();
    stages["physics_stage"] = pxr::UsdStageRefPtr();
    stages["base_stage"] = pxr::UsdStageRefPtr();
    stages["robot_stage"] = pxr::UsdStageRefPtr();

    bool multi_layer = true;
    std::string name = "";
    if (stage_identifier != "" && pxr::UsdStage::IsSupportedFile(stage_identifier))
    {
        OpenOrCreateNew(stages["stage"], stage_identifier);
        std::string directoryPath = pxr::TfGetPathName(stage_identifier);
        name = pxr::TfStringGetBeforeSuffix(pxr::TfGetBaseName(stage_identifier));

        // CARB_LOG_WARN(stage_identifier.c_str());
        // CARB_LOG_WARN(directoryPath.c_str());
        // CARB_LOG_WARN(name.c_str());

        OpenOrCreateNew(stages["sensor_stage"], directoryPath + "/configuration/" + name + "_sensor.usd");
        OpenOrCreateNew(stages["physics_stage"], directoryPath + "/configuration/" + name + "_physics.usd");
        OpenOrCreateNew(stages["base_stage"], directoryPath + "/configuration/" + name + "_base.usd");
        OpenOrCreateNew(stages["robot_stage"], directoryPath + "/configuration/" + name + "_robot.usd");


        config.makeDefaultPrim = true;
    }
    if (!stages["stage"]) // If all else fails, import on current stage
    {
        CARB_LOG_INFO("Importing MJCF to Current Stage");
        // Get the 'active' USD stage from the USD stage cache.
        const std::vector<pxr::UsdStageRefPtr> allStages = pxr::UsdUtilsStageCache::Get().GetAllStages();
        if (allStages.size() != 1)
        {
            CARB_LOG_ERROR("Cannot determine the 'active' USD stage (%zu stages present in the USD stage cache).",
                           allStages.size());
            return;
        }
        stages["stage"] = allStages[0];
        std::string identifier = pxr::TfStringGetBeforeSuffix((stages["stage"]->GetRootLayer()->GetIdentifier()));
        if (pxr::TfStringStartsWith(identifier, "anon:"))
        {
            CARB_LOG_WARN("Creating Asset in an in-memory stage, will not create layered structure");
            stages["sensor_stage"] = stages["stage"];
            stages["physics_stage"] = stages["stage"];
            stages["base_stage"] = stages["stage"];
            stages["robot_stage"] = stages["stage"];
            multi_layer = false;
            save_stage = false;
        }
        else
        {
            std::string directoryPath = pxr::TfGetPathName(identifier);
            std::string name = pxr::TfGetBaseName(identifier);
            OpenOrCreateNew(stages["sensor_stage"], directoryPath + "/configuration/" + name + "_sensor.usd");
            OpenOrCreateNew(stages["physics_stage"], directoryPath + "/configuration/" + name + "_physics.usd");
            OpenOrCreateNew(stages["base_stage"], directoryPath + "/configuration/" + name + "_base.usd");
            OpenOrCreateNew(stages["robot_stage"], directoryPath + "/configuration/" + name + "_robot.usd");
        }
    }

    for (auto& stage : stages)
    {

        {
            pxr::UsdGeomSetStageUpAxis(stage.second, pxr::UsdGeomTokens->z);
            pxr::UsdGeomSetStageMetersPerUnit(stage.second, 1.0 / config.distanceScale);
        }
    }
    // Create Root prim before adding sublayers
    createRoot(stages, trans, primName, config);


    pxr::SdfLayerRefPtr rootLayer = stages["stage"]->GetRootLayer();
    if (multi_layer)
    {
        auto subLayerPaths = rootLayer->GetSubLayerPaths();
        auto sensor_layer = resolve_relative(
            stages["stage"]->GetRootLayer()->GetIdentifier(), stages["sensor_stage"]->GetRootLayer()->GetIdentifier());
        if (std::find(subLayerPaths.begin(), subLayerPaths.end(), sensor_layer) == subLayerPaths.end())
        {
            subLayerPaths.push_back(sensor_layer);
        }
        auto physics_layer = resolve_relative(
            stages["stage"]->GetRootLayer()->GetIdentifier(), stages["physics_stage"]->GetRootLayer()->GetIdentifier());
        if (std::find(subLayerPaths.begin(), subLayerPaths.end(), physics_layer) == subLayerPaths.end())
        {
            subLayerPaths.push_back(physics_layer);
        }
        auto physics_subLayerPaths = stages["physics_stage"]->GetRootLayer()->GetSubLayerPaths();
        auto base_layer = resolve_relative(stages["physics_stage"]->GetRootLayer()->GetIdentifier(),
                                           stages["base_stage"]->GetRootLayer()->GetIdentifier());
        if (std::find(physics_subLayerPaths.begin(), physics_subLayerPaths.end(), base_layer) ==
            physics_subLayerPaths.end())
        {
            physics_subLayerPaths.push_back(base_layer);
        }
        auto robot_subLayerPaths = stages["robot_stage"]->GetRootLayer()->GetSubLayerPaths();
        auto robot_layer = resolve_relative(stages["base_stage"]->GetRootLayer()->GetIdentifier(),
                                            stages["robot_stage"]->GetRootLayer()->GetIdentifier());
        if (std::find(robot_subLayerPaths.begin(), robot_subLayerPaths.end(), robot_layer) == robot_subLayerPaths.end())
        {
            stages["base_stage"]->GetRootLayer()->GetSubLayerPaths().push_back(robot_layer);
        }
    }
    std::string result = "";

    if (stages["stage"])
    {
        for (auto const& stage : stages)
        {
            pxr::UsdGeomSetStageMetersPerUnit(stage.second, 1.0 / config.distanceScale);
            stage.second->DefinePrim(pxr::SdfPath(std::string(primName)), pxr::TfToken("Xform"));
        }
        if (config.createPhysicsScene)
        {
            pxr::UsdPhysicsScene scene =
                pxr::UsdPhysicsScene::Define(stages["physics_stage"], pxr::SdfPath("/physicsScene"));
            scene.CreateGravityDirectionAttr().Set(pxr::GfVec3f(0.0f, 0.0f, -1.0));
            scene.CreateGravityMagnitudeAttr().Set(9.81f * config.distanceScale);

            pxr::PhysxSchemaPhysxSceneAPI physxSceneAPI = pxr::PhysxSchemaPhysxSceneAPI::Apply(
                stages["physics_stage"]->GetPrimAtPath(pxr::SdfPath("/physicsScene")));
            physxSceneAPI.CreateEnableCCDAttr().Set(true);
            physxSceneAPI.CreateEnableStabilizationAttr().Set(true);
            physxSceneAPI.CreateEnableGPUDynamicsAttr().Set(false);

            physxSceneAPI.CreateBroadphaseTypeAttr().Set(pxr::TfToken("MBP"));
            physxSceneAPI.CreateSolverTypeAttr().Set(pxr::TfToken("TGS"));
        }

        {

            pxr::UsdEditContext context(stages["stage"], stages["base_stage"]->GetRootLayer());
            stages["stage"]->DefinePrim(pxr::SdfPath(std::string(primName) + "/Looks"), pxr::TfToken("Scope"));
            stages["stage"]->DefinePrim(pxr::SdfPath(std::string(primName) + "/joints"), pxr::TfToken("Scope"));
            stages["stage"]->DefinePrim(pxr::SdfPath("/meshes"), pxr::TfToken("Scope"));
            stages["stage"]->DefinePrim(pxr::SdfPath("/visuals"), pxr::TfToken("Scope"));
            stages["physics_stage"]->DefinePrim(pxr::SdfPath("/collisions"), pxr::TfToken("Scope"));

            pxr::UsdGeomImageable(stages["stage"]->GetPrimAtPath(pxr::SdfPath("/visuals")))
                .CreateVisibilityAttr()
                .Set(pxr::UsdGeomTokens->invisible);
            pxr::UsdGeomImageable(stages["stage"]->GetPrimAtPath(pxr::SdfPath("/collisions")))
                .CreateVisibilityAttr()
                .Set(pxr::UsdGeomTokens->invisible);
            pxr::UsdGeomImageable(stages["stage"]->GetPrimAtPath(pxr::SdfPath("/meshes")))
                .CreateVisibilityAttr()
                .Set(pxr::UsdGeomTokens->invisible);

            {
                pxr::UsdPhysicsCollisionGroup collisionGroupRobot = pxr::UsdPhysicsCollisionGroup::Define(
                    stages["physics_stage"], pxr::SdfPath("/collisions/robotCollisionGroup"));
                collisionGroupRobot.GetCollidersCollectionAPI().CreateIncludesRel().AddTarget(
                    pxr::SdfPath(std::string(primName)));

                pxr::UsdPhysicsCollisionGroup collisionGroupColliders = pxr::UsdPhysicsCollisionGroup::Define(
                    stages["physics_stage"], pxr::SdfPath("/collisions/collidersCollisionGroup"));
                collisionGroupColliders.GetCollidersCollectionAPI().CreateIncludesRel().AddTarget(
                    pxr::SdfPath("/collisions"));
                pxr::UsdRelationship collisionGroupCollidersRel = collisionGroupColliders.CreateFilteredGroupsRel();
                collisionGroupCollidersRel.AddTarget(collisionGroupRobot.GetPrim().GetPath());
            }
        }
        if (!mjcf.AddPhysicsEntities(stages, trans, primName, config))
        {
            printf("no physics entities found!\n");
        }
        // CARB_LOG_WARN("Import Done, saving");
        if (save_stage)
        {
            CARB_LOG_INFO("Saving Stage %s", stages["stage"]->GetRootLayer()->GetIdentifier().c_str());
            CARB_LOG_INFO("Saving Sensor Stage %s", stages["sensor_stage"]->GetRootLayer()->GetIdentifier().c_str());
            CARB_LOG_INFO("Saving Physics Stage %s", stages["physics_stage"]->GetRootLayer()->GetIdentifier().c_str());
            CARB_LOG_INFO("Saving Base Stage %s", stages["base_stage"]->GetRootLayer()->GetIdentifier().c_str());
            stages["stage"]->Save();
            stages["sensor_stage"]->Save();
            stages["physics_stage"]->Save();
            stages["base_stage"]->Save();
            stages["robot_stage"]->Save();
            pxr::UsdEditContext context(stages["stage"], stages["stage"]->GetRootLayer());
            result = stages["base_stage"]->GetDefaultPrim().GetPath().GetString();
            // Remove the physics and sensor stages from sublayers, and add them as payloads through variants
            rootLayer->GetSubLayerPaths().clear();

            auto root_prim = stages["stage"]->GetDefaultPrim();


            pxr::UsdVariantSets variantSets = root_prim.GetVariantSets();
            pxr::UsdVariantSet physics = variantSets.AddVariantSet("Physics");
            physics.AddVariant("None");
            physics.SetVariantSelection("None");
            {
                pxr::UsdEditContext ctxt(physics.GetVariantEditContext());
                root_prim.GetReferences().AddReference(
                    resolve_relative(stages["stage"]->GetRootLayer()->GetIdentifier(),
                                     stages["base_stage"]->GetRootLayer()->GetIdentifier()));
                auto joints = stages["stage"]->GetPrimAtPath(root_prim.GetPath().AppendPath(pxr::SdfPath("joints")));
                if (joints)
                {
                    joints.SetActive(false);
                }
                auto loop_joints =
                    stages["stage"]->GetPrimAtPath(root_prim.GetPath().AppendPath(pxr::SdfPath("loop_joints")));
                if (loop_joints)
                {
                    loop_joints.SetActive(false);
                }
                auto root_joint =
                    stages["stage"]->GetPrimAtPath(root_prim.GetPath().AppendPath(pxr::SdfPath("root_joint")));
                if (root_joint)
                {
                    root_joint.SetActive(false);
                }
            }
            physics.AddVariant("PhysX");
            physics.SetVariantSelection("PhysX");
            {
                pxr::UsdEditContext ctxt(physics.GetVariantEditContext());
                stages["stage"]->GetDefaultPrim().GetPayloads().AddPayload(
                    pxr::SdfPayload(resolve_relative(stages["stage"]->GetRootLayer()->GetIdentifier(),
                                                     stages["physics_stage"]->GetRootLayer()->GetIdentifier())));
            }


            pxr::UsdVariantSet sensor = variantSets.AddVariantSet("Sensor");
            sensor.AddVariant("None");
            sensor.AddVariant("Sensors");
            sensor.SetVariantSelection("Sensors");
            {
                pxr::UsdEditContext ctxt(sensor.GetVariantEditContext());
                stages["stage"]->GetDefaultPrim().GetPayloads().AddPayload(
                    pxr::SdfPayload(resolve_relative(stages["stage"]->GetRootLayer()->GetIdentifier(),
                                                     stages["sensor_stage"]->GetRootLayer()->GetIdentifier())));
            }


            CARB_LOG_INFO("Import Done, saving");

            stages["stage"]->Save();
        }
    }
}

} // namespace

CARB_EXPORT void carbOnPluginStartup()
{
    CARB_LOG_INFO("Startup MJCF Extension");
}

CARB_EXPORT void carbOnPluginShutdown()
{
}

void fillInterface(isaacsim::asset::importer::mjcf::Mjcf& iface)
{
    using namespace isaacsim::asset::importer::mjcf;
    memset(&iface, 0, sizeof(iface));
    // iface.helloWorld = helloWorld;
    iface.createAssetFromMJCF = createAssetFromMJCF;
    // iface.importRobot = importRobot;
}
