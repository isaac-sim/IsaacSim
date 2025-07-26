// SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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


// clang-format off
#include <pch/UsdPCH.h>
// clang-format on

#include <isaacsim/core/cloner/Cloner.h>
#include <omni/fabric/FabricUSD.h>
#include <omni/fabric/IFabric.h>
#include <omni/fabric/connectivity/Connectivity.h>
#include <usdrt/hierarchy/IFabricHierarchy.h>
#include <usdrt/population/IUtils.h>
#include <usdrt/scenegraph/usd/usd/stage.h>

const omni::fabric::IToken* omni::fabric::Token::iToken = nullptr;

void initializeRigidBodyBatched(const std::vector<usdrt::SdfPath>& rbPaths,
                                omni::fabric::IStageReaderWriter* iStageReaderWriter,
                                omni::fabric::StageReaderWriterId stageInProgress,
                                usdrt::UsdStageRefPtr usdrtStage)
{

    omni::fabric::Token::iToken = carb::getFramework()->tryAcquireInterface<omni::fabric::IToken>();

    omni::fabric::StageReaderWriter stage(stageInProgress);

    struct RigidBodyInitData
    {
        bool validPath;
        bool usdPath;
        omni::fabric::PathC path;
        bool dynamicBody;
    };

    const size_t numPaths = rbPaths.size();
    std::vector<RigidBodyInitData> rbInitData(numPaths);

    usdrt::TfToken kinematicEnabledToken("physics:kinematicEnabled");

    for (size_t i = 0; i < numPaths; i++)
    {
        const omni::fabric::PathC pathC(rbPaths[i]);
        usdrt::UsdPrim usdrt_prim = usdrtStage->GetPrimAtPath(usdrt::SdfPath(pathC));
        if (!usdrt_prim)
        {
            rbInitData[i].validPath = false;
            continue;
        }
        else
        {
            bool kinematic = false;
            usdrt_prim.GetAttribute(kinematicEnabledToken).Get(&kinematic);
            rbInitData[i].path = omni::fabric::PathC(rbPaths[i]);
            rbInitData[i].dynamicBody = !kinematic;
            rbInitData[i].validPath = true;
            rbInitData[i].usdPath = false;
        }
    }

    omni::fabric::Token rigidBodyWorldPositionToken = omni::fabric::Token::iToken->getHandle("_rigidBodyWorldPosition");
    omni::fabric::Token rigidBodyWorldOrientationToken =
        omni::fabric::Token::iToken->getHandle("_rigidBodyWorldOrientation");
    omni::fabric::Token rigidBodyWorldScaleToken = omni::fabric::Token::iToken->getHandle("_rigidBodyWorldScale");


    omni::fabric::Token worldForceToken = omni::fabric::Token::iToken->getHandle("_worldForce");
    omni::fabric::Token worldTorqueToken = omni::fabric::Token::iToken->getHandle("_worldTorque");

    omni::fabric::Token physXPtrToken = omni::fabric::Token::iToken->getHandle("_physxPtr");

    omni::fabric::Token dynamicBodyToken = omni::fabric::Token::iToken->getHandle("dynamicBody");

    omni::fabric::Token linVelToken = omni::fabric::Token::iToken->getHandle("physics:velocity");
    omni::fabric::Token angVelToken = omni::fabric::Token::iToken->getHandle("physics:angularVelocity");

    omni::fabric::Type float3Type(omni::fabric::BaseDataType::eFloat, 3, 0, omni::fabric::AttributeRole::eNone);
    omni::fabric::Type double3Type(omni::fabric::BaseDataType::eDouble, 3, 0, omni::fabric::AttributeRole::eNone);
    omni::fabric::Type quatType(omni::fabric::BaseDataType::eFloat, 4, 0, omni::fabric::AttributeRole::eQuaternion);
    omni::fabric::Type matrix4dType(omni::fabric::BaseDataType::eDouble, 16, 0, omni::fabric::AttributeRole::eMatrix);

    omni::fabric::Type ptrType(omni::fabric::BaseDataType::eUInt64, 1, 0, omni::fabric::AttributeRole::eNone);

    omni::fabric::Type tagType(omni::fabric::BaseDataType::eTag, 1, 0, omni::fabric::AttributeRole::eNone);

    for (const RigidBodyInitData& initData : rbInitData)
    {
        if (!initData.validPath)
            continue;

        if (initData.dynamicBody)
        {
            std::array<omni::fabric::AttrNameAndType, 9> attrNameTypeVec = {
                omni::fabric::AttrNameAndType(ptrType, physXPtrToken),

                omni::fabric::AttrNameAndType(float3Type, worldForceToken),
                omni::fabric::AttrNameAndType(float3Type, worldTorqueToken),

                omni::fabric::AttrNameAndType(float3Type, linVelToken),
                omni::fabric::AttrNameAndType(float3Type, angVelToken),

                // token
                omni::fabric::AttrNameAndType(tagType, dynamicBodyToken),

                // world trs attributes
                omni::fabric::AttrNameAndType(double3Type, rigidBodyWorldPositionToken),
                omni::fabric::AttrNameAndType(quatType, rigidBodyWorldOrientationToken),
                omni::fabric::AttrNameAndType(float3Type, rigidBodyWorldScaleToken),
            };

            stage.createAttributes(initData.path, attrNameTypeVec);
        }
    }
}


/**
 * @brief Creates clones of a source prim at specified target paths in the USD stage
 * @details This function performs a fabric clone operation, creating copies of a source prim at multiple
 * target locations within the same stage.
 *
 * @param[in] stageId The unique identifier of the USD stage where cloning will occur
 * @param[in] source_prim_path The path to the source prim that will be cloned, this prim should be a valid USD prim
 * @param[in] prim_paths Vector of target paths where clones will be created, these prims will be created in Fabric
 * stage not in USD stage
 *
 * @return true if cloning was successful, false otherwise
 *
 * @warning The source prim must exist at the specified path
 */
bool isaacsim::core::cloner::fabricClone(long int stageId,
                                         const std::string& source_prim_path,
                                         const std::vector<std::string>& prim_paths)
{

    omni::fabric::IStageReaderWriter* iStageReaderWriter = carb::getCachedInterface<omni::fabric::IStageReaderWriter>();
    if (!iStageReaderWriter)
    {
        CARB_LOG_ERROR("fabricClone - IStageReaderWriter not found");
        return false;
    }

    {
        CARB_PROFILE_ZONE(0, "fabricClone - fabric get or create");

        omni::fabric::FabricId fabricId{};
        omni::fabric::StageReaderWriterId stageReaderWriterId = iStageReaderWriter->get(stageId);
        if (!stageReaderWriterId.id)
        {
            // populate fabric
            auto iSimStageWithHistory = carb::getCachedInterface<omni::fabric::ISimStageWithHistory>();

            if (!iSimStageWithHistory)
            {
                CARB_LOG_ERROR("fabricClone - ISimStageWithHistory not found");
                return false;
            }

            iSimStageWithHistory->getOrCreate(stageId, 1, { 1, 30 }, omni::fabric::GpuComputeType::eCuda);
            iStageReaderWriter->create(stageId, 0);

            stageReaderWriterId = iStageReaderWriter->get(stageId);
            fabricId = iStageReaderWriter->getFabricId(stageReaderWriterId);

            auto iFabricUsd = carb::getCachedInterface<omni::fabric::IFabricUsd>();
            if (!iFabricUsd)
            {
                CARB_LOG_ERROR("fabricClone - IFabricUsd not found");
                return false;
            }
            iFabricUsd->setEnableChangeNotifies(fabricId, false);

            auto populationUtils = omni::core::createType<usdrt::population::IUtils>();
            if (!populationUtils)
            {
                CARB_LOG_ERROR("fabricClone - IUtils not found");
                return false;
            }
            populationUtils->setEnableUsdNoticeHandling(stageId, fabricId, true);

            // Fill the stage in progress with USD values
            {
                CARB_PROFILE_ZONE(0, "fabricClone - fabric populate");
                populationUtils->populateFromUsd(stageReaderWriterId, stageId,
                                                 omni::fabric::asInt(PXR_NS::SdfPath::AbsoluteRootPath()), nullptr, 0.0);
            }
        }
        else
        {
            fabricId = iStageReaderWriter->getFabricId(stageReaderWriterId);

            auto iFabricUsd = carb::getCachedInterface<omni::fabric::IFabricUsd>();
            if (!iFabricUsd)
            {
                CARB_LOG_ERROR("fabricClone - IFabricUsd not found");
                return false;
            }
            iFabricUsd->setEnableChangeNotifies(fabricId, false);

            auto populationUtils = omni::core::createType<usdrt::population::IUtils>();
            if (!populationUtils)
            {
                CARB_LOG_ERROR("fabricClone - IUtils not found");
                return false;
            }
            populationUtils->setEnableUsdNoticeHandling(stageId, fabricId, true);

            // Fill the stage in progress with USD values
            {
                CARB_PROFILE_ZONE(0, "fabricClone - fabric populate");
                populationUtils->populateFromUsd(stageReaderWriterId, stageId,
                                                 omni::fabric::asInt(PXR_NS::SdfPath::AbsoluteRootPath()), nullptr, 0.0);
            }

            fabricId = iStageReaderWriter->getFabricId(stageReaderWriterId);
            usdrt::UsdStageRefPtr stage = usdrt::UsdStage::Attach(stageId, stageReaderWriterId);
            stage->SynchronizeToFabric(usdrt::TimeChange::NoUpdate);
        }

        {
            // prepare the rigid bodies before cloning creating all attributes
            {
                CARB_PROFILE_ZONE(0, "FabricManager::resume:rigidBodyInitialization");
                usdrt::UsdStageRefPtr usdrtStage = usdrt::UsdStage::Attach(stageId, stageReaderWriterId);
                initializeRigidBodyBatched(usdrtStage->GetPrimsWithAppliedAPIName(usdrt::TfToken("PhysicsRigidBodyAPI")),
                                           iStageReaderWriter, stageReaderWriterId, usdrtStage);
            }
        }
    }

    {
        omni::fabric::StageReaderWriterId stageInProgress = iStageReaderWriter->get(stageId);
        omni::fabric::IStageReaderWriterLegacy* isrwLegacy =
            carb::getCachedInterface<omni::fabric::IStageReaderWriterLegacy>();
        if (!isrwLegacy)
        {
            CARB_LOG_ERROR("fabricClone - IStageReaderWriterLegacy not found");
            return false;
        }

        omni::fabric::Path world_envs_env_0(source_prim_path.c_str());

        // convert prim_paths to fabric paths, this is quite unfortunate and will slow down the cloning process
        std::vector<omni::fabric::Path> list_of_clones;
        {
            CARB_PROFILE_ZONE(0, "fabricClone - prim_paths to fabric paths");
            for (const auto& prim_path : prim_paths)
            {
                // compare if its not the same as the source prim path
                if (prim_path != source_prim_path)
                {
                    list_of_clones.push_back(omni::fabric::Path(prim_path.c_str()));
                }
            }
        }

        if (!list_of_clones.empty())
        {
            CARB_PROFILE_ZONE(0, "fabricClone - fabric batch clone");
            isrwLegacy->batchClone(iStageReaderWriter->getFabricId(stageInProgress), world_envs_env_0,
                                   { (const omni::fabric::PathC*)list_of_clones.data(), list_of_clones.size() });
        }
        else
        {
            return false;
        }
    }

    return true;
}
