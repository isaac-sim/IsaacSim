// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#pragma once

// #include <carb/filesystem/IFileSystem.h>
#include <isaacsim/core/includes/Conversions.h>
#include <isaacsim/core/includes/PhysicsEngine.h>
#include <isaacsim/core/includes/UsdUtilities.h>
#include <isaacsim/core/simulation_manager/ISimulationManager.h>
#include <omni/physics/tensors/IArticulationView.h>
#include <omni/physics/tensors/IRigidBodyView.h>
#include <omni/physics/tensors/ISimulationView.h>
#include <omni/physics/tensors/TensorApi.h>
using namespace omni::physics::tensors;

namespace isaacsim
{
namespace core
{
namespace includes
{

/**
 * @namespace transforms
 * @brief Utilities for manipulating transforms of USD prims and physics objects.
 * @details
 * Provides functions for modifying transforms of objects in the scene, handling both:
 * - Physics-based objects (articulations and rigid bodies)
 * - Regular USD prims
 *
 * Features:
 * - Transform setting with physics state preservation
 * - Scale manipulation with physics constraints
 * - Automatic physics state management (wake/sleep)
 * - Proper handling of local vs. world space transforms
 */
namespace transforms
{
static void createTensorDesc(TensorDesc& tensorDesc, void* dataPtr, int numElements, TensorDataType type)
{
    tensorDesc.dtype = type;
    tensorDesc.numDims = 1;
    tensorDesc.dims[0] = numElements;
    tensorDesc.data = dataPtr;
    tensorDesc.ownData = true;
    tensorDesc.device = -1;
}

/**
 * @brief Sets the transform (position and rotation) of a USD prim.
 * @details
 * Handles different types of objects appropriately:
 * 1. For articulated objects:
 *    - Wakes up the articulation
 *    - Sets root body pose
 *    - Resets velocities
 * 2. For rigid bodies:
 *    - Wakes up the body
 *    - Sets pose directly
 *    - Resets velocities
 * 3. For regular prims:
 *    - Computes and sets local transform
 *    - Handles parent space correctly
 *
 * @param[in,out] prim USD prim to transform
 * @param[in] bodyTranslation New position in world space
 * @param[in] bodyRotation New rotation in world space
 *
 * @note For physics objects, this function only works during simulation
 * @warning Resets linear and angular velocities to zero for physics objects
 */
inline void setTransform(pxr::UsdPrim& prim, pxr::GfVec3f bodyTranslation, pxr::GfQuatf bodyRotation)
{
    // TODO: Handle world rotation as well
    // NOTE: reverting this for now, rigid body sink publishes global so teleport should be global too
    // auto newTranslation = pxBodyTranslation; // + parentToWorldMat.ExtractTranslation();

    auto mSimulationManagerInterface = carb::getCachedInterface<simulation_manager::ISimulationManager>();
    if (!mSimulationManagerInterface)
    {
        CARB_LOG_ERROR("Failed to acquire Simulation Manager interface\n");
        return;
    }
    uint64_t stageId = pxr::UsdUtilsStageCache::Get().GetId(prim.GetStage()).ToLongInt();

    if (mSimulationManagerInterface->isSimulating())
    {
        auto mTensorInterface = carb::getCachedInterface<TensorApi>();
        if (!mTensorInterface)
        {
            CARB_LOG_ERROR("Failed to acquire Tensor Api interface\n");
            return;
        }
        auto mSimView = mTensorInterface->createSimulationView(long(stageId), getActivePhysicsEngineName());
        if (mSimView)
        {
            TensorDesc xformTensor;
            TensorDesc velTensor;
            std::vector<float> xformData(7, 0.0);
            std::vector<float> velData(6, 0.0);
            createTensorDesc(xformTensor, (void*)xformData.data(), 7, TensorDataType::eFloat32);
            createTensorDesc(velTensor, (void*)velData.data(), 6, TensorDataType::eFloat32);

            xformData[0] = bodyTranslation[0];
            xformData[1] = bodyTranslation[1];
            xformData[2] = bodyTranslation[2];
            xformData[3] = bodyRotation.GetImaginary()[0];
            xformData[4] = bodyRotation.GetImaginary()[1];
            xformData[5] = bodyRotation.GetImaginary()[2];
            xformData[6] = bodyRotation.GetReal();
            // TODO: do we need to wake up articulations?
            ObjectType objectType = mSimView->getObjectType(prim.GetPath().GetText());
            if (objectType == ObjectType::eArticulation || objectType == ObjectType::eArticulationRootLink)
            {
                auto articulation = mSimView->createArticulationView(prim.GetPath().GetText());
                articulation->setRootTransforms(&xformTensor, nullptr);
                articulation->setRootVelocities(&velTensor, nullptr);
            }
            else if (objectType == ObjectType::eRigidBody || objectType == ObjectType::eArticulationLink)
            {
                auto rigidBody = mSimView->createRigidBodyView(prim.GetPath().GetText());
                rigidBody->setTransforms(&xformTensor, nullptr);
                rigidBody->setVelocities(&velTensor, nullptr);
            }
            return;
        }
    }
    // In case we are not simulating or the object was a regular prim, go down this path
    pxr::GfTransform usdBodyPose;
    usdBodyPose.SetTranslation(bodyTranslation);
    usdBodyPose.SetRotation(pxr::GfRotation(bodyRotation));
    // Pose is global so offset by parent pose
    pxr::GfMatrix4d parentToWorldMat =
        pxr::UsdGeomXformable(prim).ComputeParentToWorldTransform(pxr::UsdTimeCode::Default());
    isaacsim::core::includes::setLocalTransformMatrix(prim, usdBodyPose.GetMatrix() * parentToWorldMat.GetInverse());
}

/**
 * @brief Sets the scale of a USD prim.
 * @details
 * Applies scaling to the prim's local transform, with special handling for physics objects:
 * - Only scales non-physics objects during simulation
 * - Preserves existing transform components
 * - Applies scale in local space
 *
 * @param[in,out] prim USD prim to scale
 * @param[in] pxBodyScale Scale factors for x, y, and z axes
 *
 * @note During simulation, only non-physics objects can be scaled
 * @warning Scaling physics objects during simulation may have unexpected results
 */
inline void setScale(pxr::UsdPrim& prim, pxr::GfVec3f pxBodyScale)
{
    bool doScale = true;
    auto mSimulationManagerInterface = carb::getCachedInterface<simulation_manager::ISimulationManager>();
    if (!mSimulationManagerInterface)
    {
        CARB_LOG_ERROR("Failed to acquire Simulation Manager interface\n");
        return;
    }
    if (mSimulationManagerInterface->isSimulating())
    {
        auto mTensorInterface = carb::getCachedInterface<TensorApi>();
        if (!mTensorInterface)
        {
            CARB_LOG_ERROR("Failed to acquire Tensor Api interface\n");
            return;
        }
        uint64_t stageId = pxr::UsdUtilsStageCache::Get().GetId(prim.GetStage()).ToLongInt();
        if (auto mSimView = mTensorInterface->createSimulationView(long(stageId), getActivePhysicsEngineName()))
        {
            auto path = prim.GetPath().GetString().c_str();
            auto articulation = mSimView->createArticulationView(path);
            auto rigidBody = mSimView->createRigidBodyView(path);
            if (articulation or rigidBody)
                doScale = false;
        }
    }

    if (doScale)
    {
        auto currentTransformMat = isaacsim::core::includes::getLocalTransformMatrix(prim);
        pxr::GfMatrix4d scaleMat;
        scaleMat.SetScale(pxBodyScale);
        auto scaledTransformMat = scaleMat * currentTransformMat;
        isaacsim::core::includes::setLocalTransformMatrix(prim, scaledTransformMat);
    }
}

} // namespace transforms
} // namespace includes
} // namespace core
} // namespace isaacsim
