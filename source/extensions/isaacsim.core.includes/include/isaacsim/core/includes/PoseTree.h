// SPDX-FileCopyrightText: Copyright (c) 2023-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include "Buffer.h"
#include "Conversions.h"
#include "Pose.h"
#include "UsdUtilities.h"

#include <carb/logging/Log.h>

#include <foundation/PxTransform.h>
#include <isaacsim/robot/schema/sensor_tokens.h>
#include <omni/physics/tensors/IArticulationMetatype.h>
#include <omni/physics/tensors/IArticulationView.h>
#include <omni/physics/tensors/IRigidBodyView.h>
#include <omni/physics/tensors/ISimulationView.h>
#include <omni/physics/tensors/ObjectTypes.h>
#include <omni/physics/tensors/TensorApi.h>
#include <physx/include/foundation/PxTransform.h>
#include <usdrt/scenegraph/usd/rt/xformable.h>

#include <cmath>
#include <unordered_map>

using namespace omni::physics::tensors;
using namespace isaacsim::core::includes::conversions;
using namespace isaacsim::robot::schema::sensors;

namespace isaacsim
{
namespace core
{
namespace includes
{
namespace posetree
{

/**
 * @class PoseTree
 * @brief A utility class for managing and traversing hierarchical pose transformations in a scene.
 * @details
 * PoseTree provides functionality to manage and process pose transformations between different frames
 * in a hierarchical scene structure. It handles both rigid bodies and articulated objects, computing
 * relative transforms between parent and child frames.
 *
 * The class supports:
 * - Parent-child relationships between prims
 * - Rigid body transformations
 * - Articulation hierarchies
 * - Camera-specific transformations
 *
 * @note This class requires a valid USD stage and DynamicControl instance to function properly.
 * @warning Frame names must be unique within the tree. Duplicate names will be automatically renamed
 *          with their full path.
 */
static void createTensorDesc(TensorDesc& tensorDesc, void* dataPtr, int numXforms, TensorDataType type, int device)
{
    tensorDesc.dtype = type;
    tensorDesc.numDims = 1;
    tensorDesc.dims[0] = numXforms * 7;
    tensorDesc.data = dataPtr;
    tensorDesc.ownData = true;
    tensorDesc.device = device;
}

class PoseTree
{
public:
    /**
     * @brief Constructs a new PoseTree instance.
     * @details Initializes the PoseTree with USD stage and dynamic control references.
     *
     * @param[in] stageId The unique identifier for the USD stage
     * @param[in] simulationView Pointer to the simulation view instance for physics interactions
     *
     * @pre stageId must be valid and correspond to an existing USD stage
     * @pre simulationView must be a valid pointer to a simulation view instance
     */
    PoseTree(const uint64_t& stageId, ISimulationView* simulationView)
    {
        // Store the USD and USDRT stage references from the stage ID
        m_usdStage = pxr::UsdUtilsStageCache::Get().Find(pxr::UsdStageCache::Id::FromLongInt(static_cast<long>(stageId)));
        omni::fabric::IStageReaderWriter* iStageReaderWriter =
            carb::getCachedInterface<omni::fabric::IStageReaderWriter>();
        omni::fabric::StageReaderWriterId stageInProgress = iStageReaderWriter->get(stageId);
        m_usdrtStage = usdrt::UsdStage::Attach(stageId, stageInProgress);
        m_simulationView = simulationView;
        m_rigidTransformData.resize(7);
        m_rigidTransformBuffer.resize(7);
        m_rigidTransformBuffer.setDevice(m_simulationView->getDeviceOrdinal());
        createTensorDesc(m_rigidTransformTensor, (void*)m_rigidTransformBuffer.data(), 1, TensorDataType::eFloat32,
                         m_simulationView->getDeviceOrdinal());
    }

    /**
     * @brief Destructor that cleans up cached physics views.
     */
    ~PoseTree()
    {
        for (auto& pair : m_articulationViewCache)
        {
            if (pair.second)
            {
                pair.second->release();
            }
        }
        for (auto& pair : m_rigidBodyViewCache)
        {
            if (pair.second)
            {
                pair.second->release();
            }
        }
        m_articulationViewCache.clear();
        m_rigidBodyViewCache.clear();
    }

    /**
     * @brief Sets the parent prim path and frame name for the pose tree.
     * @details Establishes the root reference frame for subsequent pose calculations.
     *
     * @param[in] parentPath SDF path for the parent prim in the USD stage
     * @param[in] parentFrame Name identifier for the parent frame
     *
     * @note The parent frame serves as the root reference for all child transformations
     */
    void setParentPrimPath(const pxr::SdfPath& parentPath, const std::string& parentFrame)
    {
        m_parentPath = parentPath;
        m_parentFrame = parentFrame;
    }

    /**
     * @brief Sets the target prim paths to be processed in the pose tree.
     * @details Defines the set of prims whose poses will be computed relative to the parent frame.
     *
     * @param[in] targets Vector of SDF paths representing the target prims
     */
    void setTargetPrimPaths(const pxr::SdfPathVector& targets)
    {
        m_targets = targets;
    }

    /**
     * @brief Traverses the complete pose tree and processes each transform.
     * @details
     * Performs a depth-first traversal of the pose tree, computing transforms between parent and child frames.
     * Handles different types of prims including:
     * - Articulations and their bodies
     * - Rigid bodies
     * - Regular transforms
     * - Special cases like cameras
     *
     * @param[in] processTransform Callback function to handle each computed transform
     *
     * @note The callback function receives:
     *       - Parent frame name (string)
     *       - Child frame name (string)
     *       - Relative transform between frames (PxTransform)
     *
     * @warning For cameras without RTXLidar API, an additional 180-degree rotation about the x-axis is applied
     */
    void processAllFrames(
        std::function<void(const std::string&, const std::string&, const ::physx::PxTransform&)>& processTransform)
    {
        // If the parent prim path is not empty, get the type of prim and its pose.
        if (!m_parentPath.IsEmpty())
        {
            ObjectType objectType = m_simulationView->getObjectType(m_parentPath.GetString().c_str());
            if (objectType == ObjectType::eInvalid)
            {
                CARB_LOG_WARN("[PoseTree] parent getObjectType eInvalid for '%s'", m_parentPath.GetText());
            }
            if (objectType == ObjectType::eArticulationLink || objectType == ObjectType::eArticulationRootLink ||
                objectType == ObjectType::eRigidBody)
            {
                std::string parentPathStr = m_parentPath.GetString();
                IRigidBodyView* rigidBody = nullptr;
                if (m_rigidBodyViewCache.find(parentPathStr) != m_rigidBodyViewCache.end())
                {
                    rigidBody = m_rigidBodyViewCache[parentPathStr];
                }
                else
                {
                    rigidBody = m_simulationView->createRigidBodyView(parentPathStr.c_str());
                    m_rigidBodyViewCache[parentPathStr] = rigidBody;
                }
                m_parentPose = getRigidBodyPose(rigidBody);
            }
            else
            {
                // TODO: handle non types
                m_parentPose = getXformPose(m_parentPath);
            }

            m_parentFrame =
                getUniqueFrameName(getName(m_usdStage->GetPrimAtPath(m_parentPath)), m_parentPath.GetString());
        }
        // For each target prim determine its type and compute the associated poses
        for (pxr::SdfPath primPath : m_targets)
        {
            ObjectType objectType = m_simulationView->getObjectType(primPath.GetString().c_str());
            if (objectType == ObjectType::eInvalid)
            {
                CARB_LOG_WARN("[PoseTree] target getObjectType eInvalid for '%s'", primPath.GetText());
            }
            if (objectType == ObjectType::eArticulation || objectType == ObjectType::eArticulationRootLink)
            {
                std::string primPathStr = primPath.GetString();
                IArticulationView* articulation = nullptr;
                if (m_articulationViewCache.find(primPathStr) != m_articulationViewCache.end())
                {
                    articulation = m_articulationViewCache[primPathStr];
                }
                else
                {
                    articulation = m_simulationView->createArticulationView(primPathStr.c_str());
                    m_articulationViewCache[primPathStr] = articulation;
                }
                if (!articulation)
                {
                    CARB_LOG_WARN("[PoseTree] createArticulationView returned null for '%s'", primPathStr.c_str());
                    continue;
                }
                const IArticulationMetatype* mt = articulation->getSharedMetatype();
                uint32_t linkCount = articulation->getMaxLinks();
                if (linkCount == 0)
                {
                    CARB_LOG_WARN("[PoseTree] articulation has zero links for '%s'", primPathStr.c_str());
                    continue;
                }
                std::vector<std::string> linkPaths(linkCount);
                std::vector<std::vector<int>> childLinks(linkCount);
                for (uint32_t j = 0; j < linkCount; ++j)
                {
                    linkPaths[j] = articulation->getUsdLinkPath(0, j);
                    std::string parentName =
                        mt->getLinkParentName(j) ? mt->getLinkParentName(j) : ""; // mt->getLinkParentName(j)
                                                                                  // could be
                                                                                  // nullptr
                    int32_t parentIdx = mt->findLinkIndex(mt->getLinkParentName(j));
                    if (parentIdx >= 0)
                    {
                        childLinks[parentIdx].push_back(j);
                    }
                }
                m_linkTransformData.resize(7 * linkCount);
                m_linkTransformBuffer.resize(7 * linkCount);
                m_linkTransformBuffer.setDevice(m_simulationView->getDeviceOrdinal());
                createTensorDesc(m_linkTransformTensor, (void*)m_linkTransformBuffer.data(), linkCount,
                                 TensorDataType::eFloat32, m_simulationView->getDeviceOrdinal());

                articulation->getLinkTransforms(&m_linkTransformTensor);
                m_linkTransformBuffer.copyTo(m_linkTransformData.data(), 7 * linkCount);
                float* data = m_linkTransformData.data();
                ::physx::PxTransform body1Pose = ::physx::PxTransform(
                    ::physx::PxVec3(data[0], data[1], data[2]), ::physx::PxQuat(data[3], data[4], data[5], data[6]));
                std::string framePath = linkPaths[0];
                std::string bodyName = getName(m_usdStage->GetPrimAtPath(pxr::SdfPath(framePath)));

                if (!m_parentPath.IsEmpty())
                {
                    body1Pose = m_parentPose.transformInv(body1Pose);
                }
                std::string childFrameId = getUniqueFrameName(bodyName, framePath);
                if (m_parentFrame != childFrameId)
                {
                    // articulations always have an extra transform to the base link/rigid body
                    processTransform(m_parentFrame, childFrameId, body1Pose);
                }
                for (size_t j = 0; j < linkCount; j++)
                {
                    ::physx::PxTransform body0Pose(
                        ::physx::PxVec3(data[7 * j + 0], data[7 * j + 1], data[7 * j + 2]),
                        ::physx::PxQuat(data[7 * j + 3], data[7 * j + 4], data[7 * j + 5], data[7 * j + 6]));
                    std::string parentPath = linkPaths[j];
                    std::string parentName = getName(m_usdStage->GetPrimAtPath(pxr::SdfPath(parentPath)));
                    size_t num_children = childLinks[j].size();
                    for (size_t k = 0; k < num_children; k++)
                    {
                        size_t childIdx = childLinks[j][k];
                        ::physx::PxTransform body1Pose(
                            ::physx::PxVec3(data[7 * childIdx + 0], data[7 * childIdx + 1], data[7 * childIdx + 2]),
                            ::physx::PxQuat(data[7 * childIdx + 3], data[7 * childIdx + 4], data[7 * childIdx + 5],
                                            data[7 * childIdx + 6]));
                        ::physx::PxTransform body0Tbody1(body0Pose.transformInv(body1Pose));
                        std::string framePath = linkPaths[childIdx];
                        auto bodyName = getName(m_usdStage->GetPrimAtPath(pxr::SdfPath(framePath)));
                        processTransform(getUniqueFrameName(parentName, parentPath),
                                         getUniqueFrameName(bodyName, framePath), body0Tbody1);
                    }
                }
            }
            else if (objectType == ObjectType::eArticulationLink || objectType == ObjectType::eRigidBody)
            {
                std::string primPathStr = primPath.GetString();
                IRigidBodyView* rigidBody = nullptr;
                if (m_rigidBodyViewCache.find(primPathStr) != m_rigidBodyViewCache.end())
                {
                    rigidBody = m_rigidBodyViewCache[primPathStr];
                }
                else
                {
                    rigidBody = m_simulationView->createRigidBodyView(primPathStr.c_str());
                    m_rigidBodyViewCache[primPathStr] = rigidBody;
                }
                ::physx::PxTransform body1Pose = getRigidBodyPose(rigidBody);
                std::string childFrameId =
                    getUniqueFrameName(getName(m_usdStage->GetPrimAtPath(primPath)), primPath.GetString());
                if (m_parentFrame != childFrameId)
                {
                    if (!m_parentPath.IsEmpty())
                    {
                        body1Pose = m_parentPose.transformInv(body1Pose);
                    }

                    processTransform(m_parentFrame, childFrameId, body1Pose);
                }
            }
            else
            {
                pxr::UsdPrim prim = m_usdStage->GetPrimAtPath(primPath);

                ::physx::PxTransform body1Pose = getXformPose(primPath);


                if (prim.IsA<pxr::UsdGeomCamera>() && !prim.HasAPI(kIsaacRtxLidarSensorAPI))
                {
                    // Regular camera (not RTXLidar), Rotate 180 degrees about x-axis
                    // pxr::GfMatrix4d(1, 0, 0, 0, 0, -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1);
                    ::physx::PxQuat omniTCamera(1, 0, 0, 0);
                    body1Pose = body1Pose * ::physx::PxTransform(omniTCamera);
                }

                if (!m_parentPath.IsEmpty())
                {
                    body1Pose = m_parentPose.transformInv(body1Pose);
                }

                processTransform(m_parentFrame, getUniqueFrameName(getName(prim), primPath.GetString()), body1Pose);
            }
        }
    }

    /**
     * @brief Retrieves the physics-based pose of a rigid body.
     * @details Queries the DynamicControl system for the current pose of a rigid body prim.
     *
     * @param[in] rb Pointer to the rigid body view interface
     * @return PxTransform representing the world-space pose of the rigid body
     *
     * @pre The rigid body view must be valid or nullptr (returns identity transform)
     */
    ::physx::PxTransform getRigidBodyPose(IRigidBodyView* rb)
    {
        if (rb)
        {
            rb->getTransforms(&m_rigidTransformTensor);
            m_rigidTransformBuffer.copyTo(m_rigidTransformData.data(), 7);
            float* data = m_rigidTransformData.data();
            ::physx::PxTransform trans(
                ::physx::PxVec3(data[0], data[1], data[2]), ::physx::PxQuat(data[3], data[4], data[5], data[6]));
            return trans;
        }
        else
        {
            return ::physx::PxTransform(::physx::PxIdentity);
        }
    }

    /**
     * @brief Gets the pose of a prim using fabric or USD.
     * @details Computes the world-space transform of a prim, prioritizing fabric data if available.
     *
     * @param[in] path SDF path to the target prim
     * @return PxTransform representing the world-space pose of the prim
     */
    ::physx::PxTransform getXformPose(const pxr::SdfPath& path)
    {
        return asPxTransform(isaacsim::core::includes::pose::computeWorldXformNoCache(m_usdStage, m_usdrtStage, path));
    }

    /**
     * @brief Generates a unique frame name for a given prim.
     * @details
     * Ensures unique frame names by:
     * 1. Using the provided frame name if unique
     * 2. Using a cached renamed version if previously processed
     * 3. Creating a new unique name based on the full path if needed
     *
     * @param[in] frame Desired frame name
     * @param[in] path Full USD path of the prim
     * @return std::string Unique frame name
     *
     * @note If a name collision occurs, the full path is used with '/' replaced by '_'
     * @warning Logs a warning when name collisions occur, suggesting the use of isaac:nameOverride
     */
    std::string getUniqueFrameName(const std::string& frame, const std::string& path)
    {
        std::string name(frame);
        if (m_renamedFrames.find(path) != m_renamedFrames.end())
        {
            m_publishedFrames[frame] = true;
            return m_renamedFrames[path];
        }
        else if (m_publishedFrames.find(frame) == m_publishedFrames.end())
        {
            m_renamedFrames[path] = frame;
            m_publishedFrames[frame] = true;
        }

        else
        {
            name = path;
            std::replace(name.begin(), name.end(), '/', '_');
            name = name.substr(1);
            CARB_LOG_WARN(
                "Frame with name %s already exists. Overriding frame name for %s to %s (you can add the attribute isaac:nameOverride to remove this warning)",
                frame.c_str(), path.c_str(), name.c_str());
            m_renamedFrames[path] = name;
            m_publishedFrames[name] = true;
        }
        return name;
    }

    /**
     * @brief Retrieves the world pose of a frame.
     * @details Computes the world-space transform of a frame, prioritizing fabric data if available.
     *
     * @param[in] frame Frame name
     * @param[in] getParent If true, retrieves the pose of the parent frame
     * @return PxTransform representing the world-space pose of the frame
     */
    ::physx::PxTransform getFrameWorldPose(const std::string& frame, bool getParent = false)
    {
        pxr::SdfPath path(frame);
        if (getParent)
        {
            path = path.GetParentPath();
        }
        return asPxTransform(isaacsim::core::includes::pose::computeWorldXformNoCache(m_usdStage, m_usdrtStage, path));
    }

private:
    /** @brief SDF path to the parent prim */
    pxr::SdfPath m_parentPath;

    /** @brief Name identifier for the parent frame */
    std::string m_parentFrame;

    /** @brief Vector of target prim paths to process */
    pxr::SdfPathVector m_targets;

    /** @brief Cached transform of the parent frame */
    ::physx::PxTransform m_parentPose = ::physx::PxTransform(::physx::PxIdentity);

    /** @brief Reference to the USD stage */
    pxr::UsdStageRefPtr m_usdStage;

    /** @brief Reference to the USDRT stage */
    usdrt::UsdStageRefPtr m_usdrtStage;

    /** @brief Map of original paths to renamed frames */
    std::map<std::string, std::string> m_renamedFrames;

    /** @brief Set of published frame names */
    std::map<std::string, bool> m_publishedFrames;

    /** @brief Pointer to the simulation view interface for physics queries */
    ISimulationView* m_simulationView = nullptr;

    /** @brief Tensor descriptor for rigid body transform data */
    TensorDesc m_rigidTransformTensor;

    /** @brief Tensor descriptor for articulation link transform data */
    TensorDesc m_linkTransformTensor;

    /** @brief Buffer for rigid body transform buffer (CPU/GPU attending view device) */
    GenericBufferBase<float> m_rigidTransformBuffer;

    /** @brief Buffer for articulation link transform buffer (CPU/GPU attending view device) */
    GenericBufferBase<float> m_linkTransformBuffer;

    /** @brief Storage for rigid body transform data (CPU, for PxTransform) */
    std::vector<float> m_rigidTransformData;

    /** @brief Storage for articulation link transform data (CPU, for PxTransform) */
    std::vector<float> m_linkTransformData;

    /** @brief Cache of articulation views by prim path */
    std::unordered_map<std::string, IArticulationView*> m_articulationViewCache;

    /** @brief Cache of rigid body views by prim path */
    std::unordered_map<std::string, IRigidBodyView*> m_rigidBodyViewCache;
};
}
}
}
}
