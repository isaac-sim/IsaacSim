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

#include <pch/UsdPCH.h>
// clang-format on
#include "isaacsim/robot/schema/robot_schema.h"
#include "isaacsim/robot/surface_gripper/SurfaceGripperComponent.h"

#include <extensions/PxJoint.h>
#include <omni/physics/tensors/BodyTypes.h>
#include <omni/physx/IPhysx.h>
#include <omni/physx/IPhysxSceneQuery.h>
#include <pxr/usd/sdf/layer.h>
#include <pxr/usd/usdPhysics/filteredPairsAPI.h>

#include <PxConstraint.h>
#include <PxRigidActor.h>


namespace
{
omni::physx::IPhysx* g_physx = nullptr;
}

namespace isaacsim
{
namespace robot
{
namespace surface_gripper
{

inline const pxr::SdfPath& intToPath(const uint64_t& path)
{
    static_assert(sizeof(pxr::SdfPath) == sizeof(uint64_t), "Change to make the same size as pxr::SdfPath");

    return reinterpret_cast<const pxr::SdfPath&>(path);
}


void SurfaceGripperComponent::initialize(const pxr::UsdPrim& prim, const pxr::UsdStageWeakPtr stage, bool writeToUsd)
{
    g_physx = carb::getCachedInterface<omni::physx::IPhysx>();
    isaacsim::core::includes::ComponentBase<pxr::UsdPrim>::initialize(prim, stage);
    m_primPath = prim.GetPath();
    m_writeToUsd = writeToUsd;
    mDoStart = true;

    updateGripperProperties();
}

void SurfaceGripperComponent::onComponentChange()
{
    // Update surface gripper properties from the prim
    updateGripperProperties();
}

void SurfaceGripperComponent::onStart()
{

    onComponentChange();
    // Get attachment points (D6 joints)
    updateAttachmentPoints();
    if (m_status == GripperStatus::Closed)
    {
        updateClosedGripper();
    }
    else
    {
        updateOpenGripper();
    }
    m_isInitialized = true;
}

void SurfaceGripperComponent::onPhysicsStep(double dt)
{
    // CARB_PROFILE_ZONE(0, "[IsaacSim] SurfaceGripperComponent::onPhysicsStep");
    // Early return if component is not initialized
    if (!m_isInitialized)
    {
        return;
    }

    // Use SdfChangeBlock to batch USD changes for better performance
    // pxr::SdfChangeBlock changeBlock;


    // Update joint settling counters for inactive attachment points
    for (const auto& attachmentIndex : m_inactiveAttachmentIndices)
    {
        if (attachmentIndex < m_jointSettlingCounters.size() && m_jointSettlingCounters[attachmentIndex] > 0)
        {
            m_jointSettlingCounters[attachmentIndex] = m_jointSettlingCounters[attachmentIndex] - 1;
        }
    }

    // Handle retry timeout for closing gripper
    if (m_status == GripperStatus::Closing && m_retryInterval > 0 && m_retryCloseActive &&
        !m_inactiveAttachmentIndices.empty())
    {
        m_retryElapsed += dt;
        if (m_retryElapsed > m_retryInterval)
        {
            // Timeout reached, stop trying to close
            m_retryCloseActive = false;
            m_retryElapsed = 0.0;
        }
    }

    // Update gripper state based on current status
    if (m_status == GripperStatus::Closed || m_status == GripperStatus::Closing)
    {
        if (m_retryCloseActive || !m_activeAttachmentIndices.empty())
        {
            updateClosedGripper();
        }
        else
        {
            updateOpenGripper();
        }
    }
    else
    {
        // For GripperStatus::Open or any other state
        updateOpenGripper();
    }
}

void SurfaceGripperComponent::preTick()
{
    // Nothing to do in preTick for now
}

void SurfaceGripperComponent::tick()
{
    if (!m_isInitialized || !m_isEnabled)
    {
        return;
    }

    // Update the visualization or any non-physics aspects
    // For example, we could update visual indicators of gripper state
}

void SurfaceGripperComponent::onStop()
{
    m_grippedObjects.clear();
    m_inactiveAttachmentIndices.clear();
    for (size_t i = 0; i < m_attachmentPaths.size(); ++i)
    {
        m_inactiveAttachmentIndices.insert(i);
    }
    m_activeAttachmentIndices.clear();
    m_isInitialized = false;
    mDoStart = true;
}

bool SurfaceGripperComponent::setGripperStatus(GripperStatus status)
{
    GripperStatus gripperStatus = status;
    if (gripperStatus != GripperStatus::Open && gripperStatus != GripperStatus::Closed &&
        gripperStatus != GripperStatus::Closing)
    {
        return false;
    }
    if (gripperStatus != m_status)
    {

        if (gripperStatus == GripperStatus::Open)
        {
            updateOpenGripper();
            m_status = GripperStatus::Open;
            m_retryCloseActive = false;
        }
        else if ((gripperStatus == GripperStatus::Closed && m_status != GripperStatus::Closing) ||
                 gripperStatus == GripperStatus::Closing)
        {
            m_status = GripperStatus::Closing;
            m_retryCloseActive = true;
            updateClosedGripper();
            if (m_retryInterval > 0)
            {
                // Start retry timer
                m_retryElapsed = 0.0;
            }
            else
            {
                m_retryCloseActive = false;
            }
        }
    }

    return true;
}

void SurfaceGripperComponent::updateGripperProperties()
{
    auto prim = m_stage->GetPrimAtPath(m_primPath);
    if (!prim)
    {
        return;
    }
    // Get forward axis
    pxr::UsdAttribute forwardAxisAttr =
        prim.GetAttribute(isaacsim::robot::schema::getAttributeName(isaacsim::robot::schema::Attributes::FORWARD_AXIS));
    if (forwardAxisAttr)
    {
        pxr::TfToken forwardAxis;
        forwardAxisAttr.Get(&forwardAxis);
        m_forwardAxis = forwardAxis.GetString();
    }

    // Get status
    pxr::UsdAttribute statusAttr =
        prim.GetAttribute(isaacsim::robot::schema::getAttributeName(isaacsim::robot::schema::Attributes::STATUS));
    if (statusAttr)
    {
        pxr::TfToken status;
        statusAttr.Get(&status);
        m_status = GripperStatusFromToken(status);
    }

    // Get retry interval
    pxr::UsdAttribute retryIntervalAttr =
        prim.GetAttribute(isaacsim::robot::schema::getAttributeName(isaacsim::robot::schema::Attributes::RETRY_INTERVAL));
    if (retryIntervalAttr)
    {
        retryIntervalAttr.Get(&m_retryInterval);
    }

    // Get force limits
    pxr::UsdAttribute shearForceLimitAttr = prim.GetAttribute(
        isaacsim::robot::schema::getAttributeName(isaacsim::robot::schema::Attributes::SHEAR_FORCE_LIMIT));
    if (shearForceLimitAttr)
    {
        shearForceLimitAttr.Get(&m_shearForceLimit);
    }

    pxr::UsdAttribute coaxialForceLimitAttr = prim.GetAttribute(
        isaacsim::robot::schema::getAttributeName(isaacsim::robot::schema::Attributes::COAXIAL_FORCE_LIMIT));
    if (coaxialForceLimitAttr)
    {
        coaxialForceLimitAttr.Get(&m_coaxialForceLimit);
    }

    pxr::UsdAttribute maxGripDistanceAttr = prim.GetAttribute(
        isaacsim::robot::schema::getAttributeName(isaacsim::robot::schema::Attributes::MAX_GRIP_DISTANCE));
    if (maxGripDistanceAttr)
    {
        maxGripDistanceAttr.Get(&m_maxGripDistance);
    }
}

void SurfaceGripperComponent::updateAttachmentPoints()
{
    auto prim = m_stage->GetPrimAtPath(m_primPath);
    m_attachmentPaths.clear();
    m_attachmentJoints.clear();

    pxr::UsdRelationship attachmentPointsRel = prim.GetRelationship(
        isaacsim::robot::schema::relationNames.at(isaacsim::robot::schema::Relations::ATTACHMENT_POINTS));
    if (!attachmentPointsRel)
    {
        return;
    }

    pxr::UsdRelationship rel = prim.GetRelationship(
        isaacsim::robot::schema::relationNames.at(isaacsim::robot::schema::Relations::GRIPPED_OBJECTS));
    if (!rel)
    {
        rel = prim.CreateRelationship(
            isaacsim::robot::schema::relationNames.at(isaacsim::robot::schema::Relations::GRIPPED_OBJECTS), false);
    }

    std::vector<pxr::SdfPath> attachmentPaths;
    attachmentPointsRel.GetTargets(&attachmentPaths);
    // Preallocate buffers
    m_attachmentPaths.reserve(attachmentPaths.size());
    m_grippedObjectsBuffer.reserve(attachmentPaths.size());
    m_grippedObjects.reserve(attachmentPaths.size());
    m_activeAttachmentIndices.reserve(attachmentPaths.size());
    m_inactiveAttachmentIndices.reserve(attachmentPaths.size());
    m_jointSettlingCounters.clear();
    m_jointForwardAxis.clear();
    m_jointClearanceOffset.clear();
    m_attachmentJoints.reserve(attachmentPaths.size());
    m_apIndicesToDetach.reserve(attachmentPaths.size());
    m_apIndicesToActivate.reserve(attachmentPaths.size());
    m_clearanceOffsetsToPersist.reserve(attachmentPaths.size());
    m_grippedObjectsVector.reserve(attachmentPaths.size());
    m_jointSettlingCounters.resize(attachmentPaths.size(), 0);
    // Default to X to match the IsaacRobotSchema default for `isaac:forwardAxis`.
    m_jointForwardAxis.resize(attachmentPaths.size(), Axis::X);
    m_jointClearanceOffset.resize(attachmentPaths.size(), 0.0f);
    std::vector<std::string> applyApiPaths;
    std::vector<std::string> excludeFromArticulationPaths;
    std::vector<std::pair<std::string, float>> clearanceOffsets;
    applyApiPaths.reserve(attachmentPaths.size());
    excludeFromArticulationPaths.reserve(attachmentPaths.size());
    clearanceOffsets.reserve(attachmentPaths.size());
    size_t index = 0;
    for (const auto& path : attachmentPaths)
    {
        pxr::UsdPrim attachmentPrim = prim.GetPrimAtPath(path);
        if (attachmentPrim && attachmentPrim.IsA<pxr::UsdPhysicsJoint>())
        {
            if (!attachmentPrim.HasAPI(
                    isaacsim::robot::schema::className(isaacsim::robot::schema::Classes::ATTACHMENT_POINT_API)))
            {
                applyApiPaths.push_back(path.GetString());
            }
            pxr::UsdPhysicsJoint joint(attachmentPrim);

            bool excludeFromArticulation;
            joint.GetExcludeFromArticulationAttr().Get(&excludeFromArticulation);
            if (!excludeFromArticulation)
            {
                excludeFromArticulationPaths.push_back(path.GetString());
            }
            physx::PxJoint* pxJoint =
                static_cast<physx::PxJoint*>(g_physx->getPhysXPtr((path), omni::physx::PhysXType::ePTJoint));
            if (!pxJoint)
            {
                CARB_LOG_WARN("   Gripper %s has no joint at attachment point %s", m_primPath.GetText(), path.GetText());
                continue;
            }
            pxJoint->setConstraintFlag(physx::PxConstraintFlag::eDISABLE_CONSTRAINT, m_status == GripperStatus::Open);
            m_attachmentPaths.push_back(path.GetString());
            m_attachmentJoints.push_back(pxJoint);
            m_jointSettlingCounters[index] = 0;

            // Cache per-joint forward axis
            pxr::TfToken jointAxisToken;
            attachmentPrim
                .GetAttribute(isaacsim::robot::schema::getAttributeName(isaacsim::robot::schema::Attributes::FORWARD_AXIS))
                .Get(&jointAxisToken);
            // Schema default for `isaac:forwardAxis` is "X"; an unset or
            // unrecognized token falls back to X to match.
            Axis axisEnum = Axis::X;
            if (!jointAxisToken.IsEmpty())
            {
                const char c = std::toupper(jointAxisToken.GetText()[0]);
                if (c == 'Y')
                {
                    axisEnum = Axis::Y;
                }
                else if (c == 'Z')
                {
                    axisEnum = Axis::Z;
                }
            }
            m_jointForwardAxis[index] = axisEnum;

            // Cache clearance offset if present
            float clearanceOffset = 0.0f;
            pxr::UsdAttribute clearanceOffsetAttr = attachmentPrim.GetAttribute(
                isaacsim::robot::schema::getAttributeName(isaacsim::robot::schema::Attributes::CLEARANCE_OFFSET));
            if (clearanceOffsetAttr)
            {
                clearanceOffsetAttr.Get(&clearanceOffset);
            }
            m_jointClearanceOffset[index] = clearanceOffset;

            // Cache body0 path for filtered pairs from the first joint
            if (m_body0PathForFilterPairs.empty())
            {
                pxr::SdfPathVector targets0;
                joint.GetBody0Rel().GetTargets(&targets0);
                if (!targets0.empty())
                {
                    m_body0PathForFilterPairs = targets0[0].GetString();
                }
            }
            ++index;
        }
    }
    m_activeAttachmentIndices.clear();
    m_inactiveAttachmentIndices.clear();
    for (size_t i = 0; i < m_attachmentPaths.size(); ++i)
    {
        m_inactiveAttachmentIndices.insert(i);
    }

    if (m_writeToUsd)
    {
        _queueWriteAttachmentPointBatch(applyApiPaths, excludeFromArticulationPaths, clearanceOffsets);
    }
}

void SurfaceGripperComponent::updateGrippedObjectsList()
{
    // Early return if gripper is open - no need to track gripped objects
    if (m_activeAttachmentIndices.empty())
    {
        if (!m_grippedObjects.empty())
        {
            m_grippedObjects.clear();
        }
        if (m_writeToUsd)
        {
            m_grippedObjectsVector.clear(); // empty -> clear targets
            _queueWriteGrippedObjectsAndFilters(m_grippedObjectsVector, m_body0PathForFilterPairs);
        }
    }
    else
    {
        // Create a new set for current gripped objects
        m_grippedObjectsBuffer.clear();

        // Iterate through active attachment points to find gripped objects
        for (const auto& attachmentIndex : m_activeAttachmentIndices)
        {
            physx::PxJoint* pxJoint = _getCachedJoint(attachmentIndex);

            // Skip invalid or broken joints
            if (!pxJoint || (pxJoint->getConstraintFlags() &
                             (physx::PxConstraintFlag::eBROKEN | physx::PxConstraintFlag::eDISABLE_CONSTRAINT)))
            {
                continue;
            }

            // Get actors attached to the joint
            physx::PxRigidActor *pxActor0, *pxActor1;
            pxJoint->getActors(pxActor0, pxActor1);

            // Add the second actor to gripped objects if it exists and has a name
            if (pxActor1 && pxActor1->getName())
            {
                m_grippedObjectsBuffer.insert(pxActor1->getName());
            }
        }

        // Check if the set of gripped objects has changed
        if (m_grippedObjects == m_grippedObjectsBuffer)
        {
            // No change in gripped objects, nothing to update
            return;
        }

        // Update the gripped objects
        m_grippedObjects = m_grippedObjectsBuffer;


        if (m_writeToUsd)
        {
            m_grippedObjectsVector.clear();
            m_grippedObjectsVector.assign(m_grippedObjects.begin(), m_grippedObjects.end());
            _queueWriteGrippedObjectsAndFilters(m_grippedObjectsVector, m_body0PathForFilterPairs);
        }
    }
}

void SurfaceGripperComponent::updateClosedGripper()
{
    // CARB_PROFILE_ZONE(0, "[IsaacSim] SurfaceGripperComponent::updateClosedGripper");
    // If we have no attachment points, we can't do anything
    if (m_attachmentPaths.empty())
    {
        return;
    }

    if (!m_inactiveAttachmentIndices.empty() && (m_status == GripperStatus::Closing || m_retryCloseActive))
    {
        findObjectsToGrip();
    }

    checkForceLimits();

    if (m_writeToUsd)
    {
        updateGrippedObjectsList();
    }
    {
        // CARB_PROFILE_ZONE(0, "[IsaacSim] SurfaceGripperComponent::updateClosedGripper::updateStatus");

        GripperStatus newStatus = m_status;

        if (m_inactiveAttachmentIndices.empty())
        {
            newStatus = GripperStatus::Closed;
        }
        else if (m_retryCloseActive)
        {
            newStatus = GripperStatus::Closing;
        }
        else if (m_activeAttachmentIndices.empty())
        {
            newStatus = GripperStatus::Open;
        }

        // Update status if it changed
        if (newStatus != m_status)
        {
            m_status = newStatus;

            // Reset retry elapsed time if we've finished closing
            if (newStatus == GripperStatus::Closed)
            {
                m_retryElapsed = 0.0f;
            }

            if (m_writeToUsd)
            {
                _queueWriteStatus(GripperStatusToToken(m_status).GetString());
            }
        }
    }
}

void SurfaceGripperComponent::checkForceLimits()
{
    // CARB_PROFILE_ZONE(0, "[IsaacSim] SurfaceGripperComponent::checkForceLimits");
    // Use reusable member buffer to avoid per-step allocations
    m_apIndicesToDetach.clear();

    // Only check force limits if they are set
    const bool checkShearForce = m_shearForceLimit > 0.0f;
    const bool checkCoaxialForce = m_coaxialForceLimit > 0.0f;
    const bool checkForces = m_status != GripperStatus::Open && (checkShearForce || checkCoaxialForce);
    // Skip force checking if no limits are set
    if (!checkForces)
    {
        return;
    }


    for (auto it = m_activeAttachmentIndices.begin(); it != m_activeAttachmentIndices.end(); it++)
    {
        size_t attachmentIndex = *it;


        size_t& settlingCounter = m_jointSettlingCounters[attachmentIndex];
        if (settlingCounter == 0)
        {
            physx::PxJoint* pxJoint = _getCachedJoint(attachmentIndex);


            physx::PxVec3 force, torque;
            physx::PxRigidActor *pxActor0 = nullptr, *pxActor1 = nullptr;
            pxJoint->getActors(pxActor0, pxActor1);
            pxJoint->getConstraint()->getForce(force, torque);

            Axis axis = _getJointForwardAxis(attachmentIndex);
            physx::PxTransform worldTransform = _computeJointWorldTransform(pxJoint, pxActor0);
            physx::PxVec3 direction = _directionFromAxisAndWorld(axis, worldTransform);

            // Lazy-evaluate shearForce only if needed for performance
            float shearForceMag = 0.0f;
            bool shouldRelease = false;
            float coaxialForce = force.dot(direction);
            if (checkCoaxialForce)
            {
                shouldRelease = (coaxialForce > m_coaxialForceLimit);
            }
            if (!shouldRelease && checkShearForce)
            {
                // Shear force: remove coaxial component
                physx::PxVec3 shearForce = force - (direction * coaxialForce);
                shearForceMag = shearForce.dot(shearForce);
                shouldRelease = (shearForceMag > m_shearForceLimit * m_shearForceLimit);
            }

            if (shouldRelease)
            {
                m_apIndicesToDetach.push_back(attachmentIndex);
                _queueDetachJoint(pxJoint);
            }
        }
        else
        {
            // Decrement the settling counter and move on
            --settlingCounter;
        }
    }


    // Process all joints to be removed
    for (const auto& ap : m_apIndicesToDetach)
    {
        m_activeAttachmentIndices.erase(ap);
        m_inactiveAttachmentIndices.insert(ap);
        m_jointSettlingCounters[ap] = m_settlingDelay;
    }
}

void SurfaceGripperComponent::findObjectsToGrip()
{
    // CARB_PROFILE_ZONE(0, "[IsaacSim] SurfaceGripperComponent::findObjectsToGrip");
    // Get physics query interface
    auto physxQuery = carb::getCachedInterface<omni::physx::IPhysxSceneQuery>();
    if (!physxQuery)
    {
        return;
    }

    // Iterate through each attachment point sequentially using reusable buffers
    m_apIndicesToActivate.clear();
    m_clearanceOffsetsToPersist.clear();

    for (const auto& attachmentIndex : m_inactiveAttachmentIndices)
    {
        _processAttachmentForGrip(attachmentIndex, m_apIndicesToActivate, m_clearanceOffsetsToPersist);
    }
    // Persist all clearance offset changes in one USD write
    if (!m_clearanceOffsetsToPersist.empty() && m_writeToUsd)
    {

        std::vector<std::string> emptyVec;
        _queueWriteAttachmentPointBatch(emptyVec, emptyVec, m_clearanceOffsetsToPersist);
    }
    for (const auto& ap : m_apIndicesToActivate)
    {
        m_inactiveAttachmentIndices.erase(ap);
        m_activeAttachmentIndices.insert(ap);

        m_jointSettlingCounters[ap] = m_settlingDelay; // Initialize settling counter
    }
}

void SurfaceGripperComponent::_processAttachmentForGrip(size_t attachmentIndex,
                                                        std::vector<size_t>& apToRemove,
                                                        std::vector<std::pair<std::string, float>>& clearanceOffsetsToPersist)
{
    auto physxQuery = carb::getCachedInterface<omni::physx::IPhysxSceneQuery>();
    if (!physxQuery)
    {
        return;
    }

    if (m_jointSettlingCounters[attachmentIndex] > 0)
    {
        m_jointSettlingCounters[attachmentIndex]--;
        return;
    }

    const std::string& attachmentPath = m_attachmentPaths[attachmentIndex];
    physx::PxJoint* pxJoint = _getCachedJoint(attachmentIndex);


    physx::PxRigidActor *localActor0 = nullptr, *localActor1 = nullptr;
    pxJoint->getActors(localActor0, localActor1);
    if (!localActor0)
    {
        return;
    }

    physx::PxTransform worldTransform = _computeJointWorldTransform(pxJoint, localActor0);

    Axis axis = _getJointForwardAxis(attachmentIndex);
    physx::PxVec3 dirPx = _directionFromAxisAndWorld(axis, worldTransform);
    pxr::GfVec3f direction(dirPx.x, dirPx.y, dirPx.z);

    pxr::GfVec3f worldPos(worldTransform.p.x, worldTransform.p.y, worldTransform.p.z);
    float clearanceOffset = 0.0f;
    clearanceOffset = m_jointClearanceOffset[attachmentIndex];
    bool selfCollision = true;
    bool clearanceChanged = false;
    while (selfCollision)
    {
        pxr::GfVec3f rayStart = worldPos + direction * static_cast<float>(clearanceOffset);

        carb::Float3 _rayStart{ static_cast<float>(rayStart[0]),
                                static_cast<float>(rayStart[1]), // NOLINT(readability-identifier-naming)
                                static_cast<float>(rayStart[2]) };
        carb::Float3 _rayDir{ static_cast<float>(direction[0]),
                              static_cast<float>(direction[1]), // NOLINT(readability-identifier-naming)
                              static_cast<float>(direction[2]) };

        omni::physx::RaycastHit result;
        float rayLength = static_cast<float>(m_maxGripDistance) - clearanceOffset;
        if (rayLength <= 0.0f)
        {
            rayLength = 0.001f;
        }
        bool hit = physxQuery->raycastClosest(_rayStart, _rayDir, rayLength, result, false);

        if (hit)
        {
            std::string hitPath = intToPath(result.rigidBody).GetString();
            if (hitPath == localActor0->getName())
            {
                selfCollision = true;
                clearanceOffset += 0.001f;
                continue;
            }
            selfCollision = false;
            if (clearanceOffset > 0.0f)
            {
                float originalOffset = 0.0f;
                if (attachmentIndex < m_jointClearanceOffset.size())
                {
                    originalOffset = m_jointClearanceOffset[attachmentIndex];
                }
                if (originalOffset != clearanceOffset)
                {
                    clearanceChanged = true;
                    if (attachmentIndex < m_jointClearanceOffset.size())
                    {
                        m_jointClearanceOffset[attachmentIndex] = clearanceOffset;
                    }
                }
            }
            physx::PxRigidActor* hitActor = _getOrCacheActor(hitPath);
            if (!hitActor)
            {
                return;
            }
            physx::PxTransform hitWorldTransform = hitActor->getGlobalPose();

            physx::PxVec3 offsetTranslation =
                -physx::PxVec3(direction[0], direction[1], direction[2]) * (result.distance - clearanceOffset);
            physx::PxTransform offsetTransform(offsetTranslation, physx::PxQuat(physx::PxIdentity));
            physx::PxTransform adjustedWorldTransform = offsetTransform * worldTransform;
            physx::PxTransform hitLocalTransform = hitWorldTransform.transformInv(adjustedWorldTransform);

            _queueAttachJoint(pxJoint, localActor0, hitActor, hitLocalTransform);

            apToRemove.push_back(attachmentIndex);
        }
        else
        {
            break;
        }
    }
    if (clearanceChanged)
    {
        clearanceOffsetsToPersist.emplace_back(attachmentPath, clearanceOffset);
    }
}

void SurfaceGripperComponent::updateOpenGripper()
{
    // CARB_PROFILE_ZONE(0, "[IsaacSim] SurfaceGripperComponent::updateOpenGripper");
    // Make sure we've released any gripped objects
    if (!m_activeAttachmentIndices.empty() || m_status != GripperStatus::Open)
    {
        _queueReleaseAllObjectsActions();
        m_grippedObjects.clear();
        m_activeAttachmentIndices.clear();
        m_inactiveAttachmentIndices.clear();
        for (size_t i = 0; i < m_attachmentPaths.size(); ++i)
        {
            m_inactiveAttachmentIndices.insert(i);
        }
        if (m_writeToUsd)
        {
            _queueWriteGrippedObjectsAndFilters(std::vector<std::string>(), m_body0PathForFilterPairs);
        }
    }
    if (m_status != GripperStatus::Open)
    {
        m_status = GripperStatus::Open;
        if (m_writeToUsd)
        {
            _queueWriteStatus(GripperStatusToToken(m_status).GetString());
        }
    }
}

void SurfaceGripperComponent::releaseAllObjects()
{
    // Early return if no attachment points exist
    if (m_attachmentPaths.empty())
    {
        return;
    }

    // Release all objects by disabling constraints on all attachment points
    for (size_t i = 0; i < m_attachmentPaths.size(); ++i)
    {
        // Immediate release used for non-physics-step flows (e.g., onStop)
        physx::PxJoint* px_joint = _getCachedJoint(i); // NOLINT(readability-identifier-naming)
        if (px_joint)
        {
            px_joint->setConstraintFlag(physx::PxConstraintFlag::eDISABLE_CONSTRAINT, true);
            px_joint->setConstraintFlag(physx::PxConstraintFlag::eCOLLISION_ENABLED, true);
        }
    }
    m_activeAttachmentIndices.clear();
    m_inactiveAttachmentIndices.clear();
    for (size_t i = 0; i < m_attachmentPaths.size(); ++i)
    {
        m_inactiveAttachmentIndices.insert(i);
    }


    // Reset settling counters for all attachment points
    for (size_t i = 0; i < m_attachmentPaths.size(); ++i)
    {
        m_jointSettlingCounters[i] = m_settlingDelay;
    }
}


void SurfaceGripperComponent::_queueReleaseAllObjectsActions()
{
    for (size_t i = 0; i < m_attachmentJoints.size(); ++i)
    {
        PhysxAction a;
        a.type = PhysxActionType::Detach;
        a.joint = _getCachedJoint(i);
        m_physxActions.push_back(std::move(a));
    }
}

void SurfaceGripperComponent::_queueDetachJoint(physx::PxJoint* joint)
{
    PhysxAction a;
    a.type = PhysxActionType::Detach;
    a.joint = joint;
    m_physxActions.push_back(std::move(a));
}

void SurfaceGripperComponent::_queueAttachJoint(physx::PxJoint* joint,
                                                physx::PxRigidActor* actor0,
                                                physx::PxRigidActor* actor1,
                                                const physx::PxTransform& localPose1)
{
    PhysxAction a;
    a.type = PhysxActionType::Attach;
    a.joint = joint;
    a.actor0 = actor0;
    a.actor1 = actor1;
    a.localPose1 = localPose1;
    m_physxActions.push_back(std::move(a));
}


// Removed unused direct USD writer helpers; writes are now queued and executed in the manager.


void SurfaceGripperComponent::_queueWriteStatus(const std::string& statusToken)
{
    UsdAction a;
    a.type = UsdActionType::WriteStatus;
    a.primPath = m_primPath.GetString();
    a.statusToken = statusToken;
    m_usdActions.push_back(std::move(a));
}

void SurfaceGripperComponent::_queueWriteGrippedObjectsAndFilters(const std::vector<std::string>& objectPaths,
                                                                  const std::string& body0PathForFilterPairs)
{
    UsdAction a;
    a.type = UsdActionType::WriteGrippedObjectsAndFilters;
    a.primPath = m_primPath.GetString();
    a.grippedObjectPaths = objectPaths;
    a.body0PathForFilterPairs = body0PathForFilterPairs;
    m_usdActions.push_back(std::move(a));
}

void SurfaceGripperComponent::_queueWriteAttachmentPointBatch(
    const std::vector<std::string>& applyApiPaths,
    const std::vector<std::string>& excludeFromArticulationPaths,
    const std::vector<std::pair<std::string, float>>& clearanceOffsets)
{
    UsdAction a;
    a.type = UsdActionType::WriteAttachmentPointBatch;
    a.primPath = m_primPath.GetString();
    a.apiPathsToApply = applyApiPaths;
    a.excludeFromArticulationPaths = excludeFromArticulationPaths;
    a.clearanceOffsets = clearanceOffsets;
    m_usdActions.push_back(std::move(a));
}

Axis SurfaceGripperComponent::_getJointForwardAxis(size_t attachmentIndex) const
{
    if (attachmentIndex < m_jointForwardAxis.size())
    {
        return m_jointForwardAxis[attachmentIndex];
    }
    // Match the IsaacRobotSchema default for `isaac:forwardAxis`.
    return Axis::X;
}

physx::PxTransform SurfaceGripperComponent::_computeJointWorldTransform(physx::PxJoint* joint,
                                                                        physx::PxRigidActor* actor0) const
{
    physx::PxTransform localPose0 = joint->getLocalPose(physx::PxJointActorIndex::eACTOR0);
    physx::PxTransform actorPose = actor0->getGlobalPose();
    return actorPose * localPose0;
}

physx::PxVec3 SurfaceGripperComponent::_directionFromAxisAndWorld(Axis axis, const physx::PxTransform& worldTransform) const
{
    physx::PxVec3 local(0.0f, 0.0f, 0.0f);
    switch (axis)
    {
    case Axis::X:
        local.x = 1.0f;
        break;
    case Axis::Y:
        local.y = 1.0f;
        break;
    default:
        local.z = 1.0f;
        break;
    }
    physx::PxVec3 dir = worldTransform.q.rotate(local);
    dir.normalize();
    return dir;
}

const std::vector<std::string>& SurfaceGripperComponent::getGrippedObjects() const
{
    if (!m_writeToUsd)
    {
        // Lazy refresh from physics state when not writing to USD
        const_cast<SurfaceGripperComponent*>(this)->updateGrippedObjectsList();
    }

    // Mirror set into cached vector without triggering USD writes
    m_grippedObjectsVector.clear();
    m_grippedObjectsVector.assign(m_grippedObjects.begin(), m_grippedObjects.end());
    return m_grippedObjectsVector;
}

physx::PxJoint* SurfaceGripperComponent::_getCachedJoint(size_t attachmentIndex) const
{
    if (attachmentIndex < m_attachmentJoints.size())
    {
        return m_attachmentJoints[attachmentIndex];
    }
    return nullptr;
}

physx::PxRigidActor* SurfaceGripperComponent::_getOrCacheActor(const std::string& actorPath)
{
    auto it = m_actorCache.find(actorPath);
    if (it != m_actorCache.end())
    {
        return it->second;
    }
    physx::PxRigidActor* actor = static_cast<physx::PxRigidActor*>(
        g_physx->getPhysXPtr(pxr::SdfPath(actorPath), omni::physx::PhysXType::ePTActor));
    if (actor)
    {
        m_actorCache.emplace(actorPath, actor);
    }
    return actor;
}

} // namespace surface_gripper
} // namespace robot
} // namespace isaacsim
