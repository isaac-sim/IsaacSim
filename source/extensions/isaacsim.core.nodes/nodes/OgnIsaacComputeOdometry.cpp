// SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <carb/logging/Log.h>

#include <isaacsim/core/experimental/prims/IPrimDataReader.h>
#include <isaacsim/core/experimental/prims/IPrimDataReaderManager.h>
#include <isaacsim/core/includes/BaseResetNode.h>
#include <isaacsim/core/includes/PhysicsEngine.h>
#include <isaacsim/core/nodes/ICoreNodes.h>
#include <isaacsim/core/simulation_manager/ISimulationManager.h>
#include <isaacsim/robot/schema/utils.h>
#include <omni/fabric/FabricUSD.h>
#include <omni/usd/UsdContext.h>
#include <omni/usd/UsdContextIncludes.h>
#include <pxr/usd/usdPhysics/articulationRootAPI.h>
#include <pxr/usd/usdPhysics/rigidBodyAPI.h>

#include <OgnIsaacComputeOdometryDatabase.h>
#include <atomic>
#include <cstdint>
#include <string>

namespace isaacsim
{
namespace core
{
namespace nodes
{

using experimental::prims::IArticulationDataView;
using experimental::prims::IPrimDataReader;
using experimental::prims::IPrimDataReaderManager;
using experimental::prims::IRigidBodyDataView;

static std::atomic<int> s_viewCounter{ 0 };

class OgnIsaacComputeOdometry : public isaacsim::core::includes::BaseResetNode
{
public:
    ~OgnIsaacComputeOdometry()
    {
        cleanupView();
    }

    static void initInstance(NodeObj const& nodeObj, GraphInstanceID instanceId)
    {
        auto& state = OgnIsaacComputeOdometryDatabase::sPerInstanceState<OgnIsaacComputeOdometry>(nodeObj, instanceId);
        state.m_simulationManagerFramework =
            carb::getCachedInterface<isaacsim::core::simulation_manager::ISimulationManager>();
        state.m_readerManager = carb::getCachedInterface<IPrimDataReaderManager>();
        state.m_reader = state.m_readerManager ? state.m_readerManager->getReader() : nullptr;
    }

    static bool compute(OgnIsaacComputeOdometryDatabase& db)
    {
        const GraphContextObj& context = db.abi_context();
        auto& state = db.perInstanceState<OgnIsaacComputeOdometry>();
        if (!state.ensureCurrentView(db, context))
        {
            return false;
        }

        state.computeOdometry(db);

        db.outputs.execOut() = kExecutionAttributeStateEnabled;
        return true;
    }

private:
    bool ensureCurrentView(OgnIsaacComputeOdometryDatabase& db, const GraphContextObj& context)
    {
        if (!m_simulationManagerFramework || !m_simulationManagerFramework->isSimulating())
        {
            return false;
        }

        if (!m_firstFrame && m_reader && m_reader->getGeneration() == m_readerGeneration)
        {
            return true;
        }

        const bool preserveReferencePose = !m_firstFrame;

        long stageId = context.iContext->getStageId(context);
        auto stage = pxr::UsdUtilsStageCache::Get().Find(pxr::UsdStageCache::Id::FromLongInt(stageId));
        if (!stage)
        {
            db.logError("Could not find USD stage with ID %ld", stageId);
            return false;
        }

        const auto& prim = db.inputs.chassisPrim();
        if (prim.empty())
        {
            db.logError("No chassis (target) prim specified");
            return false;
        }

        auto primSdfPath = omni::fabric::toSdfPath(prim[0]);
        auto usdPrim = stage->GetPrimAtPath(primSdfPath);
        if (!usdPrim)
        {
            db.logError(
                "The prim %s is not valid. Please specify at least one valid chassis prim", primSdfPath.GetText());
            return false;
        }

        if (!m_readerManager)
        {
            m_readerManager = carb::getCachedInterface<IPrimDataReaderManager>();
        }
        if (!m_readerManager)
        {
            db.logError("Failed to acquire IPrimDataReaderManager interface");
            return false;
        }

        if (!m_readerManager->ensureInitialized(stageId, -1))
        {
            db.logError("Failed to initialize shared prim data reader session");
            return false;
        }

        cleanupView();

        m_reader = m_readerManager->getReader();
        if (!m_reader)
        {
            db.logError("Failed to acquire shared IPrimDataReader interface");
            return false;
        }
        m_readerGeneration = m_reader->getGeneration();

        m_viewId = "odometry_" + std::to_string(s_viewCounter.fetch_add(1));
        const char* pathStr = primSdfPath.GetText();
        const char* engine = isaacsim::core::includes::getActivePhysicsEngineName();

        // Prefer RigidBodyView when the chassis prim is itself a rigid body: the articulation
        // view's root pose tracks PhysX's auto-selected root link, which may not match the
        // user-specified chassis (e.g. after Robot Assembler attaches bodies via fixed joints).
        const bool hasRigidBody = usdPrim.HasAPI<pxr::UsdPhysicsRigidBodyAPI>();
        const bool hasArticulationRoot = usdPrim.HasAPI<pxr::UsdPhysicsArticulationRootAPI>();
        const bool hasRobotApi =
            usdPrim.HasAPI(isaacsim::robot::schema::className(isaacsim::robot::schema::Classes::ROBOT_API));

        // When `chassisPrim` carries `IsaacRobotAPI` but not `UsdPhysicsRigidBodyAPI` /
        // `UsdPhysicsArticulationRootAPI` directly (a common asset layout when the articulation
        // root lives on a deeper `base_link`), use the first entry of `isaac:physics:robotLinks`.
        // The schema defines that entry as the robot's base link, so no search is needed.
        // This must happen BEFORE the rigid-body and articulation branches forward `pathStr` to
        // `physx-tensors`, otherwise the tensors plugin emits a `did not match any rigid bodies`
        // warning before our `logError` fires.
        pxr::UsdPrim effectivePrim = usdPrim;
        std::string resolvedPathStorage;
        if (!hasRigidBody && !hasArticulationRoot && hasRobotApi)
        {
            const auto links = isaacsim::robot::schema::GetAllRobotLinks(stage, usdPrim);
            if (links.empty() || !links.front())
            {
                db.logError(
                    "IsaacRobotAPI prim '%s' has no resolvable `isaac:physics:robotLinks` entry to "
                    "use for odometry",
                    pathStr);
                return false;
            }
            effectivePrim = links.front();
            resolvedPathStorage = effectivePrim.GetPath().GetString();
        }
        // `resolvedPathStorage` outlives `effectivePathStr` (same scope), so this aliasing is safe.
        const char* effectivePathStr = resolvedPathStorage.empty() ? pathStr : resolvedPathStorage.c_str();

        if (effectivePrim.HasAPI<pxr::UsdPhysicsRigidBodyAPI>())
        {
            m_rigidBodyView = m_reader->createRigidBodyView(m_viewId.c_str(), &effectivePathStr, 1, engine);
            if (!m_rigidBodyView)
            {
                db.logError("Failed to create rigid body view for '%s'", effectivePathStr);
                return false;
            }
        }
        else if (effectivePrim.HasAPI<pxr::UsdPhysicsArticulationRootAPI>())
        {
            // Articulation root is not a rigid body (e.g. ancestor Xform); odometry follows
            // PhysX's auto-selected root link. Point chassisPrim at the rigid body to override.
            m_articulationView = m_reader->createArticulationView(m_viewId.c_str(), &effectivePathStr, 1, engine);
            if (!m_articulationView)
            {
                db.logError("Failed to create articulation view for '%s'", effectivePathStr);
                return false;
            }
        }
        else
        {
            db.logError("The prim at path '%s' is not a valid rigid body or articulation root", pathStr);
            return false;
        }

        readTransformAndVelocity();
        if (!preserveReferencePose)
        {
            m_startingPos = m_position;
            m_startingQuat = m_orientation;
        }
        m_unitScale = UsdGeomGetStageMetersPerUnit(stage);

        pxr::GfRotation rotation(m_orientation);
        pxr::GfVec3d bodyLocalLinVel = rotation.GetInverse().TransformDir(m_globalLinearVel);
        m_prevLinearVelocity = bodyLocalLinVel;
        m_prevGlobalLinearVelocity = m_globalLinearVel;
        m_prevAngularVelocity = m_bodyAngularVel;
        m_lastTime = m_simulationManagerFramework->getSimulationTime();
        m_firstFrame = false;
        return true;
    }

    void readTransformAndVelocity()
    {
        if (m_articulationView)
        {
            m_articulationView->update();
        }
        else if (m_rigidBodyView)
        {
            m_rigidBodyView->update();
        }

        int count = 0;
        if (m_articulationView)
        {
            const float* tf = m_articulationView->getRootTransformsHost(&count);
            if (tf && count >= 7)
            {
                m_position = pxr::GfVec3d(tf[0], tf[1], tf[2]);
                // PhysX root transforms layout: [px, py, pz, qx, qy, qz, qw]
                m_orientation = pxr::GfQuatd(tf[6], tf[3], tf[4], tf[5]);
            }
            const float* vel = m_articulationView->getRootVelocitiesHost(&count);
            if (vel && count >= 6)
            {
                m_globalLinearVel = pxr::GfVec3d(vel[0], vel[1], vel[2]);
                m_bodyAngularVel = pxr::GfVec3d(vel[3], vel[4], vel[5]);
            }
        }
        else if (m_rigidBodyView)
        {
            const float* pos = m_rigidBodyView->getWorldPositionsHost(&count);
            if (pos && count >= 3)
            {
                m_position = pxr::GfVec3d(pos[0], pos[1], pos[2]);
            }
            const float* ori = m_rigidBodyView->getWorldOrientationsHost(&count);
            if (ori && count >= 4)
            {
                // Fabric decomposeMatrix layout: [qw, qx, qy, qz]
                m_orientation = pxr::GfQuatd(ori[0], ori[1], ori[2], ori[3]);
            }
            const float* lv = m_rigidBodyView->getLinearVelocitiesHost(&count);
            if (lv && count >= 3)
            {
                m_globalLinearVel = pxr::GfVec3d(lv[0], lv[1], lv[2]);
            }
            const float* av = m_rigidBodyView->getAngularVelocitiesHost(&count);
            if (av && count >= 3)
            {
                m_bodyAngularVel = pxr::GfVec3d(av[0], av[1], av[2]);
            }
        }
    }

    void computeOdometry(OgnIsaacComputeOdometryDatabase& db)
    {
        if (!m_articulationView && !m_rigidBodyView)
        {
            return;
        }

        readTransformAndVelocity();

        pxr::GfRotation rotation(m_orientation);
        pxr::GfVec3d bodyLocalLinVel = rotation.GetInverse().TransformDir(m_globalLinearVel);

        if (m_simulationManagerFramework->getSimulationTime() != m_lastTime)
        {
            double dt = m_simulationManagerFramework->getSimulationTime() - m_lastTime;

            m_linearAcceleration = (bodyLocalLinVel - m_prevLinearVelocity) / dt;
            m_globalLinearAcceleration = (m_globalLinearVel - m_prevGlobalLinearVelocity) / dt;
            m_angularAcceleration = (m_bodyAngularVel - m_prevAngularVelocity) / dt;

            db.outputs.linearAcceleration().Set(
                m_linearAcceleration[0], m_linearAcceleration[1], m_linearAcceleration[2]);
            db.outputs.globalLinearAcceleration().Set(
                m_globalLinearAcceleration[0], m_globalLinearAcceleration[1], m_globalLinearAcceleration[2]);
            db.outputs.angularAcceleration().Set(
                m_angularAcceleration[0], m_angularAcceleration[1], m_angularAcceleration[2]);
        }

        pxr::GfVec3d globalTranslation = m_position - m_startingPos;
        pxr::GfRotation startRotation(m_startingQuat);

        db.outputs.position() = startRotation.GetInverse().TransformDir(globalTranslation) * m_unitScale;
        db.outputs.orientation() = (rotation * startRotation.GetInverse()).GetQuat();

        db.outputs.linearVelocity().Set(bodyLocalLinVel[0], bodyLocalLinVel[1], bodyLocalLinVel[2]);
        db.outputs.globalLinearVelocity().Set(m_globalLinearVel[0], m_globalLinearVel[1], m_globalLinearVel[2]);
        db.outputs.angularVelocity().Set(m_bodyAngularVel[0], m_bodyAngularVel[1], m_bodyAngularVel[2]);

        m_prevLinearVelocity = bodyLocalLinVel;
        m_prevGlobalLinearVelocity = m_globalLinearVel;
        m_prevAngularVelocity = m_bodyAngularVel;
        m_lastTime = m_simulationManagerFramework->getSimulationTime();
    }

    virtual void reset()
    {
        cleanupView();
        m_firstFrame = true;
    }

private:
    void cleanupView()
    {
        if (m_reader && !m_viewId.empty())
        {
            m_reader->removeView(m_viewId.c_str());
        }
        m_articulationView = nullptr;
        m_rigidBodyView = nullptr;
        m_reader = nullptr;
        m_readerGeneration = 0;
        m_viewId.clear();
    }

    IPrimDataReader* m_reader = nullptr;
    IPrimDataReaderManager* m_readerManager = nullptr;
    IArticulationDataView* m_articulationView = nullptr;
    IRigidBodyDataView* m_rigidBodyView = nullptr;
    std::string m_viewId;
    uint64_t m_readerGeneration = 0;

    isaacsim::core::simulation_manager::ISimulationManager* m_simulationManagerFramework = nullptr;
    bool m_firstFrame = true;
    double m_lastTime = 0.0;
    double m_unitScale = 1.0;

    pxr::GfVec3d m_position;
    pxr::GfQuatd m_orientation = pxr::GfQuatd(1, 0, 0, 0);
    pxr::GfVec3d m_globalLinearVel;
    pxr::GfVec3d m_bodyAngularVel;

    pxr::GfVec3d m_startingPos;
    pxr::GfQuatd m_startingQuat = pxr::GfQuatd(1, 0, 0, 0);

    pxr::GfVec3d m_prevLinearVelocity;
    pxr::GfVec3d m_prevGlobalLinearVelocity;
    pxr::GfVec3d m_prevAngularVelocity;
    pxr::GfVec3d m_linearAcceleration;
    pxr::GfVec3d m_angularAcceleration;
    pxr::GfVec3d m_globalLinearAcceleration;
};

REGISTER_OGN_NODE()
}
}
}
