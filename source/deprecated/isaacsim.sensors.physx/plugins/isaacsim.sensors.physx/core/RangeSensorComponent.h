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

#include "isaacsim/core/includes/Component.h"
#include "isaacsim/core/includes/UsdUtilities.h"

#include <carb/tasking/ITasking.h>

#include <isaacsim/robot/schema/sensor_tokens.h>
#include <isaacsim/sensors/physx/IPhysxSensorInterface.h>
#include <isaacsim/util/debug_draw/PrimitiveDrawingHelper.h>
#include <omni/fabric/FabricUSD.h>
#include <omni/fabric/IToken.h>
#include <omni/fabric/SimStageWithHistory.h>
#include <omni/physx/IPhysx.h>
#include <omni/physx/IPhysxSceneQuery.h>
#include <omni/timeline/ITimeline.h>
#include <pxr/usd/usdPhysics/scene.h>

#include <PxActor.h>
#if defined(_WIN32)
#    include <PxRigidDynamic.h>
#else
#    pragma GCC diagnostic push
#    pragma GCC diagnostic ignored "-Wpragmas"
#    include <PxRigidDynamic.h>
#    pragma GCC diagnostic pop
#endif
namespace isaacsim
{
namespace sensors
{
namespace physx
{

using namespace isaacsim::robot::schema::sensors;

/**
 * @class RangeSensorComponentBase
 * @brief Base class which simulates a range sensor
 * @details This template class provides the core functionality for range-based sensors,
 *          including initialization, lifecycle management, and data processing. It handles
 *          sensor transforms, visualization, and interaction with the physics simulation.
 * @tparam PrimType The USD prim type associated with this sensor component
 */
template <class PrimType>
class RangeSensorComponentBase : public isaacsim::core::includes::ComponentBase<PrimType>
{

public:
    /**
     * @brief Constructs a new Isaac Component
     * @param[in] physxPtr Pointer to the PhysX interface for physics simulation
     * @details Initializes the component with necessary interfaces and creates visualization helpers
     */
    RangeSensorComponentBase(omni::physx::IPhysx* physxPtr)
        : m_physx(physxPtr),
          m_timeline(carb::getCachedInterface<omni::timeline::ITimeline>()),
          m_token(carb::getCachedInterface<omni::fabric::IToken>()),
          m_tasking(carb::getCachedInterface<carb::tasking::ITasking>()),
          m_lineDrawing(std::make_shared<isaacsim::util::debug_draw::drawing::PrimitiveDrawingHelper>(
              omni::usd::UsdContext::getContext(),
              isaacsim::util::debug_draw::drawing::PrimitiveDrawingHelper::RenderingMode::eLines)),
          m_pointDrawing(std::make_shared<isaacsim::util::debug_draw::drawing::PrimitiveDrawingHelper>(
              omni::usd::UsdContext::getContext(),
              isaacsim::util::debug_draw::drawing::PrimitiveDrawingHelper::RenderingMode::ePoints))
    {
    }

    /**
     * @brief Destroys the Range Sensor Component Base object
     * @details Cleans up visualization resources
     */
    ~RangeSensorComponentBase()
    {
        m_lineDrawing.reset();
        m_pointDrawing.reset();
    }

    /**
     * @brief Initializes various pointers and handles in the component
     * @details Must be called after creation, can be overridden to initialize subcomponents
     * @param[in] prim The USD prim representing this sensor
     * @param[in] stage The USD stage containing the sensor prim
     */
    virtual void initialize(const PrimType& prim, pxr::UsdStageWeakPtr stage)
    {
        isaacsim::core::includes::ComponentBase<PrimType>::initialize(prim, stage);
    }

    /**
     * @brief Function that runs after start is pressed
     * @details Initializes the component and locates the PhysX scene for sensor operations
     */
    virtual void onStart()
    {
        onComponentChange();

        pxr::UsdPrimRange range = this->m_stage->Traverse();

        // TODO: move this to the manager and only run once on start for all sensors
        m_pxScene = nullptr;
        for (pxr::UsdPrimRange::iterator iter = range.begin(); iter != range.end(); ++iter)
        {
            pxr::UsdPrim prim = *iter;

            if (prim.IsA<pxr::UsdPhysicsScene>())
            {
                m_pxScene = static_cast<::physx::PxScene*>(
                    m_physx->getPhysXPtr(prim.GetPrimPath(), omni::physx::PhysXType::ePTScene));

                if (m_pxScene)
                {
                    break;
                }
            }
        }
    }

    /**
     * @brief Called before tick, sequential, used to get sensor transforms
     * @details Empty base implementation that can be overridden by derived classes
     */
    virtual void preTick(){};

    /**
     * @brief Called every frame in parallel
     * @details Pure virtual function that must be implemented by derived classes to
     *          perform the main sensor update during each simulation frame
     */
    virtual void tick() = 0;

    /**
     * @brief Called after each physics step to update sensor data
     * @details This function is called after each physics simulation step to process and update
     *          the range sensor data based on the latest physics state
     */
    virtual void onPhysicsStep(){};

    /**
     * @brief Returns whether the sensor already ran its tick during the latest physics step
     * @return True when the next stage-update tick should skip sensor simulation
     */
    virtual bool hasTickedOnPhysicsStep() const
    {
        return false;
    };

    /**
     * @brief Clears the physics-step tick marker after the stage-update path observes it
     */
    virtual void clearTickedOnPhysicsStep(){};

    /**
     * @brief Called after all sensors have simulated to perform any drawing related tasks
     * @details Renders the debug visualization for both points and lines if enabled
     */
    virtual void draw()
    {
        m_lineDrawing->draw();
        m_pointDrawing->draw();
    }

    /**
     * @brief Run when stop is pressed
     * @details Clears all visualization data and ensures it's properly rendered
     */
    virtual void onStop()
    {
        m_lineDrawing->clear();
        m_pointDrawing->clear();
        m_lineDrawing->draw();
        m_pointDrawing->draw();
    };

    /**
     * @brief Called every time the Prim is changed
     * @details Updates component properties from the USD prim attributes and
     *          refreshes transform-related data
     */
    virtual void onComponentChange()
    {
        isaacsim::core::includes::safeGetAttribute(this->m_prim.GetAttribute(kEnabledAttr), this->m_enabled);
        isaacsim::core::includes::safeGetAttribute(this->m_prim.GetAttribute(kMinRangeAttr), m_minRange);
        isaacsim::core::includes::safeGetAttribute(this->m_prim.GetAttribute(kMaxRangeAttr), m_maxRange);
        isaacsim::core::includes::safeGetAttribute(this->m_prim.GetAttribute(kDrawPointsAttr), m_drawPoints);
        isaacsim::core::includes::safeGetAttribute(this->m_prim.GetAttribute(kDrawLinesAttr), m_drawLines);

        m_parentPrim = this->m_stage->GetPrimAtPath(this->m_prim.GetPath()).GetParent();
        m_metersPerUnit = static_cast<float>(UsdGeomGetStageMetersPerUnit(this->m_stage));

        if (m_parentPrim.IsA<pxr::UsdGeomXformable>())
        {
            std::vector<double> times;
            pxr::UsdGeomXformable(m_parentPrim).GetTimeSamples(&times);

            m_isParentPrimTimeSampled = times.size() > 1;
        }
        this->m_sequenceNumber = 0;
    }

    /**
     * @brief Updates timestamps for component
     * @param[in] timeSeconds Current simulation time in seconds
     * @param[in] dt Time step duration in seconds
     * @param[in] timeNano Current simulation time in nanoseconds
     */
    void updateTimestamp(double timeSeconds, double dt, int64_t timeNano)
    {
        isaacsim::core::includes::ComponentBase<PrimType>::updateTimestamp(timeSeconds, dt, timeNano);

        m_parentPrimTimeCode = pxr::UsdTimeCode::Default();
        if (m_isParentPrimTimeSampled)
        {
            m_parentPrimTimeCode = round(m_timeline->getCurrentTime() * this->m_stage->GetTimeCodesPerSecond());
        }
    }

    /**
     * @brief Gets the draw points visualization state
     * @return True if point visualization is enabled, false otherwise
     */
    bool getDrawPoints()
    {
        return m_drawPoints;
    }

    /**
     * @brief Gets the draw lines visualization state
     * @return True if line visualization is enabled, false otherwise
     */
    bool getDrawLines()
    {
        return m_drawLines;
    }

    /**
     * @brief Gets the latest point cloud data from the range sensor
     * @details Returns a reference to the vector containing the most recent hit positions
     *          detected by the range sensor. Each point represents a detected surface in 3D space.
     * @return Reference to vector of 3D points representing the latest point cloud data
     */
    std::vector<carb::Float3>& getPointCloud()
    {
        return m_lastHitPos;
    }

protected:
    /** @brief Flag to enable/disable point visualization */
    bool m_drawPoints = false;
    /** @brief Flag to enable/disable line visualization */
    bool m_drawLines = false;
    /** @brief Vector storing the most recent hit positions from the sensor */
    std::vector<carb::Float3> m_lastHitPos;

    /** @brief Minimum range of the sensor in meters */
    float m_minRange = 0.4f;
    /** @brief Maximum range of the sensor in meters */
    float m_maxRange = 100.0f;

    /** @brief Conversion factor from scene units to meters */
    float m_metersPerUnit = 1.0;

    /** @brief Reference to the parent USD prim containing this sensor */
    pxr::UsdPrim m_parentPrim;

    /** @brief Pointer to the PhysX interface */
    omni::physx::IPhysx* m_physx = nullptr;
    /** @brief Pointer to the PhysX scene */
    ::physx::PxScene* m_pxScene = nullptr;
    /** @brief Pointer to the timeline interface */
    omni::timeline::ITimeline* m_timeline = nullptr;
    /** @brief Pointer to the fabric token interface */
    omni::fabric::IToken* m_token = nullptr;
    /** @brief Pointer to the tasking interface */
    carb::tasking::ITasking* m_tasking = nullptr;
    /** @brief Helper for drawing debug lines */
    std::shared_ptr<isaacsim::util::debug_draw::drawing::PrimitiveDrawingHelper> m_lineDrawing;
    /** @brief Helper for drawing debug points */
    std::shared_ptr<isaacsim::util::debug_draw::drawing::PrimitiveDrawingHelper> m_pointDrawing;

    /** @brief Time code for the parent prim's current state */
    pxr::UsdTimeCode m_parentPrimTimeCode;
    /** @brief Flag indicating if the parent prim has time-sampled transforms */
    bool m_isParentPrimTimeSampled = false;

    /** @brief Flag indicating if this is the first frame */
    bool m_firstFrame = true;
};

/**
 * @typedef RangeSensorComponent
 * @brief Convenience typedef for a range sensor component using the UsdPrim type
 */
using RangeSensorComponent = RangeSensorComponentBase<pxr::UsdPrim>;

}
}
}
