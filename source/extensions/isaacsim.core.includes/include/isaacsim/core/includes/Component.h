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

// clang-format off
#include <pch/UsdPCH.h>
// clang-format on

#pragma once
#if defined(_WIN32)
#    include <usdrt/scenegraph/usd/usd/stage.h>
#else
#    pragma GCC diagnostic push
#    pragma GCC diagnostic ignored "-Wunused-variable"
#    pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#    include <usdrt/scenegraph/usd/usd/stage.h>
#    pragma GCC diagnostic pop
#endif

#include <string>
#include <vector>

namespace isaacsim
{
namespace core
{
namespace includes
{

/**
 * @class ComponentBase
 * @brief Base class template for USD prim-attached components in an Application
 * @details
 * ComponentBase provides the foundational structure for components that are attached to USD prims
 * within an Application. It manages the lifecycle, timing, and state of components while providing
 * virtual interfaces for key operations like initialization, updates, and event handling.
 *
 * @tparam PrimType The USD prim type that this component will be attached to
 *
 * @note All derived components must implement the pure virtual functions
 * @warning Components must be properly initialized with a valid USD prim and stage before use
 */
template <class PrimType>
class ComponentBase
{
public:
    /**
     * @brief Virtual destructor ensuring proper cleanup of derived classes
     */
    virtual ~ComponentBase() = default;

    /**
     * @brief Initializes the component with USD prim and stage references
     * @details Sets up the component's USD context and prepares it for execution
     *
     * @param[in] prim The USD prim to attach this component to
     * @param[in] stage The USD stage containing the prim
     *
     * @post Component is initialized with valid USD prim and stage references
     * @post mDoStart is set to true, indicating the component is ready to start
     */
    virtual void initialize(const PrimType& prim, pxr::UsdStageWeakPtr stage)
    {
        m_prim = prim;
        m_stage = stage;
        mDoStart = true;

        omni::fabric::IStageReaderWriter* iStageReaderWriter =
            carb::getCachedInterface<omni::fabric::IStageReaderWriter>();
        uint64_t stageId = static_cast<uint64_t>(pxr::UsdUtilsStageCache::Get().GetId(stage).ToLongInt());
        omni::fabric::StageReaderWriterId stageInProgress = iStageReaderWriter->get(stageId);
        m_usdrtStage = usdrt::UsdStage::Attach(stageId, stageInProgress);
    }

    /**
     * @brief Pure virtual function called after simulation start
     * @details Implement this to define component behavior at simulation start
     */
    virtual void onStart() = 0;

    /**
     * @brief Called when simulation is stopped
     * @details Override this to implement cleanup or state reset behavior
     */
    virtual void onStop()
    {
    }

    /**
     * @brief Called during each physics simulation step
     * @details Override this to implement physics-based behavior
     *
     * @param[in] dt Time step size in seconds
     */
    virtual void onPhysicsStep(float dt)
    {
    }

    /**
     * @brief Called for each rendered frame
     * @details Override this to implement render-specific behavior or visual updates
     */
    virtual void onRenderEvent()
    {
    }

    /**
     * @brief Pure virtual function called every frame
     * @details Implement this to define the component's per-frame behavior
     */
    virtual void tick() = 0;

    /**
     * @brief Pure virtual function called when the component's prim changes
     * @details Implement this to handle USD prim attribute or relationship changes
     */
    virtual void onComponentChange() = 0;

    /**
     * @brief Updates the component's internal timing information
     * @details Maintains synchronized timing state across the component
     *
     * @param[in] timeSeconds Current simulation time in seconds
     * @param[in] dt Time step size in seconds
     * @param[in] timeNano Current simulation time in nanoseconds
     */
    virtual void updateTimestamp(double timeSeconds, double dt, int64_t timeNano)
    {
        this->m_timeSeconds = timeSeconds;
        this->m_timeDelta = dt;
        this->m_timeNanoSeconds = timeNano;
    }

    /**
     * @brief Retrieves the component's USD prim
     * @return Reference to the component's USD prim
     */
    PrimType& getPrim()
    {
        return m_prim;
    }

    /**
     * @brief Checks if the component is enabled
     * @return true if the component is enabled, false otherwise
     */
    bool getEnabled()
    {
        return m_enabled;
    }

    /**
     * @brief Gets the component's sequence number
     * @return The component's sequence number
     */
    uint64_t getSequenceNumber()
    {
        return m_sequenceNumber;
    }

    /** @brief Flag indicating whether onStart should be called */
    bool mDoStart = true;

protected:
    /** @brief USD prim reference storing component settings */
    PrimType m_prim;

    /** @brief Weak pointer to the USD stage containing the prim */
    pxr::UsdStageWeakPtr m_stage = nullptr;

    /** @brief Runtime USD stage reference */
    usdrt::UsdStageRefPtr m_usdrtStage = nullptr;

    /** @brief Current simulation time in seconds */
    double m_timeSeconds = 0;

    /** @brief Current simulation time in nanoseconds */
    int64_t m_timeNanoSeconds = 0;

    /** @brief Time delta for current tick in seconds */
    double m_timeDelta = 0;

    /** @brief Component sequence number for ordering/identification */
    uint64_t m_sequenceNumber = 0;

    /** @brief Component enabled state flag */
    bool m_enabled = true;
};

/**
 * @typedef Component
 * @brief Convenience typedef for ComponentBase specialized with pxr::UsdPrim
 */
using Component = ComponentBase<pxr::UsdPrim>;

}
}
}
