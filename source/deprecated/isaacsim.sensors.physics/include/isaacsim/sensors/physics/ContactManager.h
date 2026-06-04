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

#include <carb/Framework.h>
#include <carb/events/EventsUtils.h>
#include <carb/logging/Log.h>

#include <isaacsim/sensors/physics/IsaacSensorTypes.h>
#include <omni/physx/ContactEvent.h>
#include <omni/physx/IPhysx.h>
#include <omni/physx/IPhysxSimulation.h>
#include <physxSchema/physxContactReportAPI.h>
#include <physxSchema/physxRigidBodyAPI.h>

#include <PxActor.h>

#if defined(_WIN32)
#    include <PxArticulationLink.h>
#else
#    pragma GCC diagnostic push
#    pragma GCC diagnostic ignored "-Wpragmas"
#    include <PxArticulationLink.h>
#    pragma GCC diagnostic pop
#endif

#include <PxRigidDynamic.h>
#include <PxScene.h>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace isaacsim
{
namespace sensors
{
namespace physics
{

/**
 * @brief Converts a USD path to an integer representation.
 * @details
 * Uses the same logic as SdfPath::_AsInt() to ensure path equality comparisons.
 * Assumes sizeof(pxr::SdfPath) == sizeof(uint64_t).
 *
 * @param[in] path USD path to convert.
 * @return Integer representation of the path.
 */
inline uint64_t asInt(const pxr::SdfPath& path)
{
    static_assert(sizeof(pxr::SdfPath) == sizeof(uint64_t), "Change to make the same size as pxr::SdfPath");
    uint64_t ret;
    std::memcpy(&ret, &path, sizeof(pxr::SdfPath));
    return ret;
}

/**
 * @struct ContactPair
 * @brief Represents a pair of bodies in contact.
 * @details
 * Stores and manages the identifiers of two bodies involved in a contact.
 * Ensures consistent ordering by keeping the smaller ID in body0.
 */
struct ContactPair
{
    /** @brief First body in the contact pair (always has the smaller ID). */
    uint64_t body0;

    /** @brief Second body in the contact pair. */
    uint64_t body1;

    /**
     * @brief Constructor from two body IDs.
     * @param[in] b0 First body ID.
     * @param[in] b1 Second body ID.
     */
    ContactPair(uint64_t b0, uint64_t b1) : body0(b0), body1(b1)
    {
        // keep body zero always the token with the smaller value
        if (b0 > b1)
        {
            body0 = b1;
            body1 = b0;
        }
    }

    /**
     * @brief Constructor from two USD paths.
     * @param[in] b0 First body's USD path.
     * @param[in] b1 Second body's USD path.
     */
    ContactPair(pxr::SdfPath b0, pxr::SdfPath b1) : ContactPair(asInt(b0), asInt(b1))
    {
    }

    /**
     * @brief Constructor from raw contact data.
     * @param[in] d Raw contact data containing body IDs.
     */
    ContactPair(CsRawData d) : ContactPair(d.body0, d.body1)
    {
    }

    /**
     * @brief Equality comparison operator.
     * @param[in] rhs Right-hand side contact pair to compare with.
     * @return True if both pairs represent the same contact.
     */
    bool operator==(ContactPair rhs) const
    {
        return body0 == rhs.body0 && body1 == rhs.body1;
    }
};

/**
 * @class ContactManager
 * @brief Manages contact events and data in the physics simulation.
 * @details
 * Handles the processing and storage of contact events between physical bodies,
 * providing access to raw contact data and managing contact sensor states.
 */
class ContactManager
{
public:
    /**
     * @brief Default constructor.
     */
    ContactManager();

    /**
     * @brief Virtual destructor.
     */
    virtual ~ContactManager();

    /**
     * @brief Resets all contact sensors to their initial state.
     */
    void resetSensors();

    /**
     * @brief Processes a contact event from the physics engine.
     * @param[in] c Contact event header.
     * @param[in] contactDataBuffer Buffer containing contact data.
     * @param[in,out] dataIdx Index into the contact data buffer.
     */
    void processContact(const omni::physx::ContactEventHeader c,
                        const omni::physx::ContactData* contactDataBuffer,
                        uint32_t& dataIdx);

    /**
     * @brief Gets raw contact data for a specific USD path.
     * @param[in] usdPath Path to the body in the USD stage.
     * @param[out] size Number of contact data points.
     * @return Pointer to array of raw contact data.
     */
    CsRawData* getCsRawData(const char* usdPath, size_t& size);

    /**
     * @brief Gets raw contact data for a specific body token.
     * @param[in] token Body identifier token.
     * @param[out] size Number of contact data points.
     * @return Pointer to array of raw contact data.
     */
    CsRawData* getCsRawData(uint64_t token, size_t& size);

    /**
     * @brief Removes raw contact data for a contact pair.
     * @param[in] p Contact pair to remove data for.
     */
    void removeRawData(const ContactPair& p);

    /**
     * @brief Updates contact manager state each physics step.
     * @param[in] currentTime Current simulation time.
     * @param[in] timeElapsed Time elapsed since last step.
     */
    void onPhysicsStep(const float& currentTime, const float& timeElapsed);

    /**
     * @brief Gets the current simulation time.
     * @return Current simulation time in seconds.
     */
    float getCurrentTime();

private:
    /** @brief Vector of raw contact data. */
    std::vector<CsRawData> m_contactRaw;

    /** @brief Map of body tokens to their raw contact data. */
    std::map<uint64_t, std::vector<CsRawData>> m_contactRawMap;

    /** @brief Subscription to contact event callbacks. */
    carb::events::ISubscriptionPtr m_contactCallbackPtr;

    /** @brief Number of contacts waiting to be processed. */
    size_t m_contactsToProcess{ 0 };

    /** @brief Number of contacts that have been processed. */
    size_t m_contactsProcessed{ 0 };

    /** @brief Current simulation time. */
    float m_currentTime{ 0.0f };

    /** @brief Current simulation time step. */
    float m_currentDt{ 0.0f };
};
}
}
}
