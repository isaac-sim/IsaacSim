// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

namespace isaacsim
{
namespace core
{
namespace includes
{

/**
 * @class UsdNoticeListener
 * @brief Helper base class to subscribe to pxr::TfNotice.
 * @details
 * Provides a convenient base class for listening to USD notices with automatic
 * cleanup and weak pointer management. Template parameter T specifies the
 * notice type to listen for.
 *
 * @tparam T The type of USD notice to listen for (e.g., pxr::UsdNotice::ObjectsChanged)
 *
 * @note Inherits from pxr::TfWeakBase to enable weak pointer management
 * @warning Always call revokeListener() before destruction to avoid memory leaks
 */
template <typename T>
class UsdNoticeListener : public pxr::TfWeakBase
{
public:
    virtual ~UsdNoticeListener()
    {
        revokeListener();
    }

    /**
     * @brief Registers this listener to receive USD notices of type T.
     * @details
     * Sets up the notice listener using weak pointers to avoid circular references.
     * Automatically revokes any existing listener registration to prevent leaks.
     *
     * @note Safe to call multiple times - will revoke previous registration first
     */
    void registerListener()
    {
        // To avoid leak
        revokeListener();
        m_usdNoticeListenerKey = pxr::TfNotice::Register(pxr::TfCreateWeakPtr(this), &UsdNoticeListener::handleNotice);
    }

    /**
     * @brief Revokes the notice listener registration.
     * @details
     * Safely unregisters the notice listener if it was previously registered.
     * Safe to call multiple times or when no listener is registered.
     *
     * @note Automatically called in destructor
     */
    void revokeListener()
    {
        if (m_usdNoticeListenerKey.IsValid())
        {
            pxr::TfNotice::Revoke(m_usdNoticeListenerKey);
        }
    }

    /**
     * @brief Pure virtual function to handle received notices.
     * @details
     * Derived classes must implement this function to process the received
     * USD notices. Called automatically when a notice of type T is received.
     *
     * @param[in] objectsChanged The notice object containing change information
     *
     * @note This function is called from the USD notice system thread
     */
    virtual void handleNotice(const T& objectsChanged) = 0;

private:
    pxr::TfNotice::Key m_usdNoticeListenerKey;
};

} // namespace includes
} // namespace core
} // namespace isaacsim
