// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include "RegistrationNanobindHelpers.hpp"

#include <isaacsim/physics/registration/simulator/Simulation.hpp>
#include <nanobind/nanobind.h>
#include <nanobind/stl/function.h>
#include <nanobind/stl/pair.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/tuple.h>
#include <nanobind/stl/vector.h>

#include <functional>
#include <utility>

namespace isaacsim
{
namespace physics
{
namespace registration
{
namespace details
{

/// Simple RAII handle returned from Python-facing `subscribe_*` methods.
///
/// Holds a `SubscriptionId` and an unsubscribe function; the destructor runs
/// the unsubscribe automatically once the Python reference is dropped. Also
/// exposes an explicit `unsubscribe()` method for callers that want to release
/// eagerly.
class PythonSubscription
{
public:
    using UnsubscribeFunction = std::function<void(SubscriptionId)>;

    PythonSubscription() = default;

    PythonSubscription(SubscriptionId subscriptionId, UnsubscribeFunction unsubscribeFunction)
        : m_subscriptionId(subscriptionId), m_unsubscribeFunction(std::move(unsubscribeFunction))
    {
    }

    PythonSubscription(const PythonSubscription&) = delete;
    PythonSubscription& operator=(const PythonSubscription&) = delete;

    PythonSubscription(PythonSubscription&& other) noexcept
        : m_subscriptionId(other.m_subscriptionId), m_unsubscribeFunction(std::move(other.m_unsubscribeFunction))
    {
        other.m_subscriptionId = g_kInvalidSubscriptionId;
    }

    PythonSubscription& operator=(PythonSubscription&& other) noexcept
    {
        if (this != &other)
        {
            _release();
            m_subscriptionId = other.m_subscriptionId;
            m_unsubscribeFunction = std::move(other.m_unsubscribeFunction);
            other.m_subscriptionId = g_kInvalidSubscriptionId;
        }
        return *this;
    }

    ~PythonSubscription()
    {
        _release();
    }

    /// Invoke the stored unsubscribe function and mark the handle as released.
    /// Safe to call multiple times — subsequent calls are no-ops.
    void unsubscribe()
    {
        _release();
    }

    SubscriptionId getId() const noexcept
    {
        return m_subscriptionId;
    }

    bool isValid() const noexcept
    {
        return m_subscriptionId != g_kInvalidSubscriptionId && static_cast<bool>(m_unsubscribeFunction);
    }

private:
    void _release()
    {
        if (m_subscriptionId != g_kInvalidSubscriptionId && m_unsubscribeFunction)
        {
            // Release the GIL since unsubscribe may block on a C++ mutex and
            // the subscribing thread might be trying to acquire the GIL.
            nanobind::gil_scoped_release globalInterpreterLockRelease;
            m_unsubscribeFunction(m_subscriptionId);
        }
        m_subscriptionId = g_kInvalidSubscriptionId;
        m_unsubscribeFunction = nullptr;
    }

    SubscriptionId m_subscriptionId{ g_kInvalidSubscriptionId };
    UnsubscribeFunction m_unsubscribeFunction;
};

/// Bind `PythonSubscription` exactly once on the given module.
inline void bindPythonSubscription(nanobind::module_& module)
{
    nanobind::class_<PythonSubscription>(module, "Subscription",
                                         "Handle returned by subscribe_* methods; unsubscribes automatically "
                                         "when the handle is destroyed or when .unsubscribe() is called.")
        .def("unsubscribe", &PythonSubscription::unsubscribe, "Unsubscribe immediately. Idempotent.")
        .def_prop_ro(
            "id", &PythonSubscription::getId, "Underlying SubscriptionId, or k_invalid_subscription_id if released.")
        .def_prop_ro("valid", &PythonSubscription::isValid, "Whether the subscription remains active.");
}

} // namespace details
} // namespace registration
} // namespace physics
} // namespace isaacsim
