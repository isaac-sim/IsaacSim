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

#include <isaacsim/physics/registration/simulator/Simulation.hpp>
#include <nanobind/nanobind.h>
#include <nanobind/stl/bind_vector.h>
#include <nanobind/stl/function.h>
#include <nanobind/stl/pair.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/tuple.h>
#include <nanobind/stl/unordered_map.h>
#include <nanobind/stl/variant.h>
#include <nanobind/stl/vector.h>

#include <RegistrationNanobindHelpers.hpp>
#include <functional>
#include <utility>

namespace isaacsim
{
namespace physics
{
namespace manager
{

using registration::g_kInvalidSubscriptionId;
using registration::SubscriptionId;

/// Simple RAII handle returned from Python-facing `subscribe_*` methods.
///
/// Holds a `SubscriptionId` and an unsubscribe function; the destructor runs
/// the unsubscribe automatically once the Python reference is dropped. Also
/// exposes an explicit `unsubscribe()` method for callers that want to release
/// eagerly.
class PythonSubscription
{
public:
    using Unsubscribe = std::function<void(SubscriptionId)>;

    PythonSubscription() = default;

    PythonSubscription(SubscriptionId id, Unsubscribe unsubscribeFunction)
        : m_id(id), m_unsubscribe(std::move(unsubscribeFunction))
    {
    }

    PythonSubscription(const PythonSubscription&) = delete;
    PythonSubscription& operator=(const PythonSubscription&) = delete;

    PythonSubscription(PythonSubscription&& other) noexcept
        : m_id(other.m_id), m_unsubscribe(std::move(other.m_unsubscribe))
    {
        other.m_id = g_kInvalidSubscriptionId;
    }

    PythonSubscription& operator=(PythonSubscription&& other) noexcept
    {
        if (this != &other)
        {
            _release();
            m_id = other.m_id;
            m_unsubscribe = std::move(other.m_unsubscribe);
            other.m_id = g_kInvalidSubscriptionId;
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
        return m_id;
    }

    bool isValid() const noexcept
    {
        return m_id != g_kInvalidSubscriptionId && static_cast<bool>(m_unsubscribe);
    }

private:
    void _release()
    {
        if (m_id != g_kInvalidSubscriptionId && m_unsubscribe)
        {
            // Release the GIL since unsubscribe may block on a C++ mutex and
            // the subscribing thread might be trying to acquire the GIL.
            nanobind::gil_scoped_release globalInterpreterLockRelease;
            m_unsubscribe(m_id);
        }
        m_id = g_kInvalidSubscriptionId;
        m_unsubscribe = nullptr;
    }

    SubscriptionId m_id{ g_kInvalidSubscriptionId };
    Unsubscribe m_unsubscribe;
};

/// Bind `PythonSubscription` exactly once on the given module.
inline void bindPythonSubscription(nanobind::module_& module)
{
    nanobind::class_<PythonSubscription>(module, "Subscription",
                                         "Handle returned by subscribe_* methods; unsubscribes automatically "
                                         "when the handle is destroyed or when .unsubscribe() is called.")
        .def("unsubscribe", &PythonSubscription::unsubscribe, "Unsubscribe immediately. Idempotent.")
        .def_prop_ro("id", &PythonSubscription::getId,
                     "Underlying subscription identifier, or the invalid sentinel if released.")
        .def_prop_ro("valid", &PythonSubscription::isValid, "Whether the subscription remains active.");
}

} // namespace manager
} // namespace physics
} // namespace isaacsim
