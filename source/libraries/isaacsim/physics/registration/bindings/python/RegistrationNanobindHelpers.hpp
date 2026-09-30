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

#include <isaacsim/physics/registration/simulator/Interaction.hpp>
#include <isaacsim/physics/registration/simulator/Simulation.hpp>
#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/unordered_map.h>

#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>

namespace isaacsim
{
namespace physics
{
namespace registration
{
namespace details
{

namespace nb = nanobind;

inline nb::dict convertDebugDataItemToPythonDictionary(const DebugDataItem& item)
{
    nb::dict dictionary;
    dictionary["type"] = static_cast<int>(item.type);
    if (!item.description.empty())
    {
        dictionary["doc"] = item.description;
    }

    switch (item.type)
    {
    case DebugDataItemType::eFloat:
        dictionary["value"] = item.doubleValue;
        break;
    case DebugDataItemType::eVector:
    case DebugDataItemType::ePoint:
        dictionary["value"] = nb::make_tuple(item.vector3Value[0], item.vector3Value[1], item.vector3Value[2]);
        break;
    case DebugDataItemType::eQuaternion:
        dictionary["value"] =
            nb::make_tuple(item.vector4Value[0], item.vector4Value[1], item.vector4Value[2], item.vector4Value[3]);
        break;
    case DebugDataItemType::eString:
        dictionary["value"] = item.stringValue;
        break;
    case DebugDataItemType::eBoolean:
        dictionary["value"] = item.booleanValue;
        break;
    case DebugDataItemType::eInteger:
        dictionary["value"] = item.integerValue;
        break;
    case DebugDataItemType::eUndefined:
    default:
        dictionary["value"] = nb::none();
        break;
    }
    return dictionary;
}

inline DebugDataItem convertPythonDictionaryToDebugDataItem(nb::handle source)
{
    DebugDataItem item;
    if (!nb::isinstance<nb::dict>(source))
    {
        return item;
    }
    const nb::dict dictionary = nb::cast<nb::dict>(source);

    if (dictionary.contains("type"))
    {
        item.type = static_cast<DebugDataItemType>(nb::cast<int>(dictionary["type"]));
    }
    if (dictionary.contains("doc"))
    {
        item.description = nb::cast<std::string>(dictionary["doc"]);
    }
    if (!dictionary.contains("value"))
    {
        return item;
    }
    const nb::handle valueHandle = dictionary["value"];

    switch (item.type)
    {
    case DebugDataItemType::eFloat:
        item.doubleValue = nb::cast<double>(valueHandle);
        break;
    case DebugDataItemType::eVector:
    case DebugDataItemType::ePoint:
    {
        const auto tuple = nb::cast<nb::tuple>(valueHandle);
        item.vector3Value[0] = nb::cast<double>(tuple[0]);
        item.vector3Value[1] = nb::cast<double>(tuple[1]);
        item.vector3Value[2] = nb::cast<double>(tuple[2]);
        break;
    }
    case DebugDataItemType::eQuaternion:
    {
        const auto tuple = nb::cast<nb::tuple>(valueHandle);
        item.vector4Value[0] = nb::cast<double>(tuple[0]);
        item.vector4Value[1] = nb::cast<double>(tuple[1]);
        item.vector4Value[2] = nb::cast<double>(tuple[2]);
        item.vector4Value[3] = nb::cast<double>(tuple[3]);
        break;
    }
    case DebugDataItemType::eString:
        item.stringValue = nb::cast<std::string>(valueHandle);
        break;
    case DebugDataItemType::eBoolean:
        item.booleanValue = nb::cast<bool>(valueHandle);
        break;
    case DebugDataItemType::eInteger:
        item.integerValue = nb::cast<int32_t>(valueHandle);
        break;
    default:
        break;
    }
    return item;
}

inline nb::dict convertDebugDataToPythonDictionary(const DebugDataDictionary& debugData)
{
    nb::dict result;
    for (const auto& entry : debugData)
    {
        result[nb::cast(entry.first)] = convertDebugDataItemToPythonDictionary(entry.second);
    }
    return result;
}

inline DebugDataDictionary convertPythonDictionaryToDebugData(nb::handle source)
{
    DebugDataDictionary result;
    if (!nb::isinstance<nb::dict>(source))
    {
        return result;
    }
    const nb::dict dictionary = nb::cast<nb::dict>(source);
    for (const auto item : dictionary)
    {
        std::string key = nb::cast<std::string>(item.first);
        result.emplace(std::move(key), convertPythonDictionaryToDebugDataItem(item.second));
    }
    return result;
}

} // namespace details
} // namespace registration
} // namespace physics
} // namespace isaacsim

// Make the registration strong types transparent to Python. Python retains the
// pre-strong-type integer surface while C++ keeps distinct identifier types.
namespace nanobind
{
namespace detail
{

template <>
struct type_caster<isaacsim::physics::registration::SubscriptionId>
{
    NB_TYPE_CASTER(isaacsim::physics::registration::SubscriptionId, const_name("int"))

    // NOLINTNEXTLINE(readability-identifier-naming): Required nanobind type-caster hook.
    bool from_python(handle source, uint8_t flags, cleanup_list* cleanup) noexcept
    {
        make_caster<size_t> caster;
        if (!caster.from_python(source, flags, cleanup))
        {
            return false;
        }
        value = isaacsim::physics::registration::SubscriptionId(caster.value);
        return true;
    }

    // NOLINTNEXTLINE(readability-identifier-naming): Required nanobind type-caster hook.
    static handle from_cpp(isaacsim::physics::registration::SubscriptionId subscriptionId,
                           rv_policy policy,
                           cleanup_list* cleanup) noexcept
    {
        return make_caster<size_t>::from_cpp(subscriptionId.id, policy, cleanup);
    }
};

template <>
struct type_caster<isaacsim::physics::registration::PathToken>
{
    NB_TYPE_CASTER(isaacsim::physics::registration::PathToken, const_name("int"))

    // NOLINTNEXTLINE(readability-identifier-naming): Required nanobind type-caster hook.
    bool from_python(handle source, uint8_t flags, cleanup_list* cleanup) noexcept
    {
        make_caster<uint64_t> caster;
        if (!caster.from_python(source, flags, cleanup))
        {
            return false;
        }
        value = isaacsim::physics::registration::PathToken(caster.value);
        return true;
    }

    // NOLINTNEXTLINE(readability-identifier-naming): Required nanobind type-caster hook.
    static handle from_cpp(isaacsim::physics::registration::PathToken pathToken,
                           rv_policy policy,
                           cleanup_list* cleanup) noexcept
    {
        return make_caster<uint64_t>::from_cpp(pathToken.path, policy, cleanup);
    }
};

template <>
struct type_caster<isaacsim::physics::registration::GetPrimDebugDataFunction>
{
    NB_TYPE_CASTER(isaacsim::physics::registration::GetPrimDebugDataFunction, const_name("Callable[[str], dict]"))

    // NOLINTNEXTLINE(readability-identifier-naming): Required nanobind type-caster hook.
    bool from_python(handle source, uint8_t /*flags*/, cleanup_list* /*cleanup*/) noexcept
    {
        if (!source.is_valid())
        {
            return false;
        }
        if (source.is_none())
        {
            value = nullptr;
            return true;
        }
        if (!PyCallable_Check(source.ptr()))
        {
            return false;
        }
        nanobind::callable pythonFunction = nanobind::borrow<nanobind::callable>(source);
        value = [pythonFunction](const std::string& primPath) -> isaacsim::physics::registration::DebugDataDictionary
        {
            nanobind::gil_scoped_acquire globalInterpreterLockAcquire;
            try
            {
                nanobind::object result = pythonFunction(primPath);
                if (result.is_none())
                {
                    return {};
                }
                return isaacsim::physics::registration::details::convertPythonDictionaryToDebugData(result);
            }
            catch (nanobind::python_error& error)
            {
                error.discard_as_unraisable(primPath.empty() ? "get_prim_debug_data" : primPath.c_str());
                return {};
            }
        };
        return true;
    }

    // NOLINTNEXTLINE(readability-identifier-naming): Required nanobind type-caster hook.
    static handle from_cpp(const isaacsim::physics::registration::GetPrimDebugDataFunction& source,
                           rv_policy /*policy*/,
                           cleanup_list* /*cleanup*/) noexcept
    {
        if (!source)
        {
            return nanobind::none().release();
        }
        nanobind::object wrapped = nanobind::cpp_function(
            [source](const std::string& primPath) -> nanobind::dict
            { return isaacsim::physics::registration::details::convertDebugDataToPythonDictionary(source(primPath)); });
        return wrapped.release();
    }
};

} // namespace detail
} // namespace nanobind
