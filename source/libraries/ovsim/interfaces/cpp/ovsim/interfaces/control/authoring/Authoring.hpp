// SPDX-FileCopyrightText: Copyright (c) 2024-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <cstdint>
#include <functional>
#include <string>
#include <variant>

namespace ovsim
{
namespace interfaces
{
namespace control
{
namespace authoring
{
/** @brief Values accepted by provider parameter setters. */
using InputParameterType = std::variant<bool, std::uintptr_t, std::int64_t, double, std::string>;
/** @brief Values returned by provider parameter getters. */
using OutputParameterType = std::variant<bool, std::uintptr_t, std::int64_t, double, std::string>;

/** @brief Callable signature for creating an empty stage. */
using CreateStageFunction = std::function<bool()>;
/** @brief Callable signature for opening a stage from a USD path. */
using OpenStageFunction = std::function<bool(const std::string& /*usdPath*/)>;
/** @brief Callable signature for saving a stage to a USD path. */
using SaveStageFunction = std::function<bool(const std::string& /*usdPath*/)>;
/** @brief Callable signature for importing a stage from a USD string. */
using ImportStageFromStringFunction = std::function<bool(const std::string& /*usdString*/)>;
/** @brief Callable signature for exporting a stage to a USD string. */
using ExportStageToStringFunction = std::function<std::string()>;
/** @brief Callable signature for closing the current stage. */
using CloseStageFunction = std::function<bool()>;
/** @brief Callable signature for adding a referenced USD asset to the stage. */
using AddReferenceToStageFunction =
    std::function<bool(const std::string& /*usdPath*/, const std::string& /*path*/, const std::string& /*typeName*/)>;

/** @brief Callable signature for defining a prim. */
using DefinePrimFunction = std::function<bool(const std::string& /*path*/, const std::string& /*typeName*/)>;
/** @brief Callable signature for moving a prim. */
using MovePrimFunction = std::function<bool(const std::string& /*targetPath*/, const std::string& /*destinationPath*/)>;
/** @brief Callable signature for removing a prim. */
using RemovePrimFunction = std::function<bool(const std::string& /*path*/)>;

/** @brief Callable signature for creating a prim attribute. */
using CreatePrimAttributeFunction =
    std::function<bool(const std::string& /*path*/, const std::string& /*attributeName*/, const std::string& /*typeName*/)>;
/** @brief Callable signature for removing a prim attribute. */
using RemovePrimAttributeFunction =
    std::function<bool(const std::string& /*path*/, const std::string& /*attributeName*/)>;

/** @brief Callable signature for setting a named provider parameter. */
using SetParameterFunction = std::function<void(
    const std::string& /*provider*/, const std::string& /*parameterName*/, const InputParameterType& /*value*/)>;
/** @brief Callable signature for getting a named provider parameter. */
using GetParameterFunction =
    std::function<OutputParameterType(const std::string& /*provider*/, const std::string& /*parameterName*/)>;

} // namespace authoring
} // namespace control
} // namespace interfaces
} // namespace ovsim
