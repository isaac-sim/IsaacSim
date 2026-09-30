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

#include <isaacsim/foundation/ovsim/Export.h>
#include <ovsim/interfaces/control/simulation/Simulation.hpp>

#include <string>

namespace isaacsim
{
namespace foundation
{
namespace ovsim
{
namespace control
{
namespace simulation
{

using ::ovsim::interfaces::control::simulation::InputParameterType;
using ::ovsim::interfaces::control::simulation::OutputParameterType;

/** @brief Accept a play request without changing simulation state. */
ISAACSIM_FOUNDATION_OVSIM_API void play();

/** @brief Accept a pause request without changing simulation state. */
ISAACSIM_FOUNDATION_OVSIM_API void pause();

/** @brief Accept a stop request without changing simulation state. */
ISAACSIM_FOUNDATION_OVSIM_API void stop();

/** @brief Copy the active OpenUSD stage into a retained OVStage simulation snapshot. */
ISAACSIM_FOUNDATION_OVSIM_API void initialize();

/** @brief Close and release the retained OVStage simulation snapshot, if one exists. */
ISAACSIM_FOUNDATION_OVSIM_API void invalidate();

/** @brief Accept a manual simulation step request without changing simulation state. */
ISAACSIM_FOUNDATION_OVSIM_API void step();

/**
 * @brief Reject a simulation provider parameter because simulation configuration is not implemented.
 * @param[in] provider Provider identifier.
 * @param[in] parameterName Provider-specific parameter name.
 * @param[in] value Parameter value.
 * @throws std::logic_error Always.
 */
ISAACSIM_FOUNDATION_OVSIM_API void setParameter(const std::string& provider,
                                                const std::string& parameterName,
                                                const InputParameterType& value);

/**
 * @brief Reject a simulation provider query because simulation configuration is not implemented.
 * @param[in] provider Provider identifier.
 * @param[in] parameterName Provider-specific parameter name.
 * @return This function does not return a value.
 * @throws std::logic_error Always.
 */
ISAACSIM_FOUNDATION_OVSIM_API OutputParameterType getParameter(const std::string& provider,
                                                               const std::string& parameterName);

} // namespace simulation
} // namespace control
} // namespace ovsim
} // namespace foundation
} // namespace isaacsim
