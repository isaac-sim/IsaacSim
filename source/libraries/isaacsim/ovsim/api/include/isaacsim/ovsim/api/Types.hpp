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

#include <isaacsim/ovsim/api/Export.h>
#include <ovsim/interfaces/control/authoring/Authoring.hpp>
#include <ovsim/interfaces/control/simulation/Simulation.hpp>
#include <ovsim/interfaces/data/Data.hpp>

namespace isaacsim
{
namespace ovsim
{
namespace api
{
namespace types
{

/** @brief Function interface for stage and prim authoring operations. */
struct Authoring
{
    /** @brief Create an empty stage and make it active. */
    ::ovsim::interfaces::control::authoring::CreateStageFunction createStage;
    /** @brief Open a USD stage and make it active. */
    ::ovsim::interfaces::control::authoring::OpenStageFunction openStage;
    /** @brief Save the active stage to a USD file. */
    ::ovsim::interfaces::control::authoring::SaveStageFunction saveStage;
    /** @brief Replace the active stage contents with serialized USD text. */
    ::ovsim::interfaces::control::authoring::ImportStageFromStringFunction importStageFromString;
    /** @brief Export the active stage as serialized USD text. */
    ::ovsim::interfaces::control::authoring::ExportStageToStringFunction exportStageToString;
    /** @brief Close the active stage. */
    ::ovsim::interfaces::control::authoring::CloseStageFunction closeStage;
    /** @brief Add a USD reference to a prim on the active stage. */
    ::ovsim::interfaces::control::authoring::AddReferenceToStageFunction addReferenceToStage;
    /** @brief Define a prim on the active stage. */
    ::ovsim::interfaces::control::authoring::DefinePrimFunction definePrim;
    /** @brief Move a prim to another path on the active stage. */
    ::ovsim::interfaces::control::authoring::MovePrimFunction movePrim;
    /** @brief Remove a prim from the active stage. */
    ::ovsim::interfaces::control::authoring::RemovePrimFunction removePrim;
    /** @brief Create an attribute on a prim. */
    ::ovsim::interfaces::control::authoring::CreatePrimAttributeFunction createPrimAttribute;
    /** @brief Remove an attribute from a prim. */
    ::ovsim::interfaces::control::authoring::RemovePrimAttributeFunction removePrimAttribute;
    /** @brief Set an authoring parameter exposed by a provider. */
    ::ovsim::interfaces::control::authoring::SetParameterFunction setParameter;
    /** @brief Get an authoring parameter exposed by a provider. */
    ::ovsim::interfaces::control::authoring::GetParameterFunction getParameter;
};

/** @brief Function interface for controlling simulation execution. */
struct Simulation
{
    /** @brief Start or resume automatic simulation stepping. */
    ::ovsim::interfaces::control::simulation::PlayFunction play;
    /** @brief Pause automatic simulation stepping. */
    ::ovsim::interfaces::control::simulation::PauseFunction pause;
    /** @brief Stop automatic simulation stepping. */
    ::ovsim::interfaces::control::simulation::StopFunction stop;
    /** @brief Initialize the simulation for manual stepping. */
    ::ovsim::interfaces::control::simulation::InitializeFunction initialize;
    /** @brief Invalidate the manually stepped simulation state. */
    ::ovsim::interfaces::control::simulation::InvalidateFunction invalidate;
    /** @brief Advance a manually controlled simulation by one step. */
    ::ovsim::interfaces::control::simulation::StepFunction step;
    /** @brief Set a simulation parameter exposed by a provider. */
    ::ovsim::interfaces::control::simulation::SetParameterFunction setParameter;
    /** @brief Get a simulation parameter exposed by a provider. */
    ::ovsim::interfaces::control::simulation::GetParameterFunction getParameter;
};

/** @brief Authoring and simulation-control interfaces for an OV SIM client. */
struct Control
{
    /** @brief Stage and prim authoring operations. */
    Authoring authoring;
    /** @brief Simulation execution operations. */
    Simulation simulation;
};

/** @brief Function interface for reading and writing simulation data. */
struct Data
{
    /** @brief Read an attribute from one or more prims. */
    ::ovsim::interfaces::data::ReadFunction read;
    /** @brief Write an attribute on one or more prims. */
    ::ovsim::interfaces::data::WriteFunction write;
};

/** @brief Complete control and data interface returned by an OV SIM client factory. */
struct Implementation
{
    /** @brief Authoring and simulation-control interfaces. */
    Control control;
    /** @brief Simulation data interface. */
    Data data;
};

} // namespace types
} // namespace api
} // namespace ovsim
} // namespace isaacsim
