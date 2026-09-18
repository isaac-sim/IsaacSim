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

// Compile-time signature verification for isaacsim::ovsim::clients::local.
//
// Each assignment below checks that the concrete free function in the
// local client namespace is assignable to the canonical interface type alias (a
// std::function). This catches gross mismatches (wrong parameter count,
// unrelated types) at compile time, but — unlike a raw function-pointer
// alias — std::function also accepts any callable whose return type and
// parameters are merely convertible, so a same-shaped-but-not-identical
// signature can still compile. Nothing here runs at runtime.
//
// New packages must provide an equivalent VerifyInterfaces.cpp that checks their
// own implementations against the same interface aliases.
//
// Note: interface aliases do not carry default argument values (defaults are not
// part of a std::function's type).  Callers invoking through an alias must
// always pass every parameter explicitly, even those that have defaults in
// the concrete declaration.

#include <isaacsim/ovsim/clients/local/control/authoring/Authoring.hpp>
#include <isaacsim/ovsim/clients/local/control/simulation/Simulation.hpp>
#include <isaacsim/ovsim/clients/local/data/Data.hpp>
#include <ovsim/interfaces/control/authoring/Authoring.hpp>
#include <ovsim/interfaces/control/simulation/Simulation.hpp>
#include <ovsim/interfaces/data/Data.hpp>

namespace
{

namespace authoringInterface = ovsim::interfaces::control::authoring;
namespace simulationInterface = ovsim::interfaces::control::simulation;
namespace dataInterface = ovsim::interfaces::data;
namespace localControl = isaacsim::ovsim::clients::local::control;
namespace localData = isaacsim::ovsim::clients::local::data;

// Data
[[maybe_unused]] const dataInterface::ReadFunction g_kRead = &localData::read;
[[maybe_unused]] const dataInterface::WriteFunction g_kWrite = &localData::write;

// Control
// - Authoring
// -- Stage operations
[[maybe_unused]] const authoringInterface::CreateStageFunction g_kCreateStage = &localControl::authoring::createStage;
[[maybe_unused]] const authoringInterface::OpenStageFunction g_kOpenStage = &localControl::authoring::openStage;
[[maybe_unused]] const authoringInterface::SaveStageFunction g_kSaveStage = &localControl::authoring::saveStage;
[[maybe_unused]] const authoringInterface::ImportStageFromStringFunction g_kImportStageFromString =
    &localControl::authoring::importStageFromString;
[[maybe_unused]] const authoringInterface::ExportStageToStringFunction g_kExportStageToString =
    &localControl::authoring::exportStageToString;
[[maybe_unused]] const authoringInterface::CloseStageFunction g_kCloseStage = &localControl::authoring::closeStage;
[[maybe_unused]] const authoringInterface::AddReferenceToStageFunction g_kAddReferenceToStage =
    &localControl::authoring::addReferenceToStage;
// -- Prim operations
[[maybe_unused]] const authoringInterface::DefinePrimFunction g_kDefinePrim = &localControl::authoring::definePrim;
[[maybe_unused]] const authoringInterface::MovePrimFunction g_kMovePrim = &localControl::authoring::movePrim;
[[maybe_unused]] const authoringInterface::RemovePrimFunction g_kRemovePrim = &localControl::authoring::removePrim;
// -- Attribute operations
[[maybe_unused]] const authoringInterface::CreatePrimAttributeFunction g_kCreatePrimAttribute =
    &localControl::authoring::createPrimAttribute;
[[maybe_unused]] const authoringInterface::RemovePrimAttributeFunction g_kRemovePrimAttribute =
    &localControl::authoring::removePrimAttribute;
// -- Parameters
[[maybe_unused]] const authoringInterface::SetParameterFunction g_kSetAuthoringParameter =
    &localControl::authoring::setParameter;
[[maybe_unused]] const authoringInterface::GetParameterFunction g_kGetAuthoringParameter =
    &localControl::authoring::getParameter;
// - Simulation
// -- Lifecycle operations
[[maybe_unused]] const simulationInterface::PlayFunction g_kPlay = &localControl::simulation::play;
[[maybe_unused]] const simulationInterface::PauseFunction g_kPause = &localControl::simulation::pause;
[[maybe_unused]] const simulationInterface::StopFunction g_kStop = &localControl::simulation::stop;
[[maybe_unused]] const simulationInterface::InitializeFunction g_kInitialize = &localControl::simulation::initialize;
[[maybe_unused]] const simulationInterface::InvalidateFunction g_kInvalidate = &localControl::simulation::invalidate;
[[maybe_unused]] const simulationInterface::StepFunction g_kStep = &localControl::simulation::step;
// -- Parameters
[[maybe_unused]] const simulationInterface::SetParameterFunction g_kSetSimulationParameter =
    &localControl::simulation::setParameter;
[[maybe_unused]] const simulationInterface::GetParameterFunction g_kGetSimulationParameter =
    &localControl::simulation::getParameter;

} // namespace
