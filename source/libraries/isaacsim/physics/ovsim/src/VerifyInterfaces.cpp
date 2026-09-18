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

// Compile-time signature verification for isaacsim::ovsim.
//
// Each assignment below checks that the concrete free function in the
// current namespace is assignable to the canonical iface type alias (a
// std::function). This catches gross mismatches (wrong parameter count,
// unrelated types) at compile time, but — unlike a raw function-pointer
// alias — std::function also accepts any callable whose return type and
// parameters are merely convertible, so a same-shaped-but-not-identical
// signature can still compile. Nothing here runs at runtime.
//
// New packages (isaacsim_physics, …) must provide an equivalent Verify.cpp
// that checks their own implementations against the same iface aliases.
//
// Note: iface aliases do not carry default argument values (defaults are not
// part of a std::function's type).  Callers invoking through an alias must
// always pass every parameter explicitly, even those that have defaults in
// the concrete declaration.

#include <isaacsim/physics/ovsim/control/authoring/Authoring.hpp>
#include <isaacsim/physics/ovsim/control/simulation/Simulation.hpp>
#include <isaacsim/physics/ovsim/data/Data.hpp>
#include <ovsim/interfaces/control/authoring/Authoring.hpp>
#include <ovsim/interfaces/control/simulation/Simulation.hpp>
#include <ovsim/interfaces/data/Data.hpp>

namespace
{

namespace iface_control = ovsim::interfaces::control;
namespace iface_data = ovsim::interfaces::data;
namespace ns_control = isaacsim::physics::ovsim::control;
namespace ns_data = isaacsim::physics::ovsim::data;

// Data
[[maybe_unused]] iface_data::ReadFunction g_read = &ns_data::read;
[[maybe_unused]] iface_data::WriteFunction g_write = &ns_data::write;

// Control
// - Authoring
// -- Stage operations
[[maybe_unused]] iface_control::authoring::CreateStageFunction g_createStage = &ns_control::authoring::createStage;
[[maybe_unused]] iface_control::authoring::OpenStageFunction g_openStage = &ns_control::authoring::openStage;
[[maybe_unused]] iface_control::authoring::SaveStageFunction g_saveStage = &ns_control::authoring::saveStage;
[[maybe_unused]] iface_control::authoring::ImportStageFromStringFunction g_importStageFromString =
    &ns_control::authoring::importStageFromString;
[[maybe_unused]] iface_control::authoring::ExportStageToStringFunction g_exportStageToString =
    &ns_control::authoring::exportStageToString;
[[maybe_unused]] iface_control::authoring::CloseStageFunction g_closeStage = &ns_control::authoring::closeStage;
[[maybe_unused]] iface_control::authoring::AddReferenceToStageFunction g_addReferenceToStage =
    &ns_control::authoring::addReferenceToStage;
// -- Prim operations
[[maybe_unused]] iface_control::authoring::DefinePrimFunction g_definePrim = &ns_control::authoring::definePrim;
[[maybe_unused]] iface_control::authoring::MovePrimFunction g_movePrim = &ns_control::authoring::movePrim;
[[maybe_unused]] iface_control::authoring::RemovePrimFunction g_removePrim = &ns_control::authoring::removePrim;
// -- Attribute operations
[[maybe_unused]] iface_control::authoring::CreatePrimAttributeFunction g_createPrimAttribute =
    &ns_control::authoring::createPrimAttribute;
[[maybe_unused]] iface_control::authoring::RemovePrimAttributeFunction g_removePrimAttribute =
    &ns_control::authoring::removePrimAttribute;
// -- Parameters
[[maybe_unused]] iface_control::authoring::SetParameterFunction g_authoringSetParameter =
    &ns_control::authoring::setParameter;
[[maybe_unused]] iface_control::authoring::GetParameterFunction g_authoringGetParameter =
    &ns_control::authoring::getParameter;
// - Simulation
// -- Lifecycle operations
[[maybe_unused]] iface_control::simulation::PlayFunction g_play = &ns_control::simulation::play;
[[maybe_unused]] iface_control::simulation::PauseFunction g_pause = &ns_control::simulation::pause;
[[maybe_unused]] iface_control::simulation::StopFunction g_stop = &ns_control::simulation::stop;
[[maybe_unused]] iface_control::simulation::InitializeFunction g_initialize = &ns_control::simulation::initialize;
[[maybe_unused]] iface_control::simulation::InvalidateFunction g_invalidate = &ns_control::simulation::invalidate;
[[maybe_unused]] iface_control::simulation::StepFunction g_step = &ns_control::simulation::step;
// -- Parameters
[[maybe_unused]] iface_control::simulation::SetParameterFunction g_simulationSetParameter =
    &ns_control::simulation::setParameter;
[[maybe_unused]] iface_control::simulation::GetParameterFunction g_simulationGetParameter =
    &ns_control::simulation::getParameter;

} // namespace
