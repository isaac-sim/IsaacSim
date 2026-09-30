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

#include "isaacsim/common/array/nanobind/ArrayCaster.hpp"
#include "isaacsim/physics/ovsim/control/authoring/Authoring.hpp"
#include "isaacsim/physics/ovsim/control/simulation/Simulation.hpp"
#include "isaacsim/physics/ovsim/data/Data.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/variant.h>
#include <nanobind/stl/vector.h>

namespace nb = nanobind;
using namespace isaacsim::physics::ovsim;

NB_MODULE(_bindings, m)
{
    m.doc() = "Provide native OV SIM control and data adapters for Isaac Sim physics.";

    nb::module_ dataModule = m.def_submodule("data", "Read and write attributes through physics entity views.");
    nb::module_ controlModule = m.def_submodule("control", "Control OV SIM stage and simulation operations.");
    nb::module_ authoringModule =
        controlModule.def_submodule("authoring", "Expose the unavailable stage-authoring surface for compatibility.");
    nb::module_ simulationModule =
        controlModule.def_submodule("simulation", "Configure and manually step the active physics simulation.");

    // control.authoring
    // - Stage operations
    authoringModule.def("create_stage", &control::authoring::createStage, R"doc(
Report that creating a stage is unavailable in this adapter.

Raises:
    RuntimeError: Always; stage authoring is not implemented.

)doc");
    authoringModule.def("open_stage", &control::authoring::openStage, nb::arg("usd_path"), R"doc(
Report that opening a stage is unavailable in this adapter.

Args:
    usd_path: Path or URL of the USD stage to open.

Raises:
    RuntimeError: Always; stage authoring is not implemented.

)doc");
    authoringModule.def("save_stage", &control::authoring::saveStage, nb::arg("usd_path"), R"doc(
Report that saving a stage is unavailable in this adapter.

Args:
    usd_path: Destination path or URL for the stage.

Raises:
    RuntimeError: Always; stage authoring is not implemented.

)doc");
    authoringModule.def("import_stage_from_string", &control::authoring::importStageFromString, nb::arg("usd_string"), R"doc(
Report that importing stage contents is unavailable in this adapter.

Args:
    usd_string: Serialized USD stage contents.

Raises:
    RuntimeError: Always; stage authoring is not implemented.

)doc");
    authoringModule.def("export_stage_to_string", &control::authoring::exportStageToString, R"doc(
Report that exporting stage contents is unavailable in this adapter.

Raises:
    RuntimeError: Always; stage authoring is not implemented.

)doc");
    authoringModule.def("close_stage", &control::authoring::closeStage, R"doc(
Report that closing a stage is unavailable in this adapter.

Raises:
    RuntimeError: Always; stage authoring is not implemented.

)doc");
    authoringModule.def("add_reference_to_stage", &control::authoring::addReferenceToStage, nb::arg("usd_path"),
                        nb::arg("path"), nb::arg("type_name") = "Xform",
                        R"doc(
Report that adding a USD reference is unavailable in this adapter.

Args:
    usd_path: Path or URL of the referenced USD asset.
    path: Stage path of the prim that would receive the reference.
    type_name: USD type used when the destination prim must be defined.

Raises:
    RuntimeError: Always; stage authoring is not implemented.

)doc");
    // - Prim operations
    authoringModule.def(
        "define_prim", &control::authoring::definePrim, nb::arg("path"), nb::arg("type_name") = "Xform", R"doc(
Report that defining a prim is unavailable in this adapter.

Args:
    path: Stage path of the prim to define.
    type_name: USD type of the prim.

Raises:
    RuntimeError: Always; stage authoring is not implemented.

)doc");
    authoringModule.def(
        "move_prim", &control::authoring::movePrim, nb::arg("target_path"), nb::arg("destination_path"), R"doc(
Report that moving a prim is unavailable in this adapter.

Args:
    target_path: Current stage path of the prim.
    destination_path: New stage path for the prim.

Raises:
    RuntimeError: Always; stage authoring is not implemented.

)doc");
    authoringModule.def("remove_prim", &control::authoring::removePrim, nb::arg("path"), R"doc(
Report that removing a prim is unavailable in this adapter.

Args:
    path: Stage path of the prim to remove.

Raises:
    RuntimeError: Always; stage authoring is not implemented.

)doc");
    // - Attribute operations
    authoringModule.def("create_prim_attribute", &control::authoring::createPrimAttribute, nb::arg("path"),
                        nb::arg("attribute_name"), nb::arg("type_name"),
                        R"doc(
Report that creating a prim attribute is unavailable in this adapter.

Args:
    path: Stage path of the prim.
    attribute_name: Name of the attribute to create.
    type_name: USD type name of the attribute.

Raises:
    RuntimeError: Always; stage authoring is not implemented.

)doc");
    authoringModule.def("remove_prim_attribute", &control::authoring::removePrimAttribute, nb::arg("path"),
                        nb::arg("attribute_name"),
                        R"doc(
Report that removing a prim attribute is unavailable in this adapter.

Args:
    path: Stage path of the prim.
    attribute_name: Name of the attribute to remove.

Raises:
    RuntimeError: Always; stage authoring is not implemented.

)doc");
    // - Parameters
    authoringModule.def("set_parameter", &control::authoring::setParameter, nb::arg("provider"),
                        nb::arg("parameter_name"), nb::arg("value"),
                        R"doc(
Report that setting an authoring parameter is unavailable in this adapter.

Args:
    provider: Name of the parameter provider.
    parameter_name: Name of the parameter to set.
    value: Value to assign to the parameter.

Raises:
    RuntimeError: Always; stage authoring is not implemented.

)doc");
    authoringModule.def("get_parameter", &control::authoring::getParameter, nb::arg("provider"),
                        nb::arg("parameter_name"),
                        R"doc(
Report that reading an authoring parameter is unavailable in this adapter.

Args:
    provider: Name of the parameter provider.
    parameter_name: Name of the parameter to retrieve.

Raises:
    RuntimeError: Always; stage authoring is not implemented.

)doc");

    // control.simulation
    // - Lifecycle operations
    simulationModule.def("play", &control::simulation::play, R"doc(
Preserve play-state API compatibility without changing manual simulation state.

)doc");
    simulationModule.def("pause", &control::simulation::pause, R"doc(
Preserve pause-state API compatibility without changing manual simulation state.

)doc");
    simulationModule.def("stop", &control::simulation::stop, R"doc(
Preserve stop-state API compatibility without changing manual simulation state.

)doc");
    simulationModule.def("initialize", &control::simulation::initialize, R"doc(
Initialize the active physics simulation for manual stepping.

Configure ``ovstage-stage-ptr`` before calling this function.

Raises:
    RuntimeError: If no OVStage pointer is configured or physics initialization fails.

)doc");
    simulationModule.def("invalidate", &control::simulation::invalidate, R"doc(
Invalidate the manually stepped simulation state.

Raises:
    RuntimeError: If physics invalidation fails.

)doc");
    simulationModule.def("step", &control::simulation::step, R"doc(
Advance the manually controlled simulation by one step.

Raises:
    RuntimeError: If the simulation is not initialized.

)doc");
    // - Parameters
    simulationModule.def("set_parameter", &control::simulation::setParameter, nb::arg("provider"),
                         nb::arg("parameter_name"), nb::arg("value"),
                         R"doc(
Set a parameter for the physics simulation provider.

Args:
    provider: Provider name. Only ``"physics"`` is supported.
    parameter_name: ``"ovstage-stage-ptr"`` or ``"physics-engine"``.
    value: Native OVStage pointer encoded as an integer, or physics-engine name.

Raises:
    ValueError: If the provider, parameter name, or value type is unsupported.

)doc");
    simulationModule.def("get_parameter", &control::simulation::getParameter, nb::arg("provider"),
                         nb::arg("parameter_name"),
                         R"doc(
Get a parameter from the physics simulation provider.

Args:
    provider: Provider name. Only ``"physics"`` is supported.
    parameter_name: Parameter to retrieve. Only ``"ovstage-stage-ptr"`` is supported.

Returns:
    int: Native OVStage pointer encoded as an integer.

Raises:
    ValueError: If the provider or parameter name is unsupported.

)doc");

    // data
    dataModule.def("read", &data::read, nb::arg("paths"), nb::arg("attribute_name"), nb::arg("timestamp") = nb::none(),
                   R"doc(
Read an attribute from one or more physics prims.

Args:
    paths: Prim path or sequence of prim paths to read.
    attribute_name: Name of the attribute to read.
    timestamp: Optional USD time code. Time-sampled reads are not yet applied.

Returns:
    object: Attribute values for the requested prim paths.

Raises:
    RuntimeError: If no active physics engine supports the requested attribute.

)doc");
    dataModule.def("write", &data::write, nb::arg("paths"), nb::arg("attribute_name"), nb::arg("values"),
                   nb::arg("timestamp") = nb::none(),
                   R"doc(
Write an attribute on one or more physics prims.

Args:
    paths: Prim path or sequence of prim paths to update.
    attribute_name: Name of the attribute to write.
    values: Attribute values to assign.
    timestamp: Optional USD time code. Time-sampled writes are not yet applied.

Raises:
    RuntimeError: If no active physics engine supports the requested attribute.

)doc");
}
