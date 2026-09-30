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
#include "isaacsim/foundation/ovsim/control/authoring/Authoring.hpp"
#include "isaacsim/foundation/ovsim/control/simulation/Simulation.hpp"
#include "isaacsim/foundation/ovsim/data/Data.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/variant.h>
#include <nanobind/stl/vector.h>

namespace nb = nanobind;
using namespace isaacsim::foundation::ovsim;

NB_MODULE(_bindings, m)
{
    m.doc() = "Foundation implementation of the backend-neutral OV SIM interface.";

    nb::module_ m_data = m.def_submodule("data");
    nb::module_ m_control = m.def_submodule("control");
    nb::module_ m_authoring = m_control.def_submodule("authoring");
    nb::module_ m_simulation = m_control.def_submodule("simulation");
    m_data.doc() = "Read and write attributes on the active Foundation stage.";
    m_control.doc() = "Control Foundation authoring and simulation state.";
    m_authoring.doc() = "Author prims and attributes on the active OpenUSD stage.";
    m_simulation.doc() = "Manage the Foundation OVStage simulation snapshot.";

    // control.authoring
    // - Stage operations
    m_authoring.def("create_stage", &control::authoring::createStage,
                    R"doc(Create an empty OpenUSD stage and make it active.

Returns:
    Whether the created stage is valid.
)doc");
    m_authoring.def("open_stage", &control::authoring::openStage, nb::arg("usd_path"),
                    R"doc(Open a USD stage and make it active.

Args:
    usd_path: Path or URL of the USD layer to open.

Returns:
    Whether the opened stage is valid.
)doc");
    m_authoring.def("save_stage", &control::authoring::saveStage, nb::arg("usd_path"),
                    R"doc(Save the active authoring stage.

Args:
    usd_path: Destination path for the root USD layer.

Returns:
    Whether the stage was saved successfully.
)doc");
    m_authoring.def("import_stage_from_string", &control::authoring::importStageFromString, nb::arg("usd_string"),
                    R"doc(Import serialized USDA content and make the stage active.

Args:
    usd_string: Serialized USDA content.

Returns:
    Whether the imported stage is valid.
)doc");
    m_authoring.def("export_stage_to_string", &control::authoring::exportStageToString,
                    R"doc(Export the active authoring stage as USDA text.

Returns:
    Serialized USDA content.
)doc");
    m_authoring.def("close_stage", &control::authoring::closeStage,
                    R"doc(Close retained authoring and simulation stages.

Returns:
    Whether the active stage was closed successfully.
)doc");
    m_authoring.def("add_reference_to_stage", &control::authoring::addReferenceToStage, nb::arg("usd_path"),
                    nb::arg("path"), nb::arg("type_name") = "Xform",
                    R"doc(Add a reference to the active authoring stage.

Args:
    usd_path: Path or URL of the referenced USD layer.
    path: Absolute prim path that receives the reference.
    type_name: Prim type to define when the target path does not exist.

Returns:
    Whether the reference was added successfully.
)doc");
    // - Prim operations
    m_authoring.def("define_prim", &control::authoring::definePrim, nb::arg("path"), nb::arg("type_name") = "Xform",
                    R"doc(Define a prim on the active authoring stage.

Args:
    path: Absolute path of the prim to define.
    type_name: USD or Isaac Sim prim type to create.

Returns:
    Whether the prim was defined successfully.
)doc");
    m_authoring.def("move_prim", &control::authoring::movePrim, nb::arg("target_path"), nb::arg("destination_path"),
                    R"doc(Move a prim on the active authoring stage.

Args:
    target_path: Absolute path of the prim to move.
    destination_path: Absolute destination path.

Returns:
    Whether the prim was moved successfully.
)doc");
    m_authoring.def("remove_prim", &control::authoring::removePrim, nb::arg("path"),
                    R"doc(Remove a prim from the active authoring stage.

Args:
    path: Absolute path of the prim to remove.

Returns:
    Whether the prim was removed successfully.
)doc");
    // - Attribute operations
    m_authoring.def("create_prim_attribute", &control::authoring::createPrimAttribute, nb::arg("path"),
                    nb::arg("attribute_name"), nb::arg("type_name"),
                    R"doc(Create an attribute on a prim in the active authoring stage.

Args:
    path: Absolute path of the target prim.
    attribute_name: Name of the attribute to create.
    type_name: USD type name for the attribute.

Returns:
    Whether the attribute was created successfully.
)doc");
    m_authoring.def("remove_prim_attribute", &control::authoring::removePrimAttribute, nb::arg("path"),
                    nb::arg("attribute_name"),
                    R"doc(Remove an attribute from a prim in the active authoring stage.

Args:
    path: Absolute path of the target prim.
    attribute_name: Name of the attribute to remove.

Returns:
    Whether the attribute was removed successfully.
)doc");
    // - Parameters
    m_authoring.def("set_parameter", &control::authoring::setParameter, nb::arg("provider"), nb::arg("parameter_name"),
                    nb::arg("value"),
                    R"doc(Accept an authoring provider parameter without changing stage state.

Args:
    provider: Provider identifier reserved for future authoring configuration.
    parameter_name: Provider-specific parameter name.
    value: Parameter value.

Note:
    The current implementation accepts and ignores all parameters.
)doc");
    m_authoring.def("get_parameter", &control::authoring::getParameter, nb::arg("provider"), nb::arg("parameter_name"),
                    R"doc(Query a retained stage identifier or pointer.

Args:
    provider: Provider identifier. The supported value is ``"stage"``.
    parameter_name: One of ``"openusd-stage-id"``, ``"openusd-stage-ptr"``, ``"ovstage-stage-id"``, or
        ``"ovstage-stage-ptr"``.

Returns:
    Requested identifier or pointer. A missing stage uses ``-1`` for its identifier and zero for its pointer.

Raises:
    ValueError: If the provider or parameter name is unsupported.
)doc");

    // control.simulation
    // - Lifecycle operations
    m_simulation.def("play", &control::simulation::play, "Accept a play request without changing simulation state.");
    m_simulation.def("pause", &control::simulation::pause, "Accept a pause request without changing simulation state.");
    m_simulation.def("stop", &control::simulation::stop, "Accept a stop request without changing simulation state.");
    m_simulation.def("initialize", &control::simulation::initialize,
                     "Copy the active OpenUSD stage into a retained OVStage simulation snapshot.");
    m_simulation.def("invalidate", &control::simulation::invalidate,
                     "Close and release the retained OVStage simulation snapshot, if one exists.");
    m_simulation.def("step", &control::simulation::step,
                     "Accept a manual simulation step request without changing simulation state.");
    // - Parameters
    m_simulation.def("set_parameter", &control::simulation::setParameter, nb::arg("provider"),
                     nb::arg("parameter_name"), nb::arg("value"),
                     R"doc(Reject a simulation provider parameter because configuration is not implemented.

Args:
    provider: Provider identifier.
    parameter_name: Provider-specific parameter name.
    value: Parameter value.

Raises:
    RuntimeError: Always.
)doc");
    m_simulation.def("get_parameter", &control::simulation::getParameter, nb::arg("provider"), nb::arg("parameter_name"),
                     R"doc(Reject a simulation provider query because configuration is not implemented.

Args:
    provider: Provider identifier.
    parameter_name: Provider-specific parameter name.

Raises:
    RuntimeError: Always.
)doc");

    // data
    m_data.def("read", &data::read, nb::arg("paths"), nb::arg("attribute_name"), nb::arg("timestamp") = nb::none(),
               R"doc(Read one attribute from one or more prims on the active stage.

Args:
    paths: Absolute prim path or paths to read.
    attribute_name: Name of the attribute or registered facade property to read.
    timestamp: Optional time code in stage time units. The current implementation reads default-time values.

Returns:
    Values read for the selected prim paths.
)doc");
    m_data.def("write", &data::write, nb::arg("paths"), nb::arg("attribute_name"), nb::arg("values"),
               nb::arg("timestamp") = nb::none(),
               R"doc(Write one attribute on one or more prims on the active stage.

Args:
    paths: Absolute prim path or paths to modify.
    attribute_name: Name of the attribute or registered facade property to write.
    values: Values to write to the selected prim paths.
    timestamp: Optional time code in stage time units. The current implementation writes default-time values.
)doc");
}
