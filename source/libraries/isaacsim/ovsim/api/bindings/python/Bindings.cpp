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
#include "isaacsim/ovsim/api/Factory.hpp"
#include "isaacsim/ovsim/api/Types.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/unordered_map.h>
#include <nanobind/stl/variant.h>
#include <nanobind/stl/vector.h>

namespace nb = nanobind;
using namespace isaacsim::ovsim::api;

NB_MODULE(_bindings, m)
{
    m.doc() = "Create OV SIM clients and access their control and data interfaces.";

    nb::class_<types::Authoring>(m, "Authoring",
                                 R"doc(Stage, prim, attribute, and authoring-parameter operations.

Calls execute synchronously and retain the Python GIL.

Example:
    .. code-block:: python

        from isaacsim.ovsim.api import make_client

        authoring = make_client("local").control.authoring
)doc")
        // - Stage operations
        .def(
            "create_stage", [](const types::Authoring& self) { return self.createStage(); },
            R"doc(Create an empty stage and make it active.

Returns:
    Whether the stage was created.

Raises:
    RuntimeError: If the selected client does not support empty-stage creation.

Example:
    .. code-block:: python

        created = client.control.authoring.create_stage()
)doc")
        .def(
            "open_stage",
            [](const types::Authoring& self, const std::string& usdPath) { return self.openStage(usdPath); },
            R"doc(Open a USD stage and make it active.

Args:
    usd_path: Path to the USD stage.

Returns:
    Whether the stage was opened.

Raises:
    RuntimeError: If the selected client does not support opening stage files.

Example:
    .. code-block:: python

        opened = client.control.authoring.open_stage("scene.usda")
)doc",
            nb::arg("usd_path"))
        .def(
            "save_stage",
            [](const types::Authoring& self, const std::string& usdPath) { return self.saveStage(usdPath); },
            R"doc(Save the active stage to a USD file.

Args:
    usd_path: Destination path for the USD stage.

Returns:
    Whether the stage was saved.

Raises:
    RuntimeError: If the selected client does not support saving stage files.

Example:
    .. code-block:: python

        saved = client.control.authoring.save_stage("scene.usda")
)doc",
            nb::arg("usd_path"))
        .def(
            "import_stage_from_string",
            [](const types::Authoring& self, const std::string& usdString)
            { return self.importStageFromString(usdString); },
            R"doc(Replace the active stage with serialized USD text.

Args:
    usd_string: Serialized USD stage contents.

Returns:
    Whether the contents were imported.

Example:
    .. code-block:: python

        imported = client.control.authoring.import_stage_from_string("#usda 1.0")
)doc",
            nb::arg("usd_string"))
        .def(
            "export_stage_to_string", [](const types::Authoring& self) { return self.exportStageToString(); },
            R"doc(Export the active stage as serialized USD text.

Returns:
    Serialized USD stage contents.

Raises:
    RuntimeError: If the selected client does not support stage export.

Example:
    .. code-block:: python

        usd_text = client.control.authoring.export_stage_to_string()
)doc")
        .def(
            "close_stage", [](const types::Authoring& self) { return self.closeStage(); },
            R"doc(Close the active stage.

Returns:
    Whether the stage was closed.

Example:
    .. code-block:: python

        closed = client.control.authoring.close_stage()
)doc")
        .def(
            "add_reference_to_stage",
            [](const types::Authoring& self, const std::string& usdPath, const std::string& path,
               const std::string& typeName) { return self.addReferenceToStage(usdPath, path, typeName); },
            R"doc(Add a USD reference to a prim on the active stage.

Args:
    usd_path: Path to the referenced USD asset.
    path: Path of the referencing prim.
    type_name: USD type name to use when defining the prim.

Returns:
    Whether the reference was added.

Raises:
    RuntimeError: If the selected client does not support remote stage authoring.

Example:
    .. code-block:: python

        added = client.control.authoring.add_reference_to_stage("robot.usda", "/World/Robot")
)doc",
            nb::arg("usd_path"), nb::arg("path"), nb::arg("type_name") = "Xform")
        // - Prim operations
        .def(
            "define_prim",
            [](const types::Authoring& self, const std::string& path, const std::string& typeName)
            { return self.definePrim(path, typeName); },
            R"doc(Define a prim on the active stage.

Args:
    path: Path of the prim to define.
    type_name: USD type name for the prim.

Returns:
    Whether the prim was defined.

Raises:
    RuntimeError: If the selected client does not support remote stage authoring.

Example:
    .. code-block:: python

        defined = client.control.authoring.define_prim("/World/Robot")
)doc",
            nb::arg("path"), nb::arg("type_name") = "Xform")
        .def(
            "move_prim",
            [](const types::Authoring& self, const std::string& targetPath, const std::string& destinationPath)
            { return self.movePrim(targetPath, destinationPath); },
            R"doc(Move a prim to another path on the active stage.

Args:
    target_path: Existing prim path.
    destination_path: Destination prim path.

Returns:
    Whether the prim was moved.

Raises:
    RuntimeError: If the selected client does not support remote stage authoring.

Example:
    .. code-block:: python

        moved = client.control.authoring.move_prim("/World/Old", "/World/New")
)doc",
            nb::arg("target_path"), nb::arg("destination_path"))
        .def(
            "remove_prim", [](const types::Authoring& self, const std::string& path) { return self.removePrim(path); },
            R"doc(Remove a prim from the active stage.

Args:
    path: Path of the prim to remove.

Returns:
    Whether the prim was removed.

Raises:
    RuntimeError: If the selected client does not support remote stage authoring.

Example:
    .. code-block:: python

        removed = client.control.authoring.remove_prim("/World/Robot")
)doc",
            nb::arg("path"))
        // - Attribute operations
        .def(
            "create_prim_attribute",
            [](const types::Authoring& self, const std::string& path, const std::string& attributeName,
               const std::string& typeName) { return self.createPrimAttribute(path, attributeName, typeName); },
            R"doc(Create an attribute on a prim.

Args:
    path: Path of the prim.
    attribute_name: Name of the attribute to create.
    type_name: USD value type name for the attribute.

Returns:
    Whether the attribute was created.

Raises:
    RuntimeError: If the selected client does not support remote stage authoring.

Example:
    .. code-block:: python

        created = client.control.authoring.create_prim_attribute(
            "/World/Robot", "enabled", "bool"
        )
)doc",
            nb::arg("path"), nb::arg("attribute_name"), nb::arg("type_name"))
        .def(
            "remove_prim_attribute",
            [](const types::Authoring& self, const std::string& path, const std::string& attributeName)
            { return self.removePrimAttribute(path, attributeName); },
            R"doc(Remove an attribute from a prim.

Args:
    path: Path of the prim.
    attribute_name: Name of the attribute to remove.

Returns:
    Whether the attribute was removed.

Raises:
    RuntimeError: If the selected client does not support remote stage authoring.

Example:
    .. code-block:: python

        removed = client.control.authoring.remove_prim_attribute("/World/Robot", "enabled")
)doc",
            nb::arg("path"), nb::arg("attribute_name"))
        // - Parameters
        .def(
            "set_parameter",
            [](const types::Authoring& self, const std::string& provider, const std::string& parameterName,
               const ::ovsim::interfaces::control::authoring::InputParameterType& value)
            { return self.setParameter(provider, parameterName, value); },
            R"doc(Set an authoring parameter exposed by a provider.

Args:
    provider: Name of the parameter provider.
    parameter_name: Name of the parameter.
    value: Value to assign.

Raises:
    RuntimeError: If the selected client does not support authoring parameters.

Example:
    .. code-block:: python

        client.control.authoring.set_parameter("usd", "option", True)
)doc",
            nb::arg("provider"), nb::arg("parameter_name"), nb::arg("value"))
        .def(
            "get_parameter",
            [](const types::Authoring& self, const std::string& provider, const std::string& parameterName)
            { return self.getParameter(provider, parameterName); },
            R"doc(Get an authoring parameter exposed by a provider.

Args:
    provider: Name of the parameter provider.
    parameter_name: Name of the parameter.

Returns:
    Current parameter value.

Raises:
    ValueError: If the in-process client does not recognize the provider or parameter name.
    RuntimeError: If the selected client does not support authoring parameters.

Example:
    .. code-block:: python

        value = client.control.authoring.get_parameter("usd", "option")
)doc",
            nb::arg("provider"), nb::arg("parameter_name"));

    nb::class_<types::Simulation>(m, "Simulation",
                                  R"doc(Simulation lifecycle, stepping, and parameter operations.

Calls execute synchronously. Simulation initialization releases the Python GIL while it runs.

Example:
    .. code-block:: python

        simulation = client.control.simulation
)doc")
        // - Lifecycle operations
        .def(
            "play", [](const types::Simulation& self) { return self.play(); },
            R"doc(Start or resume automatic simulation stepping.

Raises:
    RuntimeError: If the selected client does not support automatic stepping.

Example:
    .. code-block:: python

        client.control.simulation.play()
)doc")
        .def(
            "pause", [](const types::Simulation& self) { return self.pause(); },
            R"doc(Pause automatic simulation stepping.

Raises:
    RuntimeError: If the selected client does not support automatic stepping.

Example:
    .. code-block:: python

        client.control.simulation.pause()
)doc")
        .def(
            "stop", [](const types::Simulation& self) { return self.stop(); },
            R"doc(Stop automatic simulation stepping.

Raises:
    RuntimeError: If the selected client does not support automatic stepping.

Example:
    .. code-block:: python

        client.control.simulation.stop()
)doc")
        .def(
            "initialize", [](const types::Simulation& self) { return self.initialize(); },
            R"doc(Initialize the simulation for manual stepping.

Raises:
    RuntimeError: If simulation initialization fails.

Example:
    .. code-block:: python

        client.control.simulation.initialize()
)doc",
            nb::call_guard<nb::gil_scoped_release>())
        .def(
            "invalidate", [](const types::Simulation& self) { return self.invalidate(); },
            R"doc(Invalidate manually stepped simulation state.

Raises:
    RuntimeError: If invalidation fails or the selected client does not support it.

Example:
    .. code-block:: python

        client.control.simulation.invalidate()
)doc")
        .def(
            "step", [](const types::Simulation& self) { return self.step(); },
            R"doc(Advance a manually controlled simulation by one step.

Raises:
    RuntimeError: If simulation stepping fails.

Example:
    .. code-block:: python

        client.control.simulation.step()
)doc")
        // - Parameters
        .def(
            "set_parameter",
            [](const types::Simulation& self, const std::string& provider, const std::string& parameterName,
               const ::ovsim::interfaces::control::simulation::InputParameterType& value)
            { return self.setParameter(provider, parameterName, value); },
            R"doc(Set a simulation parameter exposed by a provider.

Args:
    provider: Name of the parameter provider.
    parameter_name: Name of the parameter.
    value: Value to assign.

Raises:
    ValueError: If the in-process client rejects the provider, parameter name, or value.
    RuntimeError: If the selected client does not support simulation parameters.

Example:
    .. code-block:: python

        client.control.simulation.set_parameter("physics", "option", True)
)doc",
            nb::arg("provider"), nb::arg("parameter_name"), nb::arg("value"))
        .def(
            "get_parameter",
            [](const types::Simulation& self, const std::string& provider, const std::string& parameterName)
            { return self.getParameter(provider, parameterName); },
            R"doc(Get a simulation parameter exposed by a provider.

Args:
    provider: Name of the parameter provider.
    parameter_name: Name of the parameter.

Returns:
    Current parameter value.

Raises:
    ValueError: If the in-process client does not recognize the provider or parameter name.
    RuntimeError: If the selected client does not support simulation parameters.

Example:
    .. code-block:: python

        value = client.control.simulation.get_parameter("physics", "option")
)doc",
            nb::arg("provider"), nb::arg("parameter_name"));

    nb::class_<types::Control>(m, "Control", "Authoring and simulation-control interfaces for an OV SIM client.")
        .def_ro("authoring", &types::Control::authoring,
                R"doc(Get the stage and prim authoring interface.

Returns:
    Authoring interface owned by this control object.

Example:
    .. code-block:: python

        authoring = client.control.authoring
)doc")
        .def_ro("simulation", &types::Control::simulation,
                R"doc(Get the simulation execution interface.

Returns:
    Simulation interface owned by this control object.

Example:
    .. code-block:: python

        simulation = client.control.simulation
)doc");

    nb::class_<types::Data>(m, "Data",
                            R"doc(Simulation data read and write operations.

Calls execute synchronously and retain the Python GIL.

Example:
    .. code-block:: python

        data = client.data
)doc")
        .def(
            "read",
            [](const types::Data& self, const ::ovsim::interfaces::data::PathType& paths,
               const std::string& attributeName, std::optional<double> timestamp)
            { return self.read(paths, attributeName, timestamp); },
            R"doc(Read an attribute from one or more prims.

Args:
    paths: Prim path or collection of prim paths.
    attribute_name: Name of the attribute to read.
    timestamp: Optional simulation timestamp to query.

Returns:
    Requested attribute value or values.

Raises:
    ValueError: If paths, the attribute name, or the timestamp is invalid or unsupported.
    RuntimeError: If the selected client cannot complete the read.

Example:
    .. code-block:: python

        positions = client.data.read(["/World/Robot"], "position")
)doc",
            nb::arg("paths"), nb::arg("attribute_name"), nb::arg("timestamp") = nb::none())
        .def(
            "write",
            [](const types::Data& self, const ::ovsim::interfaces::data::PathType& paths,
               const std::string& attributeName, const ::ovsim::interfaces::data::InputValueType& values,
               std::optional<double> timestamp) { return self.write(paths, attributeName, values, timestamp); },
            R"doc(Write an attribute on one or more prims.

Args:
    paths: Prim path or collection of prim paths.
    attribute_name: Name of the attribute to write.
    values: Attribute value or values to assign.
    timestamp: Optional simulation timestamp for the write.

Raises:
    ValueError: If paths, the attribute name, values, or the timestamp is invalid or unsupported.
    RuntimeError: If the selected client cannot complete the write.

Example:
    .. code-block:: python

        client.data.write(["/World/Robot"], "position", positions)
)doc",
            nb::arg("paths"), nb::arg("attribute_name"), nb::arg("values"), nb::arg("timestamp") = nb::none());

    nb::class_<types::Implementation>(
        m, "Implementation", "Complete control and data interface returned by :func:`make_client`.")
        .def_ro("control", &types::Implementation::control,
                R"doc(Get the authoring and simulation-control interfaces.

Returns:
    Control interface owned by this client implementation.

Example:
    .. code-block:: python

        control = client.control
)doc")
        .def_ro("data", &types::Implementation::data,
                R"doc(Get the simulation data interface.

Returns:
    Data interface owned by this client implementation.

Example:
    .. code-block:: python

        data = client.data
)doc");

    m.def("make_client", &makeClient,
          R"doc(Create an OV SIM client implementation.

The returned object owns the callable state used by its nested interfaces. Calls execute synchronously and retain the
Python GIL except during simulation initialization.

Args:
    name: Client implementation name. Use ``"in-process"`` or ``"local"`` for an in-process client, or ``"grpc"``
        for a remote session.
    configuration: Optional implementation-specific configuration values. A gRPC client requires an ``"endpoint"``
        entry containing a channel target such as ``"127.0.0.1:50051"``.

Returns:
    Client control and data interfaces.

Raises:
    RuntimeError: If the name does not identify a supported client implementation.
    ValueError: If the selected implementation rejects its configuration.

Example:
    .. code-block:: python

        from isaacsim.ovsim.api import make_client

        client = make_client("local")
        remote = make_client("grpc", {"endpoint": "127.0.0.1:50051"})
)doc",
          nb::arg("name"), nb::arg("configuration") = nb::none());
}
