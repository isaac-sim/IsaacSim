// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <carb/BindingsPythonUtils.h>

#include <isaacsim/ucx/core/UcxListener.h>
#include <isaacsim/ucx/core/UcxListenerRegistry.h>

CARB_BINDINGS("isaacsim.ucx.core.python")

namespace
{

namespace py = pybind11;

PYBIND11_MODULE(_ucx_core, m)
{
    using namespace isaacsim::ucx::core;

    m.doc() = R"pbdoc(
        Python bindings for the UCX Core listener.

        Example::

            import isaacsim.ucx.core as ucx_core

            listener = ucx_core.add_listener(port=13337)
            connected = listener.wait_for_connection(timeout_ms=5000)
            listener.shutdown()
            ucx_core.remove_listener(13337)
    )pbdoc";

    py::class_<UCXListener, std::shared_ptr<UCXListener>>(m, "UCXListener")
        .def("wait_for_connection", &UCXListener::waitForConnection, py::arg("timeout_ms") = -1,
             py::call_guard<py::gil_scoped_release>(),
             R"pbdoc(
                 Block until a client connects or the timeout expires.

                 The GIL is released during the wait so other Python threads and the
                 asyncio event loop remain responsive.

                 Args:
                     timeout_ms: Timeout in milliseconds. -1 waits indefinitely.

                 Returns:
                     True if a connection was established, False if timeout expired.
             )pbdoc")
        .def("is_connected", &UCXListener::isConnected, R"pbdoc(Check if a client is currently connected.)pbdoc")
        .def("get_port", &UCXListener::getPort, R"pbdoc(Return the port the listener is bound to.)pbdoc")
        .def("shutdown", &UCXListener::shutdown, R"pbdoc(Shutdown the listener and close all connections.)pbdoc");

    m.def("add_listener", &UCXListenerRegistry::addListener, py::arg("port") = 0,
          R"pbdoc(
              Create or retrieve a UCX listener.

              When port is 0 (default), a new listener is created on an automatically
              assigned ephemeral port. When port is non-zero, returns the existing listener
              for that port if one exists, otherwise creates a new one.

              Args:
                  port: Port number to listen on. 0 = auto-assign.

              Returns:
                  UCXListener instance.
          )pbdoc");

    m.def("remove_listener", &UCXListenerRegistry::removeListener, py::arg("port"),
          R"pbdoc(Remove and shutdown the listener for the specified port.)pbdoc");

    m.def("is_listener_registered", &UCXListenerRegistry::isListenerRegistered, py::arg("port"),
          R"pbdoc(Return True if a listener is registered for the specified port.)pbdoc");
}

} // namespace
