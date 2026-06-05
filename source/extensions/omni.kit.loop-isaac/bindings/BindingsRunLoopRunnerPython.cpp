// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <pybind11/chrono.h>
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <RunLoopRunner.h>

CARB_BINDINGS("omni.kit.loop-isaac.python")

namespace omni
{
namespace kit
{
}
}


namespace
{

namespace py = pybind11;

PYBIND11_MODULE(_loop, m)
{
    using namespace carb;
    using namespace omni::kit;
    // We use carb data types, must import bindings for them
    auto carbModule = py::module::import("carb");

    m.doc() = "Isaac loop bindings";

    defineInterfaceClass<IRunLoopRunnerImpl>(m, "RunLoopRunner", "acquire_loop_interface", "release_loop_interface")


        .def("set_manual_mode", wrapInterfaceFunction(&IRunLoopRunnerImpl::setManualMode),
             R"pbdoc(
                Enables or disables manual stepping mode for the run loop.

                Args:
                    enabled (:obj:`bool`): Set to true to enable manual mode, false to disable.

                    name (:obj:`str`): The name of the run loop. If name is an empty string, all active run loops are set.

                )pbdoc",
             py::arg("enabled") = true, py::arg("name") = "")
        .def("set_manual_step_size", wrapInterfaceFunction(&IRunLoopRunnerImpl::setManualStepSize),
             R"pbdoc(
                Sets the time step size (dt) for manual stepping mode.

                Args:
                    dt (:obj:`double`): The time step size in seconds.

                    name (:obj:`str`): The name of the run loop. If name is an empty string, all active run loops are set.

                )pbdoc",
             py::arg("dt") = 0.01667, py::arg("name") = "")
        .def("get_manual_mode", wrapInterfaceFunction(&IRunLoopRunnerImpl::getManualMode),
             R"pbdoc(
                Gets the manual mode state for the run loop.

                Args:
                    name (:obj:`str`): The name of the run loop. If name is an empty string, returns the state of the first active run loop.

                Returns:
                    :obj:`bool`: True if manual mode is enabled, false otherwise.

                )pbdoc",
             py::arg("name") = "")
        .def("get_manual_step_size", wrapInterfaceFunction(&IRunLoopRunnerImpl::getManualStepSize),
             R"pbdoc(
                Gets the time step size (dt) for manual stepping mode.

                Args:
                    name (:obj:`str`): The name of the run loop. If name is an empty string, returns the step size of the first active run loop.

                Returns:
                    :obj:`double`: The time step size in seconds.

                )pbdoc",
             py::arg("name") = "");
}
}
