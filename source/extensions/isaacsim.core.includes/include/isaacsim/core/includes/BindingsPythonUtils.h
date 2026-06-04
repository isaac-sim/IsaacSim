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

//! @file
//! @brief Python binding utilities for Isaac Sim extensions
//!
//! This header provides utilities for creating pybind11 bindings for Carbonite interfaces.
//! It is a minimal replacement for carb/BindingsPythonUtils.h that generates correct
//! Python type stubs (using std::optional for nullable string parameters).

#pragma once

// Include carb's base binding utilities (provides CARB_BINDINGS, wrapInterfaceFunction, etc.)
#include <carb/BindingsUtils.h>

// pybind11 includes
#include <carb/detail/PybindEpilog.h>
#include <carb/detail/PybindProlog.h>

#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <optional>
#include <string>

namespace py = pybind11;

namespace isaacsim
{
namespace core
{
namespace includes
{

/**
 * @brief Define a pybind11 class for a Carbonite interface with acquire/release functions.
 *
 * This function creates Python bindings for a Carbonite interface, including:
 * - A Python class wrapping the interface
 * - An acquire function to get an instance of the interface
 * - An optional release function to release the interface
 *
 * Unlike carb::defineInterfaceClass, this version uses std::optional<std::string> for
 * the acquire function parameters, which generates correct Python type stubs
 * (str | None = None instead of str = None).
 *
 * @tparam InterfaceType The Carbonite interface type to wrap
 * @tparam PyClassArgs Additional template arguments for py::class_
 * @param m The pybind11 module to add the class to
 * @param className The name of the Python class
 * @param acquireFuncName The name of the acquire function in Python
 * @param releaseFuncName The name of the release function (optional, can be nullptr)
 * @param classDocstring Documentation string for the class (optional)
 * @return The pybind11 class object for further method definitions
 *
 * @code{.cpp}
 * // Example usage:
 * isaacsim::core::includes::defineInterfaceClass<IMyInterface>(
 *     m, "IMyInterface", "acquire_my_interface", "release_my_interface")
 *     .def("some_method", &IMyInterface::someMethod);
 * @endcode
 */
template <typename InterfaceType, typename... PyClassArgs>
py::class_<InterfaceType, PyClassArgs...> defineInterfaceClass(py::module& m,
                                                               const char* className,
                                                               const char* acquireFuncName,
                                                               const char* releaseFuncName = nullptr,
                                                               const char* classDocstring = nullptr)
{
    auto cls = classDocstring ? py::class_<InterfaceType, PyClassArgs...>(m, className, classDocstring) :
                                py::class_<InterfaceType, PyClassArgs...>(m, className);

    // Use std::optional<std::string> instead of const char* with nullptr default.
    // This generates correct Python type stubs: str | None = None
    // instead of the incorrect: str = None
    m.def(
        acquireFuncName,
        [](std::optional<std::string> pluginName, std::optional<std::string> libraryPath)
        {
            const char* pn = pluginName ? pluginName->c_str() : nullptr;
            const char* lp = libraryPath ? libraryPath->c_str() : nullptr;
            return lp ? carb::acquireInterfaceFromLibraryForBindings<InterfaceType>(lp) :
                        carb::acquireInterfaceForBindings<InterfaceType>(pn);
        },
        py::arg("plugin_name") = py::none(), py::arg("library_path") = py::none(), py::return_value_policy::reference);

    if (releaseFuncName)
    {
        m.def(releaseFuncName, [](InterfaceType* iface) { carb::getFramework()->releaseInterface(iface); });
    }

    return cls;
}

} // namespace includes
} // namespace core
} // namespace isaacsim

// Convenience alias in carb namespace for easier migration
// This allows existing code using carb::defineInterfaceClass to work with minimal changes
namespace carb
{
using isaacsim::core::includes::defineInterfaceClass;
} // namespace carb
