// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

// Python.h must precede headers that can include the C or C++ standard library.
// clang-format off
#include <Python.h>
// clang-format on

#include <isaacsim/common/logging/Logging.hpp>

#include <exception>

namespace
{

PyObject* sayHello(PyObject*, PyObject*)
{
    try
    {
        const isaacsim::common::logging::Logger logger("isaacsim.examples.hello_world.cpp_python");
        logger.report("Hello World from C++ with Python.");
        Py_RETURN_NONE;
    }
    catch (const std::exception& error)
    {
        PyErr_SetString(PyExc_RuntimeError, error.what());
        return nullptr;
    }
    catch (...)
    {
        PyErr_SetString(PyExc_RuntimeError, "The Isaac Sim C++ API reported an unknown error.");
        return nullptr;
    }
}

PyMethodDef g_methods[] = {
    { "say_hello", sayHello, METH_NOARGS, "Print the Hello World result through the Isaac Sim C++ logging API." },
    { nullptr, nullptr, 0, nullptr },
};

PyModuleDef g_module = {
    PyModuleDef_HEAD_INIT, "_hello_world_cpp", "C++ with Python Isaac Sim Hello World example.", -1, g_methods,
};

} // namespace

PyMODINIT_FUNC PyInit__hello_world_cpp()
{
    return PyModule_Create(&g_module);
}
