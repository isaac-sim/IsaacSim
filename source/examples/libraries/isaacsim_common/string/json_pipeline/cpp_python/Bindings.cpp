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

#include <isaacsim/common/string/String.hpp>
#include <nlohmann/json.hpp>

#include <exception>
#include <string>

namespace
{

PyObject* buildDocument(PyObject*, PyObject*)
{
    try
    {
        const nlohmann::json document = {
            { "robot", { { "name", isaacsim::common::string::toUpper("carter") }, { "joints", 7 } } },
        };
        const std::string serialized = document.dump();
        return PyUnicode_FromStringAndSize(serialized.data(), static_cast<Py_ssize_t>(serialized.size()));
    }
    catch (const std::exception& error)
    {
        PyErr_SetString(PyExc_RuntimeError, error.what());
        return nullptr;
    }
    catch (...)
    {
        PyErr_SetString(PyExc_RuntimeError, "The JSON pipeline reported an unknown error.");
        return nullptr;
    }
}

PyMethodDef g_methods[] = {
    { "build_document", buildDocument, METH_NOARGS, "Build and serialize the example JSON document." },
    { nullptr, nullptr, 0, nullptr },
};

PyModuleDef g_module = {
    PyModuleDef_HEAD_INIT, "_json_pipeline_cpp", "C++ JSON producer for the JSON pipeline example.", -1, g_methods,
};

} // namespace

PyMODINIT_FUNC PyInit__json_pipeline_cpp()
{
    return PyModule_Create(&g_module);
}
