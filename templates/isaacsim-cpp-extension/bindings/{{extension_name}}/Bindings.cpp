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

#include <{{python_module_path}}/IExample.h>

#include <carb/BindingsPythonUtils.h>

CARB_BINDINGS("{{extension_name}}.python")

PYBIND11_MODULE(_{{binding_module}}, m)
{
    using namespace {{ extension_name.replace(".", "::") }};

    m.doc() = "Python bindings for {{extension_name}}";

    carb::defineInterfaceClass<IExample>(
        m, "IExample", "acquire_example_interface", "release_example_interface")
        .def("greet", &IExample::greet, "Returns a greeting string.");
}
