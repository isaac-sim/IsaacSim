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

// Thin nanobind control surface for the ovphysx backend. The data hot path stays
// in C++ (the tensor EntityView subclasses registered by activate()); this module
// only exposes lifecycle + a few USD-free queries.

#include "isaacsim/physics_engines/ovphysx/Backend.hpp"

#include <nanobind/nanobind.h>

namespace nb = nanobind;

NB_MODULE(_bindings, m)
{
    namespace ov = isaacsim::physics_engines::ovphysx;

    m.doc() = "Control the OvPhysX simulation backend.";

    m.def("activate", &ov::activate, "Create and register the OvPhysX simulation backend.");
    m.def("shutdown", &ov::shutdown, "Unregister and destroy the OvPhysX simulation backend.");
    m.def(
        "set_suppress_readback", [](bool enable) { ov::setSuppressReadback(enable); }, nb::arg("enable"),
        "Enable or disable PhysX Direct GPU operation without host readback.");
    m.def(
        "set_tensor_device_ordinal", [](int ordinal) { ov::setTensorDeviceOrdinal(ordinal); }, nb::arg("ordinal"),
        "Set the device holding backend tensors, using -1 for host memory.");
    m.def("get_tensor_device_ordinal", &ov::getTensorDeviceOrdinal,
          "Return the device holding backend tensors, or -1 for host memory.");
}
