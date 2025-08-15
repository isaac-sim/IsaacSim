// SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include <carb/BindingsPythonUtils.h>
#include <isaacsim/xr/input_devices/ManusTracker.h>

CARB_BINDINGS("isaacsim.xr.input_devices.python")

namespace
{
PYBIND11_MODULE(_isaac_xr_input_devices, m)
{
    using namespace carb;
    using namespace isaacsim::xr::input_devices;
    
    auto carbModule = py::module::import("carb");

    py::class_<IsaacSimManusTracker>(m, "IsaacSimManusTracker")
        .def(py::init<>())
        .def("initialize", &IsaacSimManusTracker::initialize, R"(
            Initialize Manus SDK (adapted from existing implementation).
            
            Returns:
                bool: True if initialization was successful, False otherwise
        )")
        .def("get_glove_data", &IsaacSimManusTracker::get_glove_data, R"(
            Get glove data in IsaacSim format.
            
            Returns:
                Dict[str, List[float]]: Dictionary mapping glove data keys to values
        )")
        .def("cleanup", &IsaacSimManusTracker::cleanup, R"(
            Cleanup SDK resources.
        )");
}
} 