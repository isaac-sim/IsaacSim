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

#include "isaacsim/ovgl_viewport/debug/Viewport.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/stl/array.h>
#include <nanobind/stl/function.h>
#include <nanobind/stl/string.h>

#include <cstdint>
#include <new>
#include <utility>

namespace nb = nanobind;
using namespace isaacsim::ovgl_viewport::debug;

NB_MODULE(_bindings, module)
{
    module.doc() = "Concrete OVGL viewport for Isaac Sim tests and debugging.";

    nb::class_<Camera>(module, "Camera", "Initial first-person debug camera state.")
        .def(nb::init<>(), "Create a camera state.")
        .def_rw("target", &Camera::target, "Camera look-at target.")
        .def_rw("yaw_radians", &Camera::yawRadians, "Horizontal view angle in radians.")
        .def_rw("pitch_radians", &Camera::pitchRadians, "Vertical view angle in radians.")
        .def_rw("distance", &Camera::distance, "Distance from the eye to the target.");
    nb::class_<CameraPose>(module, "CameraPose", "World pose produced by debug camera navigation.")
        .def_ro("position", &CameraPose::position, "Camera position in stage coordinates.")
        .def_ro("orientation", &CameraPose::orientation, "XYZW-ordered camera orientation quaternion.");
    nb::class_<ViewportConfiguration>(module, "ViewportConfig", "OVGL viewport configuration.")
        .def(nb::init<>(), "Create viewport configuration.")
        .def_rw("render_product_path", &ViewportConfiguration::renderProductPath, "Authored RenderProduct path.")
        .def_rw("title", &ViewportConfiguration::title, "Window title.")
        .def_rw("width", &ViewportConfiguration::width, "Initial width in pixels.")
        .def_rw("height", &ViewportConfiguration::height, "Initial height in pixels.")
        .def_rw("maximum_frames", &ViewportConfiguration::maximumFrames, "Frame limit, or zero for no limit.")
        .def_rw("visible", &ViewportConfiguration::visible, "Whether to create and present a user-visible SDL window.")
        .def_rw("camera", &ViewportConfiguration::camera, "Initial and reset camera state.");
    nb::class_<Frame>(module, "Frame", "One top-down OVGL RGBA8 frame.")
        .def_ro("frame_number", &Frame::frameNumber, "Monotonic viewport frame number.")
        .def_ro("stage_ordinal", &Frame::stageOrdinal, "Rendered OVStage ordinal.")
        .def_ro("width", &Frame::width, "Image width in pixels.")
        .def_ro("height", &Frame::height, "Image height in pixels.")
        .def_prop_ro(
            "rgba",
            [](const Frame& frame)
            { return nb::bytes(reinterpret_cast<const char*>(frame.rgba.data()), frame.rgba.size()); },
            "Tightly packed, top-down RGBA8 pixels.");
    nb::class_<Viewport>(module, "Viewport", "Main-thread OVGL viewport for an already-populated OVStage.")
        .def(
            "__init__",
            [](Viewport* self, uintptr_t stage, CameraPoseWriter cameraPoseWriter, ViewportConfiguration configuration)
            {
                new (self) Viewport(reinterpret_cast<ovstage_instance_t*>(stage), std::move(cameraPoseWriter),
                                    std::move(configuration));
            },
            nb::arg("stage"), nb::arg("camera_pose_writer"), nb::arg("config"),
            "Create a viewport that borrows the supplied OVStage pointer.")
        .def("close", &Viewport::close, "Release rendering and window resources.")
        .def_prop_ro("closed", &Viewport::isClosed, "Whether rendering and window resources have been released.")
        .def("poll_events", &Viewport::pollEvents,
             "Process pending events, publish camera changes, and report whether the viewport remains open.")
        .def("render", &Viewport::render, nb::rv_policy::reference_internal, nb::call_guard<nb::gil_scoped_release>(),
             "Render and optionally present the latest sealed OVStage ordinal. The returned Frame is reused by the "
             "next render; copy rgba before rendering again.");
}
