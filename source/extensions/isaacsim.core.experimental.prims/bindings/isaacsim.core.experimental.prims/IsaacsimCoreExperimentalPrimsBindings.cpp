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

#include <isaacsim/core/experimental/prims/IPrimDataReader.h>
#include <isaacsim/core/experimental/prims/IPrimDataReaderManager.h>
#include <pybind11/functional.h>
#include <pybind11/stl.h>

#include <stdexcept>
#include <string>

CARB_BINDINGS("isaacsim.core.experimental.prims.python")

namespace
{

namespace py = pybind11;

PYBIND11_MODULE(_prims_reader, m)
{
    using namespace isaacsim::core::experimental::prims;

    m.doc() = "C++ read-only prim data reader interface for Isaac Sim";

    // Base view type
    py::class_<IXformDataView>(m, "IXformDataView")
        .def("get_world_positions",
             [](IXformDataView& self)
             {
                 int count = 0;
                 const float* ptr = self.getWorldPositions(&count);
                 return std::make_tuple(reinterpret_cast<uintptr_t>(ptr), count);
             })
        .def("get_world_orientations",
             [](IXformDataView& self)
             {
                 int count = 0;
                 const float* ptr = self.getWorldOrientations(&count);
                 return std::make_tuple(reinterpret_cast<uintptr_t>(ptr), count);
             })
        .def("get_local_translations",
             [](IXformDataView& self)
             {
                 int count = 0;
                 const float* ptr = self.getLocalTranslations(&count);
                 return std::make_tuple(reinterpret_cast<uintptr_t>(ptr), count);
             })
        .def("get_local_orientations",
             [](IXformDataView& self)
             {
                 int count = 0;
                 const float* ptr = self.getLocalOrientations(&count);
                 return std::make_tuple(reinterpret_cast<uintptr_t>(ptr), count);
             })
        .def("get_local_scales",
             [](IXformDataView& self)
             {
                 int count = 0;
                 const float* ptr = self.getLocalScales(&count);
                 return std::make_tuple(reinterpret_cast<uintptr_t>(ptr), count);
             })
        .def("get_world_positions_host",
             [](IXformDataView& self)
             {
                 int count = 0;
                 const float* ptr = self.getWorldPositionsHost(&count);
                 return std::make_tuple(reinterpret_cast<uintptr_t>(ptr), count);
             })
        .def("get_world_orientations_host",
             [](IXformDataView& self)
             {
                 int count = 0;
                 const float* ptr = self.getWorldOrientationsHost(&count);
                 return std::make_tuple(reinterpret_cast<uintptr_t>(ptr), count);
             })
        .def("get_local_translations_host",
             [](IXformDataView& self)
             {
                 int count = 0;
                 const float* ptr = self.getLocalTranslationsHost(&count);
                 return std::make_tuple(reinterpret_cast<uintptr_t>(ptr), count);
             })
        .def("get_local_orientations_host",
             [](IXformDataView& self)
             {
                 int count = 0;
                 const float* ptr = self.getLocalOrientationsHost(&count);
                 return std::make_tuple(reinterpret_cast<uintptr_t>(ptr), count);
             })
        .def("get_local_scales_host",
             [](IXformDataView& self)
             {
                 int count = 0;
                 const float* ptr = self.getLocalScalesHost(&count);
                 return std::make_tuple(reinterpret_cast<uintptr_t>(ptr), count);
             })
        .def("update", &IXformDataView::update)
        .def(
            "allocate_buffer",
            [](IXformDataView& self, const std::string& fieldName, size_t count, py::object dtype)
            {
                std::string key;
                if (py::isinstance<py::str>(dtype))
                {
                    key = dtype.cast<std::string>();
                }
                else if (dtype.is_none())
                {
                    key = "float";
                }
                else if (py::hasattr(dtype, "value"))
                {
                    key = dtype.attr("value").cast<std::string>();
                }
                else
                {
                    throw std::invalid_argument("allocate_buffer dtype must be a string or BufferDtype enum");
                }
                if (key.empty())
                {
                    key = "float";
                }
                if (key == "float" || key == "float32")
                {
                    return self.allocateBufferFloat(fieldName.c_str(), count);
                }
                if (key == "uint8" || key == "uint8_t")
                {
                    return self.allocateBufferUint8(fieldName.c_str(), count);
                }
                throw std::invalid_argument(
                    "allocate_buffer dtype must be 'float'/'float32' or 'uint8'/'uint8_t' (got '" + key + "')");
            },
            py::arg("field_name"), py::arg("count"), py::arg("dtype") = "float")
        .def("get_buffer_ptr", &IXformDataView::getBufferPtr, py::arg("field_name"))
        .def("get_buffer_size", &IXformDataView::getBufferSize, py::arg("field_name"))
        .def("get_buffer_device", &IXformDataView::getBufferDevice)
        .def("register_field_callback", &IXformDataView::registerFieldCallback, py::arg("field_name"),
             py::arg("callback"))
        .def(
            "get_prim_frame_name",
            [](IXformDataView& self, const std::string& primPath) -> py::object
            {
                char buf[512] = {};
                if (self.getPrimFrameName(primPath.c_str(), buf, sizeof(buf)))
                {
                    return py::str(buf);
                }
                return py::none();
            },
            py::arg("prim_path"),
            "Resolve the frame name for a prim (checks isaac:nameOverride, falls back to prim name).\n"
            "Returns the name string, or None if the prim is not found or stage is unavailable.")
        .def(
            "get_prim_world_transform",
            [](IXformDataView& self, const std::string& primPath) -> py::object
            {
                float pos[3] = {}, ori[4] = {};
                if (self.getPrimWorldTransform(primPath.c_str(), pos, ori))
                {
                    return py::make_tuple(
                        py::make_tuple(pos[0], pos[1], pos[2]), py::make_tuple(ori[0], ori[1], ori[2], ori[3]));
                }
                return py::none();
            },
            py::arg("prim_path"),
            "World transform of an arbitrary prim via Fabric.\n"
            "Returns ((x, y, z), (qw, qx, qy, qz)), or None if unavailable.");

    // RigidBody view (inherits IXformDataView bindings)
    py::class_<IRigidBodyDataView, IXformDataView>(m, "IRigidBodyDataView")
        .def("get_linear_velocities",
             [](IRigidBodyDataView& self)
             {
                 int count = 0;
                 const float* ptr = self.getLinearVelocities(&count);
                 return std::make_tuple(reinterpret_cast<uintptr_t>(ptr), count);
             })
        .def("get_angular_velocities",
             [](IRigidBodyDataView& self)
             {
                 int count = 0;
                 const float* ptr = self.getAngularVelocities(&count);
                 return std::make_tuple(reinterpret_cast<uintptr_t>(ptr), count);
             })
        .def("get_linear_velocities_host",
             [](IRigidBodyDataView& self)
             {
                 int count = 0;
                 const float* ptr = self.getLinearVelocitiesHost(&count);
                 return std::make_tuple(reinterpret_cast<uintptr_t>(ptr), count);
             })
        .def("get_angular_velocities_host",
             [](IRigidBodyDataView& self)
             {
                 int count = 0;
                 const float* ptr = self.getAngularVelocitiesHost(&count);
                 return std::make_tuple(reinterpret_cast<uintptr_t>(ptr), count);
             });

    // Articulation view (inherits IXformDataView bindings)
    py::class_<IArticulationDataView, IXformDataView>(m, "IArticulationDataView")
        .def("get_dof_positions",
             [](IArticulationDataView& self)
             {
                 int count = 0;
                 const float* ptr = self.getDofPositions(&count);
                 return std::make_tuple(reinterpret_cast<uintptr_t>(ptr), count);
             })
        .def("get_dof_velocities",
             [](IArticulationDataView& self)
             {
                 int count = 0;
                 const float* ptr = self.getDofVelocities(&count);
                 return std::make_tuple(reinterpret_cast<uintptr_t>(ptr), count);
             })
        .def("get_dof_efforts",
             [](IArticulationDataView& self)
             {
                 int count = 0;
                 const float* ptr = self.getDofEfforts(&count);
                 return std::make_tuple(reinterpret_cast<uintptr_t>(ptr), count);
             })
        .def("get_root_transforms",
             [](IArticulationDataView& self)
             {
                 int count = 0;
                 const float* ptr = self.getRootTransforms(&count);
                 return std::make_tuple(reinterpret_cast<uintptr_t>(ptr), count);
             })
        .def("get_root_velocities",
             [](IArticulationDataView& self)
             {
                 int count = 0;
                 const float* ptr = self.getRootVelocities(&count);
                 return std::make_tuple(reinterpret_cast<uintptr_t>(ptr), count);
             })
        .def("get_dof_positions_host",
             [](IArticulationDataView& self)
             {
                 int count = 0;
                 const float* ptr = self.getDofPositionsHost(&count);
                 return std::make_tuple(reinterpret_cast<uintptr_t>(ptr), count);
             })
        .def("get_dof_velocities_host",
             [](IArticulationDataView& self)
             {
                 int count = 0;
                 const float* ptr = self.getDofVelocitiesHost(&count);
                 return std::make_tuple(reinterpret_cast<uintptr_t>(ptr), count);
             })
        .def("get_dof_efforts_host",
             [](IArticulationDataView& self)
             {
                 int count = 0;
                 const float* ptr = self.getDofEffortsHost(&count);
                 return std::make_tuple(reinterpret_cast<uintptr_t>(ptr), count);
             })
        .def("get_root_transforms_host",
             [](IArticulationDataView& self)
             {
                 int count = 0;
                 const float* ptr = self.getRootTransformsHost(&count);
                 return std::make_tuple(reinterpret_cast<uintptr_t>(ptr), count);
             })
        .def("get_root_velocities_host",
             [](IArticulationDataView& self)
             {
                 int count = 0;
                 const float* ptr = self.getRootVelocitiesHost(&count);
                 return std::make_tuple(reinterpret_cast<uintptr_t>(ptr), count);
             })
        .def("get_dof_index", &IArticulationDataView::getDofIndex, py::arg("dof_prim_path"))
        .def("get_dof_names",
             [](IArticulationDataView& self)
             {
                 int count = 0;
                 const char* const* names = self.getDofNames(&count);
                 py::list result;
                 if (names && count > 0)
                 {
                     for (int i = 0; i < count; ++i)
                     {
                         result.append(names[i] ? std::string(names[i]) : std::string());
                     }
                 }
                 return result;
             })
        .def("get_dof_types",
             [](IArticulationDataView& self)
             {
                 int count = 0;
                 const uint8_t* ptr = self.getDofTypes(&count);
                 return std::make_tuple(reinterpret_cast<uintptr_t>(ptr), count);
             })
        .def("get_dof_types_host",
             [](IArticulationDataView& self)
             {
                 int count = 0;
                 const uint8_t* ptr = self.getDofTypesHost(&count);
                 return std::make_tuple(reinterpret_cast<uintptr_t>(ptr), count);
             })
        .def(
            "get_articulation_links",
            [](IArticulationDataView& self, const std::string& rootPath) -> py::list
            {
                const LinkInfo* links = nullptr;
                size_t count = 0;
                py::list result;
                if (self.getArticulationLinks(rootPath.c_str(), &links, &count) && links && count > 0)
                {
                    for (size_t i = 0; i < count; ++i)
                    {
                        py::dict entry;
                        entry["path"] = links[i].path ? std::string(links[i].path) : std::string();
                        entry["parent_path"] = links[i].parentPath ? std::string(links[i].parentPath) : std::string();
                        result.append(entry);
                    }
                }
                return result;
            },
            py::arg("root_path"),
            "Enumerate UsdPhysicsRigidBodyAPI descendants of an articulation root.\n"
            "Returns a list of dicts with 'path' and 'parent_path' keys.\n"
            "Returns an empty list if root_path is not an articulation or stage is unavailable.");

    // Factory (Carbonite interface)
    carb::defineInterfaceClass<IPrimDataReader>(
        m, "IPrimDataReader", "acquire_prim_data_reader_interface", "release_prim_data_reader_interface")
        .def("initialize", &IPrimDataReader::initialize, py::arg("stage_id"), py::arg("device_ordinal"))
        .def("shutdown", &IPrimDataReader::shutdown)
        .def(
            "create_xform_view",
            [](IPrimDataReader& self, const std::string& viewId, const std::vector<std::string>& paths,
               const std::string& engineType)
            {
                std::vector<const char*> cStringPaths;
                cStringPaths.reserve(paths.size());
                for (auto& p : paths)
                {
                    cStringPaths.push_back(p.c_str());
                }
                return self.createXformView(viewId.c_str(), cStringPaths.data(), paths.size(), engineType.c_str());
            },
            py::arg("view_id"), py::arg("paths"), py::arg("engine_type"), py::return_value_policy::reference)
        .def(
            "create_rigid_body_view",
            [](IPrimDataReader& self, const std::string& viewId, const std::vector<std::string>& paths,
               const std::string& engineType)
            {
                std::vector<const char*> cStringPaths;
                cStringPaths.reserve(paths.size());
                for (auto& p : paths)
                {
                    cStringPaths.push_back(p.c_str());
                }
                return self.createRigidBodyView(viewId.c_str(), cStringPaths.data(), paths.size(), engineType.c_str());
            },
            py::arg("view_id"), py::arg("paths"), py::arg("engine_type"), py::return_value_policy::reference)
        .def(
            "create_articulation_view",
            [](IPrimDataReader& self, const std::string& viewId, const std::vector<std::string>& paths,
               const std::string& engineType)
            {
                std::vector<const char*> cStringPaths;
                cStringPaths.reserve(paths.size());
                for (auto& p : paths)
                {
                    cStringPaths.push_back(p.c_str());
                }
                return self.createArticulationView(viewId.c_str(), cStringPaths.data(), paths.size(), engineType.c_str());
            },
            py::arg("view_id"), py::arg("paths"), py::arg("engine_type"), py::return_value_policy::reference)
        .def("remove_view", &IPrimDataReader::removeView, py::arg("view_id"))
        .def(
            "set_articulation_dof_metadata",
            [](IPrimDataReader& self, const std::string& viewId, const std::vector<std::string>& names,
               const std::vector<int>& types)
            {
                std::vector<const char*> namePtrs;
                namePtrs.reserve(names.size());
                for (const auto& n : names)
                {
                    namePtrs.push_back(n.c_str());
                }
                std::vector<uint8_t> typeBytes(types.size());
                for (size_t i = 0; i < types.size(); ++i)
                {
                    typeBytes[i] = static_cast<uint8_t>(types[i]);
                }
                self.setArticulationDofMetadata(
                    viewId.c_str(), namePtrs.data(), namePtrs.size(), typeBytes.data(), typeBytes.size());
            },
            py::arg("view_id"), py::arg("names"), py::arg("types"))
        .def("get_generation", &IPrimDataReader::getGeneration)
        .def("get_stage_id", &IPrimDataReader::getStageId)
        .def("get_device_ordinal", &IPrimDataReader::getDeviceOrdinal);

    carb::defineInterfaceClass<IPrimDataReaderManager>(m, "IPrimDataReaderManager",
                                                       "acquire_prim_data_reader_manager_interface",
                                                       "release_prim_data_reader_manager_interface")
        .def("ensure_initialized", &IPrimDataReaderManager::ensureInitialized, py::arg("stage_id"),
             py::arg("device_ordinal"))
        .def("get_reader", &IPrimDataReaderManager::getReader, py::return_value_policy::reference)
        .def("get_generation", &IPrimDataReaderManager::getGeneration);
}

} // anonymous namespace
