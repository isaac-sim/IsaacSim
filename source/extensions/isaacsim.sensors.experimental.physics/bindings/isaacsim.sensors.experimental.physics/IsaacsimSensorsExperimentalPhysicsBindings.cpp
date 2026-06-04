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

#include <isaacsim/core/includes/BindingsPythonUtils.h>
#include <isaacsim/sensors/experimental/physics/IContactSensor.h>
#include <isaacsim/sensors/experimental/physics/IEffortSensor.h>
#include <isaacsim/sensors/experimental/physics/IImuSensor.h>
#include <isaacsim/sensors/experimental/physics/IJointStateSensor.h>
#include <isaacsim/sensors/experimental/physics/IRaycastSensor.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

#include <algorithm>

CARB_BINDINGS("isaacsim.sensors.experimental.physics.python")

namespace
{

namespace py = pybind11;

PYBIND11_MODULE(_physics_sensors, m)
{
    using namespace isaacsim::sensors::experimental::physics;

    m.doc() = "C++ physics sensor backend interfaces for Isaac Sim";

    // Lightweight orientation accessor returned by ImuSensorReading.orientation
    py::class_<ImuSensorReading> imuReadingCls(m, "ImuSensorReading");
    imuReadingCls.def(py::init<>())
        .def_readwrite("linear_acceleration_x", &ImuSensorReading::linearAccelerationX)
        .def_readwrite("linear_acceleration_y", &ImuSensorReading::linearAccelerationY)
        .def_readwrite("linear_acceleration_z", &ImuSensorReading::linearAccelerationZ)
        .def_readwrite("angular_velocity_x", &ImuSensorReading::angularVelocityX)
        .def_readwrite("angular_velocity_y", &ImuSensorReading::angularVelocityY)
        .def_readwrite("angular_velocity_z", &ImuSensorReading::angularVelocityZ)
        .def_readwrite("orientation_w", &ImuSensorReading::orientationW)
        .def_readwrite("orientation_x", &ImuSensorReading::orientationX)
        .def_readwrite("orientation_y", &ImuSensorReading::orientationY)
        .def_readwrite("orientation_z", &ImuSensorReading::orientationZ)
        .def_readwrite("time", &ImuSensorReading::time)
        .def_readwrite("is_valid", &ImuSensorReading::isValid);

    carb::defineInterfaceClass<IImuSensor>(m, "IImuSensor", "acquire_imu_sensor_interface", "release_imu_sensor_interface")
        .def("shutdown", &IImuSensor::shutdown)
        .def(
            "create_sensor",
            [](IImuSensor& self, const std::string& primPath) { return self.createSensor(primPath.c_str()); },
            py::arg("prim_path"))
        .def(
            "remove_sensor", [](IImuSensor& self, const std::string& primPath) { self.removeSensor(primPath.c_str()); },
            py::arg("prim_path"))
        .def(
            "get_sensor_reading",
            [](IImuSensor& self, const std::string& primPath, bool readGravity)
            { return self.getSensorReading(primPath.c_str(), readGravity); },
            py::arg("prim_path"), py::arg("read_gravity"));

    // --- Contact sensor ---
    py::class_<ContactSensorReading>(m, "ContactSensorReading")
        .def(py::init<>())
        .def_readwrite("time", &ContactSensorReading::time)
        .def_readwrite("value", &ContactSensorReading::value)
        .def_readwrite("in_contact", &ContactSensorReading::inContact)
        .def_readwrite("is_valid", &ContactSensorReading::isValid);

    carb::defineInterfaceClass<IContactSensor>(
        m, "IContactSensor", "acquire_contact_sensor_interface", "release_contact_sensor_interface")
        .def("shutdown", &IContactSensor::shutdown)
        .def(
            "create_sensor",
            [](IContactSensor& self, const std::string& primPath) { return self.createSensor(primPath.c_str()); },
            py::arg("prim_path"))
        .def(
            "remove_sensor",
            [](IContactSensor& self, const std::string& primPath) { self.removeSensor(primPath.c_str()); },
            py::arg("prim_path"))
        .def(
            "get_sensor_reading",
            [](IContactSensor& self, const std::string& primPath) { return self.getSensorReading(primPath.c_str()); },
            py::arg("prim_path"))
        .def(
            "get_raw_contacts",
            [](IContactSensor& self, const std::string& primPath) -> py::list
            {
                const ContactRawData* data = nullptr;
                int32_t count = 0;
                self.getRawContacts(primPath.c_str(), &data, &count);
                py::list result;
                for (int32_t i = 0; i < count; i++)
                {
                    py::dict entry;
                    entry["body0"] = py::int_(data[i].body0);
                    entry["body1"] = py::int_(data[i].body1);
                    entry["position"] = py::dict(py::arg("x") = data[i].positionX, py::arg("y") = data[i].positionY,
                                                 py::arg("z") = data[i].positionZ);
                    entry["normal"] = py::dict(
                        py::arg("x") = data[i].normalX, py::arg("y") = data[i].normalY, py::arg("z") = data[i].normalZ);
                    entry["impulse"] = py::dict(py::arg("x") = data[i].impulseX, py::arg("y") = data[i].impulseY,
                                                py::arg("z") = data[i].impulseZ);
                    entry["time"] = data[i].time;
                    entry["dt"] = data[i].dt;
                    result.append(entry);
                }
                return result;
            },
            py::arg("prim_path"));

    // --- Effort sensor ---
    py::class_<EffortSensorReading>(m, "EffortSensorReading")
        .def(py::init<>())
        .def_readwrite("value", &EffortSensorReading::value)
        .def_readwrite("time", &EffortSensorReading::time)
        .def_readwrite("is_valid", &EffortSensorReading::isValid);

    carb::defineInterfaceClass<IEffortSensor>(
        m, "IEffortSensor", "acquire_effort_sensor_interface", "release_effort_sensor_interface")
        .def("shutdown", &IEffortSensor::shutdown)
        .def(
            "create_sensor",
            [](IEffortSensor& self, const std::string& jointPrimPath)
            { return self.createSensor(jointPrimPath.c_str()); },
            py::arg("joint_prim_path"))
        .def(
            "remove_sensor",
            [](IEffortSensor& self, const std::string& jointPrimPath) { self.removeSensor(jointPrimPath.c_str()); },
            py::arg("prim_path"))
        .def(
            "get_sensor_reading",
            [](IEffortSensor& self, const std::string& jointPrimPath)
            { return self.getSensorReading(jointPrimPath.c_str()); },
            py::arg("prim_path"));

    // --- Joint state sensor ---
    // positions / velocities / efforts: one copy from C into numpy arrays (no list intermediate).
    // dof_names: list of strings (small). Data is copied so Python sees value semantics.
    py::class_<JointStateSensorReading>(m, "JointStateSensorReading")
        .def(py::init<>())
        .def_readwrite("time", &JointStateSensorReading::time)
        .def_readwrite("is_valid", &JointStateSensorReading::isValid)
        .def_readwrite("dof_count", &JointStateSensorReading::dofCount)
        .def_property_readonly("dof_names",
                               [](const JointStateSensorReading& r) -> py::list
                               {
                                   py::list result;
                                   if (r.dofNames)
                                   {
                                       for (int32_t i = 0; i < r.dofCount; i++)
                                       {
                                           result.append(py::str(r.dofNames[i]));
                                       }
                                   }
                                   return result;
                               })
        .def_property_readonly("positions",
                               [](const JointStateSensorReading& r) -> py::array_t<float>
                               {
                                   if (!r.positions || r.dofCount <= 0)
                                   {
                                       return py::array_t<float>({ 0 });
                                   }
                                   py::array_t<float> arr({ static_cast<py::ssize_t>(r.dofCount) });
                                   std::copy(r.positions, r.positions + r.dofCount, arr.mutable_data());
                                   return arr;
                               })
        .def_property_readonly("velocities",
                               [](const JointStateSensorReading& r) -> py::array_t<float>
                               {
                                   if (!r.velocities || r.dofCount <= 0)
                                   {
                                       return py::array_t<float>({ 0 });
                                   }
                                   py::array_t<float> arr({ static_cast<py::ssize_t>(r.dofCount) });
                                   std::copy(r.velocities, r.velocities + r.dofCount, arr.mutable_data());
                                   return arr;
                               })
        .def_property_readonly("efforts",
                               [](const JointStateSensorReading& r) -> py::array_t<float>
                               {
                                   if (!r.efforts || r.dofCount <= 0)
                                   {
                                       return py::array_t<float>({ 0 });
                                   }
                                   py::array_t<float> arr({ static_cast<py::ssize_t>(r.dofCount) });
                                   std::copy(r.efforts, r.efforts + r.dofCount, arr.mutable_data());
                                   return arr;
                               })
        .def_property_readonly("dof_types",
                               [](const JointStateSensorReading& r) -> py::array_t<uint8_t>
                               {
                                   if (!r.dofTypes || r.dofCount <= 0)
                                   {
                                       return py::array_t<uint8_t>({ 0 });
                                   }
                                   py::array_t<uint8_t> arr({ static_cast<py::ssize_t>(r.dofCount) });
                                   std::copy(r.dofTypes, r.dofTypes + r.dofCount, arr.mutable_data());
                                   return arr;
                               })
        .def_readwrite("stage_meters_per_unit", &JointStateSensorReading::stageMetersPerUnit);

    carb::defineInterfaceClass<IJointStateSensor>(
        m, "IJointStateSensor", "acquire_joint_state_sensor_interface", "release_joint_state_sensor_interface")
        .def("shutdown", &IJointStateSensor::shutdown)
        .def(
            "create_sensor",
            [](IJointStateSensor& self, const std::string& articulationRootPath)
            { return self.createSensor(articulationRootPath.c_str()); },
            py::arg("articulation_root_path"))
        .def(
            "remove_sensor",
            [](IJointStateSensor& self, const std::string& articulationRootPath)
            { self.removeSensor(articulationRootPath.c_str()); },
            py::arg("prim_path"))
        .def(
            "get_sensor_reading",
            [](IJointStateSensor& self, const std::string& articulationRootPath)
            { return self.getSensorReading(articulationRootPath.c_str()); },
            py::arg("prim_path"));

    // --- Raycast sensor ---
    py::class_<RaycastSensorReading>(m, "RaycastSensorReading")
        .def(py::init<>())
        .def_readonly("ray_count", &RaycastSensorReading::rayCount)
        .def_readonly("time", &RaycastSensorReading::time)
        .def_readonly("is_valid", &RaycastSensorReading::isValid)
        .def_property_readonly("depths",
                               [](const RaycastSensorReading& r) -> py::array_t<float>
                               {
                                   if (!r.depths || r.rayCount == 0)
                                   {
                                       return py::array_t<float>({ 0 });
                                   }
                                   py::array_t<float> arr({ static_cast<py::ssize_t>(r.rayCount) });
                                   std::copy(r.depths, r.depths + r.rayCount, arr.mutable_data());
                                   return arr;
                               })
        .def_property_readonly(
            "hit_positions",
            [](const RaycastSensorReading& r) -> py::array_t<float>
            {
                if (!r.hitPositions || r.rayCount == 0)
                {
                    return py::array_t<float>(std::vector<py::ssize_t>{ 0, 3 });
                }
                py::array_t<float> arr({ static_cast<py::ssize_t>(r.rayCount), static_cast<py::ssize_t>(3) });
                std::copy(r.hitPositions, r.hitPositions + r.rayCount * 3, arr.mutable_data());
                return arr;
            })
        .def_property_readonly(
            "hit_normals",
            [](const RaycastSensorReading& r) -> py::array_t<float>
            {
                if (!r.hitNormals || r.rayCount == 0)
                {
                    return py::array_t<float>(std::vector<py::ssize_t>{ 0, 3 });
                }
                py::array_t<float> arr({ static_cast<py::ssize_t>(r.rayCount), static_cast<py::ssize_t>(3) });
                std::copy(r.hitNormals, r.hitNormals + r.rayCount * 3, arr.mutable_data());
                return arr;
            })
        .def_property_readonly("hit_prim_paths",
                               [](const RaycastSensorReading& r) -> py::list
                               {
                                   py::list result;
                                   if (r.hitPrimPaths && r.rayCount > 0)
                                   {
                                       for (uint32_t i = 0; i < r.rayCount; i++)
                                       {
                                           result.append(py::str(r.hitPrimPaths[i] ? r.hitPrimPaths[i] : ""));
                                       }
                                   }
                                   return result;
                               })
        .def_property_readonly(
            "ray_origins_world",
            [](const RaycastSensorReading& r) -> py::array_t<float>
            {
                if (!r.rayOriginsWorld || r.rayCount == 0)
                {
                    return py::array_t<float>(std::vector<py::ssize_t>{ 0, 3 });
                }
                py::array_t<float> arr({ static_cast<py::ssize_t>(r.rayCount), static_cast<py::ssize_t>(3) });
                std::copy(r.rayOriginsWorld, r.rayOriginsWorld + r.rayCount * 3, arr.mutable_data());
                return arr;
            })
        .def_property_readonly(
            "ray_end_points_world",
            [](const RaycastSensorReading& r) -> py::array_t<float>
            {
                if (!r.rayEndPointsWorld || r.rayCount == 0)
                {
                    return py::array_t<float>(std::vector<py::ssize_t>{ 0, 3 });
                }
                py::array_t<float> arr({ static_cast<py::ssize_t>(r.rayCount), static_cast<py::ssize_t>(3) });
                std::copy(r.rayEndPointsWorld, r.rayEndPointsWorld + r.rayCount * 3, arr.mutable_data());
                return arr;
            });

    carb::defineInterfaceClass<IRaycastSensor>(
        m, "IRaycastSensor", "acquire_raycast_sensor_interface", "release_raycast_sensor_interface")
        .def("shutdown", &IRaycastSensor::shutdown)
        .def(
            "create_sensor",
            [](IRaycastSensor& self, const std::string& primPath) { return self.createSensor(primPath.c_str()); },
            py::arg("prim_path"))
        .def(
            "remove_sensor",
            [](IRaycastSensor& self, const std::string& primPath) { self.removeSensor(primPath.c_str()); },
            py::arg("prim_path"))
        .def(
            "get_sensor_reading",
            [](IRaycastSensor& self, const std::string& primPath) { return self.getSensorReading(primPath.c_str()); },
            py::arg("prim_path"));
}

} // anonymous namespace
