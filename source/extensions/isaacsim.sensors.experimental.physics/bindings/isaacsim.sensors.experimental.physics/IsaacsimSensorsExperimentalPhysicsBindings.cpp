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

#include <isaacsim/core/includes/BindingsPythonUtils.hpp>
#include <isaacsim/sensors/experimental/physics/IContactSensor.hpp>
#include <isaacsim/sensors/experimental/physics/IEffortSensor.hpp>
#include <isaacsim/sensors/experimental/physics/IImuSensor.hpp>
#include <isaacsim/sensors/experimental/physics/IJointStateSensor.hpp>
#include <isaacsim/sensors/experimental/physics/IRaycastSensor.hpp>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

#include <algorithm>
#include <new>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

CARB_BINDINGS("isaacsim.sensors.experimental.physics.python")

namespace
{

namespace py = pybind11;

struct PythonRaycastSensorReading
{
    uint32_t rayCount{ 0 };
    py::array_t<float> depths;
    py::array_t<float> hitPositions;
    py::array_t<float> hitNormals;
    py::list hitPrimPaths;
    py::array_t<float> rayOriginsWorld;
    py::array_t<float> rayEndPointsWorld;
    float time{ 0.0f };
    bool isValid{ false };
};

struct PythonJointStateSensorReading
{
    float time{ 0.0f };
    bool isValid{ false };
    int32_t dofCount{ 0 };
    py::list dofNames;
    py::array_t<float> positions;
    py::array_t<float> velocities;
    py::array_t<float> efforts;
    py::array_t<uint8_t> dofTypes;
    float stageMetersPerUnit{ 0.0f };
};

template <typename T>
py::array_t<T> copyToArray(const T* values, const std::vector<py::ssize_t>& shape)
{
    py::array_t<T> array(shape);
    if (values && array.size() > 0)
    {
        std::copy(values, values + array.size(), array.mutable_data());
    }
    else if (array.size() > 0)
    {
        std::fill(array.mutable_data(), array.mutable_data() + array.size(), T{});
    }
    return array;
}

PythonRaycastSensorReading copyRaycastSensorReading(
    const isaacsim::sensors::experimental::physics::RaycastSensorReading& reading)
{
    PythonRaycastSensorReading copy;
    copy.rayCount = reading.rayCount;
    copy.time = reading.time;
    copy.isValid = reading.isValid;

    if (reading.depths && reading.rayCount > 0)
    {
        copy.depths = copyToArray(reading.depths, { static_cast<py::ssize_t>(reading.rayCount) });
    }
    else
    {
        copy.depths = py::array_t<float>({ 0 });
    }
    if (reading.hitPositions && reading.rayCount > 0)
    {
        copy.hitPositions = copyToArray(reading.hitPositions, { static_cast<py::ssize_t>(reading.rayCount), 3 });
    }
    else
    {
        copy.hitPositions = py::array_t<float>(std::vector<py::ssize_t>{ 0, 3 });
    }
    if (reading.hitNormals && reading.rayCount > 0)
    {
        copy.hitNormals = copyToArray(reading.hitNormals, { static_cast<py::ssize_t>(reading.rayCount), 3 });
    }
    else
    {
        copy.hitNormals = py::array_t<float>(std::vector<py::ssize_t>{ 0, 3 });
    }
    if (reading.hitPrimPaths)
    {
        for (uint32_t i = 0; i < reading.rayCount; ++i)
        {
            // Load once: a concurrent physics step may republish the slot between reads.
            const char* const path = reading.hitPrimPaths[i];
            copy.hitPrimPaths.append(py::str(path ? path : ""));
        }
    }
    if (reading.rayOriginsWorld && reading.rayCount > 0)
    {
        copy.rayOriginsWorld = copyToArray(reading.rayOriginsWorld, { static_cast<py::ssize_t>(reading.rayCount), 3 });
    }
    else
    {
        copy.rayOriginsWorld = py::array_t<float>(std::vector<py::ssize_t>{ 0, 3 });
    }
    if (reading.rayEndPointsWorld && reading.rayCount > 0)
    {
        copy.rayEndPointsWorld =
            copyToArray(reading.rayEndPointsWorld, { static_cast<py::ssize_t>(reading.rayCount), 3 });
    }
    else
    {
        copy.rayEndPointsWorld = py::array_t<float>(std::vector<py::ssize_t>{ 0, 3 });
    }
    return copy;
}

PythonJointStateSensorReading copyJointStateSensorReading(
    const isaacsim::sensors::experimental::physics::JointStateSensorReading& reading)
{
    PythonJointStateSensorReading copy;
    copy.time = reading.time;
    copy.isValid = reading.isValid;
    copy.dofCount = reading.dofCount;
    copy.stageMetersPerUnit = reading.stageMetersPerUnit;
    if (reading.dofNames)
    {
        for (int32_t i = 0; i < reading.dofCount; ++i)
        {
            const char* const name = reading.dofNames[i];
            copy.dofNames.append(py::str(name ? name : ""));
        }
    }
    const std::vector<py::ssize_t> shape = reading.dofCount > 0 ?
                                               std::vector<py::ssize_t>{ static_cast<py::ssize_t>(reading.dofCount) } :
                                               std::vector<py::ssize_t>{ 0 };
    copy.positions = copyToArray(reading.positions, shape);
    copy.velocities = copyToArray(reading.velocities, shape);
    copy.efforts = copyToArray(reading.efforts, shape);
    copy.dofTypes = copyToArray(reading.dofTypes, shape);
    return copy;
}

class PythonRaycastSensorReadingView
{
public:
    PythonRaycastSensorReadingView() = default;

    PythonRaycastSensorReadingView(isaacsim::sensors::experimental::physics::IRaycastSensor* sensorInterface,
                                   isaacsim::sensors::experimental::physics::RaycastSensorReadingView* view)
        : m_sensorInterface(sensorInterface), m_view(view)
    {
    }

    PythonRaycastSensorReadingView(const PythonRaycastSensorReadingView&) = delete;
    PythonRaycastSensorReadingView& operator=(const PythonRaycastSensorReadingView&) = delete;

    PythonRaycastSensorReadingView(PythonRaycastSensorReadingView&& other) noexcept
        : m_sensorInterface(std::exchange(other.m_sensorInterface, nullptr)),
          m_view(std::exchange(other.m_view, nullptr))
    {
    }

    PythonRaycastSensorReadingView& operator=(PythonRaycastSensorReadingView&&) = delete;

    ~PythonRaycastSensorReadingView()
    {
        if (m_view)
        {
            m_view->release();
        }
        if (m_sensorInterface)
        {
            carb::getFramework()->releaseInterface(m_sensorInterface);
        }
    }

    isaacsim::sensors::experimental::physics::RaycastSensorReadingView* getView() const
    {
        return m_view;
    }

private:
    isaacsim::sensors::experimental::physics::IRaycastSensor* m_sensorInterface{ nullptr };
    isaacsim::sensors::experimental::physics::RaycastSensorReadingView* m_view{ nullptr };
};

class PythonJointStateSensorReadingView
{
public:
    PythonJointStateSensorReadingView() = default;

    PythonJointStateSensorReadingView(isaacsim::sensors::experimental::physics::IJointStateSensor* sensorInterface,
                                      isaacsim::sensors::experimental::physics::JointStateSensorReadingView* view)
        : m_sensorInterface(sensorInterface), m_view(view)
    {
    }

    PythonJointStateSensorReadingView(const PythonJointStateSensorReadingView&) = delete;
    PythonJointStateSensorReadingView& operator=(const PythonJointStateSensorReadingView&) = delete;

    PythonJointStateSensorReadingView(PythonJointStateSensorReadingView&& other) noexcept
        : m_sensorInterface(std::exchange(other.m_sensorInterface, nullptr)),
          m_view(std::exchange(other.m_view, nullptr))
    {
    }

    PythonJointStateSensorReadingView& operator=(PythonJointStateSensorReadingView&&) = delete;

    ~PythonJointStateSensorReadingView()
    {
        if (m_view)
        {
            m_view->release();
        }
        if (m_sensorInterface)
        {
            carb::getFramework()->releaseInterface(m_sensorInterface);
        }
    }

    isaacsim::sensors::experimental::physics::JointStateSensorReadingView* getView() const
    {
        return m_view;
    }

private:
    isaacsim::sensors::experimental::physics::IJointStateSensor* m_sensorInterface{ nullptr };
    isaacsim::sensors::experimental::physics::JointStateSensorReadingView* m_view{ nullptr };
};

PythonRaycastSensorReadingView getRaycastSensorReadingView(
    isaacsim::sensors::experimental::physics::IRaycastSensor& sensorInterface, const std::string& primPath)
{
    carb::Framework* framework = carb::getFramework();
    auto* retainedInterface =
        framework->acquireInterface<isaacsim::sensors::experimental::physics::IRaycastSensor>(&sensorInterface);
    if (!retainedInterface)
    {
        throw std::runtime_error("Failed to retain the raycast sensor interface for a reading view");
    }
    auto* view = retainedInterface->getSensorReadingView(primPath.c_str());
    if (!view)
    {
        framework->releaseInterface(retainedInterface);
        throw std::bad_alloc();
    }
    return PythonRaycastSensorReadingView(retainedInterface, view);
}

PythonJointStateSensorReadingView getJointStateSensorReadingView(
    isaacsim::sensors::experimental::physics::IJointStateSensor& sensorInterface, const std::string& articulationRootPath)
{
    carb::Framework* framework = carb::getFramework();
    auto* retainedInterface =
        framework->acquireInterface<isaacsim::sensors::experimental::physics::IJointStateSensor>(&sensorInterface);
    if (!retainedInterface)
    {
        throw std::runtime_error("Failed to retain the joint-state sensor interface for a reading view");
    }
    auto* view = retainedInterface->getSensorReadingView(articulationRootPath.c_str());
    if (!view)
    {
        framework->releaseInterface(retainedInterface);
        throw std::bad_alloc();
    }
    return PythonJointStateSensorReadingView(retainedInterface, view);
}

template <typename T, typename Owner>
py::array_t<T> makeReadOnlyView(const T* values,
                                const std::vector<py::ssize_t>& shape,
                                const std::vector<py::ssize_t>& emptyShape,
                                Owner& owner)
{
    py::array_t<T> array;
    if (values)
    {
        py::object base = py::cast(&owner, py::return_value_policy::reference);
        array = py::array_t<T>(shape, values, base);
    }
    else
    {
        array = py::array_t<T>(emptyShape);
    }
    array.attr("setflags")(false);
    return array;
}

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
    py::class_<PythonJointStateSensorReading>(
        m, "JointStateSensorReading",
        "Mutable owned snapshot of a joint-state reading. Its values and validity do not change after capture.")
        .def(py::init([]() { return copyJointStateSensorReading(JointStateSensorReading{}); }))
        .def_readonly("time", &PythonJointStateSensorReading::time, "Capture time in seconds.")
        .def_readonly("is_valid", &PythonJointStateSensorReading::isValid, "Whether the reading was valid when captured.")
        .def_readonly(
            "dof_count", &PythonJointStateSensorReading::dofCount, "Number of degrees of freedom in the snapshot.")
        .def_readonly("dof_names", &PythonJointStateSensorReading::dofNames,
                      "Mutable list of degree-of-freedom names owned by the snapshot.")
        .def_readonly(
            "positions", &PythonJointStateSensorReading::positions, "Mutable position array owned by the snapshot.")
        .def_readonly(
            "velocities", &PythonJointStateSensorReading::velocities, "Mutable velocity array owned by the snapshot.")
        .def_readonly("efforts", &PythonJointStateSensorReading::efforts, "Mutable effort array owned by the snapshot.")
        .def_readonly("dof_types", &PythonJointStateSensorReading::dofTypes,
                      "Mutable degree-of-freedom type array owned by the snapshot.")
        .def_readonly("stage_meters_per_unit", &PythonJointStateSensorReading::stageMetersPerUnit,
                      "Stage scale in meters per unit at capture time.");

    py::class_<PythonJointStateSensorReadingView>(
        m, "JointStateSensorReadingView",
        "Read-only zero-copy view of live joint-state sensor buffers. The arrays retain the backing frame, and "
        "is_valid becomes false after sensor removal, reset, or timeline stop while the last values remain readable.")
        .def(py::init<>())
        .def_property_readonly(
            "time",
            [](const PythonJointStateSensorReadingView& reading)
            {
                const JointStateSensorReadingView* view = reading.getView();
                return view ? view->getTime() : 0.0f;
            },
            "Current sensor time in seconds.")
        .def_property_readonly(
            "is_valid",
            [](const PythonJointStateSensorReadingView& reading)
            {
                const JointStateSensorReadingView* view = reading.getView();
                return view && view->isValid();
            },
            "Whether the owner-backed sensor frame is still active and contains a valid reading.")
        .def_property_readonly(
            "dof_count",
            [](const PythonJointStateSensorReadingView& reading)
            {
                const JointStateSensorReadingView* view = reading.getView();
                return view ? view->getDofCount() : 0;
            },
            "Number of degrees of freedom in the view.")
        .def_property_readonly(
            "dof_names",
            [](const PythonJointStateSensorReadingView& reading)
            {
                const JointStateSensorReadingView* view = reading.getView();
                const char* const* names = view ? view->getDofNames() : nullptr;
                if (!view || !names)
                {
                    return py::tuple();
                }
                py::tuple result(view->getDofCount());
                for (int32_t i = 0; i < view->getDofCount(); ++i)
                {
                    const char* const name = names[i];
                    result[i] = py::str(name ? name : "");
                }
                return result;
            },
            "Read-only tuple of degree-of-freedom names.")
        .def_property_readonly(
            "positions",
            [](PythonJointStateSensorReadingView& reading)
            {
                JointStateSensorReadingView* view = reading.getView();
                const py::ssize_t count = view ? view->getDofCount() : 0;
                return makeReadOnlyView(view ? view->getPositions() : nullptr, { count }, { 0 }, reading);
            },
            "Read-only zero-copy position array.")
        .def_property_readonly(
            "velocities",
            [](PythonJointStateSensorReadingView& reading)
            {
                JointStateSensorReadingView* view = reading.getView();
                const py::ssize_t count = view ? view->getDofCount() : 0;
                return makeReadOnlyView(view ? view->getVelocities() : nullptr, { count }, { 0 }, reading);
            },
            "Read-only zero-copy velocity array.")
        .def_property_readonly(
            "efforts",
            [](PythonJointStateSensorReadingView& reading)
            {
                JointStateSensorReadingView* view = reading.getView();
                const py::ssize_t count = view ? view->getDofCount() : 0;
                return makeReadOnlyView(view ? view->getEfforts() : nullptr, { count }, { 0 }, reading);
            },
            "Read-only zero-copy effort array.")
        .def_property_readonly(
            "dof_types",
            [](PythonJointStateSensorReadingView& reading)
            {
                JointStateSensorReadingView* view = reading.getView();
                const py::ssize_t count = view ? view->getDofCount() : 0;
                return makeReadOnlyView(view ? view->getDofTypes() : nullptr, { count }, { 0 }, reading);
            },
            "Read-only zero-copy degree-of-freedom type array.")
        .def_property_readonly(
            "stage_meters_per_unit",
            [](const PythonJointStateSensorReadingView& reading)
            {
                const JointStateSensorReadingView* view = reading.getView();
                return view ? view->getStageMetersPerUnit() : 0.0f;
            },
            "Stage scale in meters per unit.");

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
            { return copyJointStateSensorReading(self.getSensorReading(articulationRootPath.c_str())); },
            py::arg("prim_path"), "Return a mutable owned snapshot of the current joint state.")
        .def(
            "get_sensor_reading_view",
            [](IJointStateSensor& self, const std::string& articulationRootPath)
            { return getJointStateSensorReadingView(self, articulationRootPath); },
            py::arg("prim_path"), "Return a read-only zero-copy view of the live joint-state buffers.");

    // --- Raycast sensor ---
    py::class_<PythonRaycastSensorReading>(
        m, "RaycastSensorReading",
        "Mutable owned snapshot of a raycast reading. Its values and validity do not change after capture.")
        .def(py::init([]() { return copyRaycastSensorReading(RaycastSensorReading{}); }))
        .def_readonly("ray_count", &PythonRaycastSensorReading::rayCount, "Number of rays in the snapshot.")
        .def_readonly("time", &PythonRaycastSensorReading::time, "Capture time in seconds.")
        .def_readonly("is_valid", &PythonRaycastSensorReading::isValid, "Whether the reading was valid when captured.")
        .def_readonly("depths", &PythonRaycastSensorReading::depths, "Mutable depth array owned by the snapshot.")
        .def_readonly("hit_positions", &PythonRaycastSensorReading::hitPositions,
                      "Mutable hit-position array owned by the snapshot.")
        .def_readonly(
            "hit_normals", &PythonRaycastSensorReading::hitNormals, "Mutable hit-normal array owned by the snapshot.")
        .def_readonly("hit_prim_paths", &PythonRaycastSensorReading::hitPrimPaths,
                      "Mutable list of hit primitive paths owned by the snapshot.")
        .def_readonly("ray_origins_world", &PythonRaycastSensorReading::rayOriginsWorld,
                      "Mutable world-space ray-origin array owned by the snapshot.")
        .def_readonly("ray_end_points_world", &PythonRaycastSensorReading::rayEndPointsWorld,
                      "Mutable world-space ray-endpoint array owned by the snapshot.");

    py::class_<PythonRaycastSensorReadingView>(
        m, "RaycastSensorReadingView",
        "Read-only zero-copy view of live raycast sensor buffers. The arrays retain the backing frame, and is_valid "
        "becomes false after sensor removal, reset, or timeline stop while the last values remain readable.")
        .def(py::init<>())
        .def_property_readonly(
            "ray_count",
            [](const PythonRaycastSensorReadingView& reading)
            {
                const RaycastSensorReadingView* view = reading.getView();
                return view ? view->getRayCount() : 0;
            },
            "Number of rays in the view.")
        .def_property_readonly(
            "time",
            [](const PythonRaycastSensorReadingView& reading)
            {
                const RaycastSensorReadingView* view = reading.getView();
                return view ? view->getTime() : 0.0f;
            },
            "Current sensor time in seconds.")
        .def_property_readonly(
            "is_valid",
            [](const PythonRaycastSensorReadingView& reading)
            {
                const RaycastSensorReadingView* view = reading.getView();
                return view && view->isValid();
            },
            "Whether the owner-backed sensor frame is still active and contains a valid reading.")
        .def_property_readonly(
            "depths",
            [](PythonRaycastSensorReadingView& reading)
            {
                RaycastSensorReadingView* view = reading.getView();
                const py::ssize_t count = view ? view->getRayCount() : 0;
                return makeReadOnlyView(view ? view->getDepths() : nullptr, { count }, { 0 }, reading);
            },
            "Read-only zero-copy depth array.")
        .def_property_readonly(
            "hit_positions",
            [](PythonRaycastSensorReadingView& reading)
            {
                RaycastSensorReadingView* view = reading.getView();
                const py::ssize_t count = view ? view->getRayCount() : 0;
                return makeReadOnlyView(view ? view->getHitPositions() : nullptr, { count, 3 }, { 0, 3 }, reading);
            },
            "Read-only zero-copy hit-position array.")
        .def_property_readonly(
            "hit_normals",
            [](PythonRaycastSensorReadingView& reading)
            {
                RaycastSensorReadingView* view = reading.getView();
                const py::ssize_t count = view ? view->getRayCount() : 0;
                return makeReadOnlyView(view ? view->getHitNormals() : nullptr, { count, 3 }, { 0, 3 }, reading);
            },
            "Read-only zero-copy hit-normal array.")
        .def_property_readonly(
            "hit_prim_paths",
            [](const PythonRaycastSensorReadingView& reading)
            {
                const RaycastSensorReadingView* view = reading.getView();
                const char* const* paths = view ? view->getHitPrimPaths() : nullptr;
                if (!view || !paths)
                {
                    return py::tuple();
                }
                py::tuple result(view->getRayCount());
                for (uint32_t i = 0; i < view->getRayCount(); ++i)
                {
                    // Load once: a concurrent physics step may republish the slot between reads.
                    const char* const path = paths[i];
                    result[i] = py::str(path ? path : "");
                }
                return result;
            },
            "Read-only tuple of hit primitive paths.")
        .def_property_readonly(
            "ray_origins_world",
            [](PythonRaycastSensorReadingView& reading)
            {
                RaycastSensorReadingView* view = reading.getView();
                const py::ssize_t count = view ? view->getRayCount() : 0;
                return makeReadOnlyView(view ? view->getRayOriginsWorld() : nullptr, { count, 3 }, { 0, 3 }, reading);
            },
            "Read-only zero-copy world-space ray-origin array.")
        .def_property_readonly(
            "ray_end_points_world",
            [](PythonRaycastSensorReadingView& reading)
            {
                RaycastSensorReadingView* view = reading.getView();
                const py::ssize_t count = view ? view->getRayCount() : 0;
                return makeReadOnlyView(view ? view->getRayEndPointsWorld() : nullptr, { count, 3 }, { 0, 3 }, reading);
            },
            "Read-only zero-copy world-space ray-endpoint array.");

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
            [](IRaycastSensor& self, const std::string& primPath)
            { return copyRaycastSensorReading(self.getSensorReading(primPath.c_str())); },
            py::arg("prim_path"), "Return a mutable owned snapshot of the current raycast result.")
        .def(
            "get_sensor_reading_view",
            [](IRaycastSensor& self, const std::string& primPath) { return getRaycastSensorReadingView(self, primPath); },
            py::arg("prim_path"), "Return a read-only zero-copy view of the live raycast buffers.");
}

} // anonymous namespace
