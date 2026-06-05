// SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
#include <carb/BindingsUtils.h>

#include <isaacsim/sensors/physics/IPhysicsSensor.h>
#include <pybind11/functional.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <string>
CARB_BINDINGS("isaacsim.sensors.physics.python")


namespace isaacsim
{
namespace sensors
{
namespace physics
{
// Recreate CsRawData casting body0 and body1 to uintptr_t to pass through pybind pipeline. This allows for numpy arrays
// to be sent over with the body names pointers.
struct CsRawPython
{
    float time; //<! Simulation timestamp
    float dt; //<! Simulation timestamp
    uint64_t body0; //<! First body on contact
    uint64_t body1; //<! Second body on contact
    carb::Float3 position; //<! Contact Position, in world coordinates
    carb::Float3 normal; //<! Contact Normal, in world coordinates
    carb::Float3 impulse; //<! Contact Impulse, in world coordinates
};

}
}
}

namespace
{
namespace py = pybind11;

template <typename InterfaceType>
py::class_<InterfaceType> defineInterfaceClass(py::module& m,
                                               const char* className,
                                               const char* acquireFuncName,
                                               const char* releaseFuncName = nullptr,
                                               const char* docString = "")
{
    m.def(
        acquireFuncName,
        [](const char* pluginName, const char* libraryPath)
        {
            return libraryPath ? carb::acquireInterfaceFromLibraryForBindings<InterfaceType>(libraryPath) :
                                 carb::acquireInterfaceForBindings<InterfaceType>(pluginName);
        },
        py::arg("plugin_name") = nullptr, py::arg("library_path") = nullptr, py::return_value_policy::reference,
        "Acquire Contact Sensor interface. This is the base object that all of the Contact Sensor functions are defined on");

    if (releaseFuncName)
    {
        m.def(
            releaseFuncName, [](InterfaceType* iface) { carb::getFramework()->releaseInterface(iface); },
            "Release Contact Sensor interface. Generally this does not need to be called, the Contact Sensor interface is released on extension shutdown");
    }

    return py::class_<InterfaceType>(m, className, docString);
}


PYBIND11_MODULE(_sensor, m)
{


    using namespace carb;
    using namespace isaacsim::sensors::physics;

    auto carbModule = py::module::import("carb");
    auto numpyCommonModule = py::module::import("omni.kit.numpy.common");

    py::class_<CsRawPython>(m, "CsRawData", "Contact Raw Data")
        .def(py::init<>())
        .def_readwrite("time", &CsRawPython::time, "simulation timestamp, (:obj:`float`)")
        .def_readwrite("dt", &CsRawPython::dt, "timestep during this contact report, (:obj:`float`)")
        .def_readonly("body0", &CsRawPython::body0, "Body 0 name handle, (:obj:`int`)")
        .def_readonly("body1", &CsRawPython::body1, "Body 1 name handle, (:obj:`int`)")
        .def_readwrite("position", &CsRawPython::position, "position, global coordinates, (:obj:`carb.Float3`)")
        .def_readwrite("normal", &CsRawPython::normal, "normal, global coordinates , (:obj:`carb.Float3`)")
        .def_readwrite("impulse", &CsRawPython::impulse, "impulse, global coordinates , (:obj:`carb.Float3`)");

    // PYBIND11_NUMPY_DTYPE(carb::Float3, x, y, z);
    // PYBIND11_NUMPY_DTYPE(CsRawData, time, body0, body1, position, normal, impulse);

    py::class_<CsReading>(m, "CsSensorReading", "Contact Sensor Reading")
        .def(py::init<>())
        .def_readwrite("time", &CsReading::time, "timestamp of the reading, in seconds . (:obj:`float`)")
        .def_readwrite("value", &CsReading::value, "sensor force reading value. (:obj:`float`)")
        .def_readwrite(
            "inContact", &CsReading::inContact,
            "**Deprecation Alert**, inContact will be renamed to in_contact. boolean that flags if the sensor registers a contact. (:obj:`bool`)")
        .def_readwrite(
            "in_contact", &CsReading::inContact, "boolean that flags if the sensor registers a contact. (:obj:`bool`)")
        .def_readwrite("is_valid", &CsReading::isValid, "validity of the data. (:obj:`bool`)");

    py::class_<IsReading>(m, "IsSensorReading", "Imu Sensor Reading")
        .def(py::init<>())
        .def_readwrite("time", &IsReading::time, "timestamp of the reading, in seconds. (:obj:`float`)")
        .def_readwrite("lin_acc_x", &IsReading::linAccX, "Accelerometer reading value x axis, in m/s^2. (:obj:`float`)")
        .def_readwrite("lin_acc_y", &IsReading::linAccY, "Accelerometer reading value y axis, in m/s^2. (:obj:`float`)")
        .def_readwrite("lin_acc_z", &IsReading::linAccZ, "Accelerometer reading value z axis, in m/s^2. (:obj:`float`)")
        .def_readwrite("ang_vel_x", &IsReading::angVelX, "Gyroscope reading value x axis, in rad/s. (:obj:`float`)")
        .def_readwrite("ang_vel_y", &IsReading::angVelY, "Gyroscope reading value y axis, in rad/s. (:obj:`float`)")
        .def_readwrite("ang_vel_z", &IsReading::angVelZ, "Gyroscope reading value z axis, in rad/s. (:obj:`float`)")
        .def_readwrite(
            "orientation", &IsReading::orientation, "Orientation quaternion reading (x, y, z, w). (:obj:`carb.Float4`)")
        .def_readwrite("is_valid", &IsReading::isValid, "validity of sensor reading. (:obj:`bool`)");

    PYBIND11_NUMPY_DTYPE(CsReading, time, value, inContact, isValid);
    PYBIND11_NUMPY_DTYPE(CsRawPython, time, dt, body0, body1, position, normal, impulse);
    PYBIND11_NUMPY_DTYPE(IsReading, time, linAccX, linAccY, linAccZ, angVelX, angVelY, angVelZ, orientation, isValid);

    m.doc() = R"pbdoc(
    This Extension provides an interface to 'pxr.IsaacSensorSchemaIsaacBaseSensor' to be used in a stage.
    )pbdoc";

    defineInterfaceClass<ContactSensorInterface>(
        m, "ContactSensorInterface", "acquire_contact_sensor_interface", "release_contact_sensor_interface")
        .def(
            "get_contact_sensor_raw_data",
            [](ContactSensorInterface* li, const char* bodyPath) -> py::object
            {
                if (!li)
                {
                    return py::none();
                }
                size_t numData = 0;
                CsRawData* data = li->getSensorRawData(bodyPath, numData);
                return py::array(py::buffer_info(data, sizeof(CsRawPython), py::format_descriptor<CsRawPython>::format(),
                                                 1, { numData }, { sizeof(CsRawPython) }));
            },
            R"pbdoc(
                Args:
                    arg0 (:obj:`str`): USD Path to contact sensor as string

                Returns:
                    :obj:`numpy.array`: The list of contact raw data that contains the specified body that the contact sensor is attached to.)pbdoc")
        .def(
            "get_rigid_body_raw_data",
            [](ContactSensorInterface* li, const char* bodyPath) -> py::object
            {
                if (!li)
                {
                    return py::none();
                }
                size_t numData = 0;
                CsRawData* data = li->getBodyRawData(bodyPath, numData);
                return py::array(py::buffer_info(data, sizeof(CsRawPython), py::format_descriptor<CsRawPython>::format(),
                                                 1, { numData }, { sizeof(CsRawPython) }));
            },
            R"pbdoc(
                Get raw data from a rigid body that have contact report API enabled
                Args:
                    arg0 (:obj:`str`): USD Path to rigid body as string

                Returns:
                    :obj:`numpy.array`: The list of contact raw data that contains the specified body.)pbdoc")
        .def("decode_body_name", wrapInterfaceFunction(&ContactSensorInterface::decodeBodyName),
             R"pbdoc(
                Decodes the body name pointers from the contact raw data into a string
                Args:
                    arg0 (:obj:`int`): body name handle
                Returns:
                    :obj:`str`: The body name.)pbdoc")
        .def("get_sensor_reading", wrapInterfaceFunction(&ContactSensorInterface::getSensorReading),
             R"pbdoc(
                Args:
                    arg0 (:obj:`char*`): the sensor path
                    arg1 (:obj:`bool`): use_latest_data
                Returns:
                    :obj:`numpy.array`: The reading for the current sensor period.)pbdoc",
             py::arg("sensor_path"), py::arg("use_latest_data") = false)
        .def("is_contact_sensor", wrapInterfaceFunction(&ContactSensorInterface::isContactSensor),
             R"pbdoc(
                Args:
                    arg0 (:obj:`str`): USD Path to sensor as string
                Returns:
                    :obj:`bool`: True for is contact sensor, False for not contact sensor.)pbdoc");


    defineInterfaceClass<ImuSensorInterface>(
        m, "ImuSensorInterface", "acquire_imu_sensor_interface", "release_imu_sensor_interface")
        .def(
            "get_sensor_reading",
            [](const ImuSensorInterface* li, const char* sensorPath,
               std::function<IsReading(std::vector<IsReading>, float)> interpolationFunction = nullptr,
               bool useLatestData = false, bool readGravity = true) -> py::object
            {
                if (!li)
                {
                    return py::none();
                }
                IsReading data = IsReading();
                if (interpolationFunction)
                {
                    data = li->getSensorReading(sensorPath, carb::wrapPythonCallback(std::move(interpolationFunction)),
                                                useLatestData, readGravity);
                }
                else
                {
                    data = li->getSensorReading(sensorPath, nullptr, useLatestData, readGravity);
                }
                return py::cast(data);
            },
            R"pbdoc(
                Args:
                    arg0 (:obj:`char*`): the sensor path
                    arg1 (:obj:`std::function<IsReading(std::vector<IsReading>, float)>&`): interpolation_function
                    arg2 (:obj:`bool`): use_latest_data
                    arg3 (:obj:`bool`): read_gravity
                Returns:
                    :obj:`numpy.array`: The reading for the current sensor period.)pbdoc",
            py::arg("sensor_path"), py::arg("interpolation_function") = nullptr, py::arg("use_latest_data") = false,
            py::arg("read_gravity") = true)
        .def("is_imu_sensor", wrapInterfaceFunction(&ImuSensorInterface::isImuSensor),
             R"pbdoc(
                Args:
                    arg0 (:obj:`str`): USD Path to sensor as string
                Returns:
                    :obj:`bool`: True for is imu sensor, False for not imu sensor.)pbdoc");
}
}
