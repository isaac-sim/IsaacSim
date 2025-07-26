// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <isaacsim/sensors/physx/IPhysxSensorInterface.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

CARB_BINDINGS("isaacsim.sensors.physx.python")

namespace
{
PYBIND11_MODULE(_range_sensor, m)
{
    using namespace carb;
    using namespace isaacsim::sensors::physx;

    m.doc() = R"pbdoc(
        This extension provides an interface to a `pxr.RangeSensorSchemaLidar` prim defined in a stage.

        Example:
            To use this interface you must first call the acquire interface function.
            It is also recommended to use the `is_range_sensor` function to check if a given USD path is valid

            .. code-block:: python

                >>> import isaacsim.sensors.physx._range_sensor as _range_sensor
                >>> lidar_sensor_interface = _range_sensor.acquire_lidar_sensor_interface()
                >>> if lidar_sensor_interface.is_lidar_sensor("/World/Lidar"):
                ...     print("range_sensor is valid")
                range_sensor is valid

        Refer to the sample documentation for more examples and usage
                )pbdoc";

    defineInterfaceClass<LidarSensorInterface>(
        m, "LidarSensorInterface", "acquire_lidar_sensor_interface", "release_lidar_sensor_interface")
        .def("get_num_cols", wrapInterfaceFunction(&LidarSensorInterface::getNumCols),
             R"pbdoc(
                Args:
                    arg0 (:obj:`str`): USD path to sensor as a string

                Returns:
                    :obj:`int`: The number of vertical scans of the sensor, 0 if error occurred)pbdoc")
        .def("get_num_rows", wrapInterfaceFunction(&LidarSensorInterface::getNumRows),
             R"pbdoc(
                Args:
                    arg0 (:obj:`str`): USD path to sensor as a string

                Returns:
                     :obj:`int`: The number of horizontal scans of the sensor, 0 if error occurred)pbdoc")
        .def("get_num_cols_ticked", wrapInterfaceFunction(&LidarSensorInterface::getNumColsTicked),
             R"pbdoc(
                Args:
                    arg0 (:obj:`str`): USD path to sensor as a string

                Returns:
                     :obj:`int`: The number of vertical scans the sensor completed in the last simulation step, 0 if error occurred. Generally only useful for lidars with a non-zero rotation speed)pbdoc")

        .def(
            "get_depth_data",
            [](const LidarSensorInterface* li, const char* sensorPath) -> py::object
            {
                if (!li)
                {
                    return py::none();
                }
                uint16_t* data = li->getDepthData(sensorPath);
                int rows = li->getNumRows(sensorPath);
                int numColsTicked = li->getNumColsTicked(sensorPath);
                return py::array(py::buffer_info(data, sizeof(uint16_t), py::format_descriptor<uint16_t>::value, 2,
                                                 { numColsTicked, rows }, { sizeof(uint16_t) * rows, sizeof(uint16_t) }));
            },
            R"pbdoc(
                Args:
                    arg0 (:obj:`str`): USD path to sensor as a string

                Returns:
                    :obj:`numpy.ndarray`: The distance from the sensor to the hit for each beam in uint16 and scaled by min and max distance)pbdoc")

        .def(
            "get_linear_depth_data",
            [](const LidarSensorInterface* li, const char* sensorPath) -> py::object
            {
                if (!li)
                {
                    return py::none();
                }
                float* data = li->getLinearDepthData(sensorPath);
                int rows = li->getNumRows(sensorPath);
                int numColsTicked = li->getNumColsTicked(sensorPath);
                return py::array(py::buffer_info(data, sizeof(float), py::format_descriptor<float>::value, 2,
                                                 { numColsTicked, rows }, { sizeof(float) * rows, sizeof(float) }));
            },
            R"pbdoc(
                Args:
                    arg0 (:obj:`str`): USD path to sensor as a string

                Returns:
                    :obj:`numpy.ndarray`: The distance from the sensor to the hit for each beam in meters)pbdoc")


        .def(
            "get_intensity_data",
            [](const LidarSensorInterface* li, const char* sensorPath) -> py::object
            {
                if (!li)
                {
                    return py::none();
                }
                uint8_t* data = li->getIntensityData(sensorPath);
                int rows = li->getNumRows(sensorPath);
                int numColsTicked = li->getNumColsTicked(sensorPath);
                return py::array(py::buffer_info(data, sizeof(uint8_t), py::format_descriptor<uint8_t>::value, 2,
                                                 { numColsTicked, rows }, { sizeof(uint8_t) * rows, sizeof(uint8_t) }));
            },
            R"pbdoc(
                Args:
                    arg0 (:obj:`str`): USD path to sensor as a string

                Returns:
                    :obj:`numpy.ndarray`: The observed specular intensity of each beam, 255 if hit, 0 if not)pbdoc")

        .def(
            "get_zenith_data",
            [](const LidarSensorInterface* li, const char* sensorPath) -> py::object
            {
                if (!li)
                {
                    return py::none();
                }
                float* data = li->getZenithData(sensorPath);
                int rows = li->getNumRows(sensorPath);
                return py::array(py::buffer_info(
                    data, sizeof(float), py::format_descriptor<float>::value, 1, { rows }, { sizeof(float) }));
            },
            R"pbdoc(
                Args:
                    arg0 (:obj:`str`): USD path to sensor as a string

                Returns:
                    :obj:`numpy.ndarray`: The zenith angle in radians for each row)pbdoc")

        .def(
            "get_azimuth_data",
            [](const LidarSensorInterface* li, const char* sensorPath) -> py::object
            {
                if (!li)
                {
                    return py::none();
                }
                float* data = li->getAzimuthData(sensorPath);
                int numColsTicked = li->getNumColsTicked(sensorPath);
                return py::array(py::buffer_info(
                    data, sizeof(float), py::format_descriptor<float>::value, 1, { numColsTicked }, { sizeof(float) }));
            },
            R"pbdoc(
                Args:
                    arg0 (:obj:`str`): USD path to sensor as a string

                Returns:
                    :obj:`numpy.ndarray`: The azimuth angle in radians for each column)pbdoc")

        .def(
            "get_point_cloud_data",
            [](const LidarSensorInterface* li, const char* sensorPath) -> py::object
            {
                if (!li)
                {
                    return py::none();
                }
                carb::Float3* data = li->getPointCloud(sensorPath);
                int rows = li->getNumRows(sensorPath);
                int numColsTicked = li->getNumColsTicked(sensorPath);
                return py::array(py::buffer_info(data, sizeof(float), py::format_descriptor<float>::value, 3,
                                                 { numColsTicked, rows, 3 },
                                                 { sizeof(float) * rows * 3, sizeof(float) * 3, sizeof(float) }));
            },
            R"pbdoc(
                Args:
                    arg0 (:obj:`str`): USD path to sensor as a string

                Returns:
                    :obj:`numpy.ndarray`: The hit position in xyz relative to the sensor origin, not accounting for individual ray offsets)pbdoc")
        .def(
            "get_semantic_data",
            [](const LidarSensorInterface* li, const char* sensorPath) -> py::object
            {
                CARB_LOG_WARN(
                    "Lidar get_semantic_data is deprecated and will not return any data, use get_prim_data and access semantics via usd");
                return py::array();
            },
            R"pbdoc([Deprecated]
                Args:
                    arg0 (:obj:`str`): USD path to sensor as a string

                Returns:
                    :obj:`numpy.ndarray`: The semantic id of the hit for each beam in uint16)pbdoc")
        .def(
            "get_prim_data",
            [](const LidarSensorInterface* li, const char* sensorPath) -> py::object
            {
                if (!li)
                {
                    return py::none();
                }
                return py::list(py::cast(li->getPrimData(sensorPath)));
            },
            R"pbdoc(
                Args:
                    arg0 (:obj:`str`): USD path to sensor as a string

                Returns:
                    :obj:`list`: The prim path of the hit for each beam as a string)pbdoc")
        .def("is_lidar_sensor", wrapInterfaceFunction(&LidarSensorInterface::isLidarSensor),
             R"pbdoc(
                Args:
                    arg0 (:obj:`str`): USD path to sensor as a string

                Returns:
                    :obj:`bool`: True if a sensor exists at the give path, False otherwise)pbdoc");

    defineInterfaceClass<GenericSensorInterface>(
        m, "GenericSensorInterface", "acquire_generic_sensor_interface", "release_generic_sensor_interface")
        .def("is_generic_sensor", wrapInterfaceFunction(&GenericSensorInterface::isGenericSensor),
             R"pbdoc(
                Args:
                    arg0 (:obj:`str`): USD path to sensor as a string

                Returns:
                    :obj:`bool`: True if a sensor exists at the give path, False otherwise)pbdoc")

        .def("send_next_batch", wrapInterfaceFunction(&GenericSensorInterface::sendNextBatch), "ready for next batch")

        .def(
            "set_next_batch_rays",
            [](const GenericSensorInterface* gs, const char* sensorPath, py::array_t<float> x)
            {
                if (!gs)
                {
                    return;
                }
                const auto& r = x.unchecked<2>();
                gs->setNextBatchRays(sensorPath, r.data(0, 0), r.data(1, 0), static_cast<int>(r.shape(1)));
            },
            R"pbdoc(
                Args:
                    arg0 (:obj:`str`): USD path to sensor as a string
                    arg1 (:obj:`numpy.ndaray`): The azimuth and zenith angles in radians for each column)pbdoc")


        .def(
            "set_next_batch_offsets",
            [](const GenericSensorInterface* gs, const char* sensorPath, py::array_t<float> x)
            {
                if (!gs)
                {
                    return;
                }
                const auto& r = x.unchecked<>();
                gs->setNextBatchOffsets(sensorPath, r.data(), static_cast<int>(r.shape(0)));
            },
            R"pbdoc(
                Args:
                    arg0 (:obj:`str`): USD path to sensor as a string
                    arg1 (:obj:`numpy.ndaray`): The offset xyz, a 2D array for individual rays, or 1D array for a constant offset)pbdoc")

        .def("get_num_samples_ticked", wrapInterfaceFunction(&GenericSensorInterface::getNumSamplesTicked),
             R"pbdoc(
                Args:
                    arg0 (:obj:`str`): USD path to sensor as a string

                Returns:
                     :obj:`int`: The number of sample points the sensor completed in the last simulation step, 0 if error occurred.)pbdoc")
        .def(
            "get_depth_data",
            [](const GenericSensorInterface* gs, const char* sensorPath) -> py::object
            {
                if (!gs)
                {
                    return py::none();
                }
                uint16_t* data = gs->getDepthData(sensorPath);
                int samples = gs->getNumSamplesTicked(sensorPath);
                return py::array(py::buffer_info(data, sizeof(uint16_t), py::format_descriptor<uint16_t>::value, 1,
                                                 { samples }, { sizeof(uint16_t) }));
            },
            R"pbdoc(
            Args:
                arg0 (:obj:`str`): USD path to sensor as a string

            Returns:
                :obj:`numpy.ndarray`: The distance from the sensor to the hit for each beam in uint16 and scaled by min and max distance)pbdoc")

        .def(
            "get_linear_depth_data",
            [](const GenericSensorInterface* gs, const char* sensorPath) -> py::object
            {
                if (!gs)
                {
                    return py::none();
                }
                float* data = gs->getLinearDepthData(sensorPath);
                int samples = gs->getNumSamplesTicked(sensorPath);
                return py::array(py::buffer_info(
                    data, sizeof(float), py::format_descriptor<float>::value, 1, { samples }, { sizeof(float) }));
            },
            R"pbdoc(
            Args:
                arg0 (:obj:`str`): USD path to sensor as a string

            Returns:
                :obj:`numpy.ndarray`: The distance from the sensor to the hit for each beam in meters)pbdoc")

        .def(
            "get_intensity_data",
            [](const GenericSensorInterface* gs, const char* sensorPath) -> py::object
            {
                if (!gs)
                {
                    return py::none();
                }
                uint8_t* data = gs->getIntensityData(sensorPath);
                int samples = gs->getNumSamplesTicked(sensorPath);
                return py::array(py::buffer_info(
                    data, sizeof(uint8_t), py::format_descriptor<uint8_t>::value, 1, { samples }, { sizeof(uint8_t) }));
            },
            R"pbdoc(
            Args:
                arg0 (:obj:`str`): USD path to sensor as a string

            Returns:
                :obj:`numpy.ndarray`: The observed specular intensity of each beam, 255 if hit, 0 if not)pbdoc")

        .def(
            "get_zenith_data",
            [](const GenericSensorInterface* gs, const char* sensorPath) -> py::object
            {
                if (!gs)
                {
                    return py::none();
                }
                float* data = gs->getZenithData(sensorPath);
                int samples = gs->getNumSamplesTicked(sensorPath);
                return py::array(py::buffer_info(
                    data, sizeof(float), py::format_descriptor<float>::value, 1, { samples }, { sizeof(float) }));
            },
            R"pbdoc(
                Args:
                    arg0 (:obj:`str`): USD path to sensor as a string

                Returns:
                    :obj:`numpy.ndarray`: The zenith angle in radians for each row)pbdoc")

        .def(
            "get_azimuth_data",
            [](const GenericSensorInterface* gs, const char* sensorPath) -> py::object
            {
                if (!gs)
                {
                    return py::none();
                }
                float* data = gs->getAzimuthData(sensorPath);
                int samples = gs->getNumSamplesTicked(sensorPath);
                return py::array(py::buffer_info(
                    data, sizeof(float), py::format_descriptor<float>::value, 1, { samples }, { sizeof(float) }));
            },
            R"pbdoc(
                Args:
                    arg0 (:obj:`str`): USD path to sensor as a string

                Returns:
                    :obj:`numpy.ndarray`: The azimuth angle in radians for each column)pbdoc")
        .def(
            "get_point_cloud_data",
            [](const GenericSensorInterface* gs, const char* sensorPath) -> py::object
            {
                if (!gs)
                {
                    return py::none();
                }
                carb::Float3* data = gs->getPointCloud(sensorPath);
                int samples = gs->getNumSamplesTicked(sensorPath);
                return py::array(py::buffer_info(data, sizeof(float), py::format_descriptor<float>::value, 2,
                                                 { samples, 3 }, { sizeof(float) * 3, sizeof(float) }));
            },
            R"pbdoc(
                Args:
                    arg0 (:obj:`str`): USD path to sensor as a string

                Returns:
                    :obj:`numpy.ndarray`: The hit position in xyz relative to the sensor origin, not accounting for individual ray offsets)pbdoc");

    defineInterfaceClass<LightBeamSensorInterface>(
        m, "LightBeamSensorInterface", "acquire_lightbeam_sensor_interface", "release_lightbeam_sensor_interface")
        .def("get_num_rays", wrapInterfaceFunction(&LightBeamSensorInterface::getNumRays),
             R"pbdoc(
                Args:
                    arg0 (:obj:`str`): USD Path to sensor as string
                Returns:
                    :obj:`int`: The number of rays in the light curtain.)pbdoc")
        .def(
            "get_linear_depth_data",
            [](const LightBeamSensorInterface* li, const char* sensorPath) -> py::object
            {
                if (!li)
                {
                    return py::none();
                }
                float* data = li->getLinearDepthData(sensorPath);
                int numRays = li->getNumRays(sensorPath);
                return py::array(py::buffer_info(
                    data, sizeof(float), py::format_descriptor<float>::value, 1, { numRays }, { sizeof(float) }));
            },
            R"pbdoc(
                Args:
                    arg0 (:obj:`str`): USD path to sensor as a string

                Returns:
                    :obj:`numpy.ndarray`: The distance from the sensor to the hit for each light beam in meters)pbdoc")
        .def(
            "get_beam_hit_data",
            [](const LightBeamSensorInterface* li, const char* sensorPath) -> py::object
            {
                if (!li)
                {
                    return py::none();
                }
                uint8_t* data = li->getBeamHitData(sensorPath);
                int numRays = li->getNumRays(sensorPath);
                return py::array(py::buffer_info(
                    data, sizeof(uint8_t), py::format_descriptor<uint8_t>::value, 1, { numRays }, { sizeof(uint8_t) }));
            },
            R"pbdoc(
                Args:
                    arg0 (:obj:`str`): USD path to sensor as a string

                Returns:
                    :obj:`numpy.ndarray`: True for light beam sensor detecting a raycast hit, False for not no hit)pbdoc")

        .def(
            "get_hit_pos_data",
            [](const LightBeamSensorInterface* li, const char* sensorPath) -> py::object
            {
                if (!li)
                {
                    return py::none();
                }
                carb::Float3* data = li->getHitPosData(sensorPath);
                int numRays = li->getNumRays(sensorPath);
                return py::array(py::buffer_info(data, sizeof(float), py::format_descriptor<float>::value, 2,
                                                 { numRays, 3 }, { sizeof(float) * 3, sizeof(float) }));
            },
            R"pbdoc(
                Args:
                    arg0 (:obj:`str`): USD path to sensor as a string

                Returns:
                    :obj:`numpy.ndarray`: Hit positions in xyz for each light beam relative to sensor origin)pbdoc")

        .def("is_lightbeam_sensor", wrapInterfaceFunction(&LightBeamSensorInterface::isLightBeamSensor),
             R"pbdoc(
                Args:
                    arg0 (:obj:`str`): USD Path to sensor as string
                Returns:
                    :obj:`bool`: True for is light beam sensor, False for not light beam sensor.)pbdoc");
}
}
