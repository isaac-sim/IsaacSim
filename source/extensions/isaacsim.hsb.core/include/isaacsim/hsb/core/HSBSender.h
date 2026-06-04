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

#pragma once

#include <dlpack/dlpack.h>
#include <hololink/emulation/coe_data_plane.hpp>
#include <hololink/emulation/data_plane.hpp>
#include <hololink/emulation/hsb_emulator.hpp>
#include <hololink/emulation/linux_data_plane.hpp>
#include <hololink/emulation/sensors/vb1940_emulator.hpp>

#include <memory>
#include <string>

namespace isaacsim::hsb::core
{

/**
 * @class HSBSender
 * @brief Manages the HSB emulator lifecycle and sends DLTensor data via a DataPlane.
 * @details
 * Owns the HSBEmulator, DataPlane (Linux/RoCEv2 or COE/IEEE 1722B), and Vb1940Emulator.
 * Supports "linux" (RoCEv2 UDP) or "coe" (IEEE 1722B Camera-over-Ethernet) data planes.
 */
class HSBSender
{
public:
    /**
     * @brief Construct an HSB sender
     * @param ipAddress Source IP address for the HSB emulator
     * @param dataPlaneId Data plane identifier
     * @param sensorId Sensor identifier
     * @param dataPlaneType "linux" for RoCEv2 UDP transport, "coe" for IEEE 1722B Camera-over-Ethernet
     */
    HSBSender(const std::string& ipAddress,
              uint8_t dataPlaneId,
              uint8_t sensorId,
              const std::string& dataPlaneType = "linux");

    /**
     * @brief Connect to the HSB data plane
     * @return True if the connection was established successfully, false otherwise
     */
    bool connect();

    /**
     * @brief Disconnect from the HSB data plane
     * @return True if the disconnection was successful, false otherwise
     */
    bool disconnect();

    /**
     * @brief Check whether the sender is currently connected
     * @return True if connected, false otherwise
     */
    bool isConnected() const;

    /**
     * @brief Send a DLTensor via the HSB data plane
     * @param tensor The tensor to send (CPU or GPU data)
     * @return True if sent successfully, false otherwise
     */
    bool send(const DLTensor& tensor);

    /**
     * @brief Check if the host has set the sensor to streaming mode
     * @return True if the Vb1940Emulator is streaming, false otherwise
     */
    bool isStreaming() const;

private:
    std::unique_ptr<hololink::emulation::HSBEmulator> m_emulator;
    std::unique_ptr<hololink::emulation::DataPlane> m_dataPlane;
    std::unique_ptr<hololink::emulation::sensors::Vb1940Emulator> m_vb1940;
    std::string m_ipAddress;
    uint8_t m_dataPlaneId;
    uint8_t m_sensorId;
    std::string m_dataPlaneType;
    bool m_connected;
};

} // namespace isaacsim::hsb::core
