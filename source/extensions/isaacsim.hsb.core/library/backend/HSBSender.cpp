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

#include <carb/logging/Log.h>

#include <hololink/core/hololink.hpp>
#include <hololink/emulation/emulator_utils.hpp>
#include <hololink/emulation/hsb_config.hpp>
#include <isaacsim/hsb/core/HSBSender.h>

using namespace hololink::emulation;

namespace isaacsim::hsb::core
{

HSBSender::HSBSender(const std::string& ipAddress, uint8_t dataPlaneId, uint8_t sensorId, const std::string& dataPlaneType)
    : m_ipAddress(ipAddress),
      m_dataPlaneId(dataPlaneId),
      m_sensorId(sensorId),
      m_dataPlaneType(dataPlaneType.empty() ? "linux" : dataPlaneType),
      m_connected(false)
{
}

bool HSBSender::connect()
{
    CARB_LOG_INFO("[HSB Bridge] connect() ip=%s dataPlaneId=%u sensorId=%u dataPlaneType='%s'", m_ipAddress.c_str(),
                  (unsigned)m_dataPlaneId, (unsigned)m_sensorId, m_dataPlaneType.c_str());

    if (m_connected)
    {
        return true;
    }

    try
    {
        // Parse IP address
        IPAddress source_ip = IPAddress_from_string(m_ipAddress);

        // Use leopard_eagle config (includes i2c_bus in enumeration for VB1940 player)
        HSBConfiguration config = HSB_LEOPARD_EAGLE_CONFIG;

        // Create HSB emulator
        m_emulator = std::make_unique<HSBEmulator>(config);

        // Create data plane (Linux/RoCEv2 or COE IEEE 1722B)
        if (m_dataPlaneType == "coe")
        {
            m_dataPlane = std::make_unique<COEDataPlane>(*m_emulator, source_ip, m_dataPlaneId, m_sensorId);
        }
        else
        {
            m_dataPlane = std::make_unique<LinuxDataPlane>(*m_emulator, source_ip, m_dataPlaneId, m_sensorId);
        }

        // Create and attach Vb1940Emulator for linux_vb1940_player compatibility
        // On Leopard Eagle, the i2c bus address is the sensor_id offset from CAM_I2C_BUS
        m_vb1940 = std::make_unique<sensors::Vb1940Emulator>();
        m_vb1940->attach_to_i2c(m_emulator->get_i2c(hololink::I2C_CTRL), hololink::CAM_I2C_BUS + m_sensorId);

        // Start HSB emulator (this starts BootP broadcast AND control message listener)
        m_emulator->start();

        CARB_LOG_INFO("[HSB Bridge] Connected successfully (dataPlane: %s)", m_dataPlaneType.c_str());
        m_connected = true;
        return true;
    }
    catch (const std::exception& e)
    {
        CARB_LOG_ERROR("[HSB Bridge] Failed to connect: %s", e.what());
        m_connected = false;
        return false;
    }
}

bool HSBSender::disconnect()
{
    if (!m_connected)
    {
        return true;
    }

    try
    {
        // Stop HSB emulator (this stops BootP and control listener)
        if (m_emulator)
        {
            m_emulator->stop();
        }

        // Clean up in reverse order of creation
        m_vb1940.reset();
        m_dataPlane.reset();
        m_emulator.reset();
        m_connected = false;
        return true;
    }
    catch (const std::exception& e)
    {
        CARB_LOG_ERROR("[HSB Bridge] Failed to disconnect: %s", e.what());
        return false;
    }
}

bool HSBSender::isStreaming() const
{
    if (m_vb1940)
    {
        return m_vb1940->is_streaming();
    }
    return false;
}

bool HSBSender::isConnected() const
{
    return m_connected && m_dataPlane != nullptr;
}

bool HSBSender::send(const DLTensor& tensor)
{
    if (!isConnected())
    {
        CARB_LOG_ERROR("[HSB Bridge] Failed to send: not connected");
        return false;
    }

    // Check if host has configured streaming - if not, data won't actually be sent
    // (DataPlane::send returns 0 when no receiver has connected)
    static bool streamingWarningLogged = false;
    if (!isStreaming() && !streamingWarningLogged)
    {
        CARB_LOG_WARN("[HSB Bridge] Vb1940 not in streaming mode - waiting for host to connect");
        streamingWarningLogged = true;
    }
    else if (isStreaming() && streamingWarningLogged)
    {
        CARB_LOG_INFO("[HSB Bridge] Vb1940 now streaming - host connected");
        streamingWarningLogged = false;
    }

    int64_t bytes_sent = m_dataPlane->send(tensor);
    return bytes_sent >= 0;
}

} // namespace isaacsim::hsb::core
