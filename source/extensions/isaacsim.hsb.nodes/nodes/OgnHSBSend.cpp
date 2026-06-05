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

// clang-format off
#include <pch/UsdPCH.h>
// clang-format on

#include "dlpack/dlpack.h"

#include <carb/logging/Log.h>
#include <carb/tasking/ITasking.h>
#include <carb/tasking/TaskingUtils.h>

#include <isaacsim/core/includes/BaseResetNode.h>
#include <isaacsim/hsb/core/HSBSender.h>

#include <OgnHSBSendDatabase.h>
#include <memory>
#include <string>
#include <vector>

using namespace isaacsim::hsb::core;

/**
 * @class OgnHSBSend
 * @brief Sends data buffers via HSB (Holoscan Sensor Bridge) using DLTensor.
 * @details
 * HSB uses DLTensor format and can handle GPU data directly without CPU serialization.
 *
 * When simulation stops, reset() is called which disconnects the HSB emulator.
 * This allows the linux_vb1940_player to reconnect when simulation restarts.
 */
class OgnHSBSend : public isaacsim::core::includes::BaseResetNode
{
public:
    /**
     * @brief Compute function - called when node is executed
     */
    static bool compute(OgnHSBSendDatabase& db)
    {
        try
        {
            auto& state = db.perInstanceState<OgnHSBSend>();
            bool success = state.computeImpl(db);
            db.outputs.execOut() = omni::graph::core::kExecutionAttributeStateEnabled;
            return success;
        }
        catch (const std::exception& e)
        {
            CARB_LOG_ERROR("[HSB Bridge] compute() exception: %s", e.what());
            db.outputs.execOut() = omni::graph::core::kExecutionAttributeStateEnabled;
            return false;
        }
        catch (...)
        {
            CARB_LOG_ERROR("[HSB Bridge] compute() unknown exception");
            db.outputs.execOut() = omni::graph::core::kExecutionAttributeStateEnabled;
            return false;
        }
    }

    /**
     * @brief Release the node instance
     */
    static void releaseInstance(NodeObj const& nodeObj, GraphInstanceID instanceId)
    {
        auto& state = OgnHSBSendDatabase::sPerInstanceState<OgnHSBSend>(nodeObj, instanceId);
        state.shutdown();
    }

    /**
     * @brief Reset the node when simulation stops (from BaseResetNode)
     * @details
     * Called automatically when the timeline stop event is triggered.
     * Disconnects the HSB emulator so it can be cleanly reconnected on next play.
     */
    void reset() override
    {
        CARB_LOG_INFO("[HSB Bridge] reset() called - disconnecting HSB emulator");
        shutdown();
    }

private:
    bool isInitialized() const
    {
        return m_sender && m_sender->isConnected();
    }

    void createEndpoint(OgnHSBSendDatabase& db)
    {
        const std::string ip = db.inputs.ipAddress();
        const std::string dataPlaneType = db.inputs.dataPlaneType();

        CARB_LOG_INFO("[HSB Bridge] Creating sender: ip=%s dataPlaneType='%s'", ip.c_str(), dataPlaneType.c_str());

        m_sender = std::make_unique<HSBSender>(ip, static_cast<uint8_t>(db.inputs.dataPlaneId()),
                                               static_cast<uint8_t>(db.inputs.sensorId()), dataPlaneType);
    }

    void initialize(OgnHSBSendDatabase& db)
    {
        createEndpoint(db);
        if (m_sender)
        {
            m_sender->connect();
        }
    }

    bool computeImpl(OgnHSBSendDatabase& db)
    {
        // Drain the previous frame's send before touching m_sendBuffer
        m_sendTask.wait();

        if (!isInitialized())
        {
            initialize(db);
            if (!isInitialized())
            {
                db.logError("Failed to initialize endpoint");
                return false;
            }
        }
        return process(db);
    }

    /**
     * @brief Process and send a 1D buffer via HSB (async via carb::tasking)
     */
    bool process(OgnHSBSendDatabase& db)
    {
        if (db.inputs.data.size() == 0)
        {
            db.logError("No valid data source");
            return false;
        }

        const auto& inputData = db.inputs.data.cpu();
        size_t dataSize = inputData.size();

        if (dataSize == 0)
        {
            db.logWarning("Buffer is empty");
            return true;
        }

        // Copy frame into persistent buffer (m_sendTask.wait() above guarantees
        // the previous send is done, so m_sendBuffer is safe to overwrite)
        m_sendBuffer.assign(inputData.data(), inputData.data() + dataSize);
        m_sendShape = static_cast<int64_t>(dataSize);

        auto tasking = carb::getCachedInterface<carb::tasking::ITasking>();
        tasking->addTask(carb::tasking::Priority::eHigh, m_sendTask,
                         [this]()
                         {
                             DLTensor tensor{};
                             tensor.data = m_sendBuffer.data();
                             tensor.strides = nullptr;
                             tensor.byte_offset = 0;
                             tensor.dtype.code = DLDataTypeCode::kDLUInt;
                             tensor.dtype.bits = 8;
                             tensor.dtype.lanes = 1;
                             tensor.device.device_type = DLDeviceType::kDLCPU;
                             tensor.device.device_id = 0;
                             tensor.ndim = 1;
                             tensor.shape = &m_sendShape;
                             m_sender->send(tensor);
                         });

        return true;
    }

    void shutdown()
    {
        m_sendTask.wait(); // ensure in-flight send finishes before disconnect
        if (m_sender)
        {
            m_sender->disconnect();
            m_sender.reset();
        }
    }

    std::unique_ptr<HSBSender> m_sender;
    carb::tasking::TaskGroup m_sendTask;
    std::vector<uint8_t> m_sendBuffer;
    int64_t m_sendShape = 0;
};

REGISTER_OGN_NODE()
