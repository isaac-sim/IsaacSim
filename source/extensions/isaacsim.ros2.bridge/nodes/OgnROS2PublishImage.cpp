// SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#if !defined(_WIN32)
#    include <isaacsim/ros2/bridge/IpcBufferManager.h>
#endif

#include <carb/RenderingTypes.h>
#include <carb/profiler/Profile.h>
#include <carb/tasking/ITasking.h>
#include <carb/tasking/TaskingUtils.h>

#include <isaacsim/core/includes/Buffer.h>
#include <isaacsim/core/includes/ScopedCudaDevice.h>
#include <isaacsim/ros2/bridge/Ros2Node.h>

#include <OgnROS2PublishImageDatabase.h>

using namespace isaacsim::ros2::bridge;

extern "C" void textureFloatCopyToRawBuffer(cudaTextureObject_t, uint8_t*, uint32_t, uint32_t, cudaStream_t);

class PublishImageThreadData
{
public:
    PublishImageThreadData()
    {
    }

    void* inputDataPtr;
    void* outputDataPtr;
    carb::Format resourceFormat;
    int width;
    int height;
    size_t bufferSize;
    size_t totalBytes; // Should be the same as bufferSize, need to refactor this
    int cudaDeviceIndex;

    cudaStream_t* stream;
    int* streamDevice;
    bool* mStreamNotCreated;

    std::shared_ptr<Ros2Publisher> publisher;
    std::shared_ptr<Ros2ImageMessage> message;
};

class PublishNitrosBridgeImageThreadData
{
public:
    PublishNitrosBridgeImageThreadData()
    {
    }

    void* inputDataPtr;
    void* outputDataPtr;
    carb::Format resourceFormat;
    int width;
    int height;
    size_t bufferSize;
    size_t totalBytes; // Should be the same as bufferSize, need to refactor this
    int cudaDeviceIndex;

    cudaStream_t* nitrosBridgeStream;
    int* nitrosBridgeStreamDevice;
    bool* nitrosBridgeStreamNotCreated;

#if !defined(_WIN32)
    std::shared_ptr<IPCBufferManager> ipcBufferManager;
#endif

    std::shared_ptr<Ros2Publisher> nitrosBridgePublisher;
    std::shared_ptr<Ros2NitrosBridgeImageMessage> nitrosBridgeMessage;
};

class OgnROS2PublishImage : public Ros2Node
{
public:
    static bool compute(OgnROS2PublishImageDatabase& db)
    {
        auto& state = db.perInstanceState<OgnROS2PublishImage>();
        // Spin once calls reset automatically if it was not successful
        const auto& nodeObj = db.abi_node();
        if (!state.isInitialized())
        {
            const GraphContextObj& context = db.abi_context();
            // Find our stage
            long stageId = context.iContext->getStageId(context);
            auto stage = pxr::UsdUtilsStageCache::Get().Find(pxr::UsdStageCache::Id::FromLongInt(stageId));

            if (!state.initializeNodeHandle(
                    std::string(nodeObj.iNode->getPrimPath(nodeObj)),
                    collectNamespace(db.inputs.nodeNamespace(),
                                     stage->GetPrimAtPath(pxr::SdfPath(nodeObj.iNode->getPrimPath(nodeObj)))),
                    db.inputs.context()))
            {
                db.logError("Unable to create ROS2 node, please check that namespace is valid");
                return false;
            }
        }

        // Publisher was not valid, create a new one
        if (!state.m_publisher)
        {
            CARB_PROFILE_ZONE(0, "setup publisher");
            // Setup ROS publisher
            const std::string& topicName = db.inputs.topicName();
            std::string fullTopicName = addTopicPrefix(state.m_namespaceName, topicName);
            if (!state.m_factory->validateTopicName(fullTopicName))
            {
                db.logError("Unable to create ROS2 publisher, invalid topic name");
                return false;
            }

            // Get extension settings for multithreading
            carb::settings::ISettings* threadSettings = carb::getCachedInterface<carb::settings::ISettings>();
            static constexpr char s_kThreadDisable[] = "/exts/isaacsim.ros2.bridge/publish_multithreading_disabled";
            state.m_multithreadingDisabled = threadSettings->getAsBool(s_kThreadDisable);
            // Get extension settings for nitros bridge
            static constexpr char s_kNitrosBridgeEnabled[] = "/exts/isaacsim.ros2.bridge/enable_nitros_bridge";
            state.m_nitrosBridgeEnabled = threadSettings->getAsBool(s_kNitrosBridgeEnabled);

            state.m_message = state.m_factory->createImageMessage();

            Ros2QoSProfile qos;
            const std::string& qosProfile = db.inputs.qosProfile();
            if (qosProfile.empty())
            {
                qos.depth = db.inputs.queueSize();
            }
            else
            {
                if (!jsonToRos2QoSProfile(qos, qosProfile))
                {
                    return false;
                }
            }

            state.m_publisher = state.m_factory->createPublisher(
                state.m_nodeHandle.get(), fullTopicName.c_str(), state.m_message->getTypeSupportHandle(), qos);

            if (state.m_nitrosBridgeEnabled)
            {
                state.m_nitrosBridgeMessage = state.m_factory->createNitrosBridgeImageMessage();
                if (state.m_nitrosBridgeMessage && state.m_nitrosBridgeMessage->getPtr())
                {
                    state.m_nitrosBridgePublisher = state.m_factory->createPublisher(
                        state.m_nodeHandle.get(), (fullTopicName + "/nitros_bridge").c_str(),
                        state.m_nitrosBridgeMessage->getTypeSupportHandle(), qos);
                }
                else
                {
                    CARB_LOG_INFO(
                        "isaac_ros_nitros_bridge_interfaces NitrosBridgeImage message type not found. The NITROS bridge publisher was not created");
                }
            }

            state.m_frameId = db.inputs.frameId();

            return true;
        }

        bool status;
        status = state.publishImage(db);
        if (state.m_nitrosBridgePublisher)
        {
            state.publishNitrosBridgeImage(db);
        }
        return status;
    }

    bool publishImage(OgnROS2PublishImageDatabase& db)
    {
        CARB_PROFILE_ZONE(1, "publish image function");
        auto& state = db.perInstanceState<OgnROS2PublishImage>();
        auto tasking = carb::getCachedInterface<carb::tasking::ITasking>();

        {
            CARB_PROFILE_ZONE(1, "wait for previous publish");
            // Wait for last message to publish before starting next
            state.m_tasks.wait();
        }
        // Check if subscription count is 0
        if (!state.m_publishWithoutVerification && !state.m_publisher.get()->getSubscriptionCount())
        {
            return false;
        }

        state.m_message->writeHeader(db.inputs.timeStamp(), state.m_frameId);

        if (db.inputs.width() == 0 || db.inputs.height() == 0)
        {
            db.logError("Width %d or height %d is not valid", db.inputs.width(), db.inputs.height());
            return false;
        }

        std::string encoding = db.tokenToString(db.inputs.encoding());
        state.m_message->generateBuffer(db.inputs.height(), db.inputs.width(), encoding);
        size_t totalBytes = state.m_message->getTotalBytes();
        void* dataPtr = state.m_message->getBufferPtr();

        if (db.inputs.cudaDeviceIndex() == -1)
        {
            CARB_PROFILE_ZONE(1, "Data on host");
            if (db.inputs.dataPtr() != 0 && totalBytes == db.inputs.bufferSize())
            {
                // Data is on host as ptr, buffer size matches
                memcpy(dataPtr, reinterpret_cast<void*>(db.inputs.dataPtr()), totalBytes);
            }
            else if (db.inputs.dataPtr() == 0 && totalBytes == db.inputs.data.size())
            {
                // Data is on host as ogn data, copy from cpu
                memcpy(dataPtr, reinterpret_cast<const uint8_t*>(db.inputs.data.cpu().data()), totalBytes);
            }
            else
            {
                db.logError("image format and expected size %d bytes does not match input buffer Size of %d bytes",
                            totalBytes, db.inputs.bufferSize());
                db.logError("dataPtr null and expected size %d bytes does not match input data Size of %d bytes",
                            totalBytes, db.inputs.data.size());
                return false;
            }

            if (state.m_multithreadingDisabled)
            {
                CARB_PROFILE_ZONE(1, "image publisher publish");
                state.m_publisher.get()->publish(state.m_message->getPtr());
            }
            else
            {
                tasking->addTask(carb::tasking::Priority::eHigh, state.m_tasks,
                                 [&state]
                                 {
                                     CARB_PROFILE_ZONE(1, "image publisher publish");
                                     state.m_publisher.get()->publish(state.m_message->getPtr());
                                 });
            }
        }
        else
        {
            PublishImageThreadData publishImageThreadData = buildThreadData(db, state, dataPtr, totalBytes);
            if (state.m_multithreadingDisabled)
            {
                return publishImageHelper(publishImageThreadData);
            }
            else
            {
                // In order to get the benefits of using a separate stream, do the work in a new thread
                tasking->addTask(carb::tasking::Priority::eHigh, state.m_tasks,
                                 [data = publishImageThreadData]() mutable { return publishImageHelper(data); });
            }
        }
        return true;
    }

    PublishImageThreadData buildThreadData(OgnROS2PublishImageDatabase& db,
                                           OgnROS2PublishImage& state,
                                           void* dataPtr,
                                           size_t totalBytes)
    {
        PublishImageThreadData threadData;

        threadData.inputDataPtr = reinterpret_cast<void*>(db.inputs.dataPtr());
        threadData.outputDataPtr = dataPtr;
        threadData.resourceFormat = static_cast<carb::Format>(db.inputs.format());
        threadData.width = db.inputs.width();
        threadData.height = db.inputs.height();
        threadData.bufferSize = db.inputs.bufferSize();
        threadData.totalBytes = totalBytes;
        threadData.cudaDeviceIndex = db.inputs.cudaDeviceIndex();

        threadData.stream = &state.m_stream;
        threadData.streamDevice = &state.m_streamDevice;
        threadData.mStreamNotCreated = &state.m_streamNotCreated;

        threadData.publisher = state.m_publisher;
        threadData.message = state.m_message;

        return threadData;
    }

    static bool publishImageHelper(PublishImageThreadData& data)
    {
        CARB_PROFILE_ZONE(1, "Publish Image Thread");
        isaacsim::core::includes::ScopedDevice scopedDev(data.cudaDeviceIndex);

        // If the device doesn't match and we have created a stream, destroy it
        if (*data.streamDevice != data.cudaDeviceIndex && *data.mStreamNotCreated == false)
        {
            CARB_PROFILE_ZONE(1, "Destroy stream");
            CUDA_CHECK(cudaStreamDestroy(*data.stream));
            *data.mStreamNotCreated = true;
            *data.streamDevice = -1;
        }
        // Create a stream if it does not exist
        if (*data.mStreamNotCreated)
        {
            CARB_PROFILE_ZONE(1, "Create stream");
            CUDA_CHECK(cudaStreamCreate(data.stream));
            *data.mStreamNotCreated = false;
            *data.streamDevice = data.cudaDeviceIndex;
        }

        if (data.bufferSize == 0)
        {
            CARB_PROFILE_ZONE(1, "data in gpu texture");
            cudaArray_t levelArray = 0;
            CUDA_CHECK(
                cudaGetMipmappedArrayLevel(&levelArray, reinterpret_cast<cudaMipmappedArray_t>(data.inputDataPtr), 0));
            switch (static_cast<carb::Format>(data.resourceFormat))
            {
            case carb::Format::eR32_SFLOAT:
                if (data.width * data.height * sizeof(float) != data.totalBytes)
                {
                    CARB_LOG_ERROR("totalBytes doesn't match eR32_SFLOAT %zu %zu",
                                   data.width * data.height * sizeof(float), data.totalBytes);
                }
                else
                {
                    CUDA_CHECK(cudaMemcpy2DFromArrayAsync(data.outputDataPtr, data.width * sizeof(float), levelArray, 0,
                                                          0, data.width * sizeof(float), data.height,
                                                          cudaMemcpyDeviceToHost, *data.stream));
                    CUDA_CHECK(cudaStreamSynchronize(*data.stream));
                }
                break;

            default:
                CARB_LOG_ERROR("SdRenderVarToRawArray : input texture format (%d) is not supported.",
                               static_cast<int>(data.resourceFormat));
                return false;
            }
        }
        else
        {
            CARB_PROFILE_ZONE(1, "data in cuda memory");
            CUDA_CHECK(cudaMemcpyAsync(data.outputDataPtr, reinterpret_cast<void*>(data.inputDataPtr), data.bufferSize,
                                       cudaMemcpyDeviceToHost, *data.stream));
            CUDA_CHECK(cudaStreamSynchronize(*data.stream));
        }

        {
            CARB_PROFILE_ZONE(1, "image publisher publish");
            data.publisher.get()->publish(data.message->getPtr());
        }
        return true;
    }

    PublishNitrosBridgeImageThreadData buildNitrosBridgeThreadData(OgnROS2PublishImageDatabase& db,
                                                                   OgnROS2PublishImage& state,
                                                                   void* dataPtr,
                                                                   size_t totalBytes)
    {
        PublishNitrosBridgeImageThreadData threadData;

        threadData.inputDataPtr = reinterpret_cast<void*>(db.inputs.dataPtr());
        threadData.outputDataPtr = dataPtr;
        threadData.resourceFormat = static_cast<carb::Format>(db.inputs.format());
        threadData.width = db.inputs.width();
        threadData.height = db.inputs.height();
        threadData.bufferSize = db.inputs.bufferSize();
        threadData.totalBytes = totalBytes;
        threadData.cudaDeviceIndex = db.inputs.cudaDeviceIndex();

        threadData.nitrosBridgeStream = &state.m_nitrosBridgeStream;
        threadData.nitrosBridgeStreamDevice = &state.m_nitrosBridgeStreamDevice;
        threadData.nitrosBridgeStreamNotCreated = &state.m_nitrosBridgeStreamNotCreated;

#if !defined(_WIN32)
        threadData.ipcBufferManager = state.m_ipcBufferManager;
#endif

        threadData.nitrosBridgePublisher = state.m_nitrosBridgePublisher;
        threadData.nitrosBridgeMessage = state.m_nitrosBridgeMessage;

        return threadData;
    }

    void publishNitrosBridgeImage(OgnROS2PublishImageDatabase& db)
    {
#if !defined(_WIN32)
        CARB_PROFILE_ZONE(1, "publish nitros bridge image function");
        auto& state = db.perInstanceState<OgnROS2PublishImage>();
        auto tasking = carb::getCachedInterface<carb::tasking::ITasking>();

        {
            CARB_PROFILE_ZONE(1, "wait for previous publish");
            // Wait for last message to publish before starting next
            state.m_nitrosBridgeTasks.wait();
        }
        // Check if subscription count is 0
        if (!m_publishWithoutVerification && !state.m_nitrosBridgePublisher.get()->getSubscriptionCount())
        {
            return;
        }

        state.m_nitrosBridgeMessage->writeHeader(db.inputs.timeStamp(), state.m_frameId);

        if (db.inputs.width() == 0 || db.inputs.height() == 0)
        {
            db.logError("Width %d or height %d is not valid", db.inputs.width(), db.inputs.height());
            return;
        }

        std::string encoding = db.tokenToString(db.inputs.encoding());
        state.m_nitrosBridgeMessage->generateBuffer(db.inputs.height(), db.inputs.width(), encoding);
        size_t totalBytes = state.m_nitrosBridgeMessage->getTotalBytes();

        // IPC manager initialization
        // If initialization fails, the NITROS publisher will be disabled for this node instance
        if (!state.m_ipcBufferManager)
        {
            if (totalBytes == 0)
            {
                db.logWarning(
                    "NITROS Bridge: Cannot initialize IPCBufferManager with zero totalBytes. Disabling NITROS publisher for this node instance.");
                state.m_nitrosBridgePublisher.reset();
                return;
            }
            try
            {
                state.m_ipcBufferManager = std::make_shared<IPCBufferManager>(40, totalBytes);
            }
            catch (const std::runtime_error& e)
            {
                db.logWarning(
                    "NITROS Bridge: Failed to initialize IPCBufferManager. Disabling NITROS publisher for this node instance.");
                state.m_nitrosBridgePublisher.reset();
                return;
            }
        }

        void* dataPtr = (void*)state.m_ipcBufferManager->getCurBufferPtr();

        // Data on host
        if (db.inputs.cudaDeviceIndex() == -1)
        {
            CARB_PROFILE_ZONE(1, "Data on host");
            // Data is on host as ptr, buffer size matches
            if (db.inputs.dataPtr() != 0 && totalBytes == db.inputs.bufferSize())
            {
                CUDA_CHECK(cudaMemcpy(
                    dataPtr, reinterpret_cast<const void*>(db.inputs.dataPtr()), totalBytes, cudaMemcpyHostToDevice));
            }
            // Data is on host as ogn data, copy from CPU
            else if (db.inputs.dataPtr() == 0 && totalBytes == db.inputs.data.size())
            {
                CUDA_CHECK(cudaMemcpy(dataPtr, reinterpret_cast<const uint8_t*>(db.inputs.data.cpu().data()),
                                      totalBytes, cudaMemcpyHostToDevice));
            }
            else
            {
                db.logError("image format and expected size %d bytes does not match input buffer Size of %d bytes",
                            totalBytes, db.inputs.bufferSize());
                db.logError("dataPtr null and expected size %d bytes does not match input data Size of %d bytes",
                            totalBytes, db.inputs.data.size());
                return;
            }

            state.m_nitrosBridgeMessage->writeData(state.m_ipcBufferManager->getCurIpcMemHandle());
            state.m_ipcBufferManager->next();

            if (state.m_multithreadingDisabled)
            {
                CARB_PROFILE_ZONE(1, "nitros image publisher publish");
                state.m_nitrosBridgePublisher.get()->publish(state.m_nitrosBridgeMessage->getPtr());
            }
            else
            {
                tasking->addTask(carb::tasking::Priority::eHigh, state.m_nitrosBridgeTasks,
                                 [&state]
                                 {
                                     CARB_PROFILE_ZONE(1, "nitros image publisher publish");
                                     state.m_nitrosBridgePublisher.get()->publish(state.m_nitrosBridgeMessage->getPtr());
                                 });
            }
        }
        // Data on device
        else
        {
            PublishNitrosBridgeImageThreadData publishNitrosBridgeImageThreadData =
                buildNitrosBridgeThreadData(db, state, dataPtr, totalBytes);

            if (state.m_multithreadingDisabled)
            {
                publishNitrosBridgeHelper(publishNitrosBridgeImageThreadData);
            }
            else
            {
                // In order to get the benefits of using a separate stream, do all of the work in a new thread
                tasking->addTask(carb::tasking::Priority::eHigh, state.m_tasks,
                                 [data = publishNitrosBridgeImageThreadData]() mutable
                                 { publishNitrosBridgeHelper(data); });
            }
        }
#endif
    }

    static void publishNitrosBridgeHelper(PublishNitrosBridgeImageThreadData& data)
    {
#if !defined(_WIN32)
        CARB_PROFILE_ZONE(1, "publish nitros image thread");
        isaacsim::core::includes::ScopedDevice scopedDev(data.cudaDeviceIndex);

        // If the device doesn't match and we have created a stream, destroy it
        if (*data.nitrosBridgeStreamDevice != data.cudaDeviceIndex && *data.nitrosBridgeStreamNotCreated == false)
        {
            CARB_PROFILE_ZONE(1, "Destroy stream");
            cudaStreamDestroy(*data.nitrosBridgeStream);
            *data.nitrosBridgeStreamNotCreated = true;
            *data.nitrosBridgeStreamDevice = -1;
        }
        // Create a stream if it does not exist
        if (*data.nitrosBridgeStreamNotCreated)
        {
            CARB_PROFILE_ZONE(1, "Create stream");
            cudaStreamCreate(&*data.nitrosBridgeStream);
            *data.nitrosBridgeStreamNotCreated = false;
            *data.nitrosBridgeStreamDevice = data.cudaDeviceIndex;
        }

        // Data in gpu texture
        if (data.bufferSize == 0)
        {
            CARB_PROFILE_ZONE(1, "data in gpu texture");
            cudaArray_t levelArray = 0;
            CUDA_CHECK(
                cudaGetMipmappedArrayLevel(&levelArray, reinterpret_cast<cudaMipmappedArray_t>(data.inputDataPtr), 0));
            switch (static_cast<carb::Format>(data.resourceFormat))
            {
            case carb::Format::eR32_SFLOAT:
                if (data.width * data.height * sizeof(float) != data.totalBytes)
                {
                    CARB_LOG_ERROR("totalBytes doesn't match eR32_SFLOAT %zu %zu",
                                   data.width * data.height * sizeof(float), data.totalBytes);
                }
                else
                {
                    CUDA_CHECK(cudaMemcpy2DFromArrayAsync(data.outputDataPtr, data.width * sizeof(float), levelArray, 0,
                                                          0, data.width * sizeof(float), data.height,
                                                          cudaMemcpyDeviceToDevice, *data.nitrosBridgeStream));
                    CUDA_CHECK(cudaStreamSynchronize(*data.nitrosBridgeStream));
                }
                break;
            default:
                CARB_LOG_ERROR("SdRenderVarToRawArray : input texture format (%d) is not supported.",
                               static_cast<int>(data.resourceFormat));
                return;
            }
        }
        // Data in CUDA memory
        else
        {
            CARB_PROFILE_ZONE(1, "data in cuda memory");
            CUDA_CHECK(cudaMemcpyAsync(data.outputDataPtr, reinterpret_cast<void*>(data.inputDataPtr), data.bufferSize,
                                       cudaMemcpyDeviceToDevice, *data.nitrosBridgeStream));
            CUDA_CHECK(cudaStreamSynchronize(*data.nitrosBridgeStream));
        }

        data.nitrosBridgeMessage->writeData(data.ipcBufferManager->getCurIpcMemHandle());
        data.ipcBufferManager->next();
        {
            CARB_PROFILE_ZONE(1, "nitros image publisher publish");
            data.nitrosBridgePublisher.get()->publish(data.nitrosBridgeMessage->getPtr());
        }
#endif
    }

    static void releaseInstance(NodeObj const& nodeObj, GraphInstanceID instanceId)
    {
        auto& state = OgnROS2PublishImageDatabase::sPerInstanceState<OgnROS2PublishImage>(nodeObj, instanceId);
        state.reset();
    }

    virtual void reset()
    {
        {
            CARB_PROFILE_ZONE(1, "wait for previous publish");
            // Wait for last message to publish before starting next
            m_tasks.wait();
            m_nitrosBridgeTasks.wait();
        }
        if (m_streamNotCreated == false)
        {
            isaacsim::core::includes::ScopedDevice scopedDev(m_streamDevice);
            CUDA_CHECK(cudaStreamDestroy(m_stream));
            m_streamDevice = -1;
            m_streamNotCreated = true;
        }
        if (m_nitrosBridgeStreamNotCreated == false)
        {
            isaacsim::core::includes::ScopedDevice scopedDev(m_nitrosBridgeStreamDevice);
            CUDA_CHECK(cudaStreamDestroy(m_nitrosBridgeStream));
            m_nitrosBridgeStreamDevice = -1;
            m_nitrosBridgeStreamNotCreated = true;
        }

#if !defined(_WIN32)
        m_ipcBufferManager.reset();
        m_ipcBufferManager = nullptr;
#endif
        m_nitrosBridgePublisher.reset();
        m_nitrosBridgePublisher = nullptr;

        m_publisher.reset(); // This should be reset before we reset the handle.
        Ros2Node::reset();
    }

private:
    std::shared_ptr<Ros2Publisher> m_publisher = nullptr;
    std::shared_ptr<Ros2ImageMessage> m_message = nullptr;

#if !defined(_WIN32)
    std::shared_ptr<IPCBufferManager> m_ipcBufferManager = nullptr; // CUDA IPC memory pool manager
#endif
    std::shared_ptr<Ros2Publisher> m_nitrosBridgePublisher = nullptr;
    std::shared_ptr<Ros2NitrosBridgeImageMessage> m_nitrosBridgeMessage = nullptr;

    std::string m_frameId = "sim_camera";

    carb::tasking::TaskGroup m_tasks;
    cudaStream_t m_stream;
    int m_streamDevice = -1;
    bool m_streamNotCreated = true;

    carb::tasking::TaskGroup m_nitrosBridgeTasks;
    cudaStream_t m_nitrosBridgeStream;
    int m_nitrosBridgeStreamDevice = -1;
    bool m_nitrosBridgeStreamNotCreated = true;

    bool m_multithreadingDisabled = false;
    bool m_nitrosBridgeEnabled = false;
};

REGISTER_OGN_NODE()
