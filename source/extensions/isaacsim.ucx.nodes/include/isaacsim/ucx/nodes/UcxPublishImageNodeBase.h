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

#pragma once

#include <isaacsim/core/includes/Buffer.h>
#include <isaacsim/core/includes/ScopedCudaDevice.h>
#include <isaacsim/ucx/core/UcxListenerRegistry.h>
#include <isaacsim/ucx/nodes/UcxNode.h>
#include <omni/graph/core/CppWrappers.h>
#include <omni/graph/core/iComputeGraph.h>
#include <ucxx/api.h>

#include <cstring>
#include <string>
#include <thread>
#include <vector>

using omni::graph::core::GraphInstanceID;
using omni::graph::core::NodeObj;

namespace isaacsim::ucx::nodes
{

/**
 * @struct ImageMetadata
 * @brief Metadata structure for image messages (non-pixel data).
 * @details
 * Contains image metadata extracted from node inputs. Pixel data is handled
 * separately to allow flexibility for CPU or GPU sources.
 */
struct ImageMetadata
{
    double timestamp; //!< Timestamp value in seconds
    uint32_t width; //!< Image width in pixels
    uint32_t height; //!< Image height in pixels
    std::string encoding; //!< Image encoding (e.g., "rgb8", "rgba8")
    uint32_t step; //!< Row length in bytes
    size_t dataSize; //!< Size of pixel data in bytes
};

} // namespace isaacsim::ucx::nodes

/**
 * @class UCXPublishImageNodeBase
 * @brief Templated base class for UCX image data publishing nodes.
 * @details
 * This template provides common functionality for publishing camera image data over UCX.
 * It handles CUDA stream management for GPU-based image data.
 * Derived classes implement message generation logic via generateMessage().
 *
 * @tparam DatabaseT The OGN database type for the node
 */
template <typename DatabaseT>
class UCXPublishImageNodeBase : public isaacsim::ucx::nodes::UcxNode
{
public:
    /**
     * @brief Initialize the node instance.
     * @details
     * Default implementation - can be overridden by derived classes if needed.
     *
     * @param[in] nodeObj The node object
     * @param[in] instanceId The instance ID
     */
    static void initInstance(NodeObj const& nodeObj, GraphInstanceID instanceId)
    {
        // Default: no initialization needed
    }

    /**
     * @brief Reset the node state.
     * @details
     * Cleans up CUDA resources and resets parent state.
     */
    virtual void reset() override
    {
        // Clear any pending async send request before the listener is destroyed.
        // The request may hold the last reference to the ucxx::Endpoint; destroying
        // it after the UCXListener is gone would trigger a close callback on a
        // dangling pointer, crashing in onEndpointClosed().
        m_sendRequest.reset();
        m_tensorSendRequest.reset();
        m_messageBuffer.clear();

        if (m_streamNotCreated == false)
        {
            isaacsim::core::includes::ScopedDevice scopedDev(m_streamDevice);
            CUDA_CHECK(cudaStreamDestroy(m_stream));
            m_streamDevice = -1;
            m_streamNotCreated = true;
        }
        UcxNode::reset();
    }

protected:
    /**
     * @brief Common compute logic for image publishing nodes.
     * @details
     * Handles listener initialization, connection checking, input validation,
     * and message publishing. Extracts metadata using extractMetadata() and
     * delegates message generation to derived class via generateMessage().
     *
     * @param[in] db Database accessor for node inputs/outputs
     * @param[in] port Port number for UCX listener
     * @param[in] tag UCX tag for message identification
     * @return bool True if execution succeeded, false otherwise
     */
    bool computeImpl(DatabaseT& db, uint16_t port, uint64_t tag)
    {
        if (!this->ensureListenerReady(db, port))
        {
            return false;
        }

        if (!this->waitForConnection())
        {
            return true;
        }

        if (db.inputs.width() == 0 || db.inputs.height() == 0)
        {
            db.logError("Width %d or height %d is not valid", db.inputs.width(), db.inputs.height());
            return false;
        }

        if (db.inputs.dataPtr() == 0 && db.inputs.data.size() == 0)
        {
            db.logError("Image data is empty (both dataPtr and data array are empty)");
            return false;
        }

        isaacsim::ucx::nodes::ImageMetadata metadata = extractMetadata(db);
        return publishMessage(db, generateMessage(metadata, db), tag);
    }

    /**
     * @brief Extract image metadata from node inputs.
     * @details
     * Extracts non-pixel metadata from the database inputs. Pixel data is handled
     * separately by generateMessage() to allow flexibility for CPU or GPU sources.
     *
     * @param[in] db Database accessor for node inputs
     * @return ImageMetadata Extracted image metadata
     */
    virtual isaacsim::ucx::nodes::ImageMetadata extractMetadata(DatabaseT& db)
    {
        isaacsim::ucx::nodes::ImageMetadata metadata;
        metadata.timestamp = db.inputs.timeStamp();
        metadata.width = db.inputs.width();
        metadata.height = db.inputs.height();
        metadata.encoding = db.tokenToString(db.inputs.encoding());

        // Determine data size from inputs
        const size_t bufferSize = db.inputs.bufferSize();
        if (bufferSize > 0)
        {
            metadata.dataSize = bufferSize;
        }
        else if (db.inputs.data.size() > 0)
        {
            metadata.dataSize = db.inputs.data.size();
        }
        else
        {
            metadata.dataSize = 0;
        }

        // Calculate step (bytes per row)
        if (metadata.height > 0 && metadata.dataSize > 0)
        {
            metadata.step = static_cast<uint32_t>(metadata.dataSize / metadata.height);
        }
        else
        {
            metadata.step = 0;
        }

        return metadata;
    }

    /**
     * @brief Generate message from image metadata and pixel data.
     * @details
     * Pure virtual function that derived classes must implement to serialize
     * image data. Has access to both metadata and db for flexible pixel handling
     * (CPU copy, GPU buffer, GPU texture, or GPU-direct send).
     *
     * @param[in] metadata Image metadata
     * @param[in] db Database accessor for pixel data access
     * @return std::vector<uint8_t> Serialized message data
     */
    virtual std::vector<uint8_t> generateMessage(const isaacsim::ucx::nodes::ImageMetadata& metadata, DatabaseT& db) = 0;

    /**
     * @brief Publishes an image message over UCX.
     * @details
     * Sends the pre-generated message using UCX tagged send with async tracking.
     * Images are large (>1MB typically) so we use async send with completion tracking
     * to prevent race conditions between send and receive operations.
     *
     * @param[in] db Database accessor for logging
     * @param[in] messageData Serialized message to send
     * @param[in] tag UCX tag for message identification
     * @return bool True if publish succeeded, false otherwise
     */
    bool publishMessage(DatabaseT& db, std::vector<uint8_t> messageData, uint64_t tag)
    {
        // Check if previous async send is still in progress
        if (m_sendRequest && !m_sendRequest->isCompleted())
        {
            // Previous send still in progress - skip this frame to avoid corrupting buffer
            // This prevents buffer reuse while async transfer is ongoing
            return true;
        }

        if (messageData.empty())
        {
            db.logError("Failed to generate message - empty message data");
            return false;
        }

        // Store in persistent buffer to keep alive during async send
        m_messageBuffer = std::move(messageData);

        // Send async and get request handle to track completion
        std::string errorMessage;
        auto result = this->m_listener->tagSendWithRequest(
            m_messageBuffer.data(), m_messageBuffer.size(), tag, errorMessage, m_sendRequest);

        if (result != isaacsim::ucx::core::UcxSendResult::eSuccess)
        {
            db.logError("tagSendWithRequest failed: %s", errorMessage.c_str());
            return false;
        }

        return true;
    }

    /**
     * @brief Get the CUDA stream for asynchronous operations.
     * @details
     * Provides access to the managed CUDA stream for GPU operations.
     *
     * @return cudaStream_t The CUDA stream
     */
    cudaStream_t getCudaStream()
    {
        return m_stream;
    }

    /**
     * @brief Get the current CUDA stream device index.
     * @details
     * Returns the device index that the stream was created on.
     *
     * @return int Device index, or -1 if stream not created
     */
    int getCudaStreamDevice() const
    {
        return m_streamDevice;
    }

    /**
     * @brief Check if CUDA stream needs to be created.
     * @details
     * Returns true if no stream has been created yet.
     *
     * @return bool True if stream needs creation, false otherwise
     */
    bool isStreamNotCreated() const
    {
        return m_streamNotCreated;
    }

    /**
     * @brief Ensure CUDA stream is created for the specified device.
     * @details
     * Creates or recreates the CUDA stream if necessary for the given device index.
     *
     * @param[in] deviceIndex CUDA device index to use
     */
    void ensureCudaStream(int deviceIndex)
    {
        if (m_streamDevice != deviceIndex && m_streamNotCreated == false)
        {
            CUDA_CHECK(cudaStreamDestroy(m_stream));
            m_streamNotCreated = true;
            m_streamDevice = -1;
        }
        if (m_streamNotCreated)
        {
            CUDA_CHECK(cudaStreamCreate(&m_stream));
            m_streamNotCreated = false;
            m_streamDevice = deviceIndex;
        }
    }

    cudaStream_t m_stream; //!< CUDA stream for asynchronous operations
    int m_streamDevice = -1; //!< Device index where stream was created
    bool m_streamNotCreated = true; //!< Flag indicating if stream needs creation
    std::vector<uint8_t> m_messageBuffer; //!< Persistent buffer to keep message alive during async send
    std::shared_ptr<ucxx::Request> m_sendRequest; //!< Request handle to track async send completion
    std::shared_ptr<ucxx::Request> m_tensorSendRequest; //!< Request handle for GPU tensor send (GPU-direct path)
};

// NOTE: To use this base class:
// 1. Derive your OGN node class from UCXPublishImageNodeBase<YourDatabase>
// 2. Implement static void releaseInstance(NodeObj const& nodeObj, GraphInstanceID instanceId)
//    - Get state: auto& state = YourDatabase::template sPerInstanceState<YourClass>(nodeObj, instanceId)
//    - Call state.reset()
// 3. Implement static bool compute(YourDatabase& db) that:
//    - Extracts inputs from db (port, tag)
//    - Gets the per-instance state: auto& state = db.template perInstanceState<YourClass>()
//    - Calls state.computeImpl(db, port, tag)
// 4. Implement virtual std::vector<uint8_t> generateMessage(const ImageMetadata& metadata, DatabaseT& db) override
//    - Serialize metadata and handle pixel data (CPU copy, GPU copy, or GPU-direct)
// 5. Optionally override extractMetadata() if different metadata extraction is needed
// 6. See OgnUCXPublishImage.cpp for examples
