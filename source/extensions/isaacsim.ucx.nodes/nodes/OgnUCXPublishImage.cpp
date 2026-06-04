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


#include <carb/RenderingTypes.h>

#include <flatbuffers/flatbuffers.h>
#include <isaacsim/ucx/nodes/UcxPublishImageNodeBase.h>

#include <OgnUCXPublishImageDatabase.h>
#include <image_generated.h>

/**
 * @class OgnUCXPublishImage
 * @brief OmniGraph node for publishing camera images via UCX.
 * @details
 * This node publishes camera image data over UCX using tagged communication.
 * It supports RGB8, RGBA8, BGR8, BGRA8, and other image formats.
 * Messages are serialized as FlatBuffers Image messages.
 * GPU sources are copied to CPU before serialization.
 */
class OgnUCXPublishImage : public UCXPublishImageNodeBase<OgnUCXPublishImageDatabase>
{
public:
    /**
     * @brief Release the node instance.
     * @details
     * Cleans up resources when the node instance is destroyed.
     *
     * @param[in] nodeObj The node object
     * @param[in] instanceId The instance ID
     */
    static void releaseInstance(NodeObj const& nodeObj, GraphInstanceID instanceId)
    {
        auto& state = OgnUCXPublishImageDatabase::sPerInstanceState<OgnUCXPublishImage>(nodeObj, instanceId);
        state.reset();
    }

    /**
     * @brief Compute function - called when node is executed.
     * @details
     * Routes to GPU-direct two-message path when GPU buffer data is available,
     * otherwise falls back to the CPU-copy path.
     *
     * @param[in] db Database accessor for node inputs/outputs
     * @return bool True if execution succeeded, false otherwise
     */
    static bool compute(OgnUCXPublishImageDatabase& db)
    {
        const uint16_t port = static_cast<uint16_t>(db.inputs.port());
        const uint64_t tag = db.inputs.tag();
        auto& state = db.template perInstanceState<OgnUCXPublishImage>();

        const bool gpuBuffer = db.inputs.cudaDeviceIndex() != -1 && db.inputs.bufferSize() > 0;
        if (gpuBuffer && db.inputs.sendCudaBuffer())
        {
            return state.computeGpuDirect(db, port, tag);
        }
        return state.computeImpl(db, port, tag);
    }

    /**
     * @brief GPU-direct compute path: two-message protocol (metadata FlatBuffer + raw GPU tensor).
     * @details
     * Sends a metadata-only FlatBuffer (empty pixel bytes, kDLCUDA device) followed by the raw
     * GPU buffer as a second tagged send. The receiver must use tensor_buffer_size > 0 to post
     * a device-memory tagRecv for the second message. Eliminates the GPU-to-CPU copy.
     *
     * @param[in] db Database accessor for node inputs/outputs
     * @param[in] port UCX port
     * @param[in] tag UCX tag
     * @return bool True if execution succeeded, false otherwise
     */
    bool computeGpuDirect(OgnUCXPublishImageDatabase& db, uint16_t port, uint64_t tag)
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
        if (db.inputs.dataPtr() == 0)
        {
            db.logError("GPU data pointer is null");
            return false;
        }

        // Skip frame if either previous async send is still in progress.
        if ((m_sendRequest && !m_sendRequest->isCompleted()) ||
            (m_tensorSendRequest && !m_tensorSendRequest->isCompleted()))
        {
            return true;
        }

        isaacsim::ucx::nodes::ImageMetadata metadata = extractMetadata(db);
        std::vector<uint8_t> metaMsg = generateMetadataMessage(metadata, db.inputs.cudaDeviceIndex());
        if (metaMsg.empty())
        {
            db.logError("Failed to generate GPU-direct metadata message");
            return false;
        }

        // Send metadata FlatBuffer (CPU memory, small).
        m_messageBuffer = std::move(metaMsg);
        std::string errorMessage;
        auto result = this->m_listener->tagSendWithRequest(
            m_messageBuffer.data(), m_messageBuffer.size(), tag, errorMessage, m_sendRequest);
        if (result != isaacsim::ucx::core::UcxSendResult::eSuccess)
        {
            db.logError("GPU-direct metadata tagSend failed: %s", errorMessage.c_str());
            return false;
        }

        // Send raw GPU tensor buffer directly — no CPU copy.
        // Same tag as metadata: UCX guarantees in-order delivery on a tag, so the receiver
        // can predictably post tagRecv for metadata then tagRecv for the tensor.
        void* gpuPtr = reinterpret_cast<void*>(db.inputs.dataPtr());
        result =
            this->m_listener->tagSendWithRequest(gpuPtr, db.inputs.bufferSize(), tag, errorMessage, m_tensorSendRequest);
        if (result != isaacsim::ucx::core::UcxSendResult::eSuccess)
        {
            db.logError("GPU-direct tensor tagSend failed: %s", errorMessage.c_str());
            // Cancel the metadata send so the pair stays atomic. Without this, the
            // receiver would consume the orphan metadata and then interpret the next
            // frame's metadata as the missing tensor — sender/receiver desync.
            if (m_sendRequest)
            {
                m_sendRequest->cancel();
            }
            m_sendRequest.reset();
            m_tensorSendRequest.reset();
            return false;
        }

        return true;
    }

protected:
    /**
     * @brief Generate metadata-only FlatBuffer for the GPU-direct two-message protocol.
     * @details
     * Builds an Image FlatBuffer with empty pixel bytes and kDLCUDA device type.
     * The receiver replaces the data pointer with a separately received GPU tensor buffer.
     *
     * @param[in] metadata Image metadata (timestamp, dimensions, encoding)
     * @param[in] cudaDeviceIndex CUDA device index where the tensor lives
     * @return std::vector<uint8_t> Serialized metadata FlatBuffer
     */
    std::vector<uint8_t> generateMetadataMessage(const isaacsim::ucx::nodes::ImageMetadata& metadata, int cudaDeviceIndex)
    {
        flatbuffers::FlatBufferBuilder builder;

        // Empty pixel bytes — receiver will substitute the separately received GPU buffer.
        auto pixel_bytes = builder.CreateVector(std::vector<uint8_t>{});
        std::vector<int64_t> shape = { static_cast<int64_t>(metadata.dataSize) };
        auto shape_fb = builder.CreateVector(shape);
        isaac::DLDataType dtype(isaac::DLDataTypeCode_kDLUInt, 8, 1);
        // Mark device as CUDA so downstream consumers treat the tensor as GPU memory.
        isaac::DLDevice device(isaac::DLDeviceType_kDLCUDA, cudaDeviceIndex);
        std::vector<int64_t> strides = { 1 };
        auto strides_fb = builder.CreateVector(strides);
        auto data_tensor = isaac::CreateTensor(builder, pixel_bytes, shape_fb, &dtype, &device, 1, strides_fb);

        isaac::ImageEncoding encoding = isaac::ImageEncoding_CUSTOM;
        {
            const std::string& enc = metadata.encoding;
            if (enc == "rgb8")
                encoding = isaac::ImageEncoding_RGB8;
            else if (enc == "rgba8")
                encoding = isaac::ImageEncoding_RGBA8;
            else if (enc == "bgr8")
                encoding = isaac::ImageEncoding_BGR8;
            else if (enc == "bgra8")
                encoding = isaac::ImageEncoding_BGRA8;
            else if (enc == "r8_g8_b8")
                encoding = isaac::ImageEncoding_R8_G8_B8;
            else if (enc == "b8_g8_r8")
                encoding = isaac::ImageEncoding_B8_G8_R8;
            else if (enc == "mono8")
                encoding = isaac::ImageEncoding_MONO8;
            else if (enc == "mono16")
                encoding = isaac::ImageEncoding_MONO16;
            else if (enc == "mono32")
                encoding = isaac::ImageEncoding_MONO32;
            else if (enc == "mono32f")
                encoding = isaac::ImageEncoding_MONO32F;
        }

        const int64_t time_ns = static_cast<int64_t>(metadata.timestamp * 1e9);
        auto frame_id_fb = builder.CreateString("");
        auto stamp_fb = isaac::CreateTime(builder, time_ns, 0);
        auto header_fb = isaac::CreateHeader(builder, stamp_fb, frame_id_fb);
        auto image_fb = isaac::CreateImage(builder, header_fb, static_cast<int32_t>(metadata.height),
                                           static_cast<int32_t>(metadata.width), encoding, data_tensor);
        builder.Finish(image_fb);

        uint8_t const* bufPtr = builder.GetBufferPointer();
        return std::vector<uint8_t>(bufPtr, bufPtr + builder.GetSize());
    }
    /**
     * @brief Generate message from image metadata and pixel data.
     * @details
     * Serializes image metadata and data as a FlatBuffers Image message.
     * Pixel data is stored as a Tensor with dtype uint8 and shape [dataSize].
     * GPU sources are first copied to CPU memory, then serialized.
     *
     * @param[in] metadata Image metadata (timestamp, dimensions, encoding)
     * @param[in] db Database accessor for pixel data access
     * @return std::vector<uint8_t> Serialized FlatBuffers message, or empty on error
     */
    std::vector<uint8_t> generateMessage(const isaacsim::ucx::nodes::ImageMetadata& metadata,
                                         OgnUCXPublishImageDatabase& db) override
    {
        const int cudaDeviceIndex = db.inputs.cudaDeviceIndex();
        const size_t bufferSize = db.inputs.bufferSize();

        // Determine data size
        size_t dataSize = metadata.dataSize;
        if (dataSize == 0 && cudaDeviceIndex != -1)
        {
            // GPU texture — calculate size based on format
            const carb::Format resourceFormat = static_cast<carb::Format>(db.inputs.format());
            if (resourceFormat == carb::Format::eR32_SFLOAT)
            {
                dataSize = metadata.width * metadata.height * sizeof(float);
            }
            else
            {
                db.logError("GPU texture with unknown format: %d", static_cast<int>(resourceFormat));
                return {};
            }
        }

        if (dataSize == 0)
        {
            db.logError("Cannot determine image data size - no valid data source");
            return {};
        }

        // Collect pixel bytes into a CPU buffer
        std::vector<uint8_t> pixelBuf(dataSize);
        uint8_t* dest = pixelBuf.data();

        if (cudaDeviceIndex == -1)
        {
            // CPU source
            if (db.inputs.dataPtr() != 0 && bufferSize > 0)
            {
                std::memcpy(dest, reinterpret_cast<const void*>(db.inputs.dataPtr()), dataSize);
            }
            else if (db.inputs.data.size() > 0)
            {
                const auto& imageData = db.inputs.data.cpu();
                std::memcpy(dest, imageData.data(), dataSize);
            }
            else
            {
                db.logError("No valid CPU image data source available");
                return {};
            }
        }
        else
        {
            // GPU source — copy to host
            isaacsim::core::includes::ScopedDevice scopedDev(cudaDeviceIndex);
            ensureCudaStream(cudaDeviceIndex);

            if (bufferSize == 0)
            {
                // Data in GPU texture (mipmapped array)
                cudaArray_t levelArray = nullptr;
                CUDA_CHECK(cudaGetMipmappedArrayLevel(
                    &levelArray, reinterpret_cast<cudaMipmappedArray_t>(db.inputs.dataPtr()), 0));

                const carb::Format resourceFormat = static_cast<carb::Format>(db.inputs.format());
                switch (resourceFormat)
                {
                case carb::Format::eR32_SFLOAT:
                    if (metadata.width * metadata.height * sizeof(float) != dataSize)
                    {
                        db.logError("Data size mismatch for eR32_SFLOAT format");
                        return {};
                    }
                    CUDA_CHECK(cudaMemcpy2DFromArrayAsync(dest, metadata.width * sizeof(float), levelArray, 0, 0,
                                                          metadata.width * sizeof(float), metadata.height,
                                                          cudaMemcpyDeviceToHost, getCudaStream()));
                    CUDA_CHECK(cudaStreamSynchronize(getCudaStream()));
                    break;
                default:
                    db.logError("GPU texture format (%d) is not supported", static_cast<int>(resourceFormat));
                    return {};
                }
            }
            else
            {
                // Data in GPU buffer
                CUDA_CHECK(cudaMemcpyAsync(dest, reinterpret_cast<void*>(db.inputs.dataPtr()), bufferSize,
                                           cudaMemcpyDeviceToHost, getCudaStream()));
                CUDA_CHECK(cudaStreamSynchronize(getCudaStream()));
            }
        }

        // Map encoding string to ImageEncoding enum
        isaac::ImageEncoding encoding = isaac::ImageEncoding_CUSTOM;
        const std::string& enc = metadata.encoding;
        if (enc == "rgb8")
            encoding = isaac::ImageEncoding_RGB8;
        else if (enc == "rgba8")
            encoding = isaac::ImageEncoding_RGBA8;
        else if (enc == "bgr8")
            encoding = isaac::ImageEncoding_BGR8;
        else if (enc == "bgra8")
            encoding = isaac::ImageEncoding_BGRA8;
        else if (enc == "r8_g8_b8")
            encoding = isaac::ImageEncoding_R8_G8_B8;
        else if (enc == "b8_g8_r8")
            encoding = isaac::ImageEncoding_B8_G8_R8;
        else if (enc == "mono8")
            encoding = isaac::ImageEncoding_MONO8;
        else if (enc == "mono16")
            encoding = isaac::ImageEncoding_MONO16;
        else if (enc == "mono32")
            encoding = isaac::ImageEncoding_MONO32;
        else if (enc == "mono32f")
            encoding = isaac::ImageEncoding_MONO32F;

        // Build FlatBuffers message
        flatbuffers::FlatBufferBuilder builder;

        // Data Tensor: raw bytes as ubyte, dtype=uint8, shape=[dataSize]
        auto pixel_bytes = builder.CreateVector(dest, dataSize);
        std::vector<int64_t> shape = { static_cast<int64_t>(dataSize) };
        auto shape_fb = builder.CreateVector(shape);
        isaac::DLDataType dtype(isaac::DLDataTypeCode_kDLUInt, 8, 1);
        isaac::DLDevice device(isaac::DLDeviceType_kDLCPU, 0);
        std::vector<int64_t> strides = { 1 };
        auto strides_fb = builder.CreateVector(strides);
        auto data_tensor = isaac::CreateTensor(builder, pixel_bytes, shape_fb, &dtype, &device, 1, strides_fb);

        // Header
        const int64_t time_ns = static_cast<int64_t>(metadata.timestamp * 1e9);
        auto frame_id_fb = builder.CreateString("");
        auto stamp_fb = isaac::CreateTime(builder, time_ns, 0);
        auto header_fb = isaac::CreateHeader(builder, stamp_fb, frame_id_fb);

        // Image table
        auto image_fb = isaac::CreateImage(builder, header_fb, static_cast<int32_t>(metadata.height),
                                           static_cast<int32_t>(metadata.width), encoding, data_tensor);
        builder.Finish(image_fb);

        uint8_t const* bufPtr = builder.GetBufferPointer();
        return std::vector<uint8_t>(bufPtr, bufPtr + builder.GetSize());
    }
};


REGISTER_OGN_NODE()
