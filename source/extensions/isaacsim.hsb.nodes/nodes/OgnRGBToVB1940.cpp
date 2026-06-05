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

#include <carb/logging/Log.h>

#include <isaacsim/hsb/core/RGBToVB1940Kernels.h>

#include <OgnRGBToVB1940Database.h>
#include <cstdint>
#include <cstring>
#include <cuda_runtime.h>
#include <stdexcept>
#include <string>
#include <vector>

using namespace isaacsim::hsb::core;

namespace
{

// Reallocate a GPU buffer only when the current capacity is insufficient.
inline void ensureGPU(uint8_t*& ptr, size_t& cap, size_t needed)
{
    if (needed <= cap)
    {
        return;
    }
    if (ptr)
    {
        cudaFree(ptr);
        ptr = nullptr;
        cap = 0;
    }
    cudaError_t err = cudaMalloc(reinterpret_cast<void**>(&ptr), needed);
    if (err != cudaSuccess)
    {
        throw std::runtime_error(std::string("cudaMalloc failed: ") + cudaGetErrorString(err));
    }
    cap = needed;
}

} // anonymous namespace

class OgnRGBToVB1940
{
public:
    static bool compute(OgnRGBToVB1940Database& db)
    {
        try
        {
            auto& state = db.perInstanceState<OgnRGBToVB1940>();
            return state.computeImpl(db);
        }
        catch (const std::exception& e)
        {
            db.logError("OgnRGBToVB1940: %s", e.what());
            return false;
        }
        catch (...)
        {
            db.logError("OgnRGBToVB1940: unknown exception");
            return false;
        }
    }

    static void releaseInstance(NodeObj const& nodeObj, GraphInstanceID instanceId)
    {
        auto& state = OgnRGBToVB1940Database::sPerInstanceState<OgnRGBToVB1940>(nodeObj, instanceId);
        state.freeGPUBuffers();
    }

private:
    // Per-instance GPU buffers (persistent across frames)
    uint8_t* d_src = nullptr; // H2D copy of CPU input
    uint8_t* d_bayer = nullptr; // GBRG Bayer 8-bit (H x W)
    uint8_t* d_frame = nullptr; // Full CSI frame (image + embedded lines)
    size_t d_src_cap = 0;
    size_t d_bayer_cap = 0;
    size_t d_frame_cap = 0;

    std::vector<uint8_t> h_output; // CPU output buffer (reused each frame)
    bool loggedDimCheck = false;

    void freeGPUBuffers()
    {
        if (d_src)
        {
            cudaFree(d_src);
            d_src = nullptr;
            d_src_cap = 0;
        }
        if (d_bayer)
        {
            cudaFree(d_bayer);
            d_bayer = nullptr;
            d_bayer_cap = 0;
        }
        if (d_frame)
        {
            cudaFree(d_frame);
            d_frame = nullptr;
            d_frame_cap = 0;
        }
    }

    bool computeImpl(OgnRGBToVB1940Database& db)
    {
        const uint32_t width = db.inputs.width();
        const uint32_t height = db.inputs.height();

        if (width == 0 || height == 0)
        {
            db.logWarning("OgnRGBToVB1940: invalid dimensions %ux%u", width, height);
            return false;
        }

        const std::string encoding = db.tokenToString(db.inputs.encoding());
        const std::string outputMode = db.tokenToString(db.inputs.outputMode());

        if (encoding != "rgb8" && encoding != "rgba8")
        {
            db.logWarning("OgnRGBToVB1940: unsupported encoding '%s'", encoding.c_str());
            return false;
        }

        const uint8_t channels = (encoding == "rgb8") ? 3u : 4u;
        const uint64_t dataPtr = db.inputs.dataPtr();
        const int32_t cudaDeviceIndex = db.inputs.cudaDeviceIndex();
        const uint32_t bufferSize = db.inputs.bufferSize();

        // ---- Determine GPU source pointer ----
        const uint8_t* gpuSrc = nullptr;

        if (cudaDeviceIndex != -1 && dataPtr != 0)
        {
            // Data is already on the GPU
            gpuSrc = reinterpret_cast<const uint8_t*>(dataPtr);
        }
        else if (dataPtr != 0 && bufferSize > 0)
        {
            // CPU pointer provided via dataPtr
            const size_t srcSize = static_cast<size_t>(bufferSize);
            ensureGPU(d_src, d_src_cap, srcSize);
            cudaError_t err = cudaMemcpy(d_src, reinterpret_cast<const void*>(dataPtr), srcSize, cudaMemcpyHostToDevice);
            if (err != cudaSuccess)
            {
                throw std::runtime_error(std::string("cudaMemcpy H2D (dataPtr) failed: ") + cudaGetErrorString(err));
            }
            gpuSrc = d_src;
        }
        else
        {
            // Fall back to the OGN cpu array input
            if (db.inputs.data.size() == 0)
            {
                db.logWarning("OgnRGBToVB1940: no valid data source");
                return false;
            }
            const auto& cpuData = db.inputs.data.cpu();
            const size_t srcSize = cpuData.size();
            ensureGPU(d_src, d_src_cap, srcSize);
            cudaError_t err = cudaMemcpy(d_src, cpuData.data(), srcSize, cudaMemcpyHostToDevice);
            if (err != cudaSuccess)
            {
                throw std::runtime_error(std::string("cudaMemcpy H2D (data array) failed: ") + cudaGetErrorString(err));
            }
            gpuSrc = d_src;
        }

        // ---- RGB/RGBA -> GBRG Bayer ----
        const size_t bayerSize = static_cast<size_t>(width) * height;
        ensureGPU(d_bayer, d_bayer_cap, bayerSize);
        launchRGBToGBRG(gpuSrc, d_bayer, width, height, channels);

        // ---- Pack and copy to CPU output ----
        if (outputMode == "vb1940_csi_linux")
        {
            if (!loggedDimCheck)
            {
                CARB_LOG_INFO("[RGBToVB1940] vb1940_csi_linux mode: %ux%u", width, height);
                loggedDimCheck = true;
            }
            const uint32_t lineBytes = lineBytesRaw10(width);
            const uint32_t frameSize = lineBytes * (height + kVB1940EmbeddedLines);

            ensureGPU(d_frame, d_frame_cap, frameSize);
            cudaError_t memsetErr = cudaMemset(d_frame, 0, frameSize);
            if (memsetErr != cudaSuccess)
            {
                throw std::runtime_error(std::string("cudaMemset (linux frame) failed: ") + cudaGetErrorString(memsetErr));
            }

            // Write image lines starting after the 1 leading embedded line
            launchBayer8pTo10p(d_bayer, d_frame + lineBytes, width, height, lineBytes);

            h_output.resize(frameSize);
            cudaError_t err = cudaMemcpy(h_output.data(), d_frame, frameSize, cudaMemcpyDeviceToHost);
            if (err != cudaSuccess)
            {
                throw std::runtime_error(std::string("cudaMemcpy D2H (linux frame) failed: ") + cudaGetErrorString(err));
            }
        }
        else if (outputMode == "vb1940_csi_coe")
        {
            if (!loggedDimCheck)
            {
                CARB_LOG_INFO("[RGBToVB1940] vb1940_csi_coe mode: %ux%u", width, height);
                loggedDimCheck = true;
            }
            const uint32_t lineBytes = lineBytesRaw10Coe(width);
            const uint32_t frameSize = lineBytes * (height + kVB1940EmbeddedLines);

            ensureGPU(d_frame, d_frame_cap, frameSize);
            cudaError_t memsetErr = cudaMemset(d_frame, 0, frameSize);
            if (memsetErr != cudaSuccess)
            {
                throw std::runtime_error(std::string("cudaMemset (coe frame) failed: ") + cudaGetErrorString(memsetErr));
            }

            // Write image lines starting after the 1 leading embedded line
            launchBayer8pToTX2Rc10Rb10Ra10(d_bayer, d_frame + lineBytes, width, height, lineBytes);

            h_output.resize(frameSize);
            cudaError_t err = cudaMemcpy(h_output.data(), d_frame, frameSize, cudaMemcpyDeviceToHost);
            if (err != cudaSuccess)
            {
                throw std::runtime_error(std::string("cudaMemcpy D2H (coe frame) failed: ") + cudaGetErrorString(err));
            }
        }
        else
        {
            db.logWarning("OgnRGBToVB1940: unknown outputMode '%s'", outputMode.c_str());
            return false;
        }

        // ---- Write output ----
        db.outputs.data().resize(h_output.size());
        std::memcpy(db.outputs.data().data(), h_output.data(), h_output.size());

        db.outputs.execOut() = omni::graph::core::kExecutionAttributeStateEnabled;
        return true;
    }
};

REGISTER_OGN_NODE()
