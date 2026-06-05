// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include "OgnIsaacConvertRGBAToRGBDatabase.h"

#include <carb/logging/Log.h>

#include <isaacsim/core/includes/Buffer.h>
#include <isaacsim/core/includes/ScopedCudaDevice.h>

#include <cmath>
#include <string>
namespace isaacsim
{
namespace core
{
namespace nodes
{
extern "C" void rgbaToRgbOgn(uint8_t* dest, cudaTextureObject_t src, const int width, const int height, const int srcStride);


/**
 * @brief a node that converts from rgba to rgb
 *
 */
class OgnIsaacConvertRGBAToRGB
{

public:
    static bool compute(OgnIsaacConvertRGBAToRGBDatabase& db)
    {
        if (std::string(db.tokenToString(db.inputs.encoding())).compare(std::string("rgba8")) != 0)
        {
            db.logError("input data must be encoded as rgba8");
            return false;
        }
        // db.outputs.data.resize(db.inputs.width() * db.inputs.height() * 3);
        // CARB_LOG_ERROR(
        //     "BUFFER SIZE: %d %d", db.inputs.width() * db.inputs.height() * 3 * sizeof(uint8_t),
        //     db.inputs.bufferSize());
        // CARB_LOG_ERROR("FORMAT: %lu DEVICE: %d", db.inputs.format(), db.inputs.cudaDeviceIndex());
        auto& state = db.perInstanceState<OgnIsaacConvertRGBAToRGB>();

        // If the data is on host, pass the pointer along

        if (db.inputs.cudaDeviceIndex() == -1)
        {
            db.outputs.dataPtr() = db.inputs.dataPtr();
            db.outputs.bufferSize() = db.inputs.bufferSize();
        }
        else
        {
            isaacsim::core::includes::ScopedDevice scopedDev(db.inputs.cudaDeviceIndex());
            uint64_t handle = db.inputs.dataPtr();
            state.m_buffer.resize(db.inputs.width() * db.inputs.height() * 3);

            isaacsim::core::includes::ScopedCudaTextureObject srcTexObj(
                reinterpret_cast<cudaMipmappedArray_t>(handle), 0);
            rgbaToRgbOgn(state.m_buffer.data(), srcTexObj, db.inputs.width(), db.inputs.height(), db.inputs.width() * 4);
            db.outputs.dataPtr() = reinterpret_cast<uint64_t>(state.m_buffer.data());
            db.outputs.bufferSize() = static_cast<uint32_t>(state.m_buffer.sizeInBytes());
        }
        db.outputs.cudaDeviceIndex() = db.inputs.cudaDeviceIndex();
        db.outputs.width() = db.inputs.width();
        db.outputs.height() = db.inputs.height();
        db.outputs.encoding() = db.stringToToken("rgb8");
        db.outputs.execOut() = kExecutionAttributeStateEnabled;
        return true;
    }

    // static void releaseInstance(NodeObj const& nodeObj, GraphInstanceID instanceId)
    // {
    //     auto& state = OgnIsaacConvertRGBAToRGBDatabase::sPerInstanceState<OgnIsaacConvertRGBAToRGB>(nodeObj,
    //     instanceId);
    // }

private:
    isaacsim::core::includes::DeviceBuffer m_buffer;
};
REGISTER_OGN_NODE()
}
}
}
