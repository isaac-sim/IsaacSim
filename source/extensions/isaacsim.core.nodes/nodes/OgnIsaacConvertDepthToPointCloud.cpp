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

#include "OgnIsaacConvertDepthToPointCloudDatabase.h"

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
extern "C" void depthToPCLOgn(float3* dest,
                              const cudaTextureObject_t src,
                              const int width,
                              const int height,
                              const float fx,
                              const float fy,
                              const float cx,
                              const float cy);

class OgnIsaacConvertDepthToPointCloud
{

public:
    static bool compute(OgnIsaacConvertDepthToPointCloudDatabase& db)
    {
        if ((carb::Format)db.inputs.format() != carb::Format::eR32_SFLOAT)
        {
            db.logError("Input data must have texture format R32_SFLOAT");
            return false;
        }

        auto& height = db.inputs.height();
        auto& width = db.inputs.width();

        auto& state = db.perInstanceState<OgnIsaacConvertDepthToPointCloud>();

        float fx, fy, cx, cy;

        fx = width * db.inputs.focalLength() / db.inputs.horizontalAperture();
        fy = height * db.inputs.focalLength() / (db.inputs.horizontalAperture() * (static_cast<float>(height) / width));
        cx = width * 0.5f;
        cy = height * 0.5f;
        {
            isaacsim::core::includes::ScopedDevice scopedDev(db.inputs.cudaDeviceIndex());
            uint64_t handle = db.inputs.dataPtr();
            isaacsim::core::includes::ScopedCudaTextureObject srcTexObj(
                reinterpret_cast<cudaMipmappedArray_t>(handle), 0);
            state.m_buffer.resize(db.inputs.width() * db.inputs.height());
            depthToPCLOgn(state.m_buffer.data(), srcTexObj, width, height, fx, fy, cx, cy);
        }

        db.outputs.dataPtr() = reinterpret_cast<uint64_t>(state.m_buffer.data());
        db.outputs.cudaDeviceIndex() = db.inputs.cudaDeviceIndex();
        db.outputs.bufferSize() = static_cast<uint32_t>(state.m_buffer.sizeInBytes());
        db.outputs.execOut() = kExecutionAttributeStateEnabled;
        db.outputs.height() = 1;
        db.outputs.width() = static_cast<uint32_t>(state.m_buffer.size());
        return true;
    }

private:
    isaacsim::core::includes::DeviceBufferBase<float3> m_buffer;
};
REGISTER_OGN_NODE()
}
}
}
