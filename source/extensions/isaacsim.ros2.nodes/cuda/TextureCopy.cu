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

template <typename T>
__global__ void textureCopyToRawBufferKernel(cudaTextureObject_t srcTexObj,
                                             T* dstBuffer,
                                             unsigned int dstWidth,
                                             unsigned int dstHeight)
{
    const unsigned int x = blockIdx.x * blockDim.x + threadIdx.x;
    const unsigned int y = blockIdx.y * blockDim.y + threadIdx.y;

    if (x >= dstWidth || y >= dstHeight)
        return;

    const float tu = (x + 0.5f) / dstWidth;
    const float tv = (y + 0.5f) / dstHeight;

    dstBuffer[y * dstWidth + x] = tex2D<T>(srcTexObj, tu, tv);
}

template <typename T>
void textureCopyToRawBuffer(cudaTextureObject_t srcTexObj,
                            unsigned char* dstBuffer,
                            unsigned int dstWidth,
                            unsigned int dstHeight,
                            cudaStream_t stream)
{
    const dim3 dimBlock(32, 32);
    const dim3 dimGrid((dstWidth + dimBlock.x - 1) / dimBlock.x, (dstHeight + dimBlock.y - 1) / dimBlock.y);
    textureCopyToRawBufferKernel<T>
        <<<dimGrid, dimBlock, 0, stream>>>(srcTexObj, reinterpret_cast<T*>(dstBuffer), dstWidth, dstHeight);
}

extern "C" void textureFloatCopyToRawBuffer(cudaTextureObject_t srcTexObj,
                                            unsigned char* dstBuffer,
                                            unsigned int dstWidth,
                                            unsigned int dstHeight,
                                            cudaStream_t stream)
{
    if (dstWidth * dstHeight > 0 && dstBuffer != nullptr)
    {
        textureCopyToRawBuffer<float>(srcTexObj, dstBuffer, dstWidth, dstHeight, stream);
    }
}
