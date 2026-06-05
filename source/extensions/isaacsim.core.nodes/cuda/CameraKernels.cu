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

#include <cuda.h>
#include <stdint.h>

__global__ void rgbaToRgbKernel(uint8_t *dest, const uint8_t *src, const int width, const int height, const int srcStride)
{

    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= width*height)
        return;

	int row = idx / width;
	int col = idx % width;

	dest[idx*3] = src[row*srcStride + col*4];
	dest[idx*3+1] = src[row*srcStride + col*4+1];
	dest[idx*3+2] = src[row*srcStride + col*4+2];

}
extern "C" void rgbaToRgb(uint8_t *dest, const uint8_t *src, const int width, const int height, const int srcStride)
{

	const int num = width*height;
    const int nt = 256;
    const int nb = (num + nt - 1) / nt;

    rgbaToRgbKernel<<<nb, nt>>>(dest, src, width, height, srcStride);

}

__global__ void rgbaToRgbKernelOgn(uint8_t *dest, cudaTextureObject_t src, const int width, const int height, const int srcStride)
{

    const unsigned int x = blockIdx.x * blockDim.x + threadIdx.x;
    const unsigned int y = blockIdx.y * blockDim.y + threadIdx.y;

    if (x >= width || y >= height)
        return;

    const float tu = (x + 0.5f) / width;
    const float tv = (y + 0.5f) / height;

    dest[(y * width + x)*3+0] = tex2D<uchar4>(src, tu, tv).x;
    dest[(y * width + x)*3+1] = tex2D<uchar4>(src, tu, tv).y;
    dest[(y * width + x)*3+2] = tex2D<uchar4>(src, tu, tv).z;

}

extern "C" void rgbaToRgbOgn(uint8_t *dest, cudaTextureObject_t src, const int width, const int height, const int srcStride)
{

	const dim3 dimBlock(32, 32);
    const dim3 dimGrid((width + dimBlock.x - 1) / dimBlock.x, (height + dimBlock.y - 1) / dimBlock.y);

    rgbaToRgbKernelOgn<<<dimGrid, dimBlock>>>(dest, src, width, height, srcStride);

}


__global__ void uint32ToUint16Kernel(uint16_t *dest, const uint32_t *src, const int width, const int height, const int srcStride)
{

    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= width*height)
        return;

	int row = idx / width;
	int col = idx % width;

	uint32_t *srcRow = (uint32_t*)((uint8_t*)src + row * srcStride);

	dest[idx] = srcRow[col];

}

extern "C" void uint32ToUint16(uint16_t *dest, const uint32_t *src, const int width, const int height, const int srcStride)
{

	const int num = width*height;
    const int nt = 256;
    const int nb = (num + nt - 1) / nt;

    uint32ToUint16Kernel<<<nb, nt>>>(dest, src, width, height, srcStride);

}


__global__ void uint32ToUint8Kernel(uint8_t *dest, const uint32_t *src, int width, int height, int srcStride)
{

    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= width*height)
        return;

	int row = idx / width;
	int col = idx % width;

	uint32_t *srcRow = (uint32_t*)((uint8_t*)src + row * srcStride);

	dest[idx] = srcRow[col];

}

extern "C" void uint32ToUint8(uint8_t *dest, const uint32_t *src, int width, int height, int srcStride)
{

	const int num = width*height;
    const int nt = 256;
    const int nb = (num + nt - 1) / nt;

    uint32ToUint8Kernel<<<nb, nt>>>(dest, src, width, height, srcStride);

}

typedef struct __align__(16) {
    float x;
    float y;
    float z;
} PointXYZ;

__global__ void depthToPCLKernel(void *dest, const float *src, const int width, const int height, const float fx, const float fy, const float cx, const float cy)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;

    if (idx >= width*height)
        return;
    int row = idx / (width);
	int col = idx % (width);

    float z = src[idx];

    PointXYZ point;

    if (z != 0){
        point.x = (float)(z * (col - cx) / fx);
        point.y = (float)(z * (row - cy) / fy);
        point.z = z;
    }

    else{
        point.x = nanf("1");
        point.y = nanf("2");
        point.z = nanf("3");
    }
    PointXYZ* result = ( PointXYZ *)dest;
    result[idx] = point;

}


extern "C" void depthToPCL(void *dest, const float *src, const int width, const int height, const float fx, const float fy, const float cx, const float cy)
{

	const int num = width*height;
    const int nt = 256;
    const int nb = (num + nt - 1) / nt;

    depthToPCLKernel<<<nb, nt>>>(dest, src, width, height, fx, fy, cx, cy);

}

__global__ void depthToPCLKernelOgn(float3 * dest, const cudaTextureObject_t src, const int width, const int height, const float fx, const float fy, const float cx, const float cy)
{

    const unsigned int x = blockIdx.x * blockDim.x + threadIdx.x;
    const unsigned int y = blockIdx.y * blockDim.y + threadIdx.y;

    if (x >= width || y >= height)
        return;

    const float tu = (x + 0.5f) / width;
    const float tv = (y + 0.5f) / height;
    int idx = y * width + x;
    int row = idx / width;
	int col = idx % width;

    float z = tex2D<float>(src, tu, tv);

    if (z != INFINITY)
    {
        dest[idx].x = (float)(z * (col - cx) / fx);
        dest[idx].y = (float)(z * (row - cy) / fy);
        dest[idx].z = z;
    }

    else
    {
        dest[idx].x = nanf("1");
        dest[idx].y = nanf("2");
        dest[idx].z = nanf("3");
    }

}

extern "C" void depthToPCLOgn(float3 * dest, const cudaTextureObject_t src, const int width, const int height, const float fx, const float fy, const float cx, const float cy)
{

    const dim3 dimBlock(32, 32);
    const dim3 dimGrid((width + dimBlock.x - 1) / dimBlock.x, (height + dimBlock.y - 1) / dimBlock.y);

    depthToPCLKernelOgn<<<dimGrid, dimBlock>>>(dest, src, width, height, fx, fy, cx, cy);
}

