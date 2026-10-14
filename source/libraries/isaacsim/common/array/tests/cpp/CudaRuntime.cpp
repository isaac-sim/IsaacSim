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

// Covers the parts of the driver binding that have no equivalent in the CUDA runtime API it
// replaced, and which the Array-level tests only reach indirectly: implicit context binding, the
// pointer-residency query, and error reporting for a device that does not exist.

#include "details/CudaRuntime.hpp"

#include <doctest/doctest.h>
#include <isaacsim/common/array/Array.hpp>
#include <isaacsim/common/array/Device.hpp>
#include <isaacsim/common/exceptions/Exceptions.hpp>

#include <vector>

using namespace isaacsim::common::array;

namespace
{

bool cudaAvailable()
{
    return Device::Cuda().isAvailable();
}

} // namespace

TEST_SUITE("CudaRuntime")
{
    TEST_CASE("cudaGetDevice() binds a context without an explicit device selection")
    {
        if (!cudaAvailable())
        {
            MESSAGE("Skipping: no CUDA device available");
            return;
        }

        // The runtime API defaulted an unselected thread to device 0 and created its primary context
        // on demand; driver calls instead fail with CUDA_ERROR_INVALID_CONTEXT unless one is bound.
        // Nothing here selects a device first, so this only passes if the binding is implicit.
        int device = -1;
        CHECK_UNARY(CudaRuntime::getInstance().cudaGetDevice(&device));
        CHECK(device >= 0);
    }

    TEST_CASE("cudaMalloc() succeeds without an explicit device selection")
    {
        if (!cudaAvailable())
        {
            MESSAGE("Skipping: no CUDA device available");
            return;
        }

        auto& runtime = CudaRuntime::getInstance();
        void* allocation = nullptr;
        REQUIRE_UNARY(runtime.cudaMalloc(&allocation, 128));
        CHECK(allocation != nullptr);
        CHECK_UNARY(runtime.cudaFree(allocation));
    }

    TEST_CASE("cudaSetDevice() reports a nonexistent ordinal as an invalid device")
    {
        if (!cudaAvailable())
        {
            MESSAGE("Skipping: no CUDA device available");
            return;
        }

        auto& runtime = CudaRuntime::getInstance();
        const int missing = static_cast<int>(runtime.cudaGetDeviceCount()) + 16;

        // The failure has to keep describing the ordinal. Retaining the primary context is what
        // fails here, and reporting that as "the driver was not loaded" would send a caller looking
        // for a broken installation instead of a bad device index.
        CHECK_THROWS_AS(runtime.cudaSetDevice(missing), isaacsim::common::exceptions::CudaRuntimeError);
        CHECK_UNARY_FALSE(runtime.cudaSetDevice(missing, /*throwIfInvalid=*/false));

        // The thread's binding must survive a rejected selection.
        int device = -1;
        CHECK_UNARY(runtime.cudaGetDevice(&device));
        CHECK(device >= 0);
    }

    TEST_CASE("cudaPointerGetAttributes() distinguishes device memory from host memory")
    {
        if (!cudaAvailable())
        {
            MESSAGE("Skipping: no CUDA device available");
            return;
        }

        auto& runtime = CudaRuntime::getInstance();

        SUBCASE("device allocation reports its owning ordinal")
        {
            void* allocation = nullptr;
            REQUIRE_UNARY(runtime.cudaMalloc(&allocation, 64));
            CudaRuntime::PointerAttributes attributes{};
            CHECK_UNARY(runtime.cudaPointerGetAttributes(&attributes, allocation));
            CHECK_UNARY(attributes.isDevice);
            CHECK(attributes.device >= 0);
            CHECK_UNARY(runtime.cudaFree(allocation));
        }

        SUBCASE("unregistered host allocation is not device memory")
        {
            // The driver rejects a pointer it never saw rather than describing it as host memory, so
            // the query fails; callers read that as "not device memory" and the fields stay cleared.
            std::vector<std::byte> host(64);
            CudaRuntime::PointerAttributes attributes{};
            CHECK_UNARY_FALSE(runtime.cudaPointerGetAttributes(&attributes, host.data(), /*throwIfInvalid=*/false));
            CHECK_UNARY_FALSE(attributes.isDevice);
        }
    }

    TEST_CASE("cudaStreamGetDevice() reports a stream's device and leaves the binding alone")
    {
        if (!cudaAvailable())
        {
            MESSAGE("Skipping: no CUDA device available");
            return;
        }

        auto& runtime = CudaRuntime::getInstance();
        CUstream stream = nullptr;
        REQUIRE_UNARY(runtime.cudaStreamCreateWithFlags(&stream, CU_STREAM_NON_BLOCKING));

        int before = -1;
        REQUIRE_UNARY(runtime.cudaGetDevice(&before));

        int streamDevice = -1;
        CHECK_UNARY(runtime.cudaStreamGetDevice(stream, &streamDevice));
        CHECK(streamDevice == before);

        // Callers treat this as a pure query -- DeviceGuard in particular assumes the thread is
        // still bound to whatever it selected -- so the current device must be unchanged.
        int after = -1;
        CHECK_UNARY(runtime.cudaGetDevice(&after));
        CHECK(after == before);

        CHECK_UNARY(runtime.cudaStreamDestroy(stream));
    }
}
