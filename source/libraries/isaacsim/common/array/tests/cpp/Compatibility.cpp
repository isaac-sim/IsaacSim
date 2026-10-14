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

#include <doctest/doctest.h>
#include <isaacsim/common/array/Array.hpp>
#include <isaacsim/common/array/Functions.hpp>

#include <chrono>
#include <condition_variable>
#include <cstring>
#include <limits>
#include <memory>
#include <mutex>
#include <vector>

using namespace isaacsim::common::array;

TEST_SUITE("M1 Array ownership compatibility")
{
    TEST_CASE("contiguous fromBuffer retains owner and byte offset")
    {
        bool released = false;
        {
            auto buffer = std::shared_ptr<std::byte[]>(new std::byte[4 * sizeof(int32_t)],
                                                       [&released](std::byte* pointer)
                                                       {
                                                           released = true;
                                                           delete[] pointer;
                                                       });
            const int32_t values[] = { 99, 1, 2, 3 };
            std::memcpy(buffer.get(), values, sizeof(values));
            Array array = Array::fromBuffer(buffer, Shape{ 3 }, Dtype::Int32(), Device::Cpu(), sizeof(int32_t));
            CHECK_EQ(array.get<std::vector<int32_t>>(), std::vector<int32_t>{ 1, 2, 3 });
            CHECK_EQ(array.capacity(), sizeof(values));
            buffer.reset();
            CHECK_FALSE(released);
            array.at(0).set(int32_t{ 7 });
            CHECK_EQ(array.get<std::vector<int32_t>>(), std::vector<int32_t>{ 7, 2, 3 });
        }
        CHECK(released);
    }

    TEST_CASE("contiguous fromBuffer rejects overflowing span and preserves zero elements")
    {
        auto buffer = std::shared_ptr<std::byte[]>(new std::byte[8]);
        CHECK_THROWS_AS(
            Array::fromBuffer(buffer, Shape{ 1 }, Dtype::Int32(), Device::Cpu(), std::numeric_limits<size_t>::max()),
            std::overflow_error);
        CHECK_THROWS_AS(Array::fromBuffer(buffer, Shape{ std::numeric_limits<int64_t>::max() }, Dtype::Float64()),
                        std::overflow_error);
        CHECK_EQ(Array::fromBuffer(nullptr, Shape{ 0 }, Dtype::Int32()).size(), 0);
        CHECK_THROWS_AS(Array::fromBuffer(nullptr, Shape{ 1 }, Dtype::Int32()), std::invalid_argument);
    }

    TEST_CASE("CUDA contiguous fromBuffer retains offset storage through asynchronous retirement")
    {
        const Device device = Device::Cuda();
        if (!device.isAvailable())
        {
            MESSAGE("Skipping unavailable device: ", device.toString());
            return;
        }
        struct Retirement
        {
            std::mutex mutex;
            std::condition_variable condition;
            bool released{ false };
        };
        auto retirement = std::make_shared<Retirement>();
        {
            auto storage = std::make_shared<Array>(std::vector<int32_t>{ 99, 1, 2, 3 }, Dtype::Int32(), device);
            std::shared_ptr<std::byte[]> buffer(static_cast<std::byte*>(storage->data()),
                                                [storage, retirement](std::byte*) mutable
                                                {
                                                    storage.reset();
                                                    {
                                                        std::lock_guard<std::mutex> lock(retirement->mutex);
                                                        retirement->released = true;
                                                    }
                                                    retirement->condition.notify_one();
                                                });
            storage.reset();
            Array view = Array::fromBuffer(buffer, Shape{ 3 }, Dtype::Int32(), device, sizeof(int32_t));
            buffer.reset();
            CHECK_EQ(view.get<std::vector<int32_t>>(), std::vector<int32_t>{ 1, 2, 3 });
            Array alias = view.broadcastTo(Shape{ 3 });
            CHECK_EQ(alias.data(), view.data());
            alias.at(0).set(int32_t{ 7 });
            CHECK_EQ(view.get<std::vector<int32_t>>(), std::vector<int32_t>{ 7, 2, 3 });
            {
                std::lock_guard<std::mutex> lock(retirement->mutex);
                CHECK_FALSE(retirement->released);
            }
        }
        // CUDA owners retire on a worker after recorded operations complete. Wait on the
        // actual release condition rather than assuming scope exit synchronously frees storage.
        std::unique_lock<std::mutex> lock(retirement->mutex);
        CHECK(retirement->condition.wait_for(
            lock, std::chrono::seconds(5), [&retirement] { return retirement->released; }));
    }

    TEST_CASE("empty and offset inputs preserve legacy materialization semantics")
    {
        for (const Device device : { Device::Cpu(), Device::Cuda() })
        {
            if (!device.isAvailable())
            {
                MESSAGE("Skipping unavailable device: ", device.toString());
                continue;
            }
            Array empty = zeros(Shape{ 0, 3 }, Dtype::Int32(), device);
            Array emptyBroadcast = empty.broadcastTo(Shape{ 1, 0, 3 });
            CHECK_EQ(emptyBroadcast.shape(), Shape{ 1, 0, 3 });
            CHECK_EQ(emptyBroadcast.size(), 0);
            CHECK(emptyBroadcast.isWriteable());
            Array emptyTranspose = transpose(empty);
            CHECK_EQ(emptyTranspose.shape(), Shape{ 3, 0 });
            CHECK_EQ(emptyTranspose.size(), 0);
            CHECK(emptyTranspose.isWriteable());
            // Empty allocations may share null addresses; pointer inequality is not meaningful.
            Array storage(std::vector<std::vector<int32_t>>{ { 99, 98, 97 }, { 1, 2, 3 } }, Dtype::Int32(), device);
            Array offset = storage.at(1);
            CHECK_NE(offset.data(), storage.data());
            Array expanded = offset.broadcastTo(Shape{ 1, 3 });
            CHECK_NE(expanded.data(), offset.data());
            CHECK_EQ(expanded.at(0).get<std::vector<int32_t>>(), std::vector<int32_t>{ 1, 2, 3 });
            expanded.at(0).set(std::vector<int32_t>{ 4, 5, 6 });
            Array repeated = offset.broadcastTo(Shape{ 2, 3 });
            repeated.at(0).set(std::vector<int32_t>{ 7, 8, 9 });
            CHECK_EQ(repeated.at(1).get<std::vector<int32_t>>(), std::vector<int32_t>{ 1, 2, 3 });
            Array transposed = transpose(offset);
            CHECK_NE(transposed.data(), offset.data());
            CHECK_EQ(transposed.get<std::vector<int32_t>>(), std::vector<int32_t>{ 1, 2, 3 });
            transposed.at(0).set(int32_t{ 10 });
            CHECK_EQ(offset.get<std::vector<int32_t>>(), std::vector<int32_t>{ 1, 2, 3 });
            CHECK_EQ(storage.at(0).get<std::vector<int32_t>>(), std::vector<int32_t>{ 99, 98, 97 });
        }
    }

    TEST_CASE("broadcast and transpose own independent results when required")
    {
        for (const Device device : { Device::Cpu(), Device::Cuda() })
        {
            if (!device.isAvailable())
            {
                MESSAGE("Skipping unavailable device: ", device.toString());
                continue;
            }
            Array source(std::vector<int32_t>{ 1, 2, 3 }, Dtype::Int32(), device);
            Array alias = source.broadcastTo(Shape{ 3 });
            CHECK_EQ(alias.data(), source.data());
            Array copied = source.broadcastTo(Shape{ 3 }, true);
            CHECK_NE(copied.data(), source.data());
            Array rankExpanded = source.broadcastTo(Shape{ 1, 3 });
            CHECK_NE(rankExpanded.data(), source.data());
            CHECK(rankExpanded.isWriteable());
            rankExpanded.at(0).set(std::vector<int32_t>{ 4, 5, 6 });
            CHECK_EQ(source.get<std::vector<int32_t>>(), std::vector<int32_t>{ 1, 2, 3 });
            Array replicated = source.broadcastTo(Shape{ 2, 3 });
            CHECK(replicated.isContiguous());
            CHECK(replicated.isWriteable());
            replicated.at(0).set(std::vector<int32_t>{ 7, 8, 9 });
            CHECK_EQ(replicated.at(1).get<std::vector<int32_t>>(), std::vector<int32_t>{ 1, 2, 3 });
            CHECK_EQ(source.get<std::vector<int32_t>>(), std::vector<int32_t>{ 1, 2, 3 });
            CHECK_FALSE(source.broadcastView(Shape{ 2, 3 }).isWriteable());

            Array scalar(int32_t{ 4 }, Dtype::Int32(), device);
            Array scalarTranspose = transpose(scalar);
            CHECK_NE(scalarTranspose.data(), scalar.data());
            CHECK_EQ(scalarTranspose.item<int32_t>(), 4);
            Array vectorTranspose = transpose(source);
            CHECK_NE(vectorTranspose.data(), source.data());
            vectorTranspose.at(0).set(int32_t{ 10 });
            CHECK_EQ(source.get<std::vector<int32_t>>(), std::vector<int32_t>{ 1, 2, 3 });

            Array matrix = source.broadcastTo(Shape{ 2, 3 });
            Array identity = transpose(matrix, std::vector<int64_t>{ 0, 1 });
            Array permuted = transpose(matrix);
            CHECK_NE(identity.data(), matrix.data());
            CHECK_NE(permuted.data(), matrix.data());
            CHECK(permuted.isContiguous());
            CHECK_EQ(permuted.shape(), Shape{ 3, 2 });
            permuted.at(0).set(std::vector<int32_t>{ 8, 9 });
            CHECK_EQ(matrix.at(0).get<std::vector<int32_t>>(), std::vector<int32_t>{ 1, 2, 3 });
            CHECK_EQ(transposeView(matrix).data(), matrix.data());
        }
    }
}
