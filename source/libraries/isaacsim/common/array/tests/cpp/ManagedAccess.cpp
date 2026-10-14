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

#include "details/CudaRuntime.hpp"

#include <doctest/doctest.h>
#include <isaacsim/common/array/Array.hpp>
#include <isaacsim/common/array/Functions.hpp>

#include <atomic>
#include <chrono>
#include <cstring>
#include <future>
#include <memory>
#include <optional>
#include <stdexcept>
#include <thread>

using namespace isaacsim::common::array;

#define SKIP_IF_CUDA_UNAVAILABLE(device)                                                                               \
    do                                                                                                                 \
    {                                                                                                                  \
        if ((device) == Device::Cuda() && !(device).isAvailable())                                                     \
        {                                                                                                              \
            MESSAGE("Skipping: no CUDA device available (" << (device).toString() << ")");                             \
            return;                                                                                                    \
        }                                                                                                              \
    } while (false)

class ScopedCudaStream
{
public:
    explicit ScopedCudaStream(const Device& device) : m_device(device)
    {
        DeviceGuard deviceGuard(m_device);
        CudaRuntime::getInstance().cudaStreamCreateWithFlags(&m_stream, CU_STREAM_NON_BLOCKING);
    }

    ~ScopedCudaStream()
    {
        try
        {
            DeviceGuard deviceGuard(m_device);
            CudaRuntime::getInstance(false).cudaStreamDestroy(m_stream, false);
        }
        catch (...)
        {
        }
    }

    CUstream get() const
    {
        return m_stream;
    }

    std::uintptr_t value() const
    {
        return reinterpret_cast<std::uintptr_t>(m_stream);
    }

private:
    Device m_device;
    CUstream m_stream{ nullptr };
};

// A failed asynchronous-ordering assertion must report a failure, not deadlock
// the test process. Shared state also survives stack unwinding until CUDA runs
// and finishes the callback queued on the stream.
class ScopedCudaTestGate
{
public:
    void enqueue(CUstream stream)
    {
        auto payload = std::make_unique<std::shared_ptr<State>>(m_state);
        CudaRuntime::getInstance().cudaLaunchHostFunc(stream, wait, payload.get());
        payload.release();
    }

    ~ScopedCudaTestGate()
    {
        release();
    }

    void release()
    {
        m_state->released.store(true);
    }

    void phase(const char* name)
    {
        m_state->phase.store(name);
    }

    const char* timedOutPhase() const
    {
        return m_state->timedOutPhase.load();
    }

private:
    struct State
    {
        std::atomic<bool> released{ false };
        std::atomic<const char*> phase{ "unspecified operation" };
        std::atomic<const char*> timedOutPhase{ nullptr };
    };

    static void CUDA_CB wait(void* userData)
    {
        std::unique_ptr<std::shared_ptr<State>> payload(static_cast<std::shared_ptr<State>*>(userData));
        const auto state = *payload;
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        while (!state->released.load())
        {
            if (std::chrono::steady_clock::now() >= deadline)
            {
                state->timedOutPhase.store(state->phase.load());
                state->released.store(true);
                break;
            }
            std::this_thread::yield();
        }
    }

    std::shared_ptr<State> m_state = std::make_shared<State>();
};

TEST_SUITE("ManagedAccess")
{
    TEST_CASE("acquireRead()")
    {
        SUBCASE("Retains CPU storage until access is released")
        {
            bool released = false;
            auto data = std::shared_ptr<std::byte[]>(new std::byte[sizeof(int32_t)],
                                                     [&released](std::byte* pointer)
                                                     {
                                                         released = true;
                                                         delete[] pointer;
                                                     });
            const int32_t expected = 42;
            std::memcpy(data.get(), &expected, sizeof(expected));
            Array array = Array::fromBuffer(data, sizeof(expected), Shape({ 1 }), Dtype::Int32());
            ArrayReadAccess access = acquireRead(array);

            CHECK_UNARY(static_cast<bool>(access));
            CHECK_EQ(access.nbytes(), sizeof(expected));
            CHECK_EQ(*static_cast<const int32_t*>(access.data()), expected);

            data.reset();
            array = Array(0);
            CHECK_FALSE(released);
            access.release();
            CHECK_UNARY(released);
            CHECK_FALSE(static_cast<bool>(access));
            CHECK_NOTHROW(access.release());
        }

        SUBCASE("Validates CPU stream and CUDA write APIs")
        {
            Array array(std::vector<int32_t>{ 1, 2, 3 });
            CHECK_THROWS_AS(acquireRead(array, 1), std::invalid_argument);
            CHECK_THROWS_AS(recordCudaWrite(array), std::invalid_argument);
        }
    }

    // A read access held by the calling thread can never be released while that thread blocks
    // waiting for it, so the wait would never end. This is reachable from ordinary code: a retained
    // DLPack export holds read access for as long as its consumer keeps the tensor, and the next
    // write to the same allocation runs on the same thread.
    TEST_CASE("Array access rejects a wait the calling thread could never satisfy")
    {
        Array array = Array(std::vector<int32_t>{ 1, 2, 3, 4 });

        SUBCASE("A writer blocked by its own read access throws instead of waiting")
        {
            ArrayReadAccess read = acquireRead(array);
            CHECK_THROWS_AS(acquireWrite(array), std::runtime_error);
            // The rejected acquisition leaves the storage usable once the reader goes away.
            read.release();
            CHECK_NOTHROW(acquireWrite(array));
        }

        SUBCASE("A writer blocked by its own write access throws instead of waiting")
        {
            ArrayWriteAccess write = acquireWrite(array);
            CHECK_THROWS_AS(acquireWrite(array), std::runtime_error);
            CHECK_THROWS_AS(acquireRead(array), std::runtime_error);
        }

        SUBCASE("Views of one allocation share the check")
        {
            Array view = array.reshape(Shape({ 2, 2 }));
            ArrayReadAccess read = acquireRead(array);
            CHECK_THROWS_AS(acquireWrite(view), std::runtime_error);
        }

        SUBCASE("Concurrent readers on one thread still compose")
        {
            ArrayReadAccess first = acquireRead(array);
            CHECK_NOTHROW(acquireRead(array));
        }
    }

    // Test ordering of a preallocated device-to-device operation. Pageable host transfers and
    // allocation may synchronize, so complete both before gating the producer stream.
    TEST_CASE("A device-resident operation queues behind its producer without blocking the host")
    {
        const Device device = Device::Cuda();
        SKIP_IF_CUDA_UNAVAILABLE(device);

        ScopedCudaStream producer(device);
        const std::vector<int32_t> expected{ 5, 6, 7, 8 };
        Array seed = empty(Shape({ 4 }), Dtype::Int32(), device);
        Array source = empty(Shape({ 4 }), Dtype::Int32(), device);
        Array produced = empty(Shape({ 4 }), Dtype::Int32(), device);
        {
            ArrayWriteAccess access = acquireWrite(seed, producer.value());
            CudaRuntime::getInstance().cudaMemcpyAsync(
                access.data(), expected.data(), expected.size() * sizeof(int32_t), producer.get());
        }
        REQUIRE_UNARY(CudaRuntime::getInstance().cudaStreamSynchronize(producer.get()));

        ScopedCudaTestGate gate;
        {
            ArrayReadAccess seedAccess = acquireRead(seed, producer.value());
            ArrayWriteAccess access = acquireWrite(source, producer.value());
            // Gate the producer stream so its work cannot complete until this thread says so.
            gate.enqueue(producer.get());
            gate.phase("producer device-to-device enqueue");
            CudaRuntime::getInstance().cudaMemcpyAsync(
                access.data(), seedAccess.data(), expected.size() * sizeof(int32_t), producer.get());
            gate.phase("producer write-access release");
        }

        // set() must queue behind the producer without waiting for its gated work to finish.
        gate.phase("preallocated destination.set enqueue");
        produced.set(source);
        gate.release();

        ScopedCudaStream consumer(device);
        std::vector<int32_t> observed(expected.size(), 0);
        {
            ArrayReadAccess access = acquireRead(produced, consumer.value());
            CudaRuntime::getInstance().cudaMemcpyAsync(
                observed.data(), access.data(), observed.size() * sizeof(int32_t), consumer.get());
        }
        CHECK_UNARY(CudaRuntime::getInstance().cudaStreamSynchronize(consumer.get()));
        INFO("CUDA producer gate timeout phase: " << (gate.timedOutPhase() ? gate.timedOutPhase() : "none"));
        CHECK(gate.timedOutPhase() == nullptr);
        // Correct values prove the queued copy was still ordered after the gated producer.
        CHECK_EQ(observed, expected);
    }

    TEST_CASE("tryAcquireWrite()")
    {
        Array array = Array(std::vector<int32_t>{ 1, 2, 3, 4 });

        SUBCASE("Succeeds on an idle allocation")
        {
            std::optional<ArrayWriteAccess> access = tryAcquireWrite(array);
            REQUIRE_UNARY(access.has_value());
            CHECK_EQ(access->nbytes(), 4 * sizeof(int32_t));
        }

        SUBCASE("Reports contention instead of blocking or throwing")
        {
            ArrayReadAccess read = acquireRead(array);
            CHECK_FALSE(tryAcquireWrite(array).has_value());
            read.release();
            CHECK_UNARY(tryAcquireWrite(array).has_value());
        }

        SUBCASE("Rejects a read-only array")
        {
            Array broadcast = array.reshape(Shape({ 1, 4 })).broadcastView(Shape({ 2, 4 }));
            CHECK_THROWS_AS(tryAcquireWrite(broadcast), std::runtime_error);
        }
    }

    TEST_CASE("acquireWrite()")
    {
        using namespace std::chrono_literals;

        SUBCASE("Retains storage, exposes visible bytes, and releases idempotently")
        {
            bool released = false;
            auto data = std::shared_ptr<std::byte[]>(new std::byte[2 * sizeof(int32_t)],
                                                     [&released](std::byte* pointer)
                                                     {
                                                         released = true;
                                                         delete[] pointer;
                                                     });
            Array array = Array::fromBuffer(data, 2 * sizeof(int32_t), Shape({ 2 }), Dtype::Int32());
            ArrayWriteAccess access = acquireWrite(array);
            auto* values = static_cast<int32_t*>(access.data());
            values[0] = 17;
            values[1] = 23;

            CHECK_UNARY(static_cast<bool>(access));
            CHECK_EQ(access.nbytes(), 2 * sizeof(int32_t));
            data.reset();
            array = Array(0);
            CHECK_FALSE(released);
            access.release();
            CHECK_UNARY(released);
            CHECK_FALSE(static_cast<bool>(access));
            CHECK_NOTHROW(access.release());
        }

        SUBCASE("Serializes readers and writers across views of one allocation")
        {
            Array array(std::vector<int32_t>{ 1, 2, 3 });
            Array view = array.reshape(Shape({ 3, 1 }));
            ArrayReadAccess reader = acquireRead(view);
            std::promise<void> writerStarted;
            auto writerStartedFuture = writerStarted.get_future();
            auto writer = std::async(std::launch::async,
                                     [&]
                                     {
                                         writerStarted.set_value();
                                         ArrayWriteAccess access = acquireWrite(array);
                                         static_cast<int32_t*>(access.data())[0] = 9;
                                     });
            writerStartedFuture.wait();
            CHECK_EQ(writer.wait_for(20ms), std::future_status::timeout);
            reader.release();
            CHECK_EQ(writer.wait_for(1s), std::future_status::ready);
            writer.get();
            CHECK_EQ(array.get<std::vector<int32_t>>(), std::vector<int32_t>{ 9, 2, 3 });

            ArrayWriteAccess activeWriter = acquireWrite(view);
            std::promise<void> readerStarted;
            auto readerStartedFuture = readerStarted.get_future();
            auto blockedReader = std::async(std::launch::async,
                                            [&]
                                            {
                                                readerStarted.set_value();
                                                return acquireRead(array);
                                            });
            readerStartedFuture.wait();
            CHECK_EQ(blockedReader.wait_for(20ms), std::future_status::timeout);
            activeWriter.release();
            CHECK_EQ(blockedReader.wait_for(1s), std::future_status::ready);
            ArrayReadAccess finalReader = blockedReader.get();
            CHECK_UNARY(static_cast<bool>(finalReader));
        }

        SUBCASE("Rejects invalid CPU streams and read-only views")
        {
            Array array(std::vector<int32_t>{ 1, 2, 3 });
            CHECK_THROWS_AS(acquireWrite(array, 1), std::invalid_argument);
            Array readOnly = array.broadcastView(Shape({ 2, 3 }));
            CHECK_THROWS_AS(acquireWrite(readOnly), std::runtime_error);
        }

        SUBCASE("Orders CUDA producers and consumers across streams")
        {
            const Device device = Device::Cuda();
            SKIP_IF_CUDA_UNAVAILABLE(device);
            ScopedCudaStream producer(device);
            ScopedCudaStream consumer(device);
            std::vector<uint8_t> expected(4096, uint8_t{ 0x5A });
            std::vector<uint8_t> observed(expected.size(), uint8_t{ 0 });
            Array array = empty(Shape({ static_cast<int64_t>(expected.size()) }), Dtype::UInt8(), device);

            {
                ArrayWriteAccess access = acquireWrite(array, producer.value());
                CudaRuntime::getInstance().cudaMemcpyAsync(
                    access.data(), expected.data(), expected.size(), producer.get());
            }
            {
                ArrayReadAccess access = acquireRead(array, consumer.value());
                CudaRuntime::getInstance().cudaMemcpyAsync(
                    observed.data(), access.data(), observed.size(), consumer.get());
            }
            CudaRuntime::getInstance().cudaStreamSynchronize(consumer.get());
            CHECK_EQ(observed, expected);

            auto released = std::make_shared<std::atomic<bool>>(false);
            std::vector<uint8_t> values(4096, uint8_t{ 1 });
            void* pointer = nullptr;
            {
                DeviceGuard deviceGuard(device);
                CudaRuntime::getInstance().cudaMalloc(&pointer, 4096);
            }
            {
                auto allocation =
                    std::shared_ptr<std::byte[]>(static_cast<std::byte*>(pointer),
                                                 [device, released](std::byte* value) noexcept
                                                 {
                                                     try
                                                     {
                                                         DeviceGuard deviceGuard(device);
                                                         CudaRuntime::getInstance(false).cudaFree(value, false);
                                                         released->store(true);
                                                     }
                                                     catch (...)
                                                     {
                                                     }
                                                 });
                Array array = Array::fromBuffer(std::move(allocation), 4096, Shape({ 4096 }), Dtype::UInt8(), device);
                ArrayWriteAccess access = acquireWrite(array, producer.value());
                CudaRuntime::getInstance().cudaMemcpyAsync(access.data(), values.data(), values.size(), producer.get());
            }
            CudaRuntime::getInstance().cudaStreamSynchronize(producer.get());
            for (size_t attempt = 0; attempt < 1000 && !released->load(); ++attempt)
            {
                std::this_thread::sleep_for(1ms);
            }
            CHECK_UNARY(released->load());
        }

        SUBCASE("Releases CUDA access from another host thread")
        {
            const Device device = Device::Cuda();
            SKIP_IF_CUDA_UNAVAILABLE(device);
            ScopedCudaStream stream(device);
            Array array(std::vector<int32_t>{ 1, 2, 3 }, Dtype::Int32(), device);
            ArrayReadAccess access = acquireRead(array, stream.value());
            std::thread releaser([movedAccess = std::move(access)]() mutable { movedAccess.release(); });
            releaser.join();
            DeviceGuard deviceGuard(device);
            CHECK_UNARY(CudaRuntime::getInstance().cudaStreamSynchronize(stream.get()));
        }

        // CU_STREAM_PER_THREAD names a different physical stream on every calling thread, while a
        // scope may be released on a thread other than the one that acquired it. Completion would
        // then be recorded on the releasing thread's stream and the allocation could retire while
        // the acquiring thread's work is still pending, so admission refuses the handle outright.
        SUBCASE("Rejects CU_STREAM_PER_THREAD for scoped access but not for recordCudaWrite()")
        {
            const Device device = Device::Cuda();
            SKIP_IF_CUDA_UNAVAILABLE(device);
            const auto perThread = reinterpret_cast<std::uintptr_t>(CU_STREAM_PER_THREAD);
            Array array(std::vector<int32_t>{ 1, 2, 3 }, Dtype::Int32(), device);

            CHECK_THROWS_AS(acquireRead(array, perThread), std::invalid_argument);
            CHECK_THROWS_AS(acquireWrite(array, perThread), std::invalid_argument);
            CHECK_THROWS_AS(tryAcquireWrite(array, perThread), std::invalid_argument);
            CHECK_NOTHROW(recordCudaWrite(array, perThread));

            // The rejected acquisitions must not have left an admission behind.
            ScopedCudaStream stream(device);
            CHECK_UNARY(static_cast<bool>(acquireWrite(array, stream.value())));
        }

        SUBCASE("Rejects a CUDA stream from another device")
        {
            if (CudaRuntime::getInstance(false).cudaGetDeviceCount() < 2)
            {
                MESSAGE("Skipping: two CUDA devices are required");
                return;
            }
            const Device arrayDevice = Device::Cuda(0);
            const Device streamDevice = Device::Cuda(1);
            ScopedCudaStream wrongStream(streamDevice);
            Array array(std::vector<int32_t>{ 1 }, Dtype::Int32(), arrayDevice);
            CHECK_THROWS_AS(acquireRead(array, wrongStream.value()), std::invalid_argument);
            CHECK_THROWS_AS(acquireWrite(array, wrongStream.value()), std::invalid_argument);
        }

        SUBCASE("Retains an allocation while a CUDA consumer is pending")
        {
            using namespace std::chrono_literals;

            const Device device = Device::Cuda();
            SKIP_IF_CUDA_UNAVAILABLE(device);
            ScopedCudaStream consumer(device);
            void* pointer = nullptr;
            ScopedCudaTestGate gate;
            {
                DeviceGuard deviceGuard(device);
                CudaRuntime::getInstance().cudaMalloc(&pointer, sizeof(int32_t));
                gate.phase("consumer retirement probe");
                gate.enqueue(consumer.get());
            }

            struct ConsumerState
            {
                std::atomic<bool> released{ false };
                int32_t observed{ 0 };
            };
            // Retirement keeps both the signal and pending host destination alive on failure paths.
            auto state = std::make_shared<ConsumerState>();
            {
                auto allocation =
                    std::shared_ptr<std::byte[]>(static_cast<std::byte*>(pointer),
                                                 [device, state](std::byte* value) noexcept
                                                 {
                                                     try
                                                     {
                                                         DeviceGuard deviceGuard(device);
                                                         CudaRuntime::getInstance(false).cudaFree(value, false);
                                                         state->released.store(true);
                                                     }
                                                     catch (...)
                                                     {
                                                     }
                                                 });
                Array array =
                    Array::fromBuffer(std::move(allocation), sizeof(int32_t), Shape({ 1 }), Dtype::Int32(), device);
                ArrayReadAccess access = acquireRead(array, consumer.value());
                CudaRuntime::getInstance().cudaMemcpyAsync(
                    &state->observed, access.data(), sizeof(state->observed), consumer.get());
            }
            std::this_thread::sleep_for(10ms);
            CHECK_FALSE(state->released.load());

            gate.release();
            {
                DeviceGuard deviceGuard(device);
                CudaRuntime::getInstance().cudaStreamSynchronize(consumer.get());
            }
            INFO("CUDA consumer gate timeout phase: " << (gate.timedOutPhase() ? gate.timedOutPhase() : "none"));
            CHECK(gate.timedOutPhase() == nullptr);
            for (size_t attempt = 0; attempt < 1000 && !state->released.load(); ++attempt)
            {
                std::this_thread::sleep_for(1ms);
            }
            CHECK_UNARY(state->released.load());
        }
    }
}
