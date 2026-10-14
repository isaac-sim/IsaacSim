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

#include <HostedSimulation.hpp>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace array = isaacsim::common::array;
namespace server = isaacsim::ovsim::servers::grpc;

namespace
{

struct FakeState
{
    std::atomic<int> importCalls{ 0 };
    std::atomic<int> initializeCalls{ 0 };
    std::atomic<double> timeStep{ 0.0 };
    std::atomic<int> invalidateCalls{ 0 };
    std::atomic<int> closeCalls{ 0 };
    std::atomic<int> readCalls{ 0 };
    std::atomic<int> writeCalls{ 0 };
    std::atomic<int> stepCalls{ 0 };
    std::atomic<int> initializeFailures{ 0 };
    std::atomic<int> invalidateFailures{ 0 };
    std::atomic<int> closeFailures{ 0 };
    std::atomic<int> stepFailures{ 0 };
    std::string lastWriteAttribute;
    std::mutex importMutex;
    std::condition_variable importCondition;
    bool blockImport{ false };
    bool importStarted{ false };
    bool releaseImport{ false };
};

void failWhilePositive(std::atomic<int>& remaining, const std::string& message)
{
    if (remaining.load() > 0)
    {
        --remaining;
        throw std::runtime_error(message);
    }
}

class FakeNativeSimulation final : public server::NativeSimulation
{
public:
    explicit FakeNativeSimulation(std::shared_ptr<FakeState> state) : m_state(std::move(state))
    {
    }

    bool importStageFromString(const std::string&) override
    {
        ++m_state->importCalls;
        std::unique_lock<std::mutex> lock(m_state->importMutex);
        m_state->importStarted = true;
        m_state->importCondition.notify_all();
        m_state->importCondition.wait(lock, [this]() { return !m_state->blockImport || m_state->releaseImport; });
        return true;
    }

    void initialize(double timeStep) override
    {
        m_state->timeStep = timeStep;
        ++m_state->initializeCalls;
        failWhilePositive(m_state->initializeFailures, "injected initialization failure");
    }

    void invalidate() override
    {
        ++m_state->invalidateCalls;
        failWhilePositive(m_state->invalidateFailures, "injected invalidation failure");
    }

    bool closeStage() override
    {
        ++m_state->closeCalls;
        failWhilePositive(m_state->closeFailures, "injected stage-close failure");
        return true;
    }

    ::ovsim::interfaces::data::OutputValueType read(const std::vector<std::string>& paths, const std::string&) override
    {
        ++m_state->readCalls;
        std::vector<std::vector<double>> values(paths.size(), std::vector<double>{ 0.0, 0.0, 0.0 });
        return array::Array(values);
    }

    void write(const std::vector<std::string>&,
               const std::string& attribute,
               const ::ovsim::interfaces::data::InputValueType&) override
    {
        ++m_state->writeCalls;
        m_state->lastWriteAttribute = attribute;
    }

    void step() override
    {
        ++m_state->stepCalls;
        failWhilePositive(m_state->stepFailures, "injected step failure");
    }

private:
    std::shared_ptr<FakeState> m_state;
};

std::unique_ptr<server::HostedSimulation> makeSimulation(const std::shared_ptr<FakeState>& state)
{
    return std::make_unique<server::HostedSimulation>(std::make_unique<FakeNativeSimulation>(state));
}

server::HostedSimulation::Snapshot waitForState(server::HostedSimulation& simulation,
                                                const std::string& simulationId,
                                                server::HostedSimulation::State state)
{
    server::HostedSimulation::Snapshot snapshot;
    for (size_t attempt = 0; attempt < 100; ++attempt)
    {
        const auto status = simulation.getSnapshot(simulationId, &snapshot);
        if (status.ok() && snapshot.state == state)
        {
            return snapshot;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    throw std::runtime_error("The hosted simulation did not reach the expected state.");
}

void waitForAbsence(server::HostedSimulation& simulation, const std::string& simulationId)
{
    server::HostedSimulation::Snapshot snapshot;
    for (size_t attempt = 0; attempt < 100; ++attempt)
    {
        if (simulation.getSnapshot(simulationId, &snapshot).error_code() == ::grpc::StatusCode::NOT_FOUND)
        {
            return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    throw std::runtime_error("The hosted simulation slot was not released.");
}

void waitForImport(const std::shared_ptr<FakeState>& state)
{
    std::unique_lock<std::mutex> lock(state->importMutex);
    if (state->importCondition.wait_for(lock, std::chrono::milliseconds(500), [state]() { return state->importStarted; }))
    {
        return;
    }
    state->releaseImport = true;
    lock.unlock();
    state->importCondition.notify_all();
    throw std::runtime_error("The fake simulation did not start importing the stage.");
}

void releaseImport(const std::shared_ptr<FakeState>& state)
{
    {
        std::lock_guard<std::mutex> lock(state->importMutex);
        state->releaseImport = true;
    }
    state->importCondition.notify_all();
}

} // namespace

TEST_SUITE("HostedSimulation lifecycle")
{
    TEST_CASE("rejects time steps outside the nanosecond timestamp range")
    {
        auto simulation = makeSimulation(std::make_shared<FakeState>());
        for (const double timeStep :
             { 0.0, 0.25e-9, 1e10, std::numeric_limits<double>::max(), std::numeric_limits<double>::quiet_NaN() })
        {
            CHECK_EQ(simulation->createSimulation("timing", "usd", timeStep).error_code(),
                     ::grpc::StatusCode::INVALID_ARGUMENT);
        }
    }

    TEST_CASE("advances at the minimum nanosecond time step")
    {
        auto state = std::make_shared<FakeState>();
        auto simulation = makeSimulation(state);
        REQUIRE(simulation->createSimulation("timing", "usd", 1e-9).ok());
        (void)waitForState(*simulation, "timing", server::HostedSimulation::State::eReady);
        for (int64_t timestamp = 1; timestamp <= 3; ++timestamp)
        {
            REQUIRE(simulation->step("timing", timestamp).ok());
        }
        CHECK_EQ(state->stepCalls.load(), 3);
    }

    TEST_CASE("rejects timestamp overflow while preserving retries")
    {
        auto state = std::make_shared<FakeState>();
        auto simulation = makeSimulation(state);
        REQUIRE(simulation->createSimulation("timing", "usd", 5e9).ok());
        (void)waitForState(*simulation, "timing", server::HostedSimulation::State::eReady);
        REQUIRE(simulation->step("timing", 5'000'000'000'000'000'000).ok());
        CHECK(simulation->step("timing", 5'000'000'000'000'000'000).ok());
        CHECK_EQ(simulation->step("timing", 6'000'000'000'000'000'000).error_code(), ::grpc::StatusCode::OUT_OF_RANGE);
        CHECK_EQ(state->stepCalls.load(), 1);
    }

    TEST_CASE("uses the configured physics time step without timestamp drift")
    {
        for (const int frequency : { 60, 200 })
        {
            auto state = std::make_shared<FakeState>();
            auto simulation = makeSimulation(state);
            REQUIRE(simulation->createSimulation("timing", "usd", 1.0 / frequency).ok());
            (void)waitForState(*simulation, "timing", server::HostedSimulation::State::eReady);
            CHECK_EQ(state->timeStep.load(), 1.0 / frequency);
            for (int64_t step = 1; step <= frequency * 10; ++step)
            {
                const int64_t timestamp = (step * 1'000'000'000 + frequency - 1) / frequency;
                REQUIRE(simulation->step("timing", timestamp).ok());
            }
            server::HostedSimulation::Snapshot snapshot;
            REQUIRE(simulation->getSnapshot("timing", &snapshot).ok());
            CHECK_EQ(snapshot.simulationTimeNs, 10'000'000'000);
            CHECK_EQ(state->stepCalls.load(), frequency * 10);
            CHECK(simulation->shutdown().ok());
        }
    }

    TEST_CASE("forwards generic attribute writes and tracks conflicts per attribute")
    {
        auto state = std::make_shared<FakeState>();
        auto simulation = makeSimulation(state);
        REQUIRE(simulation->createSimulation("generic-write", "usd").ok());
        (void)waitForState(*simulation, "generic-write", server::HostedSimulation::State::eReady);

        const std::vector<std::string> paths{ "/World/Cube" };
        const ::ovsim::interfaces::data::InputValueType position(
            array::Array(std::vector<std::vector<double>>{ { 0.0, 0.0, 2.0 } }));
        REQUIRE(simulation->write("generic-write", paths, "position", position, 16'666'667).ok());
        CHECK_EQ(state->lastWriteAttribute, "position");
        CHECK_EQ(simulation->write("generic-write", paths, "position", position, 16'666'667).error_code(),
                 ::grpc::StatusCode::ABORTED);

        const ::ovsim::interfaces::data::InputValueType size(array::Array(std::vector<std::vector<double>>{ { 2.0 } }));
        REQUIRE(simulation->write("generic-write", paths, "size", size, 16'666'667).ok());
        CHECK_EQ(state->lastWriteAttribute, "size");
        CHECK_EQ(state->writeCalls.load(), 2);

        state->stepFailures = 1;
        CHECK_EQ(simulation->step("generic-write", 16'666'667).error_code(), ::grpc::StatusCode::INTERNAL);
        CHECK_EQ(simulation->write("generic-write", paths, "position", position, 16'666'667).error_code(),
                 ::grpc::StatusCode::ABORTED);
        REQUIRE(simulation->step("generic-write", 16'666'667).ok());
        REQUIRE(simulation->write("generic-write", paths, "position", position, 33'333'334).ok());
        CHECK_EQ(state->writeCalls.load(), 3);
        CHECK(simulation->shutdown().ok());
    }

    TEST_CASE("rolls back initialization and retains the failed slot")
    {
        auto state = std::make_shared<FakeState>();
        state->initializeFailures = 1;
        auto simulation = makeSimulation(state);

        REQUIRE(simulation->createSimulation("failed-init", "usd").ok());
        const auto snapshot = waitForState(*simulation, "failed-init", server::HostedSimulation::State::eFailed);
        CHECK_EQ(snapshot.error.error_code(), ::grpc::StatusCode::INTERNAL);
        CHECK(snapshot.error.error_message().find("injected initialization failure") != std::string::npos);
        CHECK_EQ(state->importCalls.load(), 1);
        CHECK_EQ(state->initializeCalls.load(), 1);
        CHECK_EQ(state->invalidateCalls.load(), 0);
        CHECK_EQ(state->closeCalls.load(), 1);
        CHECK_EQ(simulation->createSimulation("other", "usd").error_code(), ::grpc::StatusCode::RESOURCE_EXHAUSTED);

        REQUIRE(simulation->deleteSimulation("failed-init").ok());
        waitForAbsence(*simulation, "failed-init");
        CHECK_EQ(state->closeCalls.load(), 1);
        CHECK(simulation->shutdown().ok());
    }

    TEST_CASE("deletion queued during import closes the stage")
    {
        auto state = std::make_shared<FakeState>();
        state->blockImport = true;
        auto simulation = makeSimulation(state);

        REQUIRE(simulation->createSimulation("delete-during-import", "usd").ok());
        waitForImport(state);
        const auto deleteStatus = simulation->deleteSimulation("delete-during-import");
        releaseImport(state);

        REQUIRE(deleteStatus.ok());
        waitForAbsence(*simulation, "delete-during-import");
        CHECK_EQ(state->importCalls.load(), 1);
        CHECK_EQ(state->initializeCalls.load(), 0);
        CHECK_EQ(state->invalidateCalls.load(), 0);
        CHECK_EQ(state->closeCalls.load(), 1);
        CHECK(simulation->shutdown().ok());
    }

    TEST_CASE("retains cleanup failure and retries only incomplete milestones")
    {
        auto state = std::make_shared<FakeState>();
        auto simulation = makeSimulation(state);
        REQUIRE(simulation->createSimulation("cleanup-retry", "usd").ok());
        (void)waitForState(*simulation, "cleanup-retry", server::HostedSimulation::State::eReady);

        state->invalidateFailures = 1;
        REQUIRE(simulation->deleteSimulation("cleanup-retry").ok());
        const auto failed = waitForState(*simulation, "cleanup-retry", server::HostedSimulation::State::eFailed);
        CHECK_EQ(failed.error.error_code(), ::grpc::StatusCode::INTERNAL);
        CHECK(failed.error.error_message().find("injected invalidation failure") != std::string::npos);
        CHECK_EQ(state->invalidateCalls.load(), 1);
        CHECK_EQ(state->closeCalls.load(), 0);

        REQUIRE(simulation->deleteSimulation("cleanup-retry").ok());
        waitForAbsence(*simulation, "cleanup-retry");
        CHECK_EQ(state->invalidateCalls.load(), 2);
        CHECK_EQ(state->closeCalls.load(), 1);
        CHECK(simulation->shutdown().ok());
    }

    TEST_CASE("preserves rollback failure for an explicit delete retry")
    {
        auto state = std::make_shared<FakeState>();
        state->initializeFailures = 1;
        state->closeFailures = 1;
        auto simulation = makeSimulation(state);
        REQUIRE(simulation->createSimulation("rollback-retry", "usd").ok());

        const auto failed = waitForState(*simulation, "rollback-retry", server::HostedSimulation::State::eFailed);
        CHECK_EQ(failed.error.error_code(), ::grpc::StatusCode::INTERNAL);
        CHECK(failed.error.error_message().find("Initialization rollback failed") != std::string::npos);
        CHECK_EQ(state->closeCalls.load(), 1);
        REQUIRE(simulation->deleteSimulation("rollback-retry").ok());
        waitForAbsence(*simulation, "rollback-retry");
        CHECK_EQ(state->closeCalls.load(), 2);
        CHECK(simulation->shutdown().ok());
    }

    TEST_CASE("reports graceful cleanup failure")
    {
        auto state = std::make_shared<FakeState>();
        auto simulation = makeSimulation(state);
        REQUIRE(simulation->createSimulation("shutdown-failure", "usd").ok());
        (void)waitForState(*simulation, "shutdown-failure", server::HostedSimulation::State::eReady);
        state->invalidateFailures = 1;

        const auto status = simulation->shutdown();
        CHECK_EQ(status.error_code(), ::grpc::StatusCode::INTERNAL);
        CHECK(status.error_message().find("injected invalidation failure") != std::string::npos);
    }
}
