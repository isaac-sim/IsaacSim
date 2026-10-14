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

/*
Test is implemented using the doctest C++ testing framework:
  https://github.com/doctest/doctest/blob/master/doc/markdown/readme.md
*/

#include <doctest/doctest.h>
#include <isaacsim/ucx/core/UcxListener.hpp>
#include <isaacsim/ucx/core/UcxListenerRegistry.hpp>
#include <isaacsim/ucx/core/UcxUtils.hpp>
#include <ucxx/api.h>

#include <chrono>
#include <memory>
#include <thread>
#include <vector>

using namespace isaacsim::ucx::core;

// Helper class to create client endpoint for testing
class TestClient
{
public:
    std::shared_ptr<ucxx::Worker> worker;
    std::shared_ptr<ucxx::Endpoint> endpoint;

    TestClient(std::shared_ptr<ucxx::Worker> sharedWorker, const std::string& ip, uint16_t port) : worker(sharedWorker)
    {
        endpoint = worker->createEndpointFromHostname(ip, port, true);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::shared_ptr<ucxx::Request> tagSend(const void* buffer, size_t length, uint64_t tag)
    {
        return endpoint->tagSend(const_cast<void*>(buffer), length, ucxx::Tag{ tag });
    }

    std::shared_ptr<ucxx::Request> tagReceive(void* buffer, size_t length, uint64_t tag)
    {
        return endpoint->tagRecv(buffer, length, ucxx::Tag{ tag }, ucxx::TagMaskFull);
    }
};

namespace
{

/**
 * Polls a request until it completes or the bound expires. Never waits unbounded, so a
 * request that never settles fails the assertion at the call site instead of wedging the
 * test process.
 */
bool waitForCompletion(const std::shared_ptr<ucxx::Request>& request, int timeoutMs)
{
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeoutMs);
    while (std::chrono::steady_clock::now() < deadline)
    {
        if (request->isCompleted())
        {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return request->isCompleted();
}

/** Milliseconds elapsed since the given start point. */
int64_t elapsedMsSince(const std::chrono::steady_clock::time_point& start)
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count();
}

} // namespace

TEST_SUITE("UcxUtils")
{
    TEST_CASE("waitForRequestWithTimeout - null request")
    {
        std::shared_ptr<ucxx::Request> nullRequest = nullptr;
        std::string errorMsg;
        auto result = waitForRequestWithTimeout(nullRequest, 1000, errorMsg);
        CHECK(result == UcxRequestWaitResult::eFailed);
    }

    TEST_CASE("waitForRequestWithTimeout - completed request")
    {
        // Create a context and listener
        auto context = ucxx::createContext({}, ucxx::Context::defaultFeatureFlags);
        auto listener = UCXListenerRegistry::addListener(0);
        uint16_t port = listener->getPort();

        // Start progress thread
        listener->startProgressThread();

        // Create client and connect
        TestClient client(listener->getWorker(), "127.0.0.1", port);

        // Wait for connection
        REQUIRE(listener->waitForConnection(5000));

        // Send data from client (async)
        std::vector<uint8_t> sendData(1024, 42);
        auto sendRequest = client.tagSend(sendData.data(), sendData.size(), 100);
        REQUIRE(sendRequest.get() != nullptr);

        // Receive on server with timeout
        std::vector<uint8_t> recvData(1024);
        std::string errorMsg;
        auto result = listener->tagReceive(recvData.data(), recvData.size(), 100, 0xFFFFFFFFFFFFFFFF, errorMsg, 5000);
        CHECK(result == UcxReceiveResult::eSuccess);

        // Verify data
        CHECK(recvData == sendData);

        // Clean up
        UCXListenerRegistry::removeListener(port);
    }

    TEST_CASE("waitForRequestWithTimeout - timeout before completion")
    {
        // Create a context and listener
        auto context = ucxx::createContext({}, ucxx::Context::defaultFeatureFlags);
        auto listener = UCXListenerRegistry::addListener(0);
        uint16_t port = listener->getPort();

        // Start progress thread
        listener->startProgressThread();

        // Create client and connect
        TestClient client(listener->getWorker(), "127.0.0.1", port);

        // Wait for connection
        REQUIRE(listener->waitForConnection(5000));

        // Start a receive but don't send anything (so it will timeout)
        std::vector<uint8_t> recvData(1024);

        // Receive with short timeout - should timeout since no data is sent
        auto startTime = std::chrono::steady_clock::now();
        std::string errorMsg;
        auto result = listener->tagReceive(recvData.data(), recvData.size(), 200, 0xFFFFFFFFFFFFFFFF, errorMsg, 100);
        auto elapsed =
            std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - startTime);

        CHECK(result == UcxReceiveResult::eTimedOut);
        // Verify it actually waited approximately the timeout duration
        CHECK(elapsed.count() >= 100);
        CHECK(elapsed.count() < 200); // Should not wait much longer

        // Clean up
        UCXListenerRegistry::removeListener(port);
    }

    TEST_CASE("waitForRequestWithTimeout - infinite timeout (UINT32_MAX sentinel)")
    {
        // Create a context and listener
        auto context = ucxx::createContext({}, ucxx::Context::defaultFeatureFlags);
        auto listener = UCXListenerRegistry::addListener(0);
        uint16_t port = listener->getPort();

        // Start progress thread
        listener->startProgressThread();

        // Create client and connect
        TestClient client(listener->getWorker(), "127.0.0.1", port);

        // Wait for connection
        REQUIRE(listener->waitForConnection(5000));

        // Send data from client with a delay
        std::vector<uint8_t> sendData(1024, 99);
        std::vector<uint8_t> recvData(1024);

        // Send after a short delay in a separate thread
        std::thread senderThread(
            [&client, &sendData]()
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
                auto sendRequest = client.tagSend(sendData.data(), sendData.size(), 300);
                // Wait for send to complete
                while (!sendRequest->isCompleted())
                {
                    std::this_thread::sleep_for(std::chrono::microseconds(100));
                }
            });

        // Receive with infinite timeout - should wait until data arrives
        std::string errorMsg;
        auto result = listener->tagReceive(
            recvData.data(), recvData.size(), 300, 0xFFFFFFFFFFFFFFFF, errorMsg, g_kUcxInfiniteTimeout);
        CHECK(result == UcxReceiveResult::eSuccess);
        CHECK(recvData == sendData);

        senderThread.join();

        // Clean up
        UCXListenerRegistry::removeListener(port);
    }

    TEST_CASE("waitForRequestWithTimeout - literal zero does not wait")
    {
        // g_kUcxInfiniteTimeout is the only value that waits indefinitely. A timeout of 0
        // means "do not wait": the caller gets control back immediately instead of spinning
        // on a thread it may not be allowed to block, such as an application update thread.
        auto context = ucxx::createContext({}, ucxx::Context::defaultFeatureFlags);
        auto listener = UCXListenerRegistry::addListener(0);
        uint16_t port = listener->getPort();
        listener->startProgressThread();

        TestClient client(listener->getWorker(), "127.0.0.1", port);
        REQUIRE(listener->waitForConnection(5000));

        std::vector<uint8_t> recvData(1024);

        // Nothing is ever sent on this tag, so the receive cannot complete.
        std::string errorMsg;
        const auto start = std::chrono::steady_clock::now();
        auto result =
            listener->tagReceive(recvData.data(), recvData.size(), 300, 0xFFFFFFFFFFFFFFFF, errorMsg, /*timeout=*/0);
        const auto elapsedMs =
            std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count();

        CHECK(result == UcxReceiveResult::eTimedOut);
        CHECK(elapsedMs < 1000);

        UCXListenerRegistry::removeListener(port);
    }

    TEST_CASE("waitForRequestWithTimeout - bidirectional communication")
    {
        // Create a context and listener
        auto context = ucxx::createContext({}, ucxx::Context::defaultFeatureFlags);
        auto listener = UCXListenerRegistry::addListener(0);
        uint16_t port = listener->getPort();

        // Start progress thread
        listener->startProgressThread();

        // Create client and connect
        TestClient client(listener->getWorker(), "127.0.0.1", port);

        // Wait for connection
        REQUIRE(listener->waitForConnection(5000));

        // Test client -> server
        std::vector<uint8_t> clientData(512, 111);
        auto clientSendReq = client.tagSend(clientData.data(), clientData.size(), 400);

        std::vector<uint8_t> serverRecvData(512);
        std::string errorMsg1;
        auto result1 =
            listener->tagReceive(serverRecvData.data(), serverRecvData.size(), 400, 0xFFFFFFFFFFFFFFFF, errorMsg1, 2000);
        CHECK(result1 == UcxReceiveResult::eSuccess);
        CHECK(serverRecvData == clientData);

        // Test server -> client (async send)
        std::vector<uint8_t> serverData(512, 222);
        std::string errorMsg2;
        auto serverSendResult = listener->tagSend(serverData.data(), serverData.size(), 401, errorMsg2);
        CHECK(serverSendResult == UcxSendResult::eSuccess);

        std::vector<uint8_t> clientRecvData(512);
        auto clientRecvReq = client.tagReceive(clientRecvData.data(), clientRecvData.size(), 401);

        std::string errorMsg3;
        auto result2 = waitForRequestWithTimeout(clientRecvReq, 2000, errorMsg3);
        CHECK(result2 == UcxRequestWaitResult::eCompleted);
        CHECK(clientRecvData == serverData);

        // Clean up
        UCXListenerRegistry::removeListener(port);
    }

    TEST_CASE("tagReceiveWithRequest - posts without waiting and completes when the peer sends")
    {
        // The peer sends late, so a blocking implementation would be caught out here.
        auto context = ucxx::createContext({}, ucxx::Context::defaultFeatureFlags);
        auto listener = UCXListenerRegistry::addListener(0);
        uint16_t port = listener->getPort();
        listener->startProgressThread();

        TestClient client(listener->getWorker(), "127.0.0.1", port);
        REQUIRE(listener->waitForConnection(5000));

        // The peer sends only after this delay, so any implementation that waited for data
        // could not possibly return before it elapses.
        const int sendDelayMs = 500;
        std::vector<uint8_t> sendData(1024, 77);
        std::vector<uint8_t> recvData(1024, 0);

        std::thread senderThread(
            [&client, &sendData, sendDelayMs]()
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(sendDelayMs));
                auto sendRequest = client.tagSend(sendData.data(), sendData.size(), 500);
                waitForCompletion(sendRequest, 5000);
            });

        std::string errorMessage;
        std::shared_ptr<ucxx::Request> request;
        const auto start = std::chrono::steady_clock::now();
        auto result = listener->tagReceiveWithRequest(
            recvData.data(), recvData.size(), 500, 0xFFFFFFFFFFFFFFFF, errorMessage, request);
        const auto postMs = elapsedMsSince(start);

        CHECK(result == UcxReceiveResult::eSuccess);
        REQUIRE(request.get() != nullptr);
        CHECK(errorMessage.empty());
        // Returned well before the peer sent anything, so it cannot have waited for data.
        CHECK(postMs < sendDelayMs / 2);
        CHECK_FALSE(request->isCompleted());

        // Only now, outside the call under test, do we wait for the message to land.
        CHECK(waitForCompletion(request, 5000));
        CHECK(recvData == sendData);

        senderThread.join();
        request.reset();
        UCXListenerRegistry::removeListener(port);
    }

    TEST_CASE("tagReceiveWithRequest - stays outstanding when no message is pending")
    {
        // Nothing is ever sent on the tag, so the request cannot complete.
        auto context = ucxx::createContext({}, ucxx::Context::defaultFeatureFlags);
        auto listener = UCXListenerRegistry::addListener(0);
        uint16_t port = listener->getPort();
        listener->startProgressThread();

        TestClient client(listener->getWorker(), "127.0.0.1", port);
        REQUIRE(listener->waitForConnection(5000));

        std::vector<uint8_t> recvData(1024, 0);
        std::string errorMessage;
        std::shared_ptr<ucxx::Request> request;

        const auto start = std::chrono::steady_clock::now();
        auto result = listener->tagReceiveWithRequest(
            recvData.data(), recvData.size(), 600, 0xFFFFFFFFFFFFFFFF, errorMessage, request);
        const auto postMs = elapsedMsSince(start);

        CHECK(result == UcxReceiveResult::eSuccess);
        REQUIRE(request.get() != nullptr);
        CHECK(postMs < 1000);
        CHECK_FALSE(request->isCompleted());

        // Give the progress thread real time to run; with no matching send the request must
        // still be outstanding afterwards.
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        CHECK_FALSE(request->isCompleted());

        // Retire the request before its destination buffer goes out of scope.
        request->cancel();
        CHECK(waitForCompletion(request, 5000));
        request.reset();
        UCXListenerRegistry::removeListener(port);
    }

    TEST_CASE("tagReceiveWithRequest - listener already shut down")
    {
        // A shut-down listener has no worker to progress the request.
        auto context = ucxx::createContext({}, ucxx::Context::defaultFeatureFlags);
        auto listener = UCXListenerRegistry::addListener(0);
        uint16_t port = listener->getPort();
        listener->startProgressThread();

        TestClient client(listener->getWorker(), "127.0.0.1", port);
        REQUIRE(listener->waitForConnection(5000));

        listener->shutdown();

        std::vector<uint8_t> recvData(1024, 0);
        std::string errorMessage;
        std::shared_ptr<ucxx::Request> request;
        auto result = listener->tagReceiveWithRequest(
            recvData.data(), recvData.size(), 700, 0xFFFFFFFFFFFFFFFF, errorMessage, request);

        CHECK(result == UcxReceiveResult::eFailed);
        CHECK(request.get() == nullptr);
        // The shutdown check runs before the endpoint check, so the message names the real cause.
        CHECK(errorMessage == "Listener is shutting down or shut down");

        UCXListenerRegistry::removeListener(port);
    }

    TEST_CASE("tagReceiveWithRequest - no endpoint connected")
    {
        // No client ever connects, so there is no endpoint to post the receive on.
        auto context = ucxx::createContext({}, ucxx::Context::defaultFeatureFlags);
        auto listener = UCXListenerRegistry::addListener(0);
        uint16_t port = listener->getPort();
        listener->startProgressThread();

        REQUIRE_FALSE(listener->isConnected());

        std::vector<uint8_t> recvData(1024, 0);
        std::string errorMessage;
        std::shared_ptr<ucxx::Request> request;
        auto result = listener->tagReceiveWithRequest(
            recvData.data(), recvData.size(), 800, 0xFFFFFFFFFFFFFFFF, errorMessage, request);

        CHECK(result == UcxReceiveResult::eFailed);
        CHECK(request.get() == nullptr);
        CHECK(errorMessage == "No client connected");

        UCXListenerRegistry::removeListener(port);
    }

    TEST_CASE("tagReceiveWithRequest - cancelled request settles")
    {
        // Only once a cancelled request settles is UCX done with the buffer.
        auto context = ucxx::createContext({}, ucxx::Context::defaultFeatureFlags);
        auto listener = UCXListenerRegistry::addListener(0);
        uint16_t port = listener->getPort();
        listener->startProgressThread();

        TestClient client(listener->getWorker(), "127.0.0.1", port);
        REQUIRE(listener->waitForConnection(5000));

        std::vector<uint8_t> recvData(1024, 0);
        std::string errorMessage;
        std::shared_ptr<ucxx::Request> request;
        auto result = listener->tagReceiveWithRequest(
            recvData.data(), recvData.size(), 900, 0xFFFFFFFFFFFFFFFF, errorMessage, request);

        REQUIRE(result == UcxReceiveResult::eSuccess);
        REQUIRE(request.get() != nullptr);
        REQUIRE_FALSE(request->isCompleted());

        request->cancel();

        CHECK(waitForCompletion(request, 5000));
        // A cancelled request settles with an error rather than as a successful receive.
        CHECK(request->getStatus() != UCS_OK);
        CHECK_THROWS(request->checkError());

        request.reset();
        UCXListenerRegistry::removeListener(port);
    }
}
