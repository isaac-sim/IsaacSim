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

#include "TestBase.h"

#include <carb/BindingsUtils.h>

#include <doctest/doctest.h>

#include <memory>
#include <string>

// CARB_BINDINGS moved to first test file

TEST_SUITE("isaacsim.ros2.core.node_handle_tests")
{
    TEST_CASE("Ros2NodeHandle: creation and initialization")
    {
        ROS2_TEST_SETUP();

        // Create context handle first
        auto ctx = testBase.getFactory()->createContextHandle();
        CHECK(ctx != nullptr);

        int argc = 0;
        char** argv = nullptr;
        ctx->init(argc, argv);
        CHECK(ctx->isValid());

        // Create node handle
        auto node = testBase.getFactory()->createNodeHandle("test_node", "test_namespace", ctx.get());
        CHECK(node != nullptr);

        // Cleanup
        CHECK(ctx->shutdown("test-node-handle"));
    }

    TEST_CASE("Ros2NodeHandle: with empty namespace")
    {
        ROS2_TEST_SETUP();

        auto ctx = testBase.getFactory()->createContextHandle();
        CHECK(ctx != nullptr);

        int argc = 0;
        char** argv = nullptr;
        ctx->init(argc, argv);
        CHECK(ctx->isValid());

        // Create node with empty namespace
        auto node = testBase.getFactory()->createNodeHandle("test_node", "", ctx.get());
        CHECK(node != nullptr);

        CHECK(ctx->shutdown("test-empty-namespace"));
    }

    TEST_CASE("Ros2NodeHandle: with null context should fail")
    {
        ROS2_TEST_SETUP();

        // Try to create node with null context
        auto node = testBase.getFactory()->createNodeHandle("test_node", "test_namespace", nullptr);
        CHECK(node == nullptr);
    }

    TEST_CASE("Ros2NodeHandle: multiple nodes on same context")
    {
        ROS2_TEST_SETUP();

        auto ctx = testBase.getFactory()->createContextHandle();
        CHECK(ctx != nullptr);

        int argc = 0;
        char** argv = nullptr;
        ctx->init(argc, argv);
        CHECK(ctx->isValid());

        // Create multiple nodes
        auto node1 = testBase.getFactory()->createNodeHandle("test_node_1", "ns1", ctx.get());
        CHECK(node1 != nullptr);

        auto node2 = testBase.getFactory()->createNodeHandle("test_node_2", "ns2", ctx.get());
        CHECK(node2 != nullptr);

        auto node3 = testBase.getFactory()->createNodeHandle("test_node_3", "ns1", ctx.get());
        CHECK(node3 != nullptr);

        CHECK(ctx->shutdown("test-multiple-nodes"));
    }

    TEST_CASE("Ros2NodeHandle: name validation")
    {
        ROS2_TEST_SETUP();

        // Test node name validation
        CHECK(testBase.getFactory()->validateNodeName("valid_node_name"));
        CHECK(testBase.getFactory()->validateNodeName("node123"));
        CHECK(testBase.getFactory()->validateNodeName("_private_node"));

        CHECK_FALSE(testBase.getFactory()->validateNodeName(""));
        CHECK_FALSE(testBase.getFactory()->validateNodeName("node/with/slash"));
        CHECK_FALSE(testBase.getFactory()->validateNodeName("node-with-dash"));
        CHECK_FALSE(testBase.getFactory()->validateNodeName("123node")); // starts with number

        // Test namespace validation
        CHECK(testBase.getFactory()->validateNamespaceName("/valid/namespace"));
        CHECK_FALSE(testBase.getFactory()->validateNamespaceName("valid_namespace"));
        CHECK_FALSE(testBase.getFactory()->validateNamespaceName("")); // empty is valid

        CHECK_FALSE(testBase.getFactory()->validateNamespaceName("namespace with space"));
        CHECK_FALSE(testBase.getFactory()->validateNamespaceName("namespace-with-dash"));
    }
}
