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

#include <cstdlib>
#include <memory>
#include <string>

// CARB_BINDINGS moved to first test file

TEST_SUITE("isaacsim.ros2.core.context_handle_tests")
{

    TEST_CASE("Ros2ContextHandle: creation is non-null and initially invalid")
    {
        ROS2_TEST_SETUP();

        auto ctx = testBase.getFactory()->createContextHandle();
        CHECK(ctx != nullptr);
        CHECK_FALSE(ctx->isValid());
        CHECK(ctx->getContext() == nullptr);
    }

    TEST_CASE("Ros2ContextHandle: init/shutdown default path")
    {
        ROS2_TEST_SETUP();
        auto ctx = testBase.getFactory()->createContextHandle();
        CHECK(ctx != nullptr);

        int argc = 0;
        char** argv = nullptr;

        ctx->init(argc, argv);
        CHECK(ctx->isValid());
        CHECK(ctx->getContext() != nullptr);

        // shutdown returns true and invalidates the context
        CHECK(ctx->shutdown("unit-test default shutdown"));
        CHECK_FALSE(ctx->isValid());
        CHECK(ctx->getContext() == nullptr);
    }

    TEST_CASE("Ros2ContextHandle: init with explicit domain id")
    {
        ROS2_TEST_SETUP();
        auto ctx = testBase.getFactory()->createContextHandle();
        CHECK(ctx != nullptr);

        int argc = 0;
        char** argv = nullptr;

        // Use an arbitrary domain id; we cannot read it back, but init should succeed
        const size_t domainId = 42;
        ctx->init(argc, argv, /*setDomainId=*/true, domainId);
        CHECK(ctx->isValid());
        CHECK(ctx->getContext() != nullptr);

        CHECK(ctx->shutdown("unit-test explicit domain"));
        CHECK_FALSE(ctx->isValid());
    }

    TEST_CASE("Ros2ContextHandle: double shutdown is safe")
    {
        ROS2_TEST_SETUP();
        auto ctx = testBase.getFactory()->createContextHandle();
        CHECK(ctx != nullptr);

        int argc = 0;
        char** argv = nullptr;

        ctx->init(argc, argv);
        CHECK(ctx->isValid());

        CHECK(ctx->shutdown("first shutdown"));
        CHECK_FALSE(ctx->isValid());

        // Double shutdown should be a no-op that still returns true
        CHECK(ctx->shutdown("second shutdown"));
        CHECK_FALSE(ctx->isValid());
    }

    TEST_CASE("Ros2ContextHandle: re-initialize after shutdown")
    {
        ROS2_TEST_SETUP();
        auto ctx = testBase.getFactory()->createContextHandle();
        CHECK(ctx != nullptr);

        int argc = 0;
        char** argv = nullptr;

        ctx->init(argc, argv);
        CHECK(ctx->isValid());
        CHECK(ctx->shutdown("cycle-1"));
        CHECK_FALSE(ctx->isValid());

        // Re-initialize
        ctx->init(argc, argv);
        CHECK(ctx->isValid());
        CHECK(ctx->getContext() != nullptr);

        CHECK(ctx->shutdown("cycle-2"));
        CHECK_FALSE(ctx->isValid());
    }

    TEST_CASE("Ros2ContextHandle: shutdown with nullptr reason")
    {
        ROS2_TEST_SETUP();
        auto ctx = testBase.getFactory()->createContextHandle();
        CHECK(ctx != nullptr);

        int argc = 0;
        char** argv = nullptr;

        ctx->init(argc, argv);
        CHECK(ctx->isValid());

        // Passing nullptr reason should be handled gracefully
        CHECK(ctx->shutdown(nullptr));
        CHECK_FALSE(ctx->isValid());
    }
}
