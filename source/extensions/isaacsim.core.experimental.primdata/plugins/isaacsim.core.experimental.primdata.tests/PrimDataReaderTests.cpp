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

#include <carb/BindingsUtils.h>

#include <doctest/doctest.h>

#include <BufferRegistry.h>

CARB_BINDINGS("isaacsim.core.experimental.primdata.tests")

using namespace isaacsim::core::experimental::prims;

TEST_SUITE("isaacsim.core.experimental.primdata.tests")
{
    TEST_CASE("ViewData::getOrCreateField allocates buffer on CPU")
    {
        ViewData data;
        data.deviceOrdinal = -1;
        auto& field = data.getOrCreateField<float>("dof_positions", 100, -1);
        CHECK_UNARY(field.buffer != nullptr);
        CHECK_EQ(field.buffer->size(), 100);
        CHECK_EQ(field.count, 100);
        CHECK_UNARY(field.buffer->data() != nullptr);
    }

    TEST_CASE("ViewData::getOrCreateField reuses existing buffer")
    {
        ViewData data;
        data.deviceOrdinal = -1;
        auto& field1 = data.getOrCreateField<float>("dof_positions", 50, -1);
        float* ptr1 = field1.buffer->data();
        auto& field2 = data.getOrCreateField<float>("dof_positions", 50, -1);
        CHECK_EQ(field2.buffer->data(), ptr1);
    }

    TEST_CASE("ViewData::getOrCreateField grows buffer on larger request")
    {
        ViewData data;
        data.deviceOrdinal = -1;
        data.getOrCreateField<float>("dof_positions", 50, -1);
        auto& field2 = data.getOrCreateField<float>("dof_positions", 200, -1);
        CHECK_EQ(field2.count, 200);
        CHECK_EQ(field2.buffer->size(), 200);
    }

    TEST_CASE("Callback invocation and dirty tracking")
    {
        ViewData data;
        data.deviceOrdinal = -1;
        auto& field = data.getOrCreateField<float>("test_field", 10, -1);

        int callCount = 0;
        field.callback = [&callCount, &field]()
        {
            callCount++;
            for (size_t i = 0; i < field.count; ++i)
            {
                field.buffer->data()[i] = static_cast<float>(i);
            }
        };

        // lastStep starts at -1 (sentinel: never fetched)
        CHECK_EQ(field.lastStep, -1);
        int64_t currentStep = 0;

        // First access (step 0): -1 < 0 is true, so callback fires
        if (field.lastStep < currentStep && field.callback)
        {
            field.callback();
            field.lastStep = currentStep;
        }
        CHECK_EQ(callCount, 1);
        CHECK_EQ(field.buffer->data()[0], 0.0f);
        CHECK_EQ(field.buffer->data()[9], 9.0f);

        // Second access at same step: 0 < 0 is false, callback does NOT fire
        if (field.lastStep < currentStep && field.callback)
        {
            field.callback();
            field.lastStep = currentStep;
        }
        CHECK_EQ(callCount, 1);

        // Step 1: 0 < 1 is true, callback fires again
        currentStep = 1;
        if (field.lastStep < currentStep && field.callback)
        {
            field.callback();
            field.lastStep = currentStep;
        }
        CHECK_EQ(callCount, 2);
    }

    TEST_CASE("Multiple fields on same view")
    {
        ViewData data;
        data.deviceOrdinal = -1;
        data.getOrCreateField<float>("dof_positions", 18, -1);
        data.getOrCreateField<float>("dof_velocities", 18, -1);
        data.getOrCreateField<float>("root_transforms", 14, -1);

        CHECK_EQ(data.fieldsF.size(), 3);
        CHECK_UNARY(data.fieldsF.at("dof_positions").buffer->data() != nullptr);
        CHECK_UNARY(data.fieldsF.at("dof_velocities").buffer->data() != nullptr);
        CHECK_UNARY(data.fieldsF.at("root_transforms").buffer->data() != nullptr);
    }

    TEST_CASE("GPU buffer allocation")
    {
        ViewData data;
        data.deviceOrdinal = 0;
        auto& field = data.getOrCreateField<float>("dof_positions", 100, 0);
        CHECK_UNARY(field.buffer != nullptr);
        CHECK_EQ(field.buffer->size(), 100);
        CHECK_EQ(field.buffer->type(), isaacsim::core::includes::MemoryType::eDevice);
    }

    TEST_CASE("Host staging buffer created on demand")
    {
        ViewData data;
        data.deviceOrdinal = 0;
        auto& field = data.getOrCreateField<float>("test", 10, 0);
        CHECK_UNARY(field.hostStaging == nullptr);

        field.hostStaging = std::make_unique<isaacsim::core::includes::GenericBufferBase<float>>(10, -1);
        CHECK_UNARY(field.hostStaging != nullptr);
        CHECK_EQ(field.hostStaging->type(), isaacsim::core::includes::MemoryType::eHost);
    }

    TEST_CASE("FieldEntry default state")
    {
        FieldEntry<float> entry;
        CHECK_EQ(entry.lastStep, -1);
        CHECK_EQ(entry.count, 0);
        CHECK_UNARY(entry.buffer == nullptr);
        CHECK_UNARY(entry.hostStaging == nullptr);
        CHECK_UNARY(!entry.callback);
    }
}
