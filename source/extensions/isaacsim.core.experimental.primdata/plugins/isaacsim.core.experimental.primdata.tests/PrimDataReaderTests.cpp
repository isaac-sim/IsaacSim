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
#include <carb/PluginUtils.h>
#include <carb/extras/ScopeExit.h>
#include <carb/settings/ISettings.h>

#include <doctest/doctest.h>
#include <isaacsim/core/experimental/prims/IPrimDataReaderManager.hpp>
#include <omni/usd/UsdContextIncludes.h>
//
#include <omni/usd/UsdContext.h>
#include <omni/usd/UsdManager.h>
#include <pxr/base/tf/token.h>
#include <pxr/usd/sdf/path.h>
#include <pxr/usd/usdGeom/xform.h>

#include <BufferRegistry.hpp>
#include <string>

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

    TEST_CASE("Xform views respect resetXformStack in the USD fallback")
    {
        auto* settings = carb::getCachedInterface<carb::settings::ISettings>();
        REQUIRE_UNARY(settings != nullptr);
        const bool useFabricSceneDelegate = settings->getAsBool("/app/useFabricSceneDelegate");
        CARB_SCOPE_EXIT
        {
            settings->setBool("/app/useFabricSceneDelegate", useFabricSceneDelegate);
        };
        settings->setBool("/app/useFabricSceneDelegate", false);

        const std::string contextName = "PrimDataReaderResetXformStackTest";
        auto* usdContext = omni::usd::UsdManager::createContext(contextName);
        REQUIRE_UNARY(usdContext != nullptr);
        CARB_SCOPE_EXIT
        {
            usdContext->closeStage();
            omni::usd::UsdManager::destroyContext(contextName);
        };
        REQUIRE_UNARY(usdContext->newStage());

        auto stage = usdContext->getStage();
        auto parent = pxr::UsdGeomXform::Define(stage, pxr::SdfPath("/Parent"));
        auto child = pxr::UsdGeomXform::Define(stage, pxr::SdfPath("/Parent/Child"));
        auto grandchild = pxr::UsdGeomXform::Define(stage, pxr::SdfPath("/Parent/Child/Grandchild"));
        REQUIRE_UNARY(parent.AddTranslateOp().Set(pxr::GfVec3d(10.0, 0.0, 0.0)));
        REQUIRE_UNARY(child.AddTranslateOp().Set(pxr::GfVec3d(1.0, 2.0, 3.0)));
        REQUIRE_UNARY(grandchild.AddTranslateOp().Set(pxr::GfVec3d(4.0, 0.0, 0.0)));

        auto* manager = carb::getCachedInterface<IPrimDataReaderManager>();
        REQUIRE_UNARY(manager != nullptr);
        REQUIRE_UNARY(manager->ensureInitialized(usdContext->getStageId(), -1));
        const char* paths[] = { "/Parent/Child/Grandchild" };
        auto* view = manager->getReader()->createXformView("reset_xform_stack_view", paths, 1, "newton");
        REQUIRE_UNARY(view != nullptr);

        auto checkPosition = [&](const pxr::GfVec3d& expected)
        {
            float position[3] = {};
            float orientation[4] = {};
            REQUIRE_UNARY(view->getPrimWorldTransform(paths[0], position, orientation));
            for (size_t i = 0; i < 3; ++i)
            {
                CHECK_EQ(position[i], doctest::Approx(expected[i]));
            }
        };

        checkPosition(pxr::GfVec3d(15.0, 2.0, 3.0));
        REQUIRE_UNARY(child.SetResetXformStack(true));
        checkPosition(pxr::GfVec3d(5.0, 2.0, 3.0));
        REQUIRE_UNARY(grandchild.SetResetXformStack(true));
        checkPosition(pxr::GfVec3d(4.0, 0.0, 0.0));
    }

    TEST_CASE("Stage closing releases reader-owned stage references for views")
    {
        static const std::string kContextName = "PrimDataReaderCloseCleanupTest";
        omni::usd::UsdManager::destroyContext(kContextName);
        omni::usd::UsdContext* usdContext = omni::usd::UsdManager::createContext(kContextName);
        REQUIRE_UNARY(usdContext != nullptr);
        REQUIRE_UNARY(usdContext->newStage());

        long stageId = usdContext->getStageId();
        REQUIRE_UNARY(stageId != 0);

        pxr::UsdStageRefPtr stage = usdContext->getStage();
        REQUIRE_UNARY(stage != nullptr);
        stage->DefinePrim(pxr::SdfPath("/PrimDataReaderCloseCleanupTest"), pxr::TfToken("Xform"));
        const size_t baselineRefCount = stage->GetCurrentCount();

        auto* manager = carb::getCachedInterface<IPrimDataReaderManager>();
        REQUIRE_UNARY(manager != nullptr);
        REQUIRE_UNARY(manager->ensureInitialized(stageId, -1));

        IPrimDataReader* reader = manager->getReader();
        REQUIRE_UNARY(reader != nullptr);
        CHECK_EQ(reader->getStageId(), stageId);

        const char* paths[] = { "/PrimDataReaderCloseCleanupTest" };
        IXformDataView* view = reader->createXformView("close_cleanup_view", paths, 1, "newton");
        REQUIRE_UNARY(view != nullptr);
        CHECK_UNARY(stage->GetCurrentCount() > baselineRefCount);

        stage.Reset();
        const uint64_t generationBeforeClose = reader->getGeneration();
        REQUIRE_UNARY(usdContext->closeStage());

        CHECK_EQ(reader->getStageId(), 0);
        CHECK_UNARY(reader->getGeneration() > generationBeforeClose);
        omni::usd::UsdManager::destroyContext(kContextName);
    }

    TEST_CASE("Manager reinitializes reader after invalidation")
    {
        static const std::string kContextName = "PrimDataReaderReinitializeTest";
        omni::usd::UsdManager::destroyContext(kContextName);
        omni::usd::UsdContext* usdContext = omni::usd::UsdManager::createContext(kContextName);
        REQUIRE_UNARY(usdContext != nullptr);
        REQUIRE_UNARY(usdContext->newStage());

        long stageId = usdContext->getStageId();
        REQUIRE_UNARY(stageId != 0);

        auto* manager = carb::getCachedInterface<IPrimDataReaderManager>();
        REQUIRE_UNARY(manager != nullptr);
        REQUIRE_UNARY(manager->ensureInitialized(stageId, -1));

        IPrimDataReader* reader = manager->getReader();
        REQUIRE_UNARY(reader != nullptr);
        CHECK_EQ(reader->getStageId(), stageId);
        const uint64_t generationBeforeShutdown = reader->getGeneration();

        reader->shutdown();
        CHECK_EQ(reader->getStageId(), 0);

        REQUIRE_UNARY(manager->ensureInitialized(stageId, -1));
        CHECK_EQ(reader->getStageId(), stageId);
        CHECK_UNARY(reader->getGeneration() > generationBeforeShutdown);

        usdContext->closeStage();
        omni::usd::UsdManager::destroyContext(kContextName);
    }
}
