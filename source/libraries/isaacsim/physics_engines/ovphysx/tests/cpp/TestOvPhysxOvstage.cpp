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

// ovstage attach test: exercises the exact create -> populate(file) -> wait ->
// attach sequence the umbrella OvPhysxAdapter uses, against a self-contained
// bundled scene (CartPole.usda has a PhysicsScene + rigid bodies). It steps once
// so the (lazily-created) PxScene exists, asserts it resolves via
// ovphysx_get_physx_ptr (the dependency the tensor views rely on), then detaches
// and re-attaches a second Stage to exercise the per-test re-init path. This
// makes a broken attach fail loudly at the C API seam rather than as a downstream
// tensor mismatch in the Python suite.

#include "ovphysx/ovphysx.h"
#include "ovstage/ovstage.h"
#include "ovstage/ovstage_population.h"

#include <doctest/doctest.h>

#include <string>

namespace
{

ovx_string_t createOvxString(const std::string& string)
{
    ovx_string_t value;
    value.ptr = string.c_str();
    value.length = string.size();
    return value;
}

// Mirror OvPhysxAdapter::attachUsdFile: create + populate + wait + attach.
ovstage_instance_t* attach(ovphysx_handle_t handle, const std::string& path)
{
    ovstage_instance_desc_t description{};
    description.name = "ovphysx-ovstage-test";

    ovstage_instance_t* stage = nullptr;
    if (ovstage_create_instance(&description, &stage) != OVSTAGE_OK || !stage)
    {
        FAIL("ovstage_create_instance failed");
        return nullptr;
    }

    ovstage_population_enqueue_result_t enqueueResult =
        ovstage_population_open_usd_from_file(stage, createOvxString(path), 1, 0.0, OVSTAGE_POPULATION_DOMAIN_PHYSICS);
    if (enqueueResult.status != OVSTAGE_OK)
    {
        ovx_string_t errorString = ovstage_population_get_last_error();
        const std::string error(errorString.ptr ? errorString.ptr : "", errorString.ptr ? errorString.length : 0);
        ovstage_destroy_instance(stage);
        FAIL("open_usd_from_file failed: " << error);
        return nullptr;
    }

    ovstage_population_op_wait_result_t wait{};
    ovstage_api_status_t waitStatus =
        ovstage_population_wait_op(stage, enqueueResult.op_index, OVSTAGE_TIMEOUT_INFINITE, &wait);
    if (waitStatus != OVSTAGE_OK || wait.error_op_id_count != 0)
    {
        ovstage_destroy_instance(stage);
        FAIL("population_wait_op failed (status=" << static_cast<int>(waitStatus)
                                                  << ", errors=" << wait.error_op_id_count << ")");
        return nullptr;
    }

    ovstage_write_floor_desc_t floorDescription{};
    floorDescription.ordinal = 1;
    floorDescription.scope = OVSTAGE_SCOPE_ALL;
    const ovstage_enqueue_result_t floor = ovstage_advance_write_floor(stage, &floorDescription);
    if (floor.status != OVSTAGE_OK ||
        ovstage_wait_op(stage, floor.op_index, OVSTAGE_TIMEOUT_INFINITE, nullptr) != OVSTAGE_OK)
    {
        ovstage_destroy_instance(stage);
        FAIL("ovstage_advance_write_floor failed");
        return nullptr;
    }
    (void)ovstage_release_op(stage, floor.op_index);

    if (ovphysx_attach_ovstage(handle, stage, 1).status != OVPHYSX_API_SUCCESS)
    {
        ovphysx_string_t errorString = ovphysx_get_last_error();
        const std::string error(errorString.ptr ? errorString.ptr : "", errorString.ptr ? errorString.length : 0);
        ovstage_destroy_instance(stage);
        FAIL("ovphysx_attach_ovstage failed: " << error);
        return nullptr;
    }
    return stage;
}

// Mirror OvPhysxAdapter::_detachOvstage: destroy the caller-owned Stage only when
// detach succeeds -- on failure ovphysx still owns the pointer, so retain it.
bool detach(ovphysx_handle_t handle, ovstage_instance_t* stage)
{
    ovphysx_result_t result = ovphysx_detach_ovstage(handle);
    if (result.status != OVPHYSX_API_SUCCESS)
    {
        ovphysx_string_t errorString = ovphysx_get_last_error();
        const std::string error(errorString.ptr ? errorString.ptr : "", errorString.ptr ? errorString.length : 0);
        FAIL("ovphysx_detach_ovstage failed: " << error);
        return false;
    }
    ovstage_destroy_instance(stage);
    return true;
}

// Step once (creating the lazy PxScene) and confirm the live scene resolves.
bool stepAndCheckScene(ovphysx_handle_t handle)
{
    if (ovphysx_step_sync(handle, 1.0f / 60.0f).status != OVPHYSX_API_SUCCESS)
    {
        FAIL("ovphysx_step_sync failed");
        return false;
    }
    void* scene = nullptr;
    ovphysx_result_t result =
        ovphysx_get_physx_ptr(handle, OVPHYSX_LITERAL("/physicsScene"), OVPHYSX_PHYSX_TYPE_SCENE, &scene);
    if (result.status != OVPHYSX_API_SUCCESS || scene == nullptr)
    {
        FAIL("ovphysx_get_physx_ptr(/physicsScene) failed: scene not resolved after step");
        return false;
    }
    return true;
}

} // namespace

TEST_CASE("ovphysx ovstage attach: create->populate->wait->attach + re-attach")
{
    const std::string asset = "CartPole.usda";

    REQUIRE(ovphysx_initialize().status == OVPHYSX_API_SUCCESS);
    ovphysx_register_schema_paths();

    ovphysx_create_args arguments = OVPHYSX_CREATE_ARGS_DEFAULT;
    ovphysx_handle_t handle = OVPHYSX_INVALID_HANDLE;
    REQUIRE(ovphysx_create_instance(&arguments, &handle).status == OVPHYSX_API_SUCCESS);

    // First attach: populate + attach + step + confirm the scene is live.
    ovstage_instance_t* stage = attach(handle, asset);
    REQUIRE(stage != nullptr);
    REQUIRE(stepAndCheckScene(handle));

    // Re-init path: detach + destroy, then attach a second Stage on the same
    // instance (what _doInitialize does per test) and confirm it works too.
    REQUIRE(detach(handle, stage));
    ovstage_instance_t* stage2 = attach(handle, asset);
    REQUIRE(stage2 != nullptr);
    REQUIRE(stepAndCheckScene(handle));
    REQUIRE(detach(handle, stage2));

    ovphysx_destroy_instance(handle);
    ovphysx_shutdown();
}
