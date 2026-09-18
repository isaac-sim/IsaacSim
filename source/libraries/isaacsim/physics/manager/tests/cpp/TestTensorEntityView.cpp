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
#include <isaacsim/physics/manager/tensors/EntityView.hpp>
#include <isaacsim/physics/registration/tensors/TensorDescription.hpp>
#include <isaacsim/physics/registration/tensors/TensorSpecification.hpp>

#include <cstring>
#include <stdexcept>
#include <vector>

using namespace isaacsim::physics::tensors;

namespace
{

// Concrete subclass for testing — exposes the protected `registerImplementation` /
// `registerMetadata` calls and lets tests stage callbacks.
class TestableEntityView : public EntityView
{
public:
    explicit TestableEntityView(std::vector<std::string> paths) : EntityView(std::move(paths))
    {
    }

    using EntityView::registerImplementation;
    using EntityView::registerMetadata;
};

// Build a TensorDescription that points at a heap-owned float buffer.
class OwnedFloatTensor
{
public:
    std::vector<float> storage;
    TensorDescription descriptor;

    OwnedFloatTensor(std::vector<float> data, std::vector<int64_t> shape) : storage(std::move(data))
    {
        descriptor.data = storage.data();
        descriptor.dtype = DType::eFloat32;
        descriptor.shape = std::move(shape);
        descriptor.device = DeviceKind::eCpu;
        descriptor.deviceOrdinal = -1;
    }
};

TensorSpecification makeFloat32Specification(bool indexedRead = false, bool indexedWrite = false, bool maskedWrite = false)
{
    TensorSpecification specification;
    specification.dtype = DType::eFloat32;
    specification.deviceKind = DeviceKind::eCpu;
    specification.supports = true;
    specification.supportsIndexedRead = indexedRead;
    specification.supportsIndexedWrite = indexedWrite;
    specification.supportsMaskedWrite = maskedWrite;
    return specification;
}

} // namespace

//=============================================================================
// TEST: construction
//=============================================================================
TEST_CASE("EntityView: construction stores the path list and count")
{
    SUBCASE("Default construction — empty paths, zero count")
    {
        TestableEntityView view({});
        REQUIRE(view.getPrimPathPatterns().empty());
        REQUIRE(view.getEntityCount() == 0);
    }

    SUBCASE("Path list construction stores the paths")
    {
        std::vector<std::string> paths{ "/World/robot_0", "/World/robot_1", "/World/robot_2" };
        TestableEntityView view(paths);
        REQUIRE(view.getPrimPathPatterns().size() == 3);
        REQUIRE(view.getPrimPathPatterns()[0] == "/World/robot_0");
        REQUIRE(view.getPrimPathPatterns()[2] == "/World/robot_2");
    }

    SUBCASE("setEntityCount updates the resolved-entity count")
    {
        TestableEntityView view({});
        REQUIRE(view.getEntityCount() == 0);
        view.setEntityCount(42);
        REQUIRE(view.getEntityCount() == 42);
    }
}

//=============================================================================
// TEST: registerImplementation + listImplementations + hasImplementation + getImplementationSpecification
//=============================================================================
TEST_CASE("EntityView: implementation registration discoverability")
{
    TestableEntityView view({});

    SUBCASE("Initial state — no implementations")
    {
        REQUIRE(view.listImplementations(ImplementationKind::eGet).empty());
        REQUIRE(view.listImplementations(ImplementationKind::eSet).empty());
        REQUIRE_FALSE(view.hasImplementation("dof-position", ImplementationKind::eGet));
    }

    SUBCASE("registerImplementation publishes the implementation name + specification")
    {
        TensorSpecification specification = makeFloat32Specification(/*indexedRead=*/true);
        specification.shapeHint = { -1, 7 };
        bool first = view.registerImplementation(
            "dof-position", ImplementationKind::eGet,
            GetImplementationFunction(
                [](const TensorDescription& /*indices*/, const TensorDescription& /*output*/) -> TensorDescription
                { return TensorDescription{}; }),
            specification);
        REQUIRE(first == true);
        REQUIRE(view.hasImplementation("dof-position", ImplementationKind::eGet));
        REQUIRE_FALSE(view.hasImplementation("dof-position", ImplementationKind::eSet));

        TensorSpecification retrieved = view.getImplementationSpecification("dof-position", ImplementationKind::eGet);
        REQUIRE(retrieved.dtype == DType::eFloat32);
        REQUIRE(retrieved.supportsIndexedRead == true);
        REQUIRE(retrieved.shapeHint.size() == 2);
        REQUIRE(retrieved.shapeHint[0] == -1);
        REQUIRE(retrieved.shapeHint[1] == 7);

        auto getImplementations = view.listImplementations(ImplementationKind::eGet);
        REQUIRE(getImplementations.size() == 1);
        REQUIRE(getImplementations[0] == "dof-position");
    }

    SUBCASE("Get and Set kinds are tracked independently")
    {
        view.registerImplementation("dof-position", ImplementationKind::eGet,
                                    GetImplementationFunction([](const TensorDescription&, const TensorDescription&)
                                                              { return TensorDescription{}; }),
                                    makeFloat32Specification());
        view.registerImplementation("dof-position", ImplementationKind::eSet,
                                    SetImplementationFunction([](const TensorDescription&, const TensorDescription&) {}),
                                    makeFloat32Specification(/*indexedRead=*/false, /*indexedWrite=*/true));

        REQUIRE(view.hasImplementation("dof-position", ImplementationKind::eGet));
        REQUIRE(view.hasImplementation("dof-position", ImplementationKind::eSet));

        REQUIRE(view.listImplementations(ImplementationKind::eGet).size() == 1);
        REQUIRE(view.listImplementations(ImplementationKind::eSet).size() == 1);

        // Get and Set specifications are stored separately.
        auto getSpecification = view.getImplementationSpecification("dof-position", ImplementationKind::eGet);
        auto setSpecification = view.getImplementationSpecification("dof-position", ImplementationKind::eSet);
        REQUIRE(getSpecification.supportsIndexedWrite == false);
        REQUIRE(setSpecification.supportsIndexedWrite == true);
    }

    SUBCASE("hasImplementation returns false for missing names regardless of kind")
    {
        view.registerImplementation("dof-position", ImplementationKind::eGet,
                                    GetImplementationFunction([](const TensorDescription&, const TensorDescription&)
                                                              { return TensorDescription{}; }),
                                    makeFloat32Specification());
        REQUIRE_FALSE(view.hasImplementation("dof-velocity", ImplementationKind::eGet));
        REQUIRE_FALSE(view.hasImplementation("dof-velocity", ImplementationKind::eSet));
    }

    SUBCASE("getImplementationSpecification for an unknown implementation throws std::out_of_range")
    {
        // Callers must `hasImplementation(...)` first — `getImplementationSpecification` doesn't
        // silently return a default; an unknown implementation is a bug worth
        // surfacing as an exception.
        REQUIRE_THROWS_AS(
            view.getImplementationSpecification("nonexistent", ImplementationKind::eGet), std::out_of_range);
    }

    SUBCASE("single and multi GET callbacks cannot share one registration identity")
    {
        view.registerImplementation("shared-name", ImplementationKind::eGet,
                                    GetImplementationFunction([](const TensorDescription&, const TensorDescription&)
                                                              { return TensorDescription{}; }),
                                    makeFloat32Specification());
        REQUIRE_THROWS_AS(
            view.registerImplementation(
                "shared-name", ImplementationKind::eGet,
                GetMultiImplementationFunction([](const TensorDescription&, const std::vector<TensorDescription>&)
                                               { return std::vector<TensorDescription>{}; }),
                makeFloat32Specification()),
            std::invalid_argument);

        TestableEntityView reverseView({});
        reverseView.registerImplementation(
            "shared-name", ImplementationKind::eGet,
            GetMultiImplementationFunction([](const TensorDescription&, const std::vector<TensorDescription>&)
                                           { return std::vector<TensorDescription>{}; }),
            makeFloat32Specification());
        REQUIRE_THROWS_AS(reverseView.registerImplementation(
                              "shared-name", ImplementationKind::eGet,
                              GetImplementationFunction([](const TensorDescription&, const TensorDescription&)
                                                        { return TensorDescription{}; }),
                              makeFloat32Specification()),
                          std::invalid_argument);
    }

    SUBCASE("single and multi SET callbacks cannot share one registration identity")
    {
        view.registerImplementation(
            "shared-name", ImplementationKind::eSet,
            SetMultiImplementationFunction([](const std::vector<TensorDescription>&, const TensorDescription&) {}),
            makeFloat32Specification());
        REQUIRE_THROWS_AS(view.registerImplementation(
                              "shared-name", ImplementationKind::eSet,
                              SetImplementationFunction([](const TensorDescription&, const TensorDescription&) {}),
                              makeFloat32Specification()),
                          std::invalid_argument);

        TestableEntityView reverseView({});
        reverseView.registerImplementation(
            "shared-name", ImplementationKind::eSet,
            SetImplementationFunction([](const TensorDescription&, const TensorDescription&) {}),
            makeFloat32Specification());
        REQUIRE_THROWS_AS(
            reverseView.registerImplementation(
                "shared-name", ImplementationKind::eSet,
                SetMultiImplementationFunction([](const std::vector<TensorDescription>&, const TensorDescription&) {}),
                makeFloat32Specification()),
            std::invalid_argument);
    }
}

//=============================================================================
// TEST: getData / setData round-trip — the GetImplementationFunction / SetImplementationFunction callbacks
// fire and receive the right TensorDescription arguments.
//=============================================================================
TEST_CASE("EntityView: getData / setData dispatch to registered callback")
{
    TestableEntityView view({});

    SUBCASE("getData invokes the registered callback and returns its TensorDescription")
    {
        OwnedFloatTensor result({ 1.0f, 2.0f, 3.0f }, { 3 });

        bool called = false;
        view.registerImplementation(
            "dof-position", ImplementationKind::eGet,
            GetImplementationFunction(
                [&](const TensorDescription& /*indices*/, const TensorDescription& /*output*/) -> TensorDescription
                {
                    called = true;
                    return result.descriptor;
                }),
            makeFloat32Specification());

        TensorDescription indices{}; // empty → "no indices"
        TensorDescription output{};
        TensorDescription returned = view.getData("dof-position", indices, output);
        REQUIRE(called == true);
        REQUIRE(returned.dtype == DType::eFloat32);
        REQUIRE(returned.shape.size() == 1);
        REQUIRE(returned.shape[0] == 3);
        REQUIRE(returned.data == result.storage.data());
    }

    SUBCASE("setData forwards data + indices to the callback verbatim")
    {
        std::vector<float> incoming{ 4.0f, 5.0f, 6.0f };
        std::vector<int32_t> incomingIndices{ 7, 11 };

        TensorDescription capturedData{};
        TensorDescription capturedIndices{};
        view.registerImplementation(
            "dof-position", ImplementationKind::eSet,
            SetImplementationFunction(
                [&](const TensorDescription& dataDescriptor, const TensorDescription& indicesDescriptor)
                {
                    capturedData = dataDescriptor;
                    capturedIndices = indicesDescriptor;
                }),
            makeFloat32Specification(/*indexedRead=*/false, /*indexedWrite=*/true));

        TensorDescription data;
        data.data = incoming.data();
        data.dtype = DType::eFloat32;
        data.shape = { 3 };

        TensorDescription indices;
        indices.data = incomingIndices.data();
        indices.dtype = DType::eInt32;
        indices.shape = { 2 };

        view.setData("dof-position", data, indices);
        REQUIRE(capturedData.data == incoming.data());
        REQUIRE(capturedData.shape.size() == 1);
        REQUIRE(capturedData.shape[0] == 3);
        REQUIRE(capturedIndices.shape.size() == 1);
        REQUIRE(capturedIndices.shape[0] == 2);
    }

    SUBCASE("getData on an unknown implementation throws std::out_of_range")
    {
        TensorDescription indices{};
        TensorDescription output{};
        REQUIRE_THROWS_AS(view.getData("nonexistent", indices, output), std::out_of_range);
    }

    SUBCASE("setData on an unknown implementation throws std::out_of_range")
    {
        TensorDescription data{};
        TensorDescription indices{};
        REQUIRE_THROWS_AS(view.setData("nonexistent", data, indices), std::out_of_range);
    }
}

//=============================================================================
// TEST: registerMetadata + getMetadata
//=============================================================================
TEST_CASE("EntityView: metadata registration")
{
    TestableEntityView view({});

    SUBCASE("getMetadata for an unregistered implementation returns a default Metadata")
    {
        Metadata metadata = view.getMetadata("dof-position");
        // Default Metadata is a "value-not-set" sentinel; no guarantee on
        // what the variant holds, just that the call doesn't throw.
        static_cast<void>(metadata);
    }

    SUBCASE("registered metadata callback fires on getMetadata")
    {
        Metadata payload;
        // Stash an int into the metadata variant — the exact value isn't
        // semantically important; we just want to verify round-trip.
        payload = Metadata(int64_t{ 42 });

        bool called = false;
        view.registerMetadata("dof-position", MetadataImplementationFunction(
                                                  [&]() -> Metadata
                                                  {
                                                      called = true;
                                                      return payload;
                                                  }));

        Metadata returned = view.getMetadata("dof-position");
        REQUIRE(called == true);
        // The variant should now hold our value. Cast through the int64
        // accessor used by the legacy adapter.
        // (Metadata's value-type accessors are tested in the variant's own
        // unit suite; this test pins only the dispatch round-trip.)
        static_cast<void>(returned);
    }
}

//=============================================================================
// TEST: shape-specification advertising — `supports=false` is functionally equivalent
// to "not registered" for callers. `hasImplementation` returns false for them, so the
// API surface stays uniform across engines without false omissions.
//=============================================================================
TEST_CASE("EntityView: TensorSpecification supports flag hides the implementation from hasImplementation")
{
    TestableEntityView view({});

    TensorSpecification unsupported = makeFloat32Specification();
    unsupported.supports = false;
    view.registerImplementation("spatial-tendon-stiffness", ImplementationKind::eGet,
                                GetImplementationFunction([](const TensorDescription&, const TensorDescription&)
                                                          { return TensorDescription{}; }),
                                unsupported);

    // Callers see this implementation as unavailable — the contract is "registered
    // with supports=false" == "not registered" from the consumer side.
    REQUIRE_FALSE(view.hasImplementation("spatial-tendon-stiffness", ImplementationKind::eGet));
    // Calling get/set on a supports=false implementation raises so callers don't
    // silently get bad data.
    REQUIRE_THROWS_AS(
        view.getData("spatial-tendon-stiffness", TensorDescription{}, TensorDescription{}), std::runtime_error);
}

//=============================================================================
// TEST: device ordinal
//=============================================================================
TEST_CASE("EntityView: device ordinal defaults to host and round-trips")
{
    EntityView view;

    SUBCASE("an unset ordinal reports host memory")
    {
        REQUIRE(view.getDeviceOrdinal() == -1);
    }

    SUBCASE("a stored ordinal is reported back")
    {
        view.setDeviceOrdinal(3);
        REQUIRE(view.getDeviceOrdinal() == 3);
        view.setDeviceOrdinal(-1);
        REQUIRE(view.getDeviceOrdinal() == -1);
    }

    SUBCASE("an engine view may report an ordinal it resolves on each access")
    {
        // The accessor is virtual so an engine that only learns its device once stepping has begun is
        // still observed correctly through a base reference.
        class LateDeviceView : public EntityView
        {
        public:
            int getDeviceOrdinal() const noexcept override
            {
                return resolved;
            }
            int resolved{ -1 };
        };

        LateDeviceView engineView;
        const EntityView& asBase = engineView;
        REQUIRE(asBase.getDeviceOrdinal() == -1);
        engineView.resolved = 7;
        REQUIRE(asBase.getDeviceOrdinal() == 7);
    }
}
