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
#include <isaacsim/physics/registration/tensors/ArrayTensorSpecification.hpp>

#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <vector>

using namespace isaacsim::physics::tensors;

namespace
{

// Concrete subclass for testing — exposes the protected `registerArrayImplementation` /
// `registerMetadata` calls and lets tests stage callbacks.
class TestableEntityView : public EntityView
{
public:
    explicit TestableEntityView(std::vector<std::string> paths) : EntityView(std::move(paths))
    {
    }

    using EntityView::registerArrayImplementation;
    using EntityView::registerMetadata;
};

// Holds a float buffer and hands out arrays viewing it, standing in for a provider that reads into
// storage it owns.
class OwnedFloatTensor
{
public:
    std::vector<float> storage;
    std::vector<int64_t> shape;

    OwnedFloatTensor(std::vector<float> data, std::vector<int64_t> dimensions)
        : storage(std::move(data)), shape(std::move(dimensions))
    {
    }

    array::Array array() const
    {
        return array::Array::fromBuffer(
            std::shared_ptr<std::byte[]>(
                std::shared_ptr<void>{}, reinterpret_cast<std::byte*>(const_cast<float*>(storage.data()))),
            storage.size() * sizeof(float), array::Shape(shape), array::Dtype::Float32(), array::Device::Cpu());
    }
};

// Stubs for registrations whose callback is never invoked: these cases only exercise discoverability
// and the specification gates. A read must produce an array, so the read stubs throw rather than
// fabricate one.
const auto unusedGet = [](const std::optional<array::Array>&, const std::optional<array::Array>&) -> array::Array
{ throw std::logic_error("unused get callback invoked"); };
const auto unusedSet = [](const array::Array&, const std::optional<array::Array>&) {};
const auto unusedMultiGet = [](const std::optional<array::Array>&,
                               const std::optional<std::vector<array::Array>>&) -> std::vector<array::Array>
{ throw std::logic_error("unused multi-get callback invoked"); };
const auto unusedMultiSet = [](const std::vector<array::Array>&, const std::optional<array::Array>&) {};

ArrayTensorSpecification makeFloat32Specification(bool indexedRead = false,
                                                  bool indexedWrite = false,
                                                  bool maskedWrite = false)
{
    ArrayTensorSpecification specification;
    specification.dtype = array::Dtype::Float32();
    specification.device = array::Device::Cpu();
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
// TEST: registerArrayImplementation + listImplementations + hasImplementation + getArrayImplementationSpecification
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

    SUBCASE("registerArrayImplementation publishes the implementation name + specification")
    {
        ArrayTensorSpecification specification = makeFloat32Specification(/*indexedRead=*/true);
        specification.shapeHint = { -1, 7 };
        bool first = view.registerArrayImplementation(
            "dof-position", ImplementationKind::eGet, GetArrayImplementationFunction(unusedGet), specification);
        REQUIRE(first == true);
        REQUIRE(view.hasImplementation("dof-position", ImplementationKind::eGet));
        REQUIRE_FALSE(view.hasImplementation("dof-position", ImplementationKind::eSet));

        ArrayTensorSpecification retrieved =
            view.getArrayImplementationSpecification("dof-position", ImplementationKind::eGet);
        REQUIRE(retrieved.dtype == array::Dtype::Float32());
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
        view.registerArrayImplementation("dof-position", ImplementationKind::eGet,
                                         GetArrayImplementationFunction(unusedGet), makeFloat32Specification());
        view.registerArrayImplementation("dof-position", ImplementationKind::eSet,
                                         SetArrayImplementationFunction(unusedSet),
                                         makeFloat32Specification(/*indexedRead=*/false, /*indexedWrite=*/true));

        REQUIRE(view.hasImplementation("dof-position", ImplementationKind::eGet));
        REQUIRE(view.hasImplementation("dof-position", ImplementationKind::eSet));

        REQUIRE(view.listImplementations(ImplementationKind::eGet).size() == 1);
        REQUIRE(view.listImplementations(ImplementationKind::eSet).size() == 1);

        // Get and Set specifications are stored separately.
        auto getSpecification = view.getArrayImplementationSpecification("dof-position", ImplementationKind::eGet);
        auto setSpecification = view.getArrayImplementationSpecification("dof-position", ImplementationKind::eSet);
        REQUIRE(getSpecification.supportsIndexedWrite == false);
        REQUIRE(setSpecification.supportsIndexedWrite == true);
    }

    SUBCASE("hasImplementation returns false for missing names regardless of kind")
    {
        view.registerArrayImplementation("dof-position", ImplementationKind::eGet,
                                         GetArrayImplementationFunction(unusedGet), makeFloat32Specification());
        REQUIRE_FALSE(view.hasImplementation("dof-velocity", ImplementationKind::eGet));
        REQUIRE_FALSE(view.hasImplementation("dof-velocity", ImplementationKind::eSet));
    }

    SUBCASE("getArrayImplementationSpecification for an unknown implementation throws std::out_of_range")
    {
        // Callers must `hasImplementation(...)` first — `getArrayImplementationSpecification` doesn't
        // silently return a default; an unknown implementation is a bug worth
        // surfacing as an exception.
        REQUIRE_THROWS_AS(
            view.getArrayImplementationSpecification("nonexistent", ImplementationKind::eGet), std::out_of_range);
    }

    SUBCASE("single and multi GET callbacks cannot share one registration identity")
    {
        view.registerArrayImplementation("shared-name", ImplementationKind::eGet,
                                         GetArrayImplementationFunction(unusedGet), makeFloat32Specification());
        REQUIRE_THROWS_AS(view.registerArrayImplementation("shared-name", ImplementationKind::eGet,
                                                           GetArrayMultiImplementationFunction(unusedMultiGet),
                                                           makeFloat32Specification()),
                          std::invalid_argument);

        TestableEntityView reverseView({});
        reverseView.registerArrayImplementation("shared-name", ImplementationKind::eGet,
                                                GetArrayMultiImplementationFunction(unusedMultiGet),
                                                makeFloat32Specification());
        REQUIRE_THROWS_AS(reverseView.registerArrayImplementation("shared-name", ImplementationKind::eGet,
                                                                  GetArrayImplementationFunction(unusedGet),
                                                                  makeFloat32Specification()),
                          std::invalid_argument);
    }

    SUBCASE("single and multi SET callbacks cannot share one registration identity")
    {
        view.registerArrayImplementation("shared-name", ImplementationKind::eSet,
                                         SetArrayMultiImplementationFunction(unusedMultiSet), makeFloat32Specification());
        REQUIRE_THROWS_AS(
            view.registerArrayImplementation("shared-name", ImplementationKind::eSet,
                                             SetArrayImplementationFunction(unusedSet), makeFloat32Specification()),
            std::invalid_argument);

        TestableEntityView reverseView({});
        reverseView.registerArrayImplementation("shared-name", ImplementationKind::eSet,
                                                SetArrayImplementationFunction(unusedSet), makeFloat32Specification());
        REQUIRE_THROWS_AS(reverseView.registerArrayImplementation("shared-name", ImplementationKind::eSet,
                                                                  SetArrayMultiImplementationFunction(unusedMultiSet),
                                                                  makeFloat32Specification()),
                          std::invalid_argument);
    }
}

//=============================================================================
// TEST: getArrayData / setArrayData round-trip — the GetArrayImplementationFunction / SetArrayImplementationFunction
// callbacks fire and receive the right array arguments.
//=============================================================================
TEST_CASE("EntityView: getArrayData / setArrayData dispatch to registered callback")
{
    TestableEntityView view({});

    SUBCASE("getArrayData invokes the registered callback and returns its array")
    {
        OwnedFloatTensor result({ 1.0f, 2.0f, 3.0f }, { 3 });

        bool called = false;
        view.registerArrayImplementation("dof-position", ImplementationKind::eGet,
                                         GetArrayImplementationFunction(
                                             [&](const std::optional<array::Array>& /*indices*/,
                                                 const std::optional<array::Array>& /*output*/) -> array::Array
                                             {
                                                 called = true;
                                                 return result.array();
                                             }),
                                         makeFloat32Specification());

        const array::Array returned = view.getArrayData("dof-position");
        REQUIRE(called == true);
        REQUIRE(returned.dtype() == array::Dtype::Float32());
        REQUIRE(returned.ndim() == 1);
        REQUIRE(returned.shape()[0] == 3);
        REQUIRE(returned.data() == result.storage.data());
    }

    SUBCASE("setArrayData forwards data + indices to the callback verbatim")
    {
        std::optional<array::Array> capturedData;
        std::optional<array::Array> capturedIndices;
        view.registerArrayImplementation("dof-position", ImplementationKind::eSet,
                                         SetArrayImplementationFunction(
                                             [&](const array::Array& data, const std::optional<array::Array>& indices)
                                             {
                                                 capturedData = data;
                                                 capturedIndices = indices;
                                             }),
                                         makeFloat32Specification(/*indexedRead=*/false, /*indexedWrite=*/true));

        const array::Array data(std::vector<float>{ 4.0f, 5.0f, 6.0f });
        const array::Array indices(std::vector<int32_t>{ 7, 11 });

        view.setArrayData("dof-position", data, indices);
        REQUIRE(capturedData.has_value());
        REQUIRE(capturedData->data() == data.data());
        REQUIRE(capturedData->ndim() == 1);
        REQUIRE(capturedData->shape()[0] == 3);
        REQUIRE(capturedIndices.has_value());
        REQUIRE(capturedIndices->ndim() == 1);
        REQUIRE(capturedIndices->shape()[0] == 2);
    }

    SUBCASE("an omitted selection reaches the callback as std::nullopt")
    {
        OwnedFloatTensor result({ 1.0f }, { 1 });
        bool sawSelection = true;
        view.registerArrayImplementation("dof-position", ImplementationKind::eGet,
                                         GetArrayImplementationFunction(
                                             [&](const std::optional<array::Array>& indices,
                                                 const std::optional<array::Array>& /*output*/) -> array::Array
                                             {
                                                 sawSelection = indices.has_value();
                                                 return result.array();
                                             }),
                                         makeFloat32Specification());

        view.getArrayData("dof-position");
        REQUIRE(sawSelection == false);
    }

    SUBCASE("getArrayData on an unknown implementation throws std::out_of_range")
    {
        REQUIRE_THROWS_AS(view.getArrayData("nonexistent"), std::out_of_range);
    }

    SUBCASE("setArrayData on an unknown implementation throws std::out_of_range")
    {
        REQUIRE_THROWS_AS(view.setArrayData("nonexistent", array::Array(std::vector<float>{ 1.0f })), std::out_of_range);
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
TEST_CASE("EntityView: ArrayTensorSpecification supports flag hides the implementation from hasImplementation")
{
    TestableEntityView view({});

    ArrayTensorSpecification unsupported = makeFloat32Specification();
    unsupported.supports = false;
    view.registerArrayImplementation(
        "spatial-tendon-stiffness", ImplementationKind::eGet, GetArrayImplementationFunction(unusedGet), unsupported);

    // Callers see this implementation as unavailable — the contract is "registered
    // with supports=false" == "not registered" from the consumer side.
    REQUIRE_FALSE(view.hasImplementation("spatial-tendon-stiffness", ImplementationKind::eGet));
    // Calling get/set on a supports=false implementation raises so callers don't
    // silently get bad data.
    REQUIRE_THROWS_AS(view.getArrayData("spatial-tendon-stiffness"), std::runtime_error);
}

//=============================================================================
// TEST: device
//=============================================================================
TEST_CASE("EntityView: device defaults to host and round-trips")
{
    EntityView view;

    SUBCASE("an unset device reports host memory")
    {
        REQUIRE(view.getDevice() == array::Device::Cpu());
    }

    SUBCASE("a stored device is reported back")
    {
        view.setDevice(array::Device::Cuda(3));
        REQUIRE(view.getDevice() == array::Device::Cuda(3));
        view.setDevice(array::Device::Cpu());
        REQUIRE(view.getDevice() == array::Device::Cpu());
    }

    SUBCASE("an engine view may report a device it resolves on each access")
    {
        // The accessor is virtual so an engine that only learns its device once stepping has begun is
        // still observed correctly through a base reference.
        class LateDeviceView : public EntityView
        {
        public:
            array::Device getDevice() const noexcept override
            {
                return resolved;
            }
            array::Device resolved{ array::Device::Cpu() };
        };

        LateDeviceView engineView;
        const EntityView& asBase = engineView;
        REQUIRE(asBase.getDevice() == array::Device::Cpu());
        engineView.resolved = array::Device::Cuda(7);
        REQUIRE(asBase.getDevice() == array::Device::Cuda(7));
    }
}

TEST_CASE("EntityView: descriptor callbacks preserve signed strides and output ownership")
{
    EntityView view;
    auto owner = std::make_shared<std::vector<float>>(std::initializer_list<float>{ 1.f, 2.f, 3.f });
    TensorDescription output;
    output.data = owner->data() + 2;
    output.dtype = DType::eFloat32;
    output.shape = { 3 };
    output.strides = { -1 };
    output.keepAlive = owner;
    TensorSpecification specification;
    specification.dtype = DType::eUnknown;
    specification.deviceKind = DeviceKind::eEngineDefault;
    GetImplementationFunction callback = [](const TensorDescription& indices, const TensorDescription& destination)
    {
        CHECK(indices.isEmpty());
        auto* values = static_cast<float*>(destination.data);
        values[destination.strides[0]] = 7.f;
        return destination;
    };
    CHECK(view.registerImplementation("legacy", ImplementationKind::eGet, callback, specification));
    CHECK_FALSE(view.registerImplementation("legacy", ImplementationKind::eGet, callback, specification));
    const auto result = view.getData("legacy", {}, output);
    CHECK(result.data == output.data);
    CHECK(result.strides == std::vector<int64_t>{ -1 });
    CHECK((result.keepAlive == output.keepAlive));
    CHECK((*owner)[1] == 7.f);
    CHECK(view.getImplementationSpecification("legacy", ImplementationKind::eGet).dtype == DType::eUnknown);
    CHECK(view.getImplementationSpecification("legacy", ImplementationKind::eGet).deviceKind ==
          DeviceKind::eEngineDefault);
}

TEST_CASE("EntityView: legacy device overrides remain observable through Array device access")
{
    class LateDeviceView : public EntityView
    {
    public:
        int getDeviceOrdinal() const noexcept override
        {
            return selected;
        }
        int selected{ -1 };
    };
    LateDeviceView view;
    const EntityView& base = view;
    CHECK(base.getDevice().isCpu());
    view.selected = 2;
    CHECK(base.getDeviceOrdinal() == 2);
    CHECK(base.getDevice().ordinal() == 2);
}

TEST_CASE("EntityView: modern provider replacement removes the legacy raw route")
{
    EntityView view;
    GetImplementationFunction legacy = [](const TensorDescription&, const TensorDescription& output) { return output; };
    view.registerImplementation("read", ImplementationKind::eGet, legacy, TensorSpecification{});
    GetArrayImplementationFunction current = [](const std::optional<array::Array>&, const std::optional<array::Array>&)
    { return array::Array(std::vector<float>{ 4.f }); };
    CHECK_FALSE(view.registerArrayImplementation("read", ImplementationKind::eGet, current, ArrayTensorSpecification{}));
    const auto result = view.getData("read", {}, {});
    REQUIRE(static_cast<bool>(result.keepAlive));
    CHECK(result.shape == std::vector<int64_t>{ 1 });
    CHECK(static_cast<float*>(result.data)[0] == 4.f);
}

TEST_CASE("EntityView: unknown descriptor metadata cannot masquerade as Array Float32")
{
    EntityView view;
    TensorSpecification unknown;
    unknown.dtype = DType::eUnknown;
    GetImplementationFunction single = [](const TensorDescription&, const TensorDescription& output) { return output; };
    view.registerImplementation("unknown", ImplementationKind::eGet, single, unknown);
    CHECK(view.getImplementationSpecification("unknown", ImplementationKind::eGet).dtype == DType::eUnknown);
    CHECK_THROWS_AS(view.getArrayImplementationSpecification("unknown", ImplementationKind::eGet), std::invalid_argument);
    CHECK(view.getData("unknown", {}, {}).isEmpty());

    GetMultiImplementationFunction multi = [](const TensorDescription&, const std::vector<TensorDescription>& output)
    { return output; };
    view.registerImplementation(
        "multi", ImplementationKind::eGet, multi, std::vector<TensorSpecification>{ TensorSpecification{}, unknown });
    CHECK(view.getMultiImplementationSpecifications("multi", ImplementationKind::eGet)[1].dtype == DType::eUnknown);
    CHECK_THROWS_AS(
        view.getArrayMultiImplementationSpecifications("multi", ImplementationKind::eGet), std::invalid_argument);
    CHECK(view.getDataMulti("multi", {}, {}).empty());
}

TEST_CASE("EntityView: sparse legacy destinations retain ownership with modern providers")
{
    EntityView view;
    view.registerArrayImplementation(
        "multi", ImplementationKind::eGet,
        GetArrayMultiImplementationFunction{
            [](const std::optional<array::Array>&, const std::optional<std::vector<array::Array>>& output)
            {
                CHECK_FALSE(output.has_value());
                return std::vector<array::Array>{ array::Array(std::vector<float>{ 1.f, 2.f }),
                                                  array::Array(std::vector<float>{ 3.f }) };
            } },
        ArrayTensorSpecification{});
    auto owner = std::make_shared<std::vector<float>>(2, 0.f);
    TensorDescription output;
    output.data = owner->data();
    output.dtype = DType::eFloat32;
    output.shape = { 2 };
    output.keepAlive = owner;
    const auto result = view.getDataMulti("multi", {}, { output, {} });
    REQUIRE(result.size() == 2);
    CHECK(result[0].data == output.data);
    CHECK(result[0].keepAlive.get() == output.keepAlive.get());
    CHECK((*owner)[1] == 2.f);
    CHECK(static_cast<float*>(result[1].data)[0] == 3.f);
    CHECK(static_cast<bool>(result[1].keepAlive));
    CHECK_THROWS_AS(view.getDataMulti("multi", {}, { output, {}, {} }), std::invalid_argument);
    const auto omittedTail = view.getDataMulti("multi", {}, { {} });
    REQUIRE(omittedTail.size() == 2);
    CHECK(static_cast<float*>(omittedTail[1].data)[0] == 3.f);
}

TEST_CASE("EntityView: descriptor bridges retain explicitly empty arrays")
{
    TestableEntityView view({});
    TensorDescription emptySelection;
    emptySelection.dtype = DType::eInt32;
    emptySelection.shape = { 0 };
    emptySelection.device = DeviceKind::eCpu;
    TensorDescription zeroDofData;
    zeroDofData.dtype = DType::eFloat32;
    zeroDofData.shape = { 2, 0 };
    zeroDofData.device = DeviceKind::eCpu;
    CHECK_FALSE(toArray(TensorDescription{}).has_value());
    TensorDescription invalid = zeroDofData;
    invalid.shape = { 2, 1 };
    CHECK_THROWS_AS(toArray(invalid), std::invalid_argument);
    REQUIRE(toArray(emptySelection).has_value());
    CHECK(toArray(emptySelection)->shape()[0] == 0);
    REQUIRE(toArray(zeroDofData).has_value());
    CHECK(toArray(zeroDofData)->shape().shape() == std::vector<int64_t>{ 2, 0 });

    bool read = false;
    view.registerArrayImplementation("empty", ImplementationKind::eGet,
                                     GetArrayImplementationFunction(
                                         [&](const auto& indices, const auto& output)
                                         {
                                             REQUIRE(indices.has_value());
                                             CHECK(indices->shape()[0] == 0);
                                             CHECK_FALSE(output.has_value());
                                             read = true;
                                             return toArray(zeroDofData).value();
                                         }),
                                     makeFloat32Specification(true, false));
    CHECK(view.getData("empty", emptySelection, {}).shape == std::vector<int64_t>{ 2, 0 });
    CHECK(read);

    bool written = false;
    view.registerArrayImplementation("empty", ImplementationKind::eSet,
                                     SetArrayImplementationFunction(
                                         [&](const auto& data, const auto& indices)
                                         {
                                             CHECK(data.shape().shape() == std::vector<int64_t>{ 2, 0 });
                                             REQUIRE(indices.has_value());
                                             CHECK(indices->shape()[0] == 0);
                                             written = true;
                                         }),
                                     makeFloat32Specification(false, true));
    view.setData("empty", zeroDofData, emptySelection);
    CHECK(written);
}

TEST_CASE("EntityView: descriptor dispatch checks the operation before converting buffers")
{
    EntityView view;
    TensorDescription reversed;
    float values[2] = { 1.f, 2.f };
    reversed.data = values + 1;
    reversed.dtype = DType::eFloat32;
    reversed.device = DeviceKind::eCpu;
    reversed.shape = { 2 };
    reversed.strides = { -1 };
    CHECK_THROWS_AS(view.getData("missing", {}, reversed), std::out_of_range);
    CHECK_THROWS_AS(view.getDataMulti("missing", {}, { reversed }), std::out_of_range);
    CHECK_THROWS_AS(view.setData("missing", {}, {}), std::out_of_range);
    CHECK_THROWS_AS(view.setDataMulti("missing", { {} }, {}), std::out_of_range);

    ArrayTensorSpecification unsupported = makeFloat32Specification();
    unsupported.supports = false;
    view.registerArrayImplementation(
        "unsupported", ImplementationKind::eGet, GetArrayImplementationFunction(unusedGet), unsupported);
    CHECK_THROWS_AS(view.getData("unsupported", {}, reversed), std::runtime_error);
    CHECK_THROWS_AS(toArray(reversed), std::invalid_argument);
}

TEST_CASE("EntityView: descriptor bridge accepts irrelevant negative strides")
{
    EntityView view;
    auto owner = std::make_shared<std::vector<float>>(2, 0.f);
    TensorDescription output;
    output.data = owner->data();
    output.dtype = DType::eFloat32;
    output.device = DeviceKind::eCpu;
    output.shape = { 2, 1 };
    output.strides = { 1, -1 };
    output.keepAlive = owner;
    int32_t selected = 7;
    TensorDescription indices;
    indices.data = &selected;
    indices.dtype = DType::eInt32;
    indices.device = DeviceKind::eCpu;
    indices.shape = { 1 };
    indices.strides = { -1 };
    view.registerArrayImplementation("read", ImplementationKind::eGet,
                                     GetArrayImplementationFunction(
                                         [](const auto& selection, const auto& destination)
                                         {
                                             REQUIRE(selection.has_value());
                                             CHECK(static_cast<const int32_t*>(selection->data())[0] == 7);
                                             REQUIRE(destination.has_value());
                                             CHECK(destination->strides()[1] >= 0);
                                             auto* data =
                                                 const_cast<float*>(static_cast<const float*>(destination->data()));
                                             data[0] = 3.f;
                                             data[1] = 4.f;
                                             return *destination;
                                         }),
                                     makeFloat32Specification(true));
    const auto result = view.getData("read", indices, output);
    CHECK((*owner)[0] == 3.f);
    CHECK((*owner)[1] == 4.f);
    CHECK(static_cast<bool>(result.keepAlive));

    TensorDescription empty = output;
    empty.data = nullptr;
    empty.shape = { 0, 3 };
    empty.strides = { -3, -1 };
    bool sawEmpty = false;
    view.registerArrayImplementation("empty-read", ImplementationKind::eGet,
                                     GetArrayImplementationFunction(
                                         [&](const auto&, const auto& destination)
                                         {
                                             REQUIRE(destination.has_value());
                                             CHECK(destination->size() == 0);
                                             sawEmpty = true;
                                             return array::Array(std::vector<float>{ 1.f });
                                         }),
                                     makeFloat32Specification());
    view.getData("empty-read", {}, empty);
    CHECK(sawEmpty);
}

TEST_CASE("EntityView: omitted descriptor setter data keeps command slots")
{
    EntityView view;
    int32_t selected = 5;
    TensorDescription indices;
    indices.data = &selected;
    indices.dtype = DType::eInt32;
    indices.device = DeviceKind::eCpu;
    indices.shape = { 1 };
    bool called = false;
    view.registerArrayImplementation("sleep", ImplementationKind::eSet,
                                     SetArrayImplementationFunction(
                                         [&](const auto& data, const auto& selection)
                                         {
                                             CHECK(data.size() == 0);
                                             REQUIRE(selection.has_value());
                                             CHECK(static_cast<const int32_t*>(selection->data())[0] == 5);
                                             called = true;
                                         }),
                                     makeFloat32Specification(false, true));
    view.setData("sleep", {}, indices);
    CHECK(called);

    view.registerArrayImplementation("multi", ImplementationKind::eSet,
                                     SetArrayMultiImplementationFunction(
                                         [](const auto& data, const auto& selection)
                                         {
                                             CHECK_FALSE(selection.has_value());
                                             REQUIRE(data.size() == 2);
                                             CHECK(data[0].size() == 0);
                                             CHECK(data[1].size() == 1);
                                         }),
                                     makeFloat32Specification());
    float value = 9.f;
    TensorDescription supplied;
    supplied.data = &value;
    supplied.dtype = DType::eFloat32;
    supplied.device = DeviceKind::eCpu;
    supplied.shape = { 1 };
    view.setDataMulti("multi", { {}, supplied }, {});

    bool mutated = false;
    view.registerArrayImplementation("required", ImplementationKind::eSet,
                                     SetArrayImplementationFunction(
                                         [&](const auto& data, const auto&)
                                         {
                                             if (data.size() != 1)
                                                 throw std::invalid_argument("required value missing");
                                             mutated = true;
                                         }),
                                     makeFloat32Specification());
    CHECK_THROWS_AS(view.setData("required", {}, {}), std::invalid_argument);
    CHECK_FALSE(mutated);
}
