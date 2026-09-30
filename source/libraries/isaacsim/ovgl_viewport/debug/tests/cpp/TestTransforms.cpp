// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "isaacsim/ovgl_viewport/debug/Viewport.hpp"

#include <doctest/doctest.h>
#include <isaacsim/common/ovstage/TransformJournal.hpp>
#include <ovstage/ovstage.h>
#include <ovstage/ovstage_population.h>
#include <ovx/path_dictionary/path_dictionary.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

namespace viewport = isaacsim::ovgl_viewport::debug;

constexpr const char* g_kSceneSuffix = R"usda(
def Camera "Camera"
{
    float2 clippingRange = (0.1, 1000)
    float focalLength = 18.147562
    float horizontalAperture = 20.955
    matrix4d xformOp:transform = ((1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 10, 1))
    uniform token[] xformOpOrder = ["xformOp:transform"]
}

def RenderVar "Color"
{
    uniform token dataType = "color4f"
    string sourceName = "LdrColor"
}

def RenderProduct "Product"
{
    rel camera = </Camera>
    rel orderedVars = [</Color>]
    int2 resolution = (128, 128)
}
)usda";

class Stage final
{
public:
    explicit Stage(const std::string& text)
    {
        ovstage_instance_desc_t description{};
        if (ovstage_create_instance(&description, &m_stage) != OVSTAGE_OK || !m_stage)
        {
            throw std::runtime_error("Unable to create OVStage test instance");
        }
        const ovx_string_t source{ text.c_str(), text.size() };
        const ovstage_population_enqueue_result_t population =
            ovstage_population_open_usd_from_string(m_stage, source, 1, 0.0, OVSTAGE_POPULATION_DOMAIN_ALL);
        if (population.status != OVSTAGE_OK ||
            ovstage_population_wait_op(m_stage, population.op_index, OVSTAGE_TIMEOUT_INFINITE, nullptr) != OVSTAGE_OK)
        {
            ovstage_destroy_instance(m_stage);
            m_stage = nullptr;
            throw std::runtime_error("Unable to populate OVStage test instance");
        }

        ovstage_write_floor_desc_t floorDescription{};
        floorDescription.ordinal = 1;
        floorDescription.scope = OVSTAGE_SCOPE_ALL;
        const ovstage_enqueue_result_t floor = ovstage_advance_write_floor(m_stage, &floorDescription);
        if (floor.status != OVSTAGE_OK ||
            ovstage_wait_op(m_stage, floor.op_index, OVSTAGE_TIMEOUT_INFINITE, nullptr) != OVSTAGE_OK)
        {
            ovstage_destroy_instance(m_stage);
            m_stage = nullptr;
            throw std::runtime_error("Unable to seal populated OVStage test instance");
        }
        (void)ovstage_release_op(m_stage, floor.op_index);
    }

    ~Stage()
    {
        if (m_stage)
        {
            ovstage_destroy_instance(m_stage);
        }
    }

    Stage(const Stage&) = delete;
    Stage& operator=(const Stage&) = delete;

    ovstage_instance_t* get() const
    {
        return m_stage;
    }

    void setTranslation(
        const std::string& path, const char* attributeName, double x, double y, double z, ovstage_ordinal_t ordinal)
    {
        setTranslation(_getPath(path), attributeName, x, y, z, ordinal);
    }

    void setTranslation(ovx_primpath_t path, const char* attributeName, double x, double y, double z, ovstage_ordinal_t ordinal)
    {
        std::array<double, 16> matrix{ 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, x, y, z, 1 };
        int64_t shape = 1;
        DLTensor tensor{};
        tensor.data = matrix.data();
        tensor.device = DLDevice{ kDLCPU, 0 };
        tensor.ndim = 1;
        tensor.dtype = DLDataType{ kDLFloat, 64, 16 };
        tensor.shape = &shape;
        ovstage_write_data_t data{};
        data.tensors = &tensor;
        data.tensor_count = 1;
        data.semantic = OVSTAGE_SEMANTIC_MATRIX;
        _writeAttribute(path, attributeName, ordinal, data);
    }

    void setTransformJournal(
        const std::string& driverPath, double x, double y, double z, ovstage_ordinal_t base, ovstage_ordinal_t ordinal)
    {
        using namespace isaacsim::common::ovstage;
        TransformJournalHeader header{};
        header.generation = 1;
        header.baseOrdinal = base;
        header.ordinal = ordinal;
        header.entryCount = 1;
        TransformJournalEntry entry{};
        entry.primPath = static_cast<uint64_t>(_getPath(driverPath));
        const std::array<double, 16> matrix{ 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, x, y, z, 1 };
        std::copy(matrix.begin(), matrix.end(), entry.worldMatrix);
        std::vector<uint8_t> payload(sizeof(header) + sizeof(entry));
        std::memcpy(payload.data(), &header, sizeof(header));
        std::memcpy(payload.data() + sizeof(header), &entry, sizeof(entry));

        int64_t shape[2] = { 1, static_cast<int64_t>(payload.size()) };
        DLTensor tensor{};
        tensor.data = payload.data();
        tensor.device = DLDevice{ kDLCPU, 0 };
        tensor.ndim = 2;
        tensor.dtype = DLDataType{ kDLUInt, 8, 1 };
        tensor.shape = shape;
        ovstage_write_data_t data{};
        data.tensors = &tensor;
        data.tensor_count = 1;
        data.semantic = OVSTAGE_SEMANTIC_NONE;
        data.is_array = true;
        _writeAttribute(_getPath(g_kTransformJournalPrimPath), g_kTransformJournalAttribute, ordinal, data);
    }

    ovx_primpath_t getPrototypeChildPath(const std::string& instanceRoot, const char* childName)
    {
        path_dictionary_instance_t* dictionary = ovstage_get_path_dictionary(m_stage);
        if (!dictionary || !dictionary->vtable)
        {
            throw std::runtime_error("Unable to get the OVStage path dictionary");
        }

        const ovx_primpath_t instancePath = _getPath(instanceRoot);
        ovx_primpath_t prototypePath = OVX_INVALID_PRIMPATH;
        if (ovstage_instancing_get_prototype_root(m_stage, instancePath, &prototypePath) != OVSTAGE_OK ||
            prototypePath == OVX_INVALID_PRIMPATH)
        {
            throw std::runtime_error("Unable to resolve the OVStage prototype root");
        }

        std::vector<ovx_token_t> tokenBuffer(16);
        ovx_token_t* prototypeTokens = nullptr;
        size_t prototypeTokenCount = 0;
        size_t processedPathCount = 0;
        if (dictionary->vtable
                    ->get_tokens_from_paths(dictionary->context, &prototypePath, 1, tokenBuffer.data(), tokenBuffer.size(),
                                            &prototypeTokens, &prototypeTokenCount, &processedPathCount)
                    .status != OVX_API_SUCCESS ||
            processedPathCount != 1 || !prototypeTokens || prototypeTokenCount == 0)
        {
            throw std::runtime_error("Unable to resolve the OVStage prototype path tokens");
        }

        const ovx_string_t childString{ childName, std::strlen(childName) };
        ovx_token_t childToken = OVX_INVALID_TOKEN;
        if (dictionary->vtable->create_tokens_from_strings(dictionary->context, &childString, 1, &childToken).status !=
                OVX_API_SUCCESS ||
            childToken == OVX_INVALID_TOKEN)
        {
            throw std::runtime_error("Unable to create an OVStage prototype child token");
        }

        std::vector<ovx_token_t> childPathTokens(prototypeTokens, prototypeTokens + prototypeTokenCount);
        childPathTokens.push_back(childToken);
        const size_t childPathTokenCount = childPathTokens.size();
        ovx_primpath_t childPath = OVX_INVALID_PRIMPATH;
        if (dictionary->vtable
                    ->create_paths_from_tokens(
                        dictionary->context, childPathTokens.data(), &childPathTokenCount, 1, &childPath)
                    .status != OVX_API_SUCCESS ||
            childPath == OVX_INVALID_PRIMPATH)
        {
            throw std::runtime_error("Unable to create the OVStage prototype child path");
        }
        return childPath;
    }

private:
    ovx_primpath_t _getPath(const std::string& path)
    {
        path_dictionary_instance_t* dictionary = ovstage_get_path_dictionary(m_stage);
        if (!dictionary || !dictionary->vtable)
        {
            throw std::runtime_error("Unable to get the OVStage path dictionary");
        }
        const ovx_string_t pathString{ path.c_str(), path.size() };
        ovx_primpath_t result = OVX_INVALID_PRIMPATH;
        if (dictionary->vtable->create_paths_from_strings(dictionary->context, &pathString, 1, &result).status !=
                OVX_API_SUCCESS ||
            result == OVX_INVALID_PRIMPATH)
        {
            throw std::runtime_error("Unable to create an OVStage path");
        }
        return result;
    }

    void _writeAttribute(ovx_primpath_t path,
                         const char* attributeName,
                         ovstage_ordinal_t ordinal,
                         const ovstage_write_data_t& data)
    {
        path_dictionary_instance_t* dictionary = ovstage_get_path_dictionary(m_stage);
        if (!dictionary)
        {
            throw std::runtime_error("Unable to get the OVStage path dictionary");
        }
        ovx_primpath_list_t pathList = OVX_INVALID_PRIMPATH_LIST;
        if (path_dictionary_create_path_list_from_paths(dictionary, &path, 1, &pathList).status != OVX_API_SUCCESS ||
            pathList == OVX_INVALID_PRIMPATH_LIST)
        {
            throw std::runtime_error("Unable to create an OVStage path list");
        }

        ovstage_query_handle_t query = OVSTAGE_INVALID_QUERY_HANDLE;
        if (ovstage_query_from_path_list(m_stage, pathList, &query) != OVSTAGE_OK || query == OVSTAGE_INVALID_QUERY_HANDLE)
        {
            path_dictionary_release_path_list_reference(dictionary, pathList);
            throw std::runtime_error("Unable to create an OVStage path query");
        }

        ovx_string_or_token_t attribute{};
        attribute.string = ovx_string_t{ attributeName, std::strlen(attributeName) };
        const bool wrote =
            _complete(ovstage_write_attribute(m_stage, query, attribute, ordinal, data, OVSTAGE_PRIM_MODE_UPSERT));
        const bool releasedQuery = _complete(ovstage_release_query(m_stage, query));
        const bool releasedPaths =
            path_dictionary_release_path_list_reference(dictionary, pathList).status == OVX_API_SUCCESS;
        if (!wrote || !releasedQuery || !releasedPaths)
        {
            throw std::runtime_error("Unable to write an OVStage test attribute");
        }

        ovstage_write_floor_desc_t floorDescription{};
        floorDescription.ordinal = ordinal;
        floorDescription.scope = OVSTAGE_SCOPE_ALL;
        if (!_complete(ovstage_advance_write_floor(m_stage, &floorDescription)))
        {
            throw std::runtime_error("Unable to seal an OVStage test attribute");
        }
    }

    bool _complete(ovstage_enqueue_result_t operation)
    {
        if (operation.status != OVSTAGE_OK || operation.op_index == OVSTAGE_INVALID_OP_ID)
        {
            return false;
        }
        ovstage_op_wait_result_t wait{};
        const bool waited = ovstage_wait_op(m_stage, operation.op_index, OVSTAGE_TIMEOUT_INFINITE, &wait) == OVSTAGE_OK &&
                            wait.error_op_id_count == 0;
        const bool released = ovstage_release_op(m_stage, operation.op_index) == OVSTAGE_OK;
        return waited && released;
    }

    ovstage_instance_t* m_stage{ nullptr };
};

viewport::Frame renderScene(const std::string& scene)
{
    Stage stage("#usda 1.0\n" + scene + g_kSceneSuffix);
    viewport::ViewportConfiguration configuration;
    configuration.visible = false;
    configuration.renderProductPath = "/Product";
    viewport::Viewport renderer(
        stage.get(), [](const viewport::CameraPose&) {}, configuration);
    return renderer.render();
}

double getForegroundCentroidX(const viewport::Frame& frame)
{
    double weightedX = 0.0;
    size_t foregroundPixels = 0;
    for (uint32_t y = 0; y < frame.height; ++y)
    {
        for (uint32_t x = 0; x < frame.width; ++x)
        {
            const size_t offset = (static_cast<size_t>(y) * frame.width + x) * 4;
            const uint32_t brightness = std::to_integer<uint8_t>(frame.rgba[offset]) +
                                        std::to_integer<uint8_t>(frame.rgba[offset + 1]) +
                                        std::to_integer<uint8_t>(frame.rgba[offset + 2]);
            if (brightness > 60)
            {
                weightedX += x;
                ++foregroundPixels;
            }
        }
    }
    REQUIRE_UNARY(foregroundPixels > 0);
    return weightedX / static_cast<double>(foregroundPixels);
}

size_t getForegroundPixelCount(const viewport::Frame& frame)
{
    size_t foregroundPixels = 0;
    for (uint32_t y = 0; y < frame.height; ++y)
    {
        for (uint32_t x = 0; x < frame.width; ++x)
        {
            const size_t offset = (static_cast<size_t>(y) * frame.width + x) * 4;
            const uint32_t brightness = std::to_integer<uint8_t>(frame.rgba[offset]) +
                                        std::to_integer<uint8_t>(frame.rgba[offset + 1]) +
                                        std::to_integer<uint8_t>(frame.rgba[offset + 2]);
            if (brightness > 60)
                ++foregroundPixels;
        }
    }
    return foregroundPixels;
}

} // namespace

TEST_SUITE("isaacsim.ovgl_viewport.debug transforms")
{
    TEST_CASE("Publishes camera orientations in xyzw order")
    {
        Stage stage("#usda 1.0\n" + std::string(g_kSceneSuffix));
        viewport::ViewportConfiguration configuration;
        configuration.visible = false;
        configuration.renderProductPath = "/Product";
        configuration.camera.yawRadians = -std::acos(-1.0) / 2.0;
        configuration.camera.pitchRadians = 0.0;
        viewport::CameraPose publishedPose;
        bool posePublished = false;
        viewport::Viewport renderer(
            stage.get(),
            [&publishedPose, &posePublished](const viewport::CameraPose& pose)
            {
                publishedPose = pose;
                posePublished = true;
            },
            configuration);

        CHECK_UNARY(renderer.pollEvents());
        REQUIRE_UNARY(posePublished);
        const double halfSqrtTwo = std::sqrt(0.5);
        CHECK(publishedPose.orientation[0] == doctest::Approx(halfSqrtTwo));
        CHECK(publishedPose.orientation[1] == doctest::Approx(0.0));
        CHECK(publishedPose.orientation[2] == doctest::Approx(0.0));
        CHECK(publishedPose.orientation[3] == doctest::Approx(halfSqrtTwo));
    }

    TEST_CASE("Close releases resources while borrowed frame remains alive")
    {
        Stage stage("#usda 1.0\n" + std::string(g_kSceneSuffix));
        viewport::ViewportConfiguration configuration;
        configuration.visible = false;
        configuration.renderProductPath = "/Product";
        viewport::Viewport renderer(
            stage.get(), [](const viewport::CameraPose&) {}, configuration);

        const viewport::Frame& frame = renderer.render();
        renderer.close();

        CHECK_UNARY(renderer.isClosed());
        CHECK_FALSE(renderer.pollEvents());
        CHECK_UNARY(frame.width > 0);
        CHECK_THROWS_AS(renderer.render(), std::runtime_error);
        CHECK_NOTHROW(renderer.close());
    }

    TEST_CASE("Composes an ancestor transform")
    {
        const viewport::Frame frame = renderScene(R"usda(
def Xform "World"
{
    def Xform "Offset"
    {
        double3 xformOp:translate = (3, 0, 0)
        uniform token[] xformOpOrder = ["xformOp:translate"]

        def Cube "Cube"
        {
            color3f[] primvars:displayColor = [(1, 1, 1)]
            double size = 1
        }
    }
}
)usda");

        CHECK_UNARY(getForegroundCentroidX(frame) > 80.0);
    }

    TEST_CASE("Composes a scene-graph instance transform")
    {
        const viewport::Frame frame = renderScene(R"usda(
def Xform "Prototype" (
    instanceable = true
)
{
    def Cube "Cube"
    {
        color3f[] primvars:displayColor = [(1, 1, 1)]
        double size = 1
    }
}

def Xform "Instance" (
    instanceable = true
    prepend references = </Prototype>
)
{
    double3 xformOp:translate = (3, 0, 0)
    uniform token[] xformOpOrder = ["xformOp:translate"]
}
)usda");

        CHECK_UNARY(getForegroundCentroidX(frame) > 70.0);
    }

    TEST_CASE("Refreshes only the scene-graph instance below a changed ancestor")
    {
        Stage stage("#usda 1.0\n" + std::string(R"usda(
def Xform "Prototype" (
    instanceable = true
)
{
    def Xform "Nested"
    {
        double3 xformOp:translate = (0.5, 0, 0)
        uniform token[] xformOpOrder = ["xformOp:translate"]

        def Cube "Cube"
        {
            color3f[] primvars:displayColor = [(1, 1, 1)]
            double size = 1
        }
    }
}

def Xform "Left"
{
    double3 xformOp:translate = (-2, 0, 0)
    uniform token[] xformOpOrder = ["xformOp:translate"]

    def Xform "Visual" (
        instanceable = true
        prepend references = </Prototype>
    )
    {
    }
}

def Xform "Right"
{
    double3 xformOp:translate = (2, 0, 0)
    uniform token[] xformOpOrder = ["xformOp:translate"]

    def Xform "Visual" (
        instanceable = true
        prepend references = </Prototype>
    )
    {
    }
}
)usda") + g_kSceneSuffix);
        viewport::ViewportConfiguration configuration;
        configuration.visible = false;
        configuration.renderProductPath = "/Product";
        viewport::Viewport renderer(
            stage.get(), [](const viewport::CameraPose&) {}, configuration);

        const viewport::Frame& initialFrame = renderer.render();
        const double initialCentroid = getForegroundCentroidX(initialFrame);
        const size_t initialPixelCount = getForegroundPixelCount(initialFrame);

        SUBCASE("Local matrix")
        {
            stage.setTranslation("/Left", "omni:xform", 0.0, 0.0, 0.0, 2);
            const viewport::Frame& movedFrame = renderer.render();
            const double movedCentroid = getForegroundCentroidX(movedFrame);
            const size_t movedPixelCount = getForegroundPixelCount(movedFrame);

            CAPTURE(initialCentroid);
            CAPTURE(movedCentroid);
            CAPTURE(initialPixelCount);
            CAPTURE(movedPixelCount);
            CHECK_UNARY(movedCentroid > initialCentroid + 5.0);
            CHECK_UNARY(movedPixelCount > initialPixelCount * 3 / 4);
        }

        SUBCASE("Fabric world matrix")
        {
            // Physics transform publication writes this attribute spelling.
            stage.setTranslation("/Left", "omni:fabric:worldMatrix", 0.0, 0.0, 0.0, 2);
            const viewport::Frame& movedFrame = renderer.render();
            const double movedCentroid = getForegroundCentroidX(movedFrame);
            const size_t movedPixelCount = getForegroundPixelCount(movedFrame);

            CAPTURE(initialCentroid);
            CAPTURE(movedCentroid);
            CAPTURE(initialPixelCount);
            CAPTURE(movedPixelCount);
            CHECK_UNARY(movedCentroid > initialCentroid + 5.0);
            CHECK_UNARY(movedPixelCount > initialPixelCount * 3 / 4);
        }
    }

    TEST_CASE("Refreshes every scene-graph instance after a prototype transform changes")
    {
        Stage stage("#usda 1.0\n" + std::string(R"usda(
def Xform "Prototype" (
    instanceable = true
)
{
    def Xform "Nested"
    {
        def Cube "Cube"
        {
            color3f[] primvars:displayColor = [(1, 1, 1)]
            double size = 1
        }
    }
}

def Xform "Left" (
    instanceable = true
    prepend references = </Prototype>
)
{
    double3 xformOp:translate = (-2, 0, 0)
    uniform token[] xformOpOrder = ["xformOp:translate"]
}

def Xform "Right" (
    instanceable = true
    prepend references = </Prototype>
)
{
    double3 xformOp:translate = (2, 0, 0)
    uniform token[] xformOpOrder = ["xformOp:translate"]
}
)usda") + g_kSceneSuffix);
        viewport::ViewportConfiguration configuration;
        configuration.visible = false;
        configuration.renderProductPath = "/Product";
        viewport::Viewport renderer(
            stage.get(), [](const viewport::CameraPose&) {}, configuration);

        const double initialCentroid = getForegroundCentroidX(renderer.render());
        const ovx_primpath_t prototypeChild = stage.getPrototypeChildPath("/Left", "Nested");
        stage.setTranslation(prototypeChild, "omni:xform", 2.5, 0.0, 0.0, 2);
        const double movedCentroid = getForegroundCentroidX(renderer.render());

        CAPTURE(initialCentroid);
        CAPTURE(movedCentroid);
        CHECK_UNARY(movedCentroid > initialCentroid + 18.0);
    }

    TEST_CASE("Keeps consecutive direct transforms and mirror recovery coherent")
    {
        Stage stage("#usda 1.0\n" + std::string(R"usda(
def Xform "Mover"
{
    double3 xformOp:translate = (-2, 0, 0)
    uniform token[] xformOpOrder = ["xformOp:translate"]

    def Cube "Cube"
    {
        color3f[] primvars:displayColor = [(1, 1, 1)]
        double size = 1
    }
}
)usda") + g_kSceneSuffix);
        viewport::ViewportConfiguration configuration;
        configuration.visible = false;
        configuration.renderProductPath = "/Product";
        viewport::Viewport renderer(
            stage.get(), [](const viewport::CameraPose&) {}, configuration);

        const double initialCentroid = getForegroundCentroidX(renderer.render());

        stage.setTranslation("/Mover", "omni:fabric:worldMatrix", 2.0, 0.0, 0.0, 2);
        const double rightCentroid = getForegroundCentroidX(renderer.render());
        stage.setTranslation("/Mover", "omni:fabric:worldMatrix", -1.0, 0.0, 0.0, 3);
        const double returnedCentroid = getForegroundCentroidX(renderer.render());

        // Moving the camera after direct mesh updates is intentionally
        // unsupported by the batch path. It must resynchronize the stale
        // mirror before rendering this ordinal.
        stage.setTranslation("/Camera", "omni:xform", 3.0, 0.0, 10.0, 4);
        const double cameraMovedCentroid = getForegroundCentroidX(renderer.render());

        CAPTURE(initialCentroid);
        CAPTURE(rightCentroid);
        CAPTURE(returnedCentroid);
        CAPTURE(cameraMovedCentroid);
        CHECK_UNARY(rightCentroid > initialCentroid + 20.0);
        CHECK_UNARY(returnedCentroid < rightCentroid - 15.0);
        CHECK_UNARY(cameraMovedCentroid < returnedCentroid - 15.0);
    }

    TEST_CASE("Consumes continuous producer transform journals")
    {
        Stage stage("#usda 1.0\n" + std::string(R"usda(
def Xform "Mover"
{
    double3 xformOp:translate = (-2, 0, 0)
    uniform token[] xformOpOrder = ["xformOp:translate"]

    def Cube "Cube"
    {
        color3f[] primvars:displayColor = [(1, 1, 1)]
        double size = 1
    }
}
)usda") + g_kSceneSuffix);
        viewport::ViewportConfiguration configuration;
        configuration.visible = false;
        configuration.renderProductPath = "/Product";
        viewport::Viewport renderer(
            stage.get(), [](const viewport::CameraPose&) {}, configuration);

        const double initialCentroid = getForegroundCentroidX(renderer.render());
        stage.setTransformJournal("/Mover", 2.0, 0.0, 0.0, 1, 2);
        const double rightCentroid = getForegroundCentroidX(renderer.render());
        stage.setTransformJournal("/Mover", 0.0, 0.0, 0.0, 1, 3);
        stage.setTransformJournal("/Mover", -1.0, 0.0, 0.0, 1, 4);
        const double returnedCentroid = getForegroundCentroidX(renderer.render());
        stage.setTranslation("/Mover", "omni:fabric:worldMatrix", 0.0, 0.0, 0.0, 5);
        const double recoveredCentroid = getForegroundCentroidX(renderer.render());

        CAPTURE(initialCentroid);
        CAPTURE(rightCentroid);
        CAPTURE(returnedCentroid);
        CAPTURE(recoveredCentroid);
        CHECK_UNARY(rightCentroid > initialCentroid + 20.0);
        CHECK_UNARY(returnedCentroid < rightCentroid - 15.0);
        CHECK_UNARY(std::fabs(recoveredCentroid - initialCentroid) < 1.0);
    }
}
