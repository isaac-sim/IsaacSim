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

// Python.h must precede headers that can include the C or C++ standard library.
// clang-format off
#include <Python.h>
// clang-format on

#include <isaacsim/common/array/Array.hpp>
#include <isaacsim/foundation/objects/Camera.hpp>
#include <isaacsim/foundation/objects/Prim.hpp>
#include <isaacsim/foundation/objects/Stage.hpp>
#include <isaacsim/foundation/objects/lights/DistantLight.hpp>
#include <isaacsim/foundation/objects/physics_scenes/PhysicsScene.hpp>
#include <isaacsim/foundation/objects/shapes/Cube.hpp>
#include <isaacsim/foundation/prims/physics/ColliderBody.hpp>
#include <isaacsim/foundation/prims/physics/GroundPlane.hpp>
#include <isaacsim/foundation/prims/physics/RigidBody.hpp>
#include <isaacsim/physics/entities/RigidBodyEntity.hpp>
#include <isaacsim/physics/manager/PhysicsManager.hpp>
#include <ovrtx/ovrtx.h>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <exception>
#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <tuple>
#include <vector>

namespace
{

namespace array = isaacsim::common::array;
namespace objects = isaacsim::foundation::objects;
namespace prims = isaacsim::foundation::prims::physics;
namespace entities = isaacsim::physics::entities;
namespace manager = isaacsim::physics::manager;

constexpr const char* g_kCameraPath = "/Environment/Camera";
constexpr const char* g_kLightPath = "/Environment/Light";
constexpr const char* g_kRenderProductPath = "/Render/Product";
constexpr const char* g_kRenderVarPath = "/Render/Product/LdrColor";
constexpr const char* g_kRenderVarName = "LdrColor";
constexpr const char* g_kCubePath = "/World/Cube";
constexpr float g_kTimeStep = 1.0f / 60.0f;
constexpr int g_kCaptureInterval = 15;
// The cube has reached the ground once it stops descending. The step budget only bounds
// the run if it never settles.
constexpr int g_kMaxSimulationSteps = 300;
constexpr float g_kRestTravel = 1.0e-3f;
constexpr int g_kRestSteps = 3;
// Population writes the imported scene at ordinal 1, and every published pose commits a
// new revision of that ordinal, which the next render observes.
constexpr ovstage_ordinal_t g_kPopulationOrdinal = 1;

void checkOvrtx(ovrtx_api_status_t status, const char* context)
{
    if (status != OVRTX_API_SUCCESS)
    {
        const ovx_string_t error = ovrtx_get_last_error();
        throw std::runtime_error(std::string(context) + ": " +
                                 std::string(error.ptr != nullptr ? error.ptr : "", error.length));
    }
}

/**
 * @brief Adds the render-product relationships that the object API does not yet expose.
 */
void insertRenderProductRelationships(std::string& stageText)
{
    const std::string declaration =
        "def RenderProduct \"" + std::filesystem::path(g_kRenderProductPath).filename().string() + "\"";
    const size_t primStart = stageText.find(declaration);
    const size_t primBody = primStart == std::string::npos ? std::string::npos : stageText.find("{\n", primStart);
    if (primBody == std::string::npos)
    {
        throw std::runtime_error("The authored stage does not contain the render product.");
    }
    const std::string relationships = "    rel camera = <" + std::string(g_kCameraPath) + ">\n" +
                                      "    rel orderedVars = [<" + std::string(g_kRenderVarPath) + ">]\n";
    stageText.insert(primBody + 2, relationships);
}

/**
 * @brief Authors the simulated and rendered scene and returns its USDA representation.
 */
std::string authorStage()
{
    objects::Stage stage("openusd");
    stage.createStage();
    try
    {
        stage.definePrim("/World");
        const prims::GroundPlane ground("/World/Ground", array::Array(10.0f));
        objects::shapes::Cube cube(g_kCubePath, array::Array(1.0f));
        cube.setLocalPoses(array::Array(std::vector<float>{ 0.0f, 0.0f, 2.0f }));

        objects::physics_scenes::PhysicsScene physicsScene("/World/PhysicsScene");
        physicsScene.setGravities(array::Array(std::vector<float>{ 0.0f, 0.0f, -9.81f }));
        const prims::ColliderBody cubeCollider(g_kCubePath);
        const prims::RigidBody cubeRigidBody(g_kCubePath);

        objects::lights::DistantLight light(g_kLightPath);
        light.setAngles(array::Array(1.0f));
        light.setIntensities(array::Array(4000.0f));
        light.setLocalPoses(std::nullopt, array::Array(std::vector<float>{ 0.4545f, 0.0801f, 0.1540f, 0.8733f }));

        objects::Camera camera(g_kCameraPath);
        // Camera lengths use scene units; 2.4 authors the USD focalLength value 24.
        camera.setFocalLengths(array::Array(2.4));
        camera.setLocalPoses(array::Array(std::vector<double>{ 7.0, -7.0, 5.0 }),
                             array::Array(std::vector<double>{ 0.5166, 0.2140, 0.3171, 0.7658 }));

        stage.definePrim(g_kRenderProductPath, "RenderProduct");
        objects::Prim renderProduct(g_kRenderProductPath);
        renderProduct.createAttribute("resolution", "int2");
        renderProduct.setAttributeValues("resolution", array::Array(std::vector<int32_t>{ 480, 320 }));

        stage.definePrim(g_kRenderVarPath, "RenderVar");
        objects::Prim renderVar(g_kRenderVarPath);
        renderVar.createAttribute("dataType", "token");
        renderVar.createAttribute("sourceName", "string");
        renderVar.setAttributeValues("dataType", std::string("color4f"));
        renderVar.setAttributeValues("sourceName", std::string(g_kRenderVarName));

        std::string stageText = stage.exportStageToString();
        insertRenderProductRelationships(stageText);
        stage.closeStage();
        return stageText;
    }
    catch (...)
    {
        if (stage.isValid())
        {
            stage.closeStage();
        }
        throw;
    }
}

/**
 * @brief Writes an RGBA image to a binary PPM file, dropping the alpha channel.
 */
void writePpm(const std::filesystem::path& path, const uint8_t* pixels, int64_t width, int64_t height)
{
    std::ofstream file(path, std::ios::binary);
    file << "P6\n" << width << " " << height << "\n255\n";
    for (int64_t pixel = 0; pixel < width * height; ++pixel)
    {
        file.write(reinterpret_cast<const char*>(pixels + pixel * 4), 3);
    }
    if (!file)
    {
        throw std::runtime_error("Cannot write the output file: " + path.string());
    }
}

/**
 * @brief Owns the renderer, the simulation, and the stage the two share.
 *
 * The stage owns the OVStage instance that both consumers read, so it must outlive them.
 * The teardown order is expressed here rather than left to process shutdown.
 */
class Workflow
{
public:
    ~Workflow()
    {
        if (m_simulationInitialized)
        {
            manager::PhysicsManager::getInstance().invalidate();
        }
        if (m_rendererAttached)
        {
            ovrtx_detach_ovstage(m_renderer);
        }
        if (m_renderer != nullptr)
        {
            ovrtx_destroy_renderer(m_renderer);
        }
        if (m_stage != nullptr)
        {
            m_stage->closeStage();
        }
    }

    Workflow() = default;
    Workflow(const Workflow&) = delete;
    Workflow& operator=(const Workflow&) = delete;

    /**
     * @brief Simulates the falling cube and renders every captured frame.
     */
    void run(const std::filesystem::path& outputDirectory)
    {
        const std::string stageText = authorStage();

        // The renderer and the physics engine share one OVStage: the engine reads the
        // scene it simulates from the stage, and the renderer reads the poses this
        // example publishes back onto it.
        const ovrtx_config_t rendererConfig{ nullptr, 0 };
        checkOvrtx(ovrtx_create_renderer(&rendererConfig, &m_renderer).status, "ovrtx_create_renderer");

        m_stage = std::make_unique<objects::Stage>("ovstage");
        m_stage->importStageFromString(stageText, /*makeDefault=*/false);
        checkOvrtx(ovrtx_attach_ovstage(m_renderer, static_cast<ovstage_instance_t*>(m_stage->getStagePtr())).status,
                   "ovrtx_attach_ovstage");
        m_rendererAttached = true;

        manager::PhysicsManager& physicsManager = manager::PhysicsManager::getInstance();
        if (!physicsManager.switchPhysicsEngine("ovphysx"))
        {
            throw std::runtime_error("The OvPhysX physics engine is unavailable.");
        }
        physicsManager.configure(g_kTimeStep);
        if (!physicsManager.initialize(m_stage->getStagePtr(), m_stage->getStageId()))
        {
            throw std::runtime_error("Physics initialization failed.");
        }
        m_simulationInitialized = true;

        entities::RigidBodyEntity simulatedCube("ovphysx", g_kCubePath);

        const auto cubeHeight = [&simulatedCube]() -> float
        {
            const array::Array positions = std::get<0>(simulatedCube.getWorldPoses());
            return static_cast<const float*>(positions.data())[2];
        };

        if (!physicsManager.publishTransformsToStage())
        {
            throw std::runtime_error("Physics transform publication failed.");
        }
        const float initialHeight = cubeHeight();
        const size_t firstChecksum = captureFrame(outputDirectory, 0);
        size_t lastChecksum = firstChecksum;
        int capturedFrames = 1;
        float height = initialHeight;
        int restingSteps = 0;
        int step = 0;
        while (restingSteps < g_kRestSteps && step < g_kMaxSimulationSteps)
        {
            ++step;
            physicsManager.step();
            if (!physicsManager.publishTransformsToStage())
            {
                throw std::runtime_error("Physics transform publication failed.");
            }
            const float previousHeight = height;
            height = cubeHeight();
            restingSteps = std::abs(height - previousHeight) < g_kRestTravel ? restingSteps + 1 : 0;
            if (step % g_kCaptureInterval == 0)
            {
                lastChecksum = captureFrame(outputDirectory, step);
                ++capturedFrames;
            }
        }
        // Capture where the cube settled, so the last frame always shows it landed.
        if (step % g_kCaptureInterval != 0)
        {
            lastChecksum = captureFrame(outputDirectory, step);
            ++capturedFrames;
        }

        std::printf("Cube height: %.3f -> %.3f over %d steps.\n", initialHeight, height, step);
        std::printf("Captured %d frames in %s.\n", capturedFrames, outputDirectory.string().c_str());
        if (restingSteps < g_kRestSteps)
        {
            throw std::runtime_error("The cube did not reach the ground.");
        }
        if (height >= initialHeight)
        {
            throw std::runtime_error("The cube did not fall.");
        }
        std::printf("The cube fell.\n");
        if (firstChecksum == lastChecksum)
        {
            throw std::runtime_error("The render did not follow the simulation.");
        }
        std::printf("The render followed the simulation.\n");
        std::fflush(stdout);
    }

private:
    /**
     * @brief Renders one frame from the attached stage and writes it to the output directory.
     *
     * @return Checksum of the rendered pixels.
     */
    size_t captureFrame(const std::filesystem::path& outputDirectory, int step)
    {
        const ovx_string_t productPath{ g_kRenderProductPath, std::strlen(g_kRenderProductPath) };
        const ovrtx_render_product_set_t productSet{ &productPath, 1 };

        ovrtx_step_result_handle_t resultHandle = 0;
        const ovrtx_enqueue_result_t enqueued =
            ovrtx_step_with_stage(m_renderer, productSet, g_kTimeStep, g_kPopulationOrdinal, &resultHandle);
        checkOvrtx(enqueued.status, "ovrtx_step_with_stage");

        ovrtx_op_wait_result_t waitResult{};
        checkOvrtx(
            ovrtx_wait_op(m_renderer, enqueued.op_index, ovrtx_timeout_infinite, &waitResult).status, "ovrtx_wait_op");

        ovrtx_render_product_set_outputs_t outputs{};
        checkOvrtx(ovrtx_fetch_results(m_renderer, resultHandle, ovrtx_timeout_infinite, &outputs).status,
                   "ovrtx_fetch_results");

        std::optional<size_t> checksum;
        for (size_t productIndex = 0; productIndex < outputs.output_count && !checksum.has_value(); ++productIndex)
        {
            const ovrtx_render_product_output_t& product = outputs.outputs[productIndex];
            for (size_t frameIndex = 0; frameIndex < product.output_frame_count && !checksum.has_value(); ++frameIndex)
            {
                const ovrtx_render_product_frame_output_t& producedFrame = product.output_frames[frameIndex];
                for (size_t varIndex = 0; varIndex < producedFrame.render_var_count; ++varIndex)
                {
                    const ovrtx_render_product_render_var_output_t& renderVar =
                        producedFrame.output_render_vars[varIndex];
                    if (std::string(renderVar.render_var_path.ptr, renderVar.render_var_path.length) != g_kRenderVarPath)
                    {
                        continue;
                    }
                    checksum = readRenderVar(renderVar, outputDirectory, step);
                    break;
                }
            }
        }
        checkOvrtx(ovrtx_destroy_results(m_renderer, resultHandle).status, "ovrtx_destroy_results");

        if (!checksum.has_value())
        {
            throw std::runtime_error("The renderer produced no frame.");
        }
        return *checksum;
    }

    /**
     * @brief Maps one render variable output, writes it out, and checksums its pixels.
     *
     * @return Checksum of the written pixels, or no value when the output carried none.
     */
    std::optional<size_t> readRenderVar(const ovrtx_render_product_render_var_output_t& renderVar,
                                        const std::filesystem::path& outputDirectory,
                                        int step)
    {
        const ovrtx_map_output_description_t mapDescription{ OVRTX_MAP_DEVICE_TYPE_CPU, 0 };
        ovrtx_render_var_output_t mapped{};
        checkOvrtx(ovrtx_map_render_var_output(
                       m_renderer, renderVar.output_handle, &mapDescription, ovrtx_timeout_infinite, &mapped)
                       .status,
                   "ovrtx_map_render_var_output");

        const ovrtx_cuda_sync_t sync{ 0, 0 };
        std::optional<size_t> checksum;
        try
        {
            if (mapped.num_tensors > 0 && mapped.tensors[0].dl != nullptr)
            {
                const DLTensor& tensor = *mapped.tensors[0].dl;
                const int64_t height = tensor.shape[0];
                const int64_t width = tensor.shape[1];
                const auto* pixels = static_cast<const uint8_t*>(tensor.data);
                char name[64];
                std::snprintf(name, sizeof(name), "frame_%04d.ppm", step);
                writePpm(outputDirectory / name, pixels, width, height);
                checksum = std::hash<std::string_view>{}(
                    std::string_view(reinterpret_cast<const char*>(pixels), static_cast<size_t>(width * height * 4)));
            }
        }
        catch (...)
        {
            ovrtx_unmap_render_var_output(m_renderer, mapped.map_handle, sync);
            throw;
        }
        checkOvrtx(
            ovrtx_unmap_render_var_output(m_renderer, mapped.map_handle, sync).status, "ovrtx_unmap_render_var_output");
        return checksum;
    }

    ovrtx_renderer_t* m_renderer{ nullptr };
    std::unique_ptr<objects::Stage> m_stage;
    bool m_rendererAttached{ false };
    bool m_simulationInitialized{ false };
};

PyObject* renderCube(PyObject*, PyObject* args)
{
    const char* outputDirectory = nullptr;
    if (PyArg_ParseTuple(args, "s", &outputDirectory) == 0)
    {
        return nullptr;
    }
    try
    {
        Workflow workflow;
        workflow.run(std::filesystem::path(outputDirectory));
        Py_RETURN_NONE;
    }
    catch (const std::exception& error)
    {
        PyErr_SetString(PyExc_RuntimeError, error.what());
        return nullptr;
    }
    catch (...)
    {
        PyErr_SetString(PyExc_RuntimeError, "The Isaac Sim C++ API reported an unknown error.");
        return nullptr;
    }
}

PyMethodDef g_methods[] = {
    { "render_cube", renderCube, METH_VARARGS,
      "Simulate a falling cube and render every captured frame into the given directory." },
    { nullptr, nullptr, 0, nullptr },
};

PyModuleDef g_module = {
    PyModuleDef_HEAD_INIT,
    "_render_cube_cpp",
    "C++ with Python Isaac Sim falling cube rendering example.",
    -1,
    g_methods,
};

} // namespace

PyMODINIT_FUNC PyInit__render_cube_cpp()
{
    return PyModule_Create(&g_module);
}
