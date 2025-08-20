// SPDX-FileCopyrightText: Copyright (c) 2023-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#pragma once
// clang-format off
#include <pch/UsdPCH.h>
// clang-format on

#include "omniverse_asset_converter.h"

#include <isaacsim/asset/importer/mjcf/Mesh.h>
#include <isaacsim/core/includes/utils/Path.h>
#include <isaacsim/core/includes/utils/Usd.h>

#include <fstream>
using namespace isaacsim::core::includes::utils::path;
namespace mesh
{

inline std::string StatusToString(OmniConverterStatus status)
{
    switch (status)
    {
    case OmniConverterStatus::OK:
        return "OK";
    case OmniConverterStatus::CANCELLED:
        return "Cancelled";
    case OmniConverterStatus::IN_PROGRESS:
        return "In Progress";
    case OmniConverterStatus::UNSUPPORTED_IMPORT_FORMAT:
        return "Unsupported Format";
    case OmniConverterStatus::INCOMPLETE_IMPORT_FORMAT:
        return "Incomplete File";
    case OmniConverterStatus::FILE_READ_ERROR:
        return "Asset Not Found";
    case OmniConverterStatus::FILE_WRITE_ERROR:
        return "Output Path Cannot be Opened";
    case OmniConverterStatus::UNKNOWN:
        return "Unknown";
    default:
        return "Unknown";
    }
}

/**
 * @brief Utility class for importing and converting mesh assets to USD format.
 * @details
 * Handles the import pipeline for various mesh formats, converting them to USD
 * geometry with materials and textures. Manages asset conversion using the
 * Omni Converter and handles material binding and mesh optimization.
 */
class MeshImporter
{
private:
    // std::map<std::string, std::pair<Mesh*, carb::gym::GymMeshHandle>>
    // gymGraphicsMeshCache; std::map<std::pair<float, float>,
    // carb::gym::TriangleMeshHandle> cylinderCache; std::map<std::string, Mesh*>
    // simulationMeshCache;

    /**
     * @brief Extracts folder paths from source and destination stages.
     * @param[in] sourceStage Source USD stage
     * @param[in] dstStage Destination USD stage
     * @return Pair of source folder path and destination folder path
     */
    std::pair<std::string, std::string> _extractFolderPaths(const pxr::UsdStageRefPtr& sourceStage,
                                                            const pxr::UsdStageRefPtr& dstStage)
    {
        std::string source_stage_path = sourceStage->GetRootLayer()->GetIdentifier();
        std::string dst_stage_path = dstStage->GetRootLayer()->GetIdentifier();
        std::string source_folder = source_stage_path.substr(0, source_stage_path.find_last_of("/"));
        std::string dst_folder = dst_stage_path.substr(0, dst_stage_path.find_last_of("/"));
        return std::make_pair(source_folder, dst_folder);
    }

    /**
     * @brief Copies material asset files from source to destination folder.
     * @param[in] newMaterial Material primitive containing asset references
     * @param[in] sourceFolderPath Source folder path for resolving relative paths
     * @param[in] dstFolderPath Destination folder path for copying files
     */
    void _copyMaterialAssetFiles(const pxr::UsdShadeMaterial& newMaterial,
                                 const std::string& sourceFolderPath,
                                 const std::string& dstFolderPath)
    {
        // Iterate through all child prims (shaders) of the material
        for (const auto& childPrim : newMaterial.GetPrim().GetChildren())
        {
            pxr::UsdShadeShader shader(childPrim);
            if (shader)
            {
                // Check all attributes for file path references
                for (const auto& attr : childPrim.GetAttributes())
                {
                    pxr::VtValue value;
                    if (attr.Get(&value) && value.IsHolding<pxr::SdfAssetPath>())
                    {
                        pxr::SdfAssetPath assetPath = value.Get<pxr::SdfAssetPath>();
                        std::string originalPath = assetPath.GetAssetPath();
                        if (originalPath == "OmniPBR.mdl") // Ignore OmniPBR.mdl
                        {
                            continue;
                        }
                        if (!originalPath.empty() && originalPath.find("://") == std::string::npos) // Is Local file and
                                                                                                    // authored in USD
                        {
                            _copyAssetFile(originalPath, sourceFolderPath, dstFolderPath);
                        }
                    }
                }
            }
        }
    }

    /**
     * @brief Copies a single asset file from source to destination.
     * @param[in] originalPath Original asset path from material attribute
     * @param[in] sourceFolderPath Source folder path for resolving relative paths
     * @param[in] dstFolderPath Destination folder path for copying files
     */
    void _copyAssetFile(const std::string& originalPath,
                        const std::string& sourceFolderPath,
                        const std::string& dstFolderPath)
    {
        // Resolve the source file path
        std::string sourceFilePath;
        // Check if the path is relative
        if (originalPath[1] != ':' && originalPath[0] != '/')
        {
            sourceFilePath = resolve_relative(sourceFolderPath, originalPath);
        }
        else
        {
            sourceFilePath = resolve_relative(sourceFolderPath, sourceFolderPath + "/" + originalPath);
        }

        // Create destination file path
        std::string dstFilePath = dstFolderPath + "/" + sourceFilePath;
        std::string userData[2] = { (sourceFolderPath + "/" + sourceFilePath), dstFilePath };

        // Check if destination file already exists
        omniClientWait(omniClientStat(
            dstFilePath.c_str(), &userData,
            [](void* userData, OmniClientResult result, OmniClientListEntry const* entry) noexcept
            {
                std::string* userPaths = static_cast<std::string*>(userData);
                if (result != eOmniClientResult_Ok)
                {
                    omniClientCopy(userPaths[0].c_str(), userPaths[1].c_str(), userData,
                                   [](void* userData, OmniClientResult result) noexcept
                                   {
                                       std::string* userPaths = static_cast<std::string*>(userData);
                                       if (result != eOmniClientResult_Ok)
                                       {
                                           CARB_LOG_WARN("Failed to copy file from %s to %s (omniClient error: %s)",
                                                         userPaths[0].c_str(), userPaths[1].c_str(),
                                                         omniClientGetResultString(result));
                                       }
                                   });
                }
            }));
    }

    /**
     * @brief Gets or creates material path in destination stage.
     * @param[in] sourceStage Source USD stage containing the material
     * @param[in] dstStage Destination USD stage
     * @param[in] rootPath Root path in destination stage
     * @param[in] material Source material to copy
     * @param[in] sourceFolderPath Source folder path for asset copying
     * @param[in] dstFolderPath Destination folder path for asset copying
     * @param[in,out] materialPaths Map tracking material paths to avoid duplicates
     * @return Path to material in destination stage
     */
    pxr::SdfPath _getOrCreateMaterialPath(const pxr::UsdStageRefPtr& sourceStage,
                                          const pxr::UsdStageRefPtr& dstStage,
                                          const pxr::SdfPath& rootPath,
                                          const pxr::UsdShadeMaterial& material,
                                          const std::string& sourceFolderPath,
                                          const std::string& dstFolderPath,
                                          std::map<pxr::TfToken, pxr::SdfPath>& materialPaths)
    {
        pxr::TfToken materialToken = getMaterialToken(sourceStage, material.GetPrim().GetPath());

        if (materialPaths.count(materialToken))
        {
            return materialPaths[materialToken];
        }

        // Create new material path
        pxr::SdfPath materialPath = pxr::SdfPath(isaacsim::core::includes::utils::usd::GetNewSdfPathString(
            dstStage,
            rootPath.AppendChild(pxr::TfToken("Looks")).AppendChild(pxr::TfToken(material.GetPath().GetName())).GetString()));

        pxr::SdfCopySpec(sourceStage->GetRootLayer(), material.GetPath(), dstStage->GetRootLayer(), materialPath);
        materialPaths.emplace(materialToken, materialPath);

        // Check for attributes containing file path references and copy files
        pxr::UsdShadeMaterial newMaterial = pxr::UsdShadeMaterial(dstStage->GetPrimAtPath(materialPath));
        if (newMaterial)
        {
            _copyMaterialAssetFiles(newMaterial, sourceFolderPath, dstFolderPath);
        }

        return materialPath;
    }

public:
    /**
     * @brief Default constructor for MeshImporter.
     */
    MeshImporter()
    {
    }

    /**
     * @brief Resolves a mesh file path to its absolute location.
     * @param[in] filePath Relative or absolute path to mesh file
     * @return Resolved absolute path to mesh file
     */
    std::string resolveMeshPath(const std::string& filePath)
    {
        // noop for now
        return filePath;
    }

    /**
     * @brief Gets a unique token for a material based on its properties.
     * @param[in] stage USD stage containing the material
     * @param[in] materialPath Path to the material primitive
     * @return Unique token identifying the material properties
     */
    pxr::TfToken getMaterialToken(pxr::UsdStageWeakPtr stage, const pxr::SdfPath& materialPath)
    {
        // Get the material object from the stage
        pxr::UsdShadeMaterial material = pxr::UsdShadeMaterial(stage->GetPrimAtPath(materialPath));

        // If the material doesn't exist, return an empty token
        if (!material)
        {
            return pxr::TfToken();
        }

        auto shaderPrim = *material.GetPrim().GetChildren().begin();
        // Get the shader object for the material
        pxr::UsdShadeShader shader(shaderPrim);
        if (!shader)
        {
            return pxr::TfToken();
        }

        std::stringstream ss;
        for (const auto& attr : shaderPrim.GetAttributes())
        {
            ss << "<";
            ss << attr.GetTypeName() << " : " << attr.GetName() << " = ";

            pxr::VtValue value;
            if (attr.Get(&value))
            {
                ss << value << ">\n";
            }
            else
            {
                ss << ">\n"; // Handle cases where the attribute has no value
            }
        }

        // Create a TfToken from the token string
        pxr::TfToken token(ss.str());
        return token;
    }

    /**
     * @brief Moves and binds a material from source stage to destination stage.
     * @param[in] sourceStage Source USD stage containing the material
     * @param[in] dstStage Destination USD stage to move material to
     * @param[in] rootPath Root path in destination stage
     * @param[in] sourcePath Path to source primitive with material binding
     * @param[in] dstPath Path to destination primitive for material binding
     * @param[in,out] materialPaths Map tracking material paths to avoid duplicates
     */
    void moveAndBindMaterial(const pxr::UsdStageRefPtr& sourceStage,
                             const pxr::UsdStageRefPtr& dstStage,
                             const pxr::SdfPath& rootPath,
                             const pxr::SdfPath& sourcePath,
                             const pxr::SdfPath& dstPath,
                             std::map<pxr::TfToken, pxr::SdfPath>& materialPaths)
    {
        auto [sourceFolderPath, dstFolderPath] = _extractFolderPaths(sourceStage, dstStage);
        auto sourceImageable = sourceStage->GetPrimAtPath(sourcePath);
        auto dstImageable = dstStage->GetPrimAtPath(dstPath);

        pxr::UsdShadeMaterialBindingAPI bindingAPI(sourceImageable);
        pxr::UsdShadeMaterialBindingAPI::DirectBinding directBinding = bindingAPI.GetDirectBinding();
        pxr::UsdShadeMaterial material = directBinding.GetMaterial();

        if (material)
        {
            pxr::SdfPath materialPath = _getOrCreateMaterialPath(
                sourceStage, dstStage, rootPath, material, sourceFolderPath, dstFolderPath, materialPaths);

            pxr::UsdShadeMaterial newMaterial = pxr::UsdShadeMaterial(dstStage->GetPrimAtPath(materialPath));
            if (newMaterial)
            {
                bindingAPI = pxr::UsdShadeMaterialBindingAPI(dstImageable);
                bindingAPI.Bind(newMaterial);
            }
        }
    }

    /**
     * @brief Moves mesh geometry and associated materials between USD stages.
     * @param[in] sourceStage Source USD stage containing the mesh
     * @param[in] dstStage Destination USD stage to move mesh to
     * @param[in] rootPath Root path in destination stage
     * @param[in] meshPath Path to source mesh primitive
     * @param[in] targetPrimPath Path where new mesh should be created
     * @param[in,out] materialPaths Map tracking material paths to avoid duplicates
     */
    void moveMeshAndMaterials(const pxr::UsdStageRefPtr& sourceStage,
                              const pxr::UsdStageRefPtr& dstStage,
                              const pxr::SdfPath& rootPath,
                              const pxr::SdfPath& meshPath,
                              const pxr::SdfPath& targetPrimPath,
                              std::map<pxr::TfToken, pxr::SdfPath>& materialPaths)
    {


        pxr::UsdGeomMesh mesh = pxr::UsdGeomMesh(sourceStage->GetPrimAtPath(meshPath));
        if (!mesh)
        {
            CARB_LOG_ERROR("Error: Could not find mesh at path %s", meshPath.GetText());
            return;
        }

        pxr::UsdGeomMesh newMesh = pxr::UsdGeomMesh::Define(dstStage, targetPrimPath);
        if (!newMesh)
        {
            CARB_LOG_ERROR("Error: Could not create new mesh at path %s", targetPrimPath.GetText());
            return;
        }

        pxr::SdfCopySpec(sourceStage->GetRootLayer(), mesh.GetPath(), dstStage->GetRootLayer(), newMesh.GetPath());

        // Get the global transform of the source mesh and apply it to the new mesh
        pxr::UsdGeomXformable sourceMeshXformable(mesh);
        pxr::UsdGeomXformable newMeshXformable(newMesh);

        // Get the global transform matrix using Omni USD
        pxr::GfMatrix4d globalTransform = mesh.ComputeLocalToWorldTransform(pxr::UsdTimeCode::Default());

        // Clear existing transform operations on the new mesh
        newMeshXformable.ClearXformOpOrder();

        // Decompose the global transform matrix
        pxr::GfVec3d translation = globalTransform.ExtractTranslation();
        pxr::GfRotation rotation = globalTransform.ExtractRotation();

        // Calculate scale from the magnitude of the rotation matrix columns
        pxr::GfMatrix3d rotationMatrix = globalTransform.ExtractRotationMatrix();
        pxr::GfVec3d scaleX = rotationMatrix.GetColumn(0);
        pxr::GfVec3d scaleY = rotationMatrix.GetColumn(1);
        pxr::GfVec3d scaleZ = rotationMatrix.GetColumn(2);
        pxr::GfVec3d scale(scaleX.GetLength(), scaleY.GetLength(), scaleZ.GetLength());


        // Create standardized transform operations: Translate, Orient, Scale
        pxr::UsdGeomXformOp translateOp = newMeshXformable.AddTranslateOp(pxr::UsdGeomXformOp::PrecisionDouble);
        pxr::UsdGeomXformOp orientOp = newMeshXformable.AddOrientOp(pxr::UsdGeomXformOp::PrecisionDouble);
        pxr::UsdGeomXformOp scaleOp = newMeshXformable.AddScaleOp(pxr::UsdGeomXformOp::PrecisionDouble);

        // Set the decomposed values
        translateOp.Set(translation);
        orientOp.Set(rotation.GetQuat().GetNormalized());
        scaleOp.Set(pxr::GfVec3d(scale));

        // Move materials and bind them to the new mesh

        for (const auto& subset : pxr::UsdGeomSubset::GetAllGeomSubsets(mesh))
        {
            pxr::TfToken subsetName = subset.GetPrim().GetName();
            pxr::UsdGeomSubset dstSubset =
                pxr::UsdGeomSubset(dstStage->GetPrimAtPath(newMesh.GetPath().AppendChild(pxr::TfToken(subsetName))));
            moveAndBindMaterial(sourceStage, dstStage, rootPath, subset.GetPath(), dstSubset.GetPath(), materialPaths);
        }

        // Bind the material to the new mesh
        moveAndBindMaterial(sourceStage, dstStage, rootPath, mesh.GetPath(), newMesh.GetPath(), materialPaths);
    }

    /**
     * @brief Waits for asset conversion to complete and processes the results.
     * @param[in] future Future object tracking conversion progress
     * @param[in] usdStage USD stage to add converted mesh to
     * @param[in] mesh_usd_path Path to converted USD mesh file
     * @param[in] meshStagePath Path in stage where mesh should be placed
     * @param[in] rootPath Root path for organizing assets
     * @param[in,out] materialPaths Map tracking material paths to avoid duplicates
     * @return Path to the imported mesh primitive in the stage
     */
    pxr::SdfPath waitForConverter(OmniConverterFuture* future,
                                  pxr::UsdStageRefPtr usdStage,
                                  const std::string& mesh_usd_path,
                                  const std::string& meshStagePath,
                                  const pxr::SdfPath& rootPath,
                                  std::map<pxr::TfToken, pxr::SdfPath>& materialPaths)
    {
        OmniConverterStatus status = OmniConverterStatus::OK;
        // Wait for the converter to finish
        if (future == nullptr)
        {
            CARB_LOG_ERROR("Error: Future is null");
            return pxr::SdfPath();
        }
        while (omniConverterCheckFutureStatus(future) == OmniConverterStatus::IN_PROGRESS)
        {
        }
        status = omniConverterCheckFutureStatus(future);
        omniConverterReleaseFuture(future);

        if (status == OmniConverterStatus::OK)
        {
            CARB_LOG_INFO("Asset %s convert successfully.", mesh_usd_path.c_str());
        }
        else
        {
            CARB_LOG_WARN("Asset convert failed with error status: %s (%s)", StatusToString(status).c_str(),
                          meshStagePath.c_str());
        }
        // Open the mesh stage and get the mesh prims
        pxr::UsdStageRefPtr meshStage = pxr::UsdStage::Open(mesh_usd_path);
        pxr::UsdPrimRange primRange(meshStage->GetDefaultPrim());
        std::vector<pxr::UsdPrim> meshPrims;
        std::copy_if(primRange.begin(), primRange.end(), std::back_inserter(meshPrims),
                     [](pxr::UsdPrim const& prim) { return prim.IsA<pxr::UsdGeomMesh>(); });


        // Create a base prim and move meshes and materials
        auto basePrim = pxr::UsdGeomXform::Define(usdStage, pxr::SdfPath(meshStagePath));

        for (const auto& mesh : meshPrims)
        {
            pxr::TfToken meshName = mesh.GetName();
            if (meshName == "mesh")
            {
                meshName = pxr::TfToken(basePrim.GetPath().GetName());
            }
            // CARB_LOG_INFO("Mesh name: %s", mesh.GetName().c_str());
            moveMeshAndMaterials(meshStage, usdStage, usdStage->GetDefaultPrim().GetPath(), mesh.GetPath(),
                                 basePrim.GetPath().AppendChild(meshName), materialPaths);
        }
        // Clean up
        omniClientWait(omniClientDelete(getParent(mesh_usd_path).c_str(), {}, {}));
        pxr::UsdGeomImageable(usdStage->GetPrimAtPath(pxr::SdfPath(meshStagePath)))
            .CreateVisibilityAttr()
            .Set(pxr::UsdGeomTokens->inherited);
        return pxr::SdfPath(meshStagePath);
    }

    /**
     * @brief Imports a mesh from file using the Omni Converter.
     * @param[in,out] mesh Mesh object to store conversion results
     * @param[in] relativeMeshPath Relative path to source mesh file
     * @param[in] scale Scale factors to apply during conversion
     * @param[in] flip Whether to flip normals during conversion (default: false)
     */
    void importMesh(Mesh* mesh, std::string relativeMeshPath, const Vec3& scale, bool flip = false)
    {
        std::string meshPath = resolveMeshPath(relativeMeshPath);
        std::string mesh_usd_path = pathJoin(pathJoin(getParent(meshPath), getPathStem(meshPath.c_str())) + "_tmp",
                                             getPathStem(meshPath.c_str()) + ".tmp.usd");

        CARB_LOG_INFO("Importing Mesh %s\n    (%s)", meshPath.c_str(), mesh_usd_path.c_str());
        // Set up the Omni Converter flags
        int flags = OMNI_CONVERTER_FLAGS_SINGLE_MESH_FILE | OMNI_CONVERTER_FLAGS_IGNORE_CAMERAS |
                    OMNI_CONVERTER_FLAGS_USE_METER_PER_UNIT | OMNI_CONVERTER_FLAGS_MERGE_ALL_MESHES |
                    OMNI_CONVERTER_FLAGS_IGNORE_LIGHTS | OMNI_CONVERTER_FLAGS_FBX_CONVERT_TO_Z_UP |
                    OMNI_CONVERTER_FLAGS_FBX_BAKING_SCALES_INTO_MESH | OMNI_CONVERTER_FLAGS_IGNORE_PIVOTS;

        // Set up the log and progress callbacks for the Omni Converter
        omniConverterSetLogCallback([](const char* message) { CARB_LOG_INFO("%s", message); });
        omniConverterSetProgressCallback([](OmniConverterFuture* future, uint32_t progress, uint32_t total)
                                         { CARB_LOG_INFO("Progress: %d / %d", progress, total); });

        // Create a new Omni Converter future for the mesh import
        mesh->m_assetConvertStatus = omniConverterCreateAsset(meshPath.c_str(), mesh_usd_path.c_str(), flags);
        mesh->m_convertedUsdMesh = mesh_usd_path;
    }
};

} // namespace mesh
