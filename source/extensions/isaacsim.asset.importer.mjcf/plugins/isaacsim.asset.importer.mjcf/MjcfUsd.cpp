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

#include <carb/logging/Log.h>

#include <isaacsim/asset/importer/mjcf/MeshImporter.h>
#include <isaacsim/asset/importer/mjcf/MjcfUsd.h>
#include <isaacsim/core/includes/utils/Path.h>
#include <isaacsim/core/includes/utils/Usd.h>
#include <physxSchema/physxMeshMergeCollisionAPI.h>

namespace isaacsim
{
namespace asset
{
namespace importer
{
namespace mjcf
{
using namespace isaacsim::core::includes::utils::usd;

pxr::SdfPath getNextFreePath(pxr::UsdStageWeakPtr stage, const pxr::SdfPath& primPath)
{
    auto uniquePath = primPath;
    auto prim = stage->GetPrimAtPath(uniquePath);
    const std::string& name = uniquePath.GetName();

    int startIndex = 1;
    while (prim)
    {
        uniquePath = primPath.ReplaceName(pxr::TfToken(name + "_" + std::to_string(startIndex)));
        prim = stage->GetPrimAtPath(uniquePath);
        startIndex++;
    }

    return uniquePath;
}

void setStageMetadata(pxr::UsdStageWeakPtr stage, const isaacsim::asset::importer::mjcf::ImportConfig config)
{


    pxr::UsdGeomSetStageMetersPerUnit(stage, 1.0f / config.distanceScale);
    pxr::UsdGeomSetStageUpAxis(stage, pxr::TfToken("Z"));
}

void createRoot(std::unordered_map<std::string, pxr::UsdStageRefPtr> stages,
                Transform trans,
                const std::string rootPrimPath,
                const isaacsim::asset::importer::mjcf::ImportConfig config)
{
    for (auto& stage : stages)
    {
        pxr::UsdGeomXform robotPrim = pxr::UsdGeomXform::Define(stage.second, pxr::SdfPath(rootPrimPath));
        if (stage.first != "stage" || (config.makeDefaultPrim))
        {
            stage.second->SetDefaultPrim(robotPrim.GetPrim());
        }
    }
}

void createFixedRoot(std::unordered_map<std::string, pxr::UsdStageRefPtr> stages,
                     const std::string jointPath,
                     const std::string bodyPath)
{
    pxr::UsdEditContext context(stages["stage"], stages["physics_stage"]->GetRootLayer());

    pxr::UsdPhysicsFixedJoint rootJoint = pxr::UsdPhysicsFixedJoint::Define(stages["stage"], pxr::SdfPath(jointPath));
    pxr::SdfPathVector val1{ pxr::SdfPath(bodyPath) };
    rootJoint.CreateBody1Rel().SetTargets(val1);
}

void applyArticulationAPI(std::unordered_map<std::string, pxr::UsdStageRefPtr> stages,
                          pxr::UsdGeomXformable prim,
                          const isaacsim::asset::importer::mjcf::ImportConfig config)
{
    pxr::UsdEditContext context(stages["stage"], stages["physics_stage"]->GetRootLayer());
    pxr::UsdPhysicsArticulationRootAPI physicsSchema = pxr::UsdPhysicsArticulationRootAPI::Apply(prim.GetPrim());
    pxr::PhysxSchemaPhysxArticulationAPI physxSchema = pxr::PhysxSchemaPhysxArticulationAPI::Apply(prim.GetPrim());
    physxSchema.CreateEnabledSelfCollisionsAttr().Set(config.selfCollision);
}

std::string ReplaceBackwardSlash(std::string in)
{
    for (auto& c : in)
    {
        if (c == '\\')
        {
            c = '/';
        }
    }
    return in;
}

std::string copyTexture(std::string usdStageIdentifier, std::string texturePath)
{
    // switch any windows-style path into linux backwards slash (omniclient
    // handles windows paths)
    usdStageIdentifier = ReplaceBackwardSlash(usdStageIdentifier);
    texturePath = ReplaceBackwardSlash(texturePath);

    // Assumes the folder structure has already been created.
    int path_idx = (int)usdStageIdentifier.rfind('/');
    std::string parent_folder = usdStageIdentifier.substr(0, path_idx);
    int basename_idx = (int)texturePath.rfind('/');
    std::string textureName = texturePath.substr(basename_idx + 1);
    std::string out = (parent_folder + "/materials/" + textureName);
    omniClientWait(omniClientCopy(texturePath.c_str(), out.c_str(), {}, {}));
    return out;
}

void createMaterial(pxr::UsdStageWeakPtr usdStage,
                    const pxr::SdfPath path,
                    Mesh* mesh,
                    pxr::UsdGeomMesh usdMesh,
                    std::map<int, pxr::VtArray<int>>& materialMap)
{
    std::string prefix_path;
    prefix_path = pxr::SdfPath(path).GetParentPath().GetParentPath().GetString(); // Robot root

    // for each material, store the face indices and create GeomSubsets
    usdStage->DefinePrim(pxr::SdfPath(prefix_path + "/Looks"), pxr::TfToken("Scope"));
    for (auto const& mat : materialMap)
    {
        Material& material = mesh->m_materials[mat.first];

        pxr::UsdPrim prim;
        pxr::UsdShadeMaterial matPrim;
        std::string mat_path(prefix_path + "/Looks/" +
                             makeValidUSDIdentifier("material_" + SanitizeUsdName(material.name)));

        prim = usdStage->GetPrimAtPath(pxr::SdfPath(mat_path));
        int counter = 0;
        while (prim)
        {
            mat_path = std::string(
                prefix_path + "/Looks/" +
                makeValidUSDIdentifier("material_" + SanitizeUsdName(material.name) + "_" + std::to_string(++counter)));
            prim = usdStage->GetPrimAtPath(pxr::SdfPath(mat_path));
        }

        matPrim = pxr::UsdShadeMaterial::Define(usdStage, pxr::SdfPath(mat_path));
        pxr::UsdShadeShader pbrShader = pxr::UsdShadeShader::Define(usdStage, pxr::SdfPath(mat_path + "/Shader"));
        pbrShader.CreateIdAttr(pxr::VtValue(pxr::UsdImagingTokens->UsdPreviewSurface));

        auto shader_out = pbrShader.CreateOutput(pxr::TfToken("out"), pxr::SdfValueTypeNames->Token);
        matPrim.CreateSurfaceOutput(pxr::TfToken("mdl")).ConnectToSource(shader_out);
        matPrim.CreateVolumeOutput(pxr::TfToken("mdl")).ConnectToSource(shader_out);
        matPrim.CreateDisplacementOutput(pxr::TfToken("mdl")).ConnectToSource(shader_out);
        pbrShader.GetImplementationSourceAttr().Set(pxr::UsdShadeTokens->sourceAsset);
        pbrShader.SetSourceAsset(pxr::SdfAssetPath("OmniPBR.mdl"), pxr::TfToken("mdl"));
        pbrShader.SetSourceAssetSubIdentifier(pxr::TfToken("OmniPBR"), pxr::TfToken("mdl"));
        bool has_emissive_map = false;

        // diffuse, normal/bump, metallic, emissive, reflection/shininess
        std::string materialMapPaths[5] = { material.mapKd, material.mapBump, material.mapMetallic, material.mapEnv,
                                            material.mapKs };
        std::string materialMapTokens[5] = { "diffuse_texture", "normalmap_texture", "metallic_texture",
                                             "emissive_mask_texture", "reflectionroughness_texture" };
        for (int i = 0; i < 5; i++)
        {
            if (materialMapPaths[i] != "")
            {
                if (!usdStage->GetRootLayer()->IsAnonymous())
                {
                    auto texture_path = copyTexture(usdStage->GetRootLayer()->GetIdentifier(), materialMapPaths[i]);
                    int basename_idx = (int)texture_path.rfind('/');
                    std::string filename = texture_path.substr(basename_idx + 1);
                    std::string texture_relative_path = "materials/" + filename;
                    pbrShader.CreateInput(pxr::TfToken(materialMapTokens[i]), pxr::SdfValueTypeNames->Asset)
                        .Set(pxr::SdfAssetPath(texture_relative_path));
                    if (i == 3)
                    {
                        pbrShader.CreateInput(pxr::TfToken("emissive_color"), pxr::SdfValueTypeNames->Color3f)
                            .Set(pxr::GfVec3f(1.0f, 1.0f, 1.0f));
                        pbrShader.CreateInput(pxr::TfToken("enable_emission"), pxr::SdfValueTypeNames->Bool).Set(true);
                        pbrShader.CreateInput(pxr::TfToken("emissive_intensity"), pxr::SdfValueTypeNames->Float).Set(10000.0f);
                        has_emissive_map = true;
                    }
                }
                else
                {
                    CARB_LOG_WARN(
                        "Material %s has an image texture, but it won't be imported "
                        "since the asset is being loaded on memory. Please import it "
                        "into a destination folder to get all textures.",
                        material.name.c_str());
                }
            }
        }

        if (material.hasDiffuse)
        {
            pbrShader.CreateInput(pxr::TfToken("diffuse_color_constant"), pxr::SdfValueTypeNames->Color3f)
                .Set(pxr::GfVec3f(material.Ks.x, material.Ks.y, material.Ks.z));
        }
        if (material.hasMetallic)
        {
            pbrShader.CreateInput(pxr::TfToken("metallic_constant"), pxr::SdfValueTypeNames->Float).Set(material.metallic);
        }
        if (material.hasSpecular)
        {
            pbrShader.CreateInput(pxr::TfToken("specular_level"), pxr::SdfValueTypeNames->Float).Set(material.specular);
        }
        if (!has_emissive_map && material.hasEmissive)
        {
            pbrShader.CreateInput(pxr::TfToken("emissive_color"), pxr::SdfValueTypeNames->Color3f)
                .Set(pxr::GfVec3f(material.emissive.x, material.emissive.y, material.emissive.z));
        }

        if (materialMap.size() > 1)
        {
            auto geomSubset = pxr::UsdGeomSubset::Define(
                usdStage, pxr::SdfPath(usdMesh.GetPath().GetString() + "/material_" + SanitizeUsdName(material.name)));
            geomSubset.CreateElementTypeAttr(pxr::VtValue(pxr::TfToken("face")));
            geomSubset.CreateFamilyNameAttr(pxr::VtValue(pxr::TfToken("materialBind")));
            geomSubset.CreateIndicesAttr(pxr::VtValue(mat.second));

            if (matPrim)
            {
                pxr::UsdShadeMaterialBindingAPI mbi(geomSubset);
                mbi.Bind(matPrim);
                // pxr::UsdShadeMaterialBindingAPI::Apply(geomSubset).Bind(matPrim);
            }
        }
        else
        {
            if (matPrim)
            {
                pxr::UsdShadeMaterialBindingAPI mbi(usdMesh);
                mbi.Bind(matPrim);
                // pxr::UsdShadeMaterialBindingAPI::Apply(usdMesh).Bind(matPrim);
            }
        }
    }
}

// convert from internal Gym mesh to USD mesh
pxr::UsdPrim createUsdMesh(pxr::UsdStageWeakPtr stage,
                           const pxr::SdfPath path,
                           Mesh* mesh,
                           std::map<pxr::TfToken, pxr::SdfPath>& materialPaths,
                           float scale,
                           bool importMaterials)
{
    if (mesh->m_assetConvertStatus != nullptr)
    {
        mesh::MeshImporter mesh_importer;
        pxr::SdfPath mesh_usd_path =
            mesh_importer.waitForConverter(mesh->m_assetConvertStatus, stage, mesh->m_convertedUsdMesh, path.GetText(),
                                           stage->GetDefaultPrim().GetPath(), materialPaths);
        mesh->m_assetConvertStatus = nullptr;
        pxr::UsdPrim prim = stage->GetPrimAtPath(mesh_usd_path);
        pxr::UsdGeomXformable xformable(prim);
        return stage->GetPrimAtPath(mesh_usd_path);
    }
    else
    {
        CARB_LOG_ERROR("Error: Mesh converter Future is null");
        return pxr::UsdPrim();
    }
    // basic mesh data
    pxr::VtArray<pxr::VtArray<pxr::GfVec2f>> uvs;

    size_t vertexOffset = 0;
    std::map<int, pxr::VtArray<int>> materialMap;
    for (size_t m = 0; m < mesh->m_usdMeshPrims.size(); m++)
    {
        auto& meshPrim = mesh->m_usdMeshPrims[m];

        for (size_t k = 0; k < meshPrim.uvs.size(); k++)
        {
            uvs.push_back(meshPrim.uvs[k]);
        }

        for (size_t i = vertexOffset; i < vertexOffset + meshPrim.faceVertexCounts.size(); i++)
        {
            int materialIdx = mesh->m_materialAssignments[m].material;
            materialMap[materialIdx].push_back(static_cast<int>(i));
        }
        vertexOffset = vertexOffset + meshPrim.faceVertexCounts.size();
    }

    std::vector<pxr::GfVec3f> points(mesh->m_positions.size());
    std::vector<pxr::GfVec3f> normals(mesh->m_normals.size());
    std::vector<int> indices(mesh->m_indices.size());
    std::vector<int> vertexCounts(mesh->GetNumFaces(), 3);
    for (size_t i = 0; i < mesh->m_positions.size(); i++)
    {
        Point3 p = scale * mesh->m_positions[i];
        points[i].Set(&p.x);
    }
    for (size_t i = 0; i < mesh->m_normals.size(); i++)
    {
        normals[i].Set(&mesh->m_normals[i].x);
    }
    for (size_t i = 0; i < mesh->m_indices.size(); i++)
    {
        indices[i] = mesh->m_indices[i];
    }

    pxr::UsdGeomMesh usdMesh = createMesh(stage, path, points, normals, indices, vertexCounts);

    // texture UV
    for (size_t j = 0; j < uvs.size(); j++)
    {
        pxr::TfToken stName;
        if (j == 0)
        {
            stName = pxr::TfToken("st");
        }
        else
        {
            stName = pxr::TfToken("st_" + std::to_string(j));
        }
        pxr::UsdGeomPrimvarsAPI primvarsAPI(usdMesh);
        pxr::UsdGeomPrimvar Primvar =
            primvarsAPI.CreatePrimvar(stName, pxr::SdfValueTypeNames->TexCoord2fArray, pxr::UsdGeomTokens->faceVarying);
        Primvar.Set(uvs[j]);
    }

    if (!materialMap.empty() && importMaterials)
    {
        createMaterial(stage, path, mesh, usdMesh, materialMap);
    }

    return usdMesh.GetPrim();
}

pxr::UsdGeomMesh createMesh(pxr::UsdStageWeakPtr stage,
                            const pxr::SdfPath path,
                            const std::vector<pxr::GfVec3f>& points,
                            const std::vector<pxr::GfVec3f>& normals,
                            const std::vector<int>& indices,
                            const std::vector<int>& vertexCounts)
{
    pxr::UsdGeomMesh mesh = pxr::UsdGeomMesh::Define(stage, path);
    // fill in VtArrays
    pxr::VtArray<int> vertexCountsVt;
    vertexCountsVt.assign(vertexCounts.begin(), vertexCounts.end());
    pxr::VtArray<int> vertexIndicesVt;
    vertexIndicesVt.assign(indices.begin(), indices.end());
    pxr::VtArray<pxr::GfVec3f> pointArrayVt;
    pointArrayVt.assign(points.begin(), points.end());
    pxr::VtArray<pxr::GfVec3f> normalsVt;
    normalsVt.assign(normals.begin(), normals.end());
    mesh.CreateFaceVertexCountsAttr().Set(vertexCountsVt);
    mesh.CreateFaceVertexIndicesAttr().Set(vertexIndicesVt);
    mesh.CreatePointsAttr().Set(pointArrayVt);
    mesh.CreateDoubleSidedAttr().Set(true);

    if (!normals.empty())
    {
        mesh.CreateNormalsAttr().Set(normalsVt);
        mesh.SetNormalsInterpolation(pxr::UsdGeomTokens->faceVarying);
    }

    return mesh;
}

pxr::UsdGeomXformable createBody(pxr::UsdStageWeakPtr stage,
                                 const std::string primPath,
                                 const Transform& trans,
                                 const ImportConfig& config)
{
    // translate the prim before xform is created automatically
    pxr::UsdGeomXform xform = pxr::UsdGeomXform::Define(stage, pxr::SdfPath(primPath));
    pxr::UsdGeomXformable gprim = pxr::UsdGeomXformable(xform);
    gprim.ClearXformOpOrder();
    gprim.AddTranslateOp(pxr::UsdGeomXformOp::PrecisionDouble)
        .Set(config.distanceScale * pxr::GfVec3d(trans.p.x, trans.p.y, trans.p.z));
    gprim.AddOrientOp(pxr::UsdGeomXformOp::PrecisionDouble).Set(pxr::GfQuatd(trans.q.w, trans.q.x, trans.q.y, trans.q.z));
    gprim.AddScaleOp(pxr::UsdGeomXformOp::PrecisionDouble)
        .Set(pxr::GfVec3d(config.distanceScale, config.distanceScale, config.distanceScale));

    return gprim;
}

void applyRigidBody(std::unordered_map<std::string, pxr::UsdStageRefPtr> stages,
                    pxr::UsdGeomXformable bodyPrim,
                    const MJCFBody* body,
                    const ImportConfig& config)
{
    pxr::UsdEditContext context(stages["stage"], stages["physics_stage"]->GetRootLayer());
    pxr::UsdPhysicsRigidBodyAPI physicsAPI = pxr::UsdPhysicsRigidBodyAPI::Apply(bodyPrim.GetPrim());
    pxr::PhysxSchemaPhysxRigidBodyAPI::Apply(bodyPrim.GetPrim());

    pxr::UsdPhysicsMassAPI massAPI = pxr::UsdPhysicsMassAPI::Apply(bodyPrim.GetPrim());
    // TODO: need to support override computation
    if (body->inertial) // && config.importInertiaTensor)
    {
        massAPI.CreateMassAttr().Set(body->inertial->mass);

        if (!config.overrideCoM)
        {
            massAPI.CreateCenterOfMassAttr().Set(
                config.distanceScale * pxr::GfVec3f(body->inertial->pos.x, body->inertial->pos.y, body->inertial->pos.z));
        }

        if (!config.overrideInertia)
        {
            massAPI.CreateDiagonalInertiaAttr().Set(
                config.distanceScale * config.distanceScale *
                pxr::GfVec3f(body->inertial->diaginertia.x, body->inertial->diaginertia.y, body->inertial->diaginertia.z));

            if (body->inertial->hasFullInertia == true)
            {
                massAPI.CreatePrincipalAxesAttr().Set(
                    pxr::GfQuatf(body->inertial->principalAxes.w, body->inertial->principalAxes.x,
                                 body->inertial->principalAxes.y, body->inertial->principalAxes.z));
            }
        }
    }
    else
    {
        massAPI.CreateDensityAttr().Set(config.density / config.distanceScale / config.distanceScale /
                                        config.distanceScale);
    }
}

void createAndBindMaterial(pxr::UsdStageWeakPtr stage,
                           pxr::UsdPrim prim,
                           MJCFMaterial* material,
                           MJCFTexture* texture,
                           Vec4& color,
                           bool colorOnly,
                           std::map<pxr::TfToken, pxr::SdfPath>& materialPaths)
{

    // First create the material in a temp prim
    pxr::SdfPath path = prim.GetPath();
    std::string prefix_path("/Tmp");
    stage->DefinePrim(pxr::SdfPath("/Tmp"), pxr::TfToken("Scope"));
    // prefix_path = path.GetParentPath().GetString(); // body category root
    stage->DefinePrim(pxr::SdfPath(prefix_path + "/Looks"), pxr::TfToken("Scope"));

    pxr::UsdShadeMaterial matPrim;
    std::string materialName = SanitizeUsdName(material ? material->name : "rgba");
    std::string mat_path(prefix_path + "/Looks/" + makeValidUSDIdentifier("material_" + materialName));
    pxr::UsdPrim tmpPrim = stage->GetPrimAtPath(pxr::SdfPath(mat_path));
    int counter = 0;
    while (tmpPrim)
    {
        mat_path = std::string(prefix_path + "/Looks/" +
                               makeValidUSDIdentifier("material_" + materialName + "_" + std::to_string(++counter)));
        tmpPrim = stage->GetPrimAtPath(pxr::SdfPath(mat_path));
    }
    matPrim = pxr::UsdShadeMaterial::Define(stage, pxr::SdfPath(mat_path));
    pxr::UsdShadeShader pbrShader = pxr::UsdShadeShader::Define(stage, pxr::SdfPath(mat_path + "/Shader"));
    pbrShader.CreateIdAttr(pxr::VtValue(pxr::UsdImagingTokens->UsdPreviewSurface));

    auto shader_out = pbrShader.CreateOutput(pxr::TfToken("out"), pxr::SdfValueTypeNames->Token);
    matPrim.CreateSurfaceOutput(pxr::TfToken("mdl")).ConnectToSource(shader_out);
    matPrim.CreateVolumeOutput(pxr::TfToken("mdl")).ConnectToSource(shader_out);
    matPrim.CreateDisplacementOutput(pxr::TfToken("mdl")).ConnectToSource(shader_out);
    pbrShader.GetImplementationSourceAttr().Set(pxr::UsdShadeTokens->sourceAsset);
    pbrShader.SetSourceAsset(pxr::SdfAssetPath("OmniPBR.mdl"), pxr::TfToken("mdl"));
    pbrShader.SetSourceAssetSubIdentifier(pxr::TfToken("OmniPBR"), pxr::TfToken("mdl"));

    if (colorOnly)
    {
        pbrShader.CreateInput(pxr::TfToken("diffuse_color_constant"), pxr::SdfValueTypeNames->Color3f)
            .Set(pxr::GfVec3f(color.x, color.y, color.z));
    }
    else
    {
        pbrShader.CreateInput(pxr::TfToken("diffuse_color_constant"), pxr::SdfValueTypeNames->Color3f)
            .Set(pxr::GfVec3f(material->rgba.x, material->rgba.y, material->rgba.z));

        pbrShader.CreateInput(pxr::TfToken("metallic_constant"), pxr::SdfValueTypeNames->Float).Set(material->shininess);

        pbrShader.CreateInput(pxr::TfToken("specular_level"), pxr::SdfValueTypeNames->Float).Set(material->specular);

        pbrShader.CreateInput(pxr::TfToken("reflection_roughness_constant"), pxr::SdfValueTypeNames->Float)
            .Set(material->roughness);
    }

    if (texture)
    {
        if (texture->type == "2d")
        {
            // ensures there is a texture filename to copy
            if (texture->filename != "")
            {
                if (!stage->GetRootLayer()->IsAnonymous())
                {
                    auto texture_path = copyTexture(stage->GetRootLayer()->GetIdentifier(), texture->filename);
                    int basename_idx = (int)texture_path.rfind('/');
                    std::string filename = texture_path.substr(basename_idx + 1);
                    std::string texture_relative_path = "materials/" + filename;
                    pbrShader.CreateInput(pxr::TfToken("diffuse_texture"), pxr::SdfValueTypeNames->Asset)
                        .Set(pxr::SdfAssetPath(texture_relative_path));
                    if (material->project_uvw == true)
                    {
                        pbrShader.CreateInput(pxr::TfToken("project_uvw"), pxr::SdfValueTypeNames->Bool).Set(true);
                    }
                }
                else
                {
                    CARB_LOG_WARN(
                        "Material %s has an image texture, but it won't be imported "
                        "since the asset is being loaded on memory. Please import it "
                        "into a destination folder to get all textures.",
                        material->name.c_str());
                }
            }
        }
        else if (texture->type == "cube")
        {
            // ensures there is a texture filename to copy
            if (texture->filename != "")
            {
                if (!stage->GetRootLayer()->IsAnonymous())
                {
                    auto texture_path = copyTexture(stage->GetRootLayer()->GetIdentifier(), texture->filename);
                    int basename_idx = (int)texture_path.rfind('/');
                    std::string filename = texture_path.substr(basename_idx + 1);
                    std::string texture_relative_path = "materials/" + filename;
                    pbrShader.CreateInput(pxr::TfToken("diffuse_texture"), pxr::SdfValueTypeNames->Asset)
                        .Set(pxr::SdfAssetPath(texture_relative_path));
                }
                else
                {
                    CARB_LOG_WARN(
                        "Material %s has an image texture, but it won't be imported "
                        "since the asset is being loaded on memory. Please import it "
                        "into a destination folder to get all textures.",
                        material->name.c_str());
                }
            }
        }
        else
        {
            CARB_LOG_WARN("Only '2d' and 'cube' texture types are supported.\n");
        }
    }

    if (matPrim)
    {
        // pxr::UsdShadeMaterialBindingAPI mbi(prim);
        // mbi.Apply(matPrim);
        // mbi.Bind(matPrim);
        pxr::UsdShadeMaterialBindingAPI::Apply(prim).Bind(matPrim);
        mesh::MeshImporter mesh_importer;
        mesh_importer.moveAndBindMaterial(
            stage, stage, stage->GetDefaultPrim().GetPath(), prim.GetPath(), prim.GetPath(), materialPaths);
        stage->RemovePrim(pxr::SdfPath("/Tmp"));
    }
}


pxr::GfVec3f evalSphereCoord(float u, float v)
{
    float theta = u * 2.0f * kPi;
    float phi = (v - 0.5f) * kPi;
    float cos_phi = cos(phi);

    float x = cos_phi * cos(theta);
    float y = cos_phi * sin(theta);
    float z = sin(phi);

    return pxr::GfVec3f(x, y, z);
}

int calcSphereIndex(int i, int j, int num_v_verts, int num_u_verts, std::vector<pxr::GfVec3f>& points)
{
    if (j == 0)
    {
        return 0;
    }
    else if (j == num_v_verts - 1)
    {
        return (int)points.size() - 1;
    }
    else
    {
        i = (i < num_u_verts) ? i : 0;
        return (j - 1) * num_u_verts + i + 1;
    }
}

pxr::UsdGeomMesh createSphereMesh(pxr::UsdStageWeakPtr stage, const pxr::SdfPath path, float scale)
{
    int u_patches = 32;
    int v_patches = 16;

    int num_u_verts_scale = 1;
    int num_v_verts_scale = 1;

    u_patches = u_patches * num_u_verts_scale;
    v_patches = v_patches * num_v_verts_scale;
    u_patches = (u_patches > 3) ? u_patches : 3;
    v_patches = (v_patches > 3) ? v_patches : 2;

    float u_delta = 1.0f / (float)u_patches;
    float v_delta = 1.0f / (float)v_patches;

    int num_u_verts = u_patches;
    int num_v_verts = v_patches + 1;

    std::vector<pxr::GfVec3f> points;
    std::vector<pxr::GfVec3f> normals;
    std::vector<int> face_indices;
    std::vector<int> face_vertex_counts;

    pxr::GfVec3f bottom_point = pxr::GfVec3f(0.0f, 0.0f, -1.0f);
    points.push_back(bottom_point);

    for (int j = 0; j < num_v_verts - 1; j++)
    {
        float v = (float)j * v_delta;
        for (int i = 0; i < num_u_verts; i++)
        {
            float u = (float)i * u_delta;
            pxr::GfVec3f point = evalSphereCoord(u, v);
            points.push_back(point);
        }
    }

    pxr::GfVec3f top_point = pxr::GfVec3f(0.0f, 0.0f, 1.0f);
    points.push_back(top_point);

    // generate body
    for (int j = 0; j < v_patches; j++)
    {
        for (int i = 0; i < u_patches; i++)
        {
            // index 0 is the bottom hat point
            int vindex00 = calcSphereIndex(i, j, num_v_verts, num_u_verts, points);
            int vindex10 = calcSphereIndex(i + 1, j, num_v_verts, num_u_verts, points);
            int vindex11 = calcSphereIndex(i + 1, j + 1, num_v_verts, num_u_verts, points);
            int vindex01 = calcSphereIndex(i, j + 1, num_v_verts, num_u_verts, points);

            pxr::GfVec3f p0 = points[vindex00];
            pxr::GfVec3f p1 = points[vindex10];
            pxr::GfVec3f p2 = points[vindex11];
            pxr::GfVec3f p3 = points[vindex01];

            if (vindex11 == vindex01)
            {
                face_indices.push_back(vindex00);
                face_indices.push_back(vindex10);
                face_indices.push_back(vindex01);
                face_vertex_counts.push_back(3);
                normals.push_back(p0);
                normals.push_back(p1);
                normals.push_back(p3);
            }
            else if (vindex00 == vindex10)
            {
                face_indices.push_back(vindex00);
                face_indices.push_back(vindex11);
                face_indices.push_back(vindex01);
                face_vertex_counts.push_back(3);
                normals.push_back(p0);
                normals.push_back(p2);
                normals.push_back(p3);
            }
            else
            {
                face_indices.push_back(vindex00);
                face_indices.push_back(vindex10);
                face_indices.push_back(vindex11);
                face_indices.push_back(vindex01);
                face_vertex_counts.push_back(4);
                normals.push_back(p0);
                normals.push_back(p1);
                normals.push_back(p2);
                normals.push_back(p3);
            }
        }
    }

    pxr::UsdGeomMesh usdMesh = createMesh(stage, path, points, normals, face_indices, face_vertex_counts);
    return usdMesh;
}

pxr::UsdPrim createPrimitiveGeom(pxr::UsdStageWeakPtr stage,
                                 const std::string geomPath,
                                 const MJCFGeom* geom,
                                 const std::map<std::string, MeshInfo>& simulationMeshCache,
                                 const ImportConfig& config,
                                 std::map<pxr::TfToken, pxr::SdfPath>& materialPaths,
                                 bool importMaterials,
                                 const std::string rootPrimPath,
                                 bool collisionGeom)
{
    pxr::SdfPath path = pxr::SdfPath(geomPath);

    if (geom->type != MJCFVisualElement::MESH)
    {
        pxr::UsdGeomXform::Define(stage, path);
        path = path.AppendPath(pxr::SdfPath(path.GetName()));
    }
    if (geom->type == MJCFVisualElement::PLANE)
    {
        // add visual plane
        pxr::UsdGeomMesh groundPlane = pxr::UsdGeomMesh::Define(stage, path);
        groundPlane.CreateDisplayColorAttr().Set(pxr::VtArray<pxr::GfVec3f>({ pxr::GfVec3f(0.5f, 0.5f, 0.5f) }));
        pxr::VtIntArray faceVertexCounts({ 4 });
        pxr::VtIntArray faceVertexIndices({ 0, 1, 2, 3 });

        pxr::GfVec3f normalsBase[] = { pxr::GfVec3f(0.0f, 0.0f, 1.0f), pxr::GfVec3f(0.0f, 0.0f, 1.0f),
                                       pxr::GfVec3f(0.0f, 0.0f, 1.0f), pxr::GfVec3f(0.0f, 0.0f, 1.0f) };
        const size_t normalCount = sizeof(normalsBase) / sizeof(normalsBase[0]);
        pxr::VtVec3fArray normals;
        normals.resize(normalCount);
        for (uint32_t i = 0; i < normalCount; i++)
        {
            const pxr::GfVec3f& pointSrc = normalsBase[i];
            pxr::GfVec3f& pointDst = normals[i];
            pointDst[0] = pointSrc[0];
            pointDst[1] = pointSrc[1];
            pointDst[2] = pointSrc[2];
        }

        float planeSize[] = { geom->size.x, geom->size.y };
        pxr::GfVec3f pointsBase[] = {
            pxr::GfVec3f(-planeSize[0], -planeSize[1], 0.0f) * config.distanceScale,
            pxr::GfVec3f(-planeSize[0], planeSize[1], 0.0f) * config.distanceScale,
            pxr::GfVec3f(planeSize[0], planeSize[1], 0.0f) * config.distanceScale,
            pxr::GfVec3f(planeSize[0], -planeSize[1], 0.0f) * config.distanceScale,
        };

        const size_t pointCount = sizeof(pointsBase) / sizeof(pointsBase[0]);
        pxr::VtVec3fArray points;
        points.resize(pointCount);
        for (uint32_t i = 0; i < pointCount; i++)
        {
            const pxr::GfVec3f& pointSrc = pointsBase[i];
            pxr::GfVec3f& pointDst = points[i];
            pointDst[0] = pointSrc[0];
            pointDst[1] = pointSrc[1];
            pointDst[2] = pointSrc[2];
        }

        groundPlane.CreateFaceVertexCountsAttr().Set(faceVertexCounts);
        groundPlane.CreateFaceVertexIndicesAttr().Set(faceVertexIndices);
        groundPlane.CreateNormalsAttr().Set(normals);
        groundPlane.CreatePointsAttr().Set(points);
    }
    else if (geom->type == MJCFVisualElement::SPHERE)
    {

        pxr::UsdGeomSphere spherePrim = pxr::UsdGeomSphere::Define(stage, path);
        pxr::VtVec3fArray extentArray(2);

        spherePrim.ComputeExtent(geom->size.x, &extentArray);
        spherePrim.GetRadiusAttr().Set(double(geom->size.x));
        spherePrim.GetExtentAttr().Set(extentArray);
    }
    else if (geom->type == MJCFVisualElement::ELLIPSOID)
    {
        if (collisionGeom)
        {
            // use mesh for collision, or else collision mesh does not work properly
            createSphereMesh(stage, path, config.distanceScale);
        }
        else
        {
            // use shape prim for visual
            pxr::UsdGeomSphere ellipsePrim = pxr::UsdGeomSphere::Define(stage, path);
            pxr::VtVec3fArray extentArray(2);

            ellipsePrim.ComputeExtent(geom->size.x, &extentArray);
            ellipsePrim.GetExtentAttr().Set(extentArray);
        }
    }
    else if (geom->type == MJCFVisualElement::CAPSULE)
    {
        pxr::UsdGeomCapsule capsulePrim = pxr::UsdGeomCapsule::Define(stage, path);
        pxr::VtVec3fArray extentArray(4);
        pxr::TfToken axis = pxr::TfToken("X");
        float height;
        if (geom->hasFromTo)
        {
            Vec3 dif = geom->to - geom->from;
            height = Length(dif);
        }
        else
        {
            // half length
            height = 2.0f * geom->size.y;
        }

        capsulePrim.GetRadiusAttr().Set(double(geom->size.x));
        capsulePrim.GetHeightAttr().Set(double(height));
        capsulePrim.GetAxisAttr().Set(axis);
        capsulePrim.ComputeExtent(double(height), double(geom->size.x), axis, &extentArray);
        capsulePrim.GetExtentAttr().Set(extentArray);
    }
    else if (geom->type == MJCFVisualElement::CYLINDER)
    {
        pxr::UsdGeomCylinder cylinderPrim = pxr::UsdGeomCylinder::Define(stage, path);
        pxr::VtVec3fArray extentArray(2);
        float height;
        if (geom->hasFromTo)
        {
            Vec3 dif = geom->to - geom->from;
            height = Length(dif);
        }
        else
        {
            height = 2.0f * geom->size.y;
        }
        pxr::TfToken axis = pxr::TfToken("X");
        cylinderPrim.ComputeExtent(double(height), double(geom->size.x), axis, &extentArray);
        cylinderPrim.GetAxisAttr().Set(pxr::UsdGeomTokens->z);
        cylinderPrim.GetExtentAttr().Set(extentArray);
        cylinderPrim.GetHeightAttr().Set(double(height));
        cylinderPrim.GetRadiusAttr().Set(double(geom->size.x));
    }
    else if (geom->type == MJCFVisualElement::BOX)
    {
        pxr::UsdGeomCube boxPrim = pxr::UsdGeomCube::Define(stage, path);
        pxr::VtVec3fArray extentArray(2);
        extentArray[1] = pxr::GfVec3f(geom->size.x, geom->size.y, geom->size.z);
        extentArray[0] = -extentArray[1];
        boxPrim.GetExtentAttr().Set(extentArray);
    }
    else if (geom->type == MJCFVisualElement::MESH)
    {
        CARB_LOG_INFO("Mesh Type");
        MeshInfo meshInfo = simulationMeshCache.find(geom->mesh)->second;
        path = createUsdMesh(stage, path, meshInfo.mesh, materialPaths, config.distanceScale, importMaterials)
                   .GetPrim()
                   .GetChildren()
                   .begin()
                   ->GetPath();
        CARB_LOG_INFO("Created mesh at path %s", geomPath.c_str());
    }


    return stage->GetPrimAtPath(pxr::SdfPath(geomPath));
}

pxr::UsdPrim createPrimitiveGeom(pxr::UsdStageWeakPtr stage,
                                 const std::string geomPath,
                                 const MJCFSite* site,
                                 const ImportConfig& config,
                                 bool importMaterials)
{
    pxr::SdfPath path = pxr::SdfPath(geomPath);
    {
        pxr::UsdGeomXform::Define(stage, path);
        path = path.AppendPath(pxr::SdfPath(path.GetName()));
    }
    if (site->type == MJCFVisualElement::SPHERE)
    {
        pxr::UsdGeomSphere spherePrim = pxr::UsdGeomSphere::Define(stage, path);
        pxr::VtVec3fArray extentArray(2);

        spherePrim.ComputeExtent(site->size.x, &extentArray);
        spherePrim.GetRadiusAttr().Set(double(site->size.x));
        spherePrim.GetExtentAttr().Set(extentArray);
    }
    else if (site->type == MJCFVisualElement::ELLIPSOID)
    {
        pxr::UsdGeomSphere ellipsePrim = pxr::UsdGeomSphere::Define(stage, path);
        pxr::VtVec3fArray extentArray(2);

        ellipsePrim.ComputeExtent(site->size.x, &extentArray);
        ellipsePrim.GetExtentAttr().Set(extentArray);
    }
    else if (site->type == MJCFVisualElement::CAPSULE)
    {
        pxr::UsdGeomCapsule capsulePrim = pxr::UsdGeomCapsule::Define(stage, path);
        pxr::VtVec3fArray extentArray(4);
        pxr::TfToken axis = pxr::TfToken("X");
        float height;
        if (site->hasFromTo)
        {
            Vec3 dif = site->to - site->from;
            height = Length(dif);
        }
        else
        {
            // half length
            height = 2.0f * site->size.y;
        }

        capsulePrim.GetRadiusAttr().Set(double(site->size.x));
        capsulePrim.GetHeightAttr().Set(double(height));
        capsulePrim.GetAxisAttr().Set(axis);
        capsulePrim.ComputeExtent(double(height), double(site->size.x), axis, &extentArray);
        capsulePrim.GetExtentAttr().Set(extentArray);
    }
    else if (site->type == MJCFVisualElement::CYLINDER)
    {
        pxr::UsdGeomCylinder cylinderPrim = pxr::UsdGeomCylinder::Define(stage, path);
        pxr::VtVec3fArray extentArray(2);
        float height;
        if (site->hasFromTo)
        {
            Vec3 dif = site->to - site->from;
            height = Length(dif);
        }
        else
        {
            height = 2.0f * site->size.y;
        }
        pxr::TfToken axis = pxr::TfToken("X");
        cylinderPrim.ComputeExtent(double(height), double(site->size.x), axis, &extentArray);
        cylinderPrim.GetAxisAttr().Set(pxr::UsdGeomTokens->z);
        cylinderPrim.GetExtentAttr().Set(extentArray);
        cylinderPrim.GetHeightAttr().Set(double(height));
        cylinderPrim.GetRadiusAttr().Set(double(site->size.x));
    }
    else if (site->type == MJCFVisualElement::BOX)
    {
        pxr::UsdGeomCube boxPrim = pxr::UsdGeomCube::Define(stage, path);
        pxr::VtVec3fArray extentArray(2);
        extentArray[1] = pxr::GfVec3f(site->size.x, site->size.y, site->size.z);
        extentArray[0] = -extentArray[1];
        boxPrim.GetExtentAttr().Set(extentArray);
    }

    pxr::UsdPrim prim = stage->GetPrimAtPath(path);
    if (prim)
    {
        // set the transformations first
        pxr::GfMatrix4d mat;
        mat.SetIdentity();
        mat.SetTranslateOnly(pxr::GfVec3d(site->pos.x, site->pos.y, site->pos.z));
        mat.SetRotateOnly(pxr::GfQuatd(site->quat.w, site->quat.x, site->quat.y, site->quat.z));

        pxr::GfMatrix4d scale;
        scale.SetIdentity();
        scale.SetScale(pxr::GfVec3d(config.distanceScale, config.distanceScale, config.distanceScale));
        if (site->type == MJCFVisualElement::ELLIPSOID)
        {
            scale.SetScale(config.distanceScale * pxr::GfVec3d(site->size.x, site->size.y, site->size.z));
        }
        else if (site->type == MJCFVisualElement::CAPSULE)
        {
            Vec3 cen;
            Quat q;

            if (site->hasFromTo)
            {
                Vec3 diff = site->to - site->from;
                diff = Normalize(diff);
                Vec3 rotVec = Cross(Vec3(1.0f, 0.0f, 0.0f), diff);
                if (Length(rotVec) < 1e-5)
                {
                    rotVec = Vec3(0.0f, 1.0f, 0.0f); // default rotation about y-axis
                }
                else
                {
                    rotVec = Normalize(rotVec); // z axis
                }

                float angle = acos(diff.x);
                cen = 0.5f * (site->from + site->to);
                q = QuatFromAxisAngle(rotVec, angle);
            }
            else
            {
                cen = site->pos;
                q = site->quat * QuatFromAxisAngle(Vec3(0.0f, 1.0f, 0.0f), -kPi * 0.5f);
            }

            mat.SetTranslateOnly(config.distanceScale * pxr::GfVec3d(cen.x, cen.y, cen.z));
            mat.SetRotateOnly(pxr::GfQuatd(q.w, q.x, q.y, q.z));
        }
        else if (site->type == MJCFVisualElement::CYLINDER)
        {
            Vec3 cen;
            Quat q;
            if (site->hasFromTo)
            {
                cen = 0.5f * (site->from + site->to);
                Vec3 axis = site->to - site->from;
                q = GetRotationQuat(Vec3(0.0f, 0.0f, 1.0f), Normalize(axis));
            }
            else
            {
                cen = site->pos;
                q = site->quat;
            }

            mat.SetRotateOnly(pxr::GfQuatd(q.w, q.x, q.y, q.z));
            mat.SetTranslateOnly(pxr::GfVec3d(cen.x, cen.y, cen.z));
        }
        else if (site->type == MJCFVisualElement::BOX)
        {
            Vec3 s = site->size;
            Vec3 cen = site->pos;
            Quat q = site->quat;
            scale.SetScale(config.distanceScale * pxr::GfVec3d(s.x, s.y, s.z));
            mat.SetTranslateOnly(config.distanceScale * pxr::GfVec3d(cen.x, cen.y, cen.z));
            mat.SetRotateOnly(pxr::GfQuatd(q.w, q.x, q.y, q.z));
        }

        pxr::UsdGeomXformable gprim = pxr::UsdGeomXformable(prim);
        gprim.ClearXformOpOrder();
        gprim.AddTranslateOp(pxr::UsdGeomXformOp::PrecisionDouble).Set(mat.ExtractTranslation());
        gprim.AddOrientOp(pxr::UsdGeomXformOp::PrecisionDouble).Set(mat.ExtractRotationQuat());
        gprim.AddScaleOp(pxr::UsdGeomXformOp::PrecisionDouble)
            .Set(pxr::GfVec3d(scale.GetRow3(0)[0], scale.GetRow3(1)[1], scale.GetRow3(2)[2]));
    }

    return stage->GetPrimAtPath(pxr::SdfPath(geomPath));
}

void applyCollisionGeom(pxr::UsdStageWeakPtr stage, pxr::UsdPrim prim, bool ConvexDecomposition)
{
    pxr::UsdPhysicsCollisionAPI::Apply(prim);
    for (const auto mesh_prim : pxr::UsdPrimRange(prim))
    {
        if (pxr::UsdGeomMesh(mesh_prim))
        {
            pxr::UsdPhysicsMeshCollisionAPI physicsMeshAPI = pxr::UsdPhysicsMeshCollisionAPI::Apply(prim);
            // pxr::PhysxSchemaPhysxMeshMergeCollisionAPI mergeAPI =
            // pxr::PhysxSchemaPhysxMeshMergeCollisionAPI::Apply(prim);
            // mergeAPI.GetCollisionMeshesCollectionAPI().GetIncludesRel().AddTarget(prim.GetPath());
            if (ConvexDecomposition)
            {
                physicsMeshAPI.CreateApproximationAttr().Set(pxr::UsdPhysicsTokens.Get()->convexDecomposition);
            }
            else
            {
                physicsMeshAPI.CreateApproximationAttr().Set(pxr::UsdPhysicsTokens.Get()->convexHull);
            }
            break;
        }
    }
}

pxr::UsdPhysicsJoint createFixedJoint(pxr::UsdStageWeakPtr stage,
                                      const std::string jointPath,
                                      const Transform& poseJointToParentBody,
                                      const Transform& poseJointToChildBody,
                                      const std::string parentBodyPath,
                                      const std::string bodyPath,
                                      const ImportConfig& config)
{
    pxr::UsdPhysicsJoint jointPrim = pxr::UsdPhysicsFixedJoint::Define(stage, pxr::SdfPath(jointPath));

    pxr::GfVec3f localPos0 = config.distanceScale * pxr::GfVec3f(poseJointToParentBody.p.x, poseJointToParentBody.p.y,
                                                                 poseJointToParentBody.p.z);
    pxr::GfQuatf localRot0 = pxr::GfQuatf(
        poseJointToParentBody.q.w, poseJointToParentBody.q.x, poseJointToParentBody.q.y, poseJointToParentBody.q.z);
    pxr::GfVec3f localPos1 = config.distanceScale *
                             pxr::GfVec3f(poseJointToChildBody.p.x, poseJointToChildBody.p.y, poseJointToChildBody.p.z);
    pxr::GfQuatf localRot1 = pxr::GfQuatf(
        poseJointToChildBody.q.w, poseJointToChildBody.q.x, poseJointToChildBody.q.y, poseJointToChildBody.q.z);

    pxr::SdfPathVector val0{ pxr::SdfPath(parentBodyPath) };
    pxr::SdfPathVector val1{ pxr::SdfPath(bodyPath) };

    jointPrim.CreateBody0Rel().SetTargets(val0);
    jointPrim.CreateLocalPos0Attr().Set(localPos0);
    jointPrim.CreateLocalRot0Attr().Set(localRot0);

    jointPrim.CreateBody1Rel().SetTargets(val1);
    jointPrim.CreateLocalPos1Attr().Set(localPos1);
    jointPrim.CreateLocalRot1Attr().Set(localRot1);

    jointPrim.CreateBreakForceAttr().Set(FLT_MAX);
    jointPrim.CreateBreakTorqueAttr().Set(FLT_MAX);

    return jointPrim;
}

pxr::UsdPhysicsJoint createD6Joint(pxr::UsdStageWeakPtr stage,
                                   const std::string jointPath,
                                   const Transform& poseJointToParentBody,
                                   const Transform& poseJointToChildBody,
                                   const std::string parentBodyPath,
                                   const std::string bodyPath,
                                   const ImportConfig& config)
{
    pxr::UsdPhysicsJoint jointPrim = pxr::UsdPhysicsJoint::Define(stage, pxr::SdfPath(jointPath));

    pxr::GfVec3f localPos0 = config.distanceScale * pxr::GfVec3f(poseJointToParentBody.p.x, poseJointToParentBody.p.y,
                                                                 poseJointToParentBody.p.z);
    pxr::GfQuatf localRot0 = pxr::GfQuatf(
        poseJointToParentBody.q.w, poseJointToParentBody.q.x, poseJointToParentBody.q.y, poseJointToParentBody.q.z);
    pxr::GfVec3f localPos1 = config.distanceScale *
                             pxr::GfVec3f(poseJointToChildBody.p.x, poseJointToChildBody.p.y, poseJointToChildBody.p.z);
    pxr::GfQuatf localRot1 = pxr::GfQuatf(
        poseJointToChildBody.q.w, poseJointToChildBody.q.x, poseJointToChildBody.q.y, poseJointToChildBody.q.z);

    pxr::SdfPathVector val0{ pxr::SdfPath(parentBodyPath) };
    pxr::SdfPathVector val1{ pxr::SdfPath(bodyPath) };

    jointPrim.CreateBody0Rel().SetTargets(val0);
    jointPrim.CreateLocalPos0Attr().Set(localPos0);
    jointPrim.CreateLocalRot0Attr().Set(localRot0);

    jointPrim.CreateBody1Rel().SetTargets(val1);
    jointPrim.CreateLocalPos1Attr().Set(localPos1);
    jointPrim.CreateLocalRot1Attr().Set(localRot1);

    jointPrim.CreateBreakForceAttr().Set(FLT_MAX);
    jointPrim.CreateBreakTorqueAttr().Set(FLT_MAX);

    return jointPrim;
}

void initPhysicsJoint(pxr::UsdPhysicsJoint& jointPrim,
                      const Transform& poseJointToParentBody,
                      const Transform& poseJointToChildBody,
                      const std::string parentBodyPath,
                      const std::string bodyPath,
                      const float& distanceScale)
{
    pxr::GfVec3f localPos0 =
        distanceScale * pxr::GfVec3f(poseJointToParentBody.p.x, poseJointToParentBody.p.y, poseJointToParentBody.p.z);
    pxr::GfQuatf localRot0 = pxr::GfQuatf(
        poseJointToParentBody.q.w, poseJointToParentBody.q.x, poseJointToParentBody.q.y, poseJointToParentBody.q.z);
    pxr::GfVec3f localPos1 =
        distanceScale * pxr::GfVec3f(poseJointToChildBody.p.x, poseJointToChildBody.p.y, poseJointToChildBody.p.z);
    pxr::GfQuatf localRot1 = pxr::GfQuatf(
        poseJointToChildBody.q.w, poseJointToChildBody.q.x, poseJointToChildBody.q.y, poseJointToChildBody.q.z);

    pxr::SdfPathVector val0{ pxr::SdfPath(parentBodyPath) };
    pxr::SdfPathVector val1{ pxr::SdfPath(bodyPath) };

    jointPrim.CreateBody0Rel().SetTargets(val0);
    jointPrim.CreateLocalPos0Attr().Set(localPos0);
    jointPrim.CreateLocalRot0Attr().Set(localRot0);

    jointPrim.CreateBody1Rel().SetTargets(val1);
    jointPrim.CreateLocalPos1Attr().Set(localPos1);
    jointPrim.CreateLocalRot1Attr().Set(localRot1);

    jointPrim.CreateBreakForceAttr().Set(FLT_MAX);
    jointPrim.CreateBreakTorqueAttr().Set(FLT_MAX);
}

void applyPhysxJoint(pxr::UsdPhysicsJoint& jointPrim, const MJCFJoint* joint)
{
    pxr::PhysxSchemaPhysxJointAPI physxJoint = pxr::PhysxSchemaPhysxJointAPI::Apply(jointPrim.GetPrim());
    physxJoint.CreateArmatureAttr().Set(joint->armature);
}

void applyJointLimits(pxr::UsdPhysicsJoint jointPrim,
                      const MJCFJoint* joint,
                      const MJCFActuator* actuator,
                      const int* axisMap,
                      const int jointIdx,
                      const int numJoints,
                      const ImportConfig& config)
{
    // enable limits if set
    JointAxis axisHinge[3] = { eJointAxisTwist, eJointAxisSwing1, eJointAxisSwing2 };
    JointAxis axisSlide[3] = { eJointAxisX, eJointAxisY, eJointAxisZ };
    std::string d6Axes[6] = { "transX", "transY", "transZ", "rotX", "rotY", "rotZ" };
    int axis = -1;
    std::string limitAttr = "";

    // assume we can only have one of slide or hinge per d6 joint
    if (joint->type == MJCFJoint::SLIDE)
    {
        // lock all rotation axes
        for (int i = 3; i < 6; ++i)
        {
            pxr::UsdPhysicsLimitAPI limitAPI =
                pxr::UsdPhysicsLimitAPI::Apply(jointPrim.GetPrim(), pxr::TfToken(d6Axes[i]));
            limitAPI.CreateLowAttr().Set(1.0f);
            limitAPI.CreateHighAttr().Set(-1.0f);
        }

        axis = int(axisSlide[axisMap[jointIdx]]);

        if (joint->limited)
        {
            pxr::UsdPhysicsLimitAPI limitAPI =
                pxr::UsdPhysicsLimitAPI::Apply(jointPrim.GetPrim(), pxr::TfToken(d6Axes[axis]));
            limitAPI.CreateLowAttr().Set(config.distanceScale * joint->range.x);
            limitAPI.CreateHighAttr().Set(config.distanceScale * joint->range.y);
        }
        pxr::PhysxSchemaPhysxLimitAPI physxLimitAPI =
            pxr::PhysxSchemaPhysxLimitAPI::Apply(jointPrim.GetPrim(), pxr::TfToken(d6Axes[axis]));
        pxr::PhysxSchemaJointStateAPI::Apply(jointPrim.GetPrim(), pxr::TfToken("linear"));
        physxLimitAPI.CreateStiffnessAttr().Set(joint->stiffness);
        physxLimitAPI.CreateDampingAttr().Set(joint->damping);
    }
    else if (joint->type == MJCFJoint::HINGE)
    {
        // lock all translation axes
        for (int i = 0; i < 3; ++i)
        {
            pxr::UsdPhysicsLimitAPI limitAPI =
                pxr::UsdPhysicsLimitAPI::Apply(jointPrim.GetPrim(), pxr::TfToken(d6Axes[i]));
            limitAPI.CreateLowAttr().Set(1.0f);
            limitAPI.CreateHighAttr().Set(-1.0f);
        }
        // TODO: locking all axes at the beginning doesn't work?
        if (numJoints == 1)
        {
            pxr::UsdPhysicsLimitAPI limitAPI =
                pxr::UsdPhysicsLimitAPI::Apply(jointPrim.GetPrim(), pxr::TfToken(d6Axes[axisHinge[axisMap[1]]]));
            limitAPI.CreateLowAttr().Set(1.0f);
            limitAPI.CreateHighAttr().Set(-1.0f);
            limitAPI = pxr::UsdPhysicsLimitAPI::Apply(jointPrim.GetPrim(), pxr::TfToken(d6Axes[axisHinge[axisMap[2]]]));
            limitAPI.CreateLowAttr().Set(1.0f);
            limitAPI.CreateHighAttr().Set(-1.0f);
        }
        else if (numJoints == 2)
        {
            pxr::UsdPhysicsLimitAPI limitAPI =
                pxr::UsdPhysicsLimitAPI::Apply(jointPrim.GetPrim(), pxr::TfToken(d6Axes[axisHinge[axisMap[2]]]));
            limitAPI.CreateLowAttr().Set(1.0f);
            limitAPI.CreateHighAttr().Set(-1.0f);
        }

        axis = int(axisHinge[axisMap[jointIdx]]);

        if (joint->limited)
        {
            pxr::UsdPhysicsLimitAPI limitAPI =
                pxr::UsdPhysicsLimitAPI::Apply(jointPrim.GetPrim(), pxr::TfToken(d6Axes[axis]));
            limitAPI.CreateLowAttr().Set(joint->range.x * 180 / kPi);
            limitAPI.CreateHighAttr().Set(joint->range.y * 180 / kPi);
            pxr::PhysxSchemaPhysxLimitAPI physxLimitAPI =
                pxr::PhysxSchemaPhysxLimitAPI::Apply(jointPrim.GetPrim(), pxr::TfToken(d6Axes[axis]));
            physxLimitAPI.CreateStiffnessAttr().Set(joint->stiffness);
            physxLimitAPI.CreateDampingAttr().Set(joint->damping);
        }
        pxr::PhysxSchemaJointStateAPI::Apply(jointPrim.GetPrim(), pxr::TfToken("angular"));
    }

    jointPrim.GetPrim()
        .CreateAttribute(pxr::TfToken("mjcf:" + d6Axes[axis] + ":name"), pxr::SdfValueTypeNames->Token)
        .Set(pxr::TfToken(SanitizeUsdName(joint->name)));

    createJointDrives(jointPrim, joint, actuator, d6Axes[axis], config);
}

void createJointDrives(pxr::UsdPhysicsJoint jointPrim,
                       const MJCFJoint* joint,
                       const MJCFActuator* actuator,
                       const std::string axis,
                       const ImportConfig& config)
{
    pxr::UsdPhysicsDriveAPI driveAPI = pxr::UsdPhysicsDriveAPI::Apply(jointPrim.GetPrim(), pxr::TfToken(axis));

    driveAPI = pxr::UsdPhysicsDriveAPI::Apply(jointPrim.GetPrim(), pxr::TfToken(axis));
    driveAPI.CreateTypeAttr().Set(pxr::TfToken("force")); // TODO: when will this be acceleration?

    driveAPI.CreateDampingAttr().Set(joint->damping);
    driveAPI.CreateStiffnessAttr().Set(joint->stiffness);

    if (actuator)
    {
        MJCFActuator::Type actuatorType = actuator->type;
        if (actuatorType == MJCFActuator::MOTOR || actuatorType == MJCFActuator::GENERAL)
        {
            // nothing special
        }
        else if (actuatorType == MJCFActuator::POSITION)
        {
            driveAPI.CreateStiffnessAttr().Set(actuator->kp);
        }
        else if (actuatorType == MJCFActuator::VELOCITY)
        {
            driveAPI.CreateStiffnessAttr().Set(actuator->kv);
        }

        const Vec2& forcerange = actuator->forcerange;
        float maxForce = std::max(abs(forcerange.x), abs(forcerange.y));
        if (maxForce == FLT_MAX)
        {
            maxForce = std::max(abs(joint->forcerange.x), abs(joint->forcerange.y));
        }
        if (maxForce != FLT_MAX)
        {
            driveAPI.CreateMaxForceAttr().Set(maxForce);
        }
    }
    else
    {
        float maxForce = std::max(abs(joint->forcerange.x), abs(joint->forcerange.y));
        if (maxForce != FLT_MAX)
        {
            driveAPI.CreateMaxForceAttr().Set(maxForce);
        }
    }
}
} // namespace mjcf
} // namespace importer
} // namespace asset
} // namespace isaacsim
