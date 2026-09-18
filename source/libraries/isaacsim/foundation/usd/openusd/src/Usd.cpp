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

#include "details/UsdHelpers.hpp"

#include <isaacsim/common/exceptions/Exceptions.hpp>
#include <isaacsim/foundation/usd/openusd/Usd.hpp>
#include <pxr/base/gf/rotation.h>
#include <pxr/base/gf/transform.h>
#include <pxr/base/tf/errorMark.h>
#include <pxr/usd/ar/resolverScopedCache.h>
#include <pxr/usd/sdf/fileFormat.h>
#include <pxr/usd/sdf/layer.h>
#include <pxr/usd/sdf/namespaceEdit.h>
#include <pxr/usd/sdf/reference.h>
#include <pxr/usd/sdf/schema.h>
#include <pxr/usd/usd/attribute.h>
#include <pxr/usd/usd/crateInfo.h>
#include <pxr/usd/usd/interpolation.h>
#include <pxr/usd/usd/primRange.h>
#include <pxr/usd/usd/references.h>
#include <pxr/usd/usd/schemaRegistry.h>
#include <pxr/usd/usd/variantSets.h>
#include <pxr/usd/usdGeom/metrics.h>
#include <pxr/usd/usdGeom/tokens.h>
#include <pxr/usd/usdGeom/xformCache.h>
#include <pxr/usd/usdPhysics/metrics.h>

#include <algorithm>
#include <atomic>
#include <cctype>
#include <cstdio>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <type_traits>
#include <unordered_map>
#include <variant>


namespace isaacsim
{
namespace foundation
{
namespace usd
{
namespace openusd
{


int64_t createStage()
{
    // Create a new stage in-memory.
    static std::atomic<size_t> s_newStageCount = 0;
    PXR_NS::UsdStageRefPtr stage =
        PXR_NS::UsdStage::CreateInMemory("Stage_" + std::to_string(s_newStageCount.fetch_add(1)) + ".usd");
    if (!stage)
    {
        return -1;
    }

    // Insert the stage into the stage cache.
    PXR_NS::UsdStageCache::Id id = PXR_NS::UsdUtilsStageCache::Get().Insert(stage);
    int64_t stageId = static_cast<int64_t>(id.ToLongInt());

    // Author default stage attributes.
    setStageUnits(stageId, 1.0f, 1.0f);
    setStageUpAxis(stageId, "Z");
    setStageTimeCode(stageId, 0.0f, 1000000.0f, 60.0f);
    stage->GetRootLayer()->SetPermissionToEdit(true);
    stage->SetInterpolationType(PXR_NS::UsdInterpolationTypeLinear);

    return stageId;
}

int64_t openStage(const std::string& usdPath)
{
    // Check for version mismatch in crate files.
    if (PXR_NS::SdfFileFormat::GetFileExtension(usdPath) == "usdc" ||
        PXR_NS::SdfFileFormat::GetFileExtension(usdPath) == "usd")
    {
        // Only binary crate files can have a version mismatch; skip if the file is actually usda.
        if (!PXR_NS::SdfFileFormat::FindByExtension(".usda")->CanRead(usdPath))
        {
            PXR_NS::TfErrorMark mark;
            PXR_NS::UsdCrateInfo crateInfo = PXR_NS::UsdCrateInfo::Open(usdPath);
            if (!mark.IsClean())
            {
                for (const PXR_NS::TfError& err : mark)
                {
                    const std::string& comment = err.GetCommentary();
                    if (comment.find("crate file version mismatch") != std::string::npos)
                    {
                        std::regex versionRegex("crate file version mismatch -- file is (\\d+\\.\\d+\\.\\d+)");
                        std::smatch match;
                        if (std::regex_search(comment, match, versionRegex))
                        {
                            const std::string version = match[1].str();
                            const std::string supportedVersion = crateInfo.GetSoftwareVersion().GetText();
                            fprintf(stderr,
                                    "Unable to open file %s. The current OpenUSD runtime does not support crate file "
                                    "version %s; re-save the file to crate version lower than %s. "
                                    "To update, set USD_WRITE_NEW_USDC_FILES_AS_VERSION=%s (new files only).\n",
                                    usdPath.c_str(), version.c_str(), supportedVersion.c_str(), supportedVersion.c_str());
                            return -1;
                        }
                    }
                }
                mark.Clear();
            }
        }
    }

    PXR_NS::SdfLayerRefPtr rootLayer = PXR_NS::SdfLayer::FindOrOpen(usdPath);
    if (!rootLayer)
    {
        return -1;
    }

    // Use a scoped resolver cache to avoid re-resolving layers during reload
    // that were already resolved during UsdStage::Open (important for remote stages).
    PXR_NS::ArResolverScopedCache resolverCache;
    PXR_NS::UsdStageRefPtr stage = PXR_NS::UsdStage::Open(rootLayer, PXR_NS::UsdStage::LoadAll);
    if (!stage)
    {
        return -1;
    }

    // Reload to pick up any on-disk changes for layers that were already cached in memory.
    stage->Reload();

    stage->GetRootLayer()->SetPermissionToSave(true);
    stage->GetRootLayer()->SetPermissionToEdit(true);
    stage->SetInterpolationType(PXR_NS::UsdInterpolationTypeLinear);

    // Insert the stage into the stage cache.
    PXR_NS::UsdStageCache::Id id = PXR_NS::UsdUtilsStageCache::Get().Insert(stage);
    return static_cast<int64_t>(id.ToLongInt());
}

bool saveStage(int64_t stageId, const std::string& usdPath)
{
    PXR_NS::UsdStageRefPtr stage = details::getStage(stageId, true);
    if (!PXR_NS::UsdStage::IsSupportedFile(usdPath))
    {
        throw std::invalid_argument("The file '" + usdPath + "' is not a supported USD file");
    }

    bool created = false;
    PXR_NS::SdfLayerRefPtr layer = PXR_NS::SdfLayer::Find(usdPath);
    if (!layer)
    {
        std::remove(usdPath.c_str()); // CreateNew refuses to overwrite an existing file
        layer = PXR_NS::SdfLayer::CreateNew(usdPath);
        if (!layer)
        {
            return false;
        }
        created = true;
    }

    PXR_NS::SdfLayerRefPtr rootLayer = stage->GetRootLayer();
    layer->TransferContent(rootLayer);
    // TODO: resolve asset paths from rootLayer->GetIdentifier() to layer->GetIdentifier()
    // so that relative references (sublayers, payloads, references) remain valid when the file is opened from usdPath.
    bool result = layer->Save();
    if (created)
    {
        layer.Reset(); // drop our strong ref; evicts from registry when no other holders remain
    }
    return result;
}

bool closeStage(int64_t stageId)
{
    PXR_NS::UsdStageCache& cache = PXR_NS::UsdUtilsStageCache::Get();
    PXR_NS::UsdStageCache::Id id = PXR_NS::UsdStageCache::Id::FromLongInt(stageId);
    return cache.Contains(id) ? cache.Erase(id) : false;
}

std::string exportStageToString(int64_t stageId)
{
    PXR_NS::UsdStageRefPtr stage = details::getStage(stageId, true);
    std::string result;
    return stage->ExportToString(&result) ? result : "";
}

int64_t importStageFromString(const std::string& usdString)
{
    static std::atomic<size_t> s_importCount = 0;
    PXR_NS::SdfLayerRefPtr layer =
        PXR_NS::SdfLayer::CreateAnonymous("ImportedStage_" + std::to_string(s_importCount.fetch_add(1)) + ".usda");
    if (!layer || !layer->ImportFromString(usdString))
    {
        return -1;
    }

    PXR_NS::UsdStageRefPtr stage = PXR_NS::UsdStage::Open(layer, PXR_NS::UsdStage::LoadAll);
    if (!stage)
    {
        return -1;
    }

    stage->GetRootLayer()->SetPermissionToEdit(true);
    stage->SetInterpolationType(PXR_NS::UsdInterpolationTypeLinear);

    PXR_NS::UsdStageCache::Id id = PXR_NS::UsdUtilsStageCache::Get().Insert(stage);
    return static_cast<int64_t>(id.ToLongInt());
}

bool isStageValid(int64_t stageId)
{
    return PXR_NS::UsdUtilsStageCache::Get().Contains(PXR_NS::UsdStageCache::Id::FromLongInt(stageId));
}

void* getStagePtr(int64_t stageId)
{
    PXR_NS::UsdStageRefPtr stage = details::getStage(stageId, false);
    return stage ? stage.operator->() : nullptr;
}

void definePrim(int64_t stageId, const std::string& path, const std::string& typeName)
{
    if (!PXR_NS::SdfPath::IsValidPathString(path) || !PXR_NS::SdfPath(path).IsAbsolutePath())
    {
        throw isaacsim::common::exceptions::PrimPathStringError(path);
    }
    PXR_NS::UsdStageRefPtr stage = details::getStage(stageId, true);
    PXR_NS::UsdPrim prim = details::getPrimAtPath(stage, path);
    if (prim.IsValid())
    {
        if (prim.GetTypeName() != PXR_NS::TfToken(typeName))
        {
            throw std::runtime_error("A prim already exists at the given path (" + path + ") with a different type (" +
                                     prim.GetTypeName().GetString() + ")");
        }
        return;
    }
    stage->DefinePrim(PXR_NS::SdfPath(path), PXR_NS::TfToken(typeName));
}

std::tuple<bool, std::string> movePrim(int64_t stageId, const std::string& targetPath, const std::string& destinationPath)
{
    if (!PXR_NS::SdfPath::IsValidPathString(destinationPath))
    {
        throw isaacsim::common::exceptions::PrimPathStringError(destinationPath);
    }
    PXR_NS::UsdStageRefPtr stage = details::getStage(stageId, true);
    PXR_NS::SdfPath target(targetPath);
    PXR_NS::UsdPrim targetPrim = details::getPrimAtPath(stage, targetPath, true);
    // Determine the final destination path
    PXR_NS::SdfPath destination(destinationPath);
    PXR_NS::SdfPath finalDestination;
    if (stage->GetPrimAtPath(destination).IsValid())
    {
        // Destination exists (or is the pseudo-root): move target into it as a child, keeping its name
        finalDestination = destination.AppendChild(target.GetNameToken());
    }
    else if (!stage->GetPrimAtPath(destination.GetParentPath()).IsValid())
    {
        throw std::invalid_argument("Destination path '" + destinationPath + "' has unexisting parent '" +
                                    destination.GetParentPath().GetString() + "'");
    }
    else
    {
        finalDestination = destination;
    }

    // Apply the move via a namespace edit on the root layer
    PXR_NS::SdfBatchNamespaceEdit batchEdit;
    batchEdit.Add(PXR_NS::SdfNamespaceEdit(target, finalDestination));
    bool result = stage->GetRootLayer()->Apply(batchEdit);
    return { result, result ? finalDestination.GetString() : "" };
}

bool removePrim(int64_t stageId, const std::string& path)
{
    PXR_NS::UsdStageRefPtr stage = details::getStage(stageId, true);
    details::getPrimAtPath(stage, path, true);
    return stage->RemovePrim(PXR_NS::SdfPath(path));
}

std::tuple<float, float> getStageUnits(int64_t stageId)
{
    PXR_NS::UsdStageRefPtr stage = details::getStage(stageId, true);
    return { static_cast<float>(PXR_NS::UsdGeomGetStageMetersPerUnit(stage)),
             static_cast<float>(PXR_NS::UsdPhysicsGetStageKilogramsPerUnit(stage)) };
}

void traversePrim(int64_t stageId, const std::string& path, std::function<bool(const std::string&)> callback)
{
    for (const auto& prim : PXR_NS::UsdPrimRange(details::getPrimAtPath(stageId, path, true)))
    {
        if (!callback(prim.GetPath().GetString()))
        {
            break;
        }
    }
}

void setStageUnits(int64_t stageId, std::optional<float> metersPerUnit, std::optional<float> kilogramsPerUnit)
{
    PXR_NS::UsdStageRefPtr stage = details::getStage(stageId, true);
    if (metersPerUnit.has_value())
    {
        PXR_NS::UsdGeomSetStageMetersPerUnit(stage, static_cast<double>(*metersPerUnit));
    }
    if (kilogramsPerUnit.has_value())
    {
        PXR_NS::UsdPhysicsSetStageKilogramsPerUnit(stage, static_cast<double>(*kilogramsPerUnit));
    }
}

std::string getStageUpAxis(int64_t stageId)
{
    PXR_NS::UsdStageRefPtr stage = details::getStage(stageId, true);
    return PXR_NS::UsdGeomGetStageUpAxis(stage).GetString();
}

void setStageUpAxis(int64_t stageId, const std::string& upAxis)
{
    std::string upAxisUpper = upAxis;
    std::transform(upAxisUpper.begin(), upAxisUpper.end(), upAxisUpper.begin(),
                   [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
    if (upAxisUpper != "Y" && upAxisUpper != "Z")
    {
        throw std::invalid_argument("Invalid up axis: '" + upAxis + "'. Valid values are 'Y' and 'Z'");
    }
    PXR_NS::UsdStageRefPtr stage = details::getStage(stageId, true);
    PXR_NS::UsdGeomSetStageUpAxis(stage, PXR_NS::TfToken(upAxisUpper));
}

std::tuple<float, float, float> getStageTimeCode(int64_t stageId)
{
    PXR_NS::UsdStageRefPtr stage = details::getStage(stageId, true);
    return { static_cast<float>(stage->GetStartTimeCode()), static_cast<float>(stage->GetEndTimeCode()),
             static_cast<float>(stage->GetTimeCodesPerSecond()) };
}

void setStageTimeCode(int64_t stageId,
                      std::optional<float> startTimeCode,
                      std::optional<float> endTimeCode,
                      std::optional<float> timeCodesPerSecond)
{
    PXR_NS::UsdStageRefPtr stage = details::getStage(stageId, true);
    if (startTimeCode.has_value())
    {
        stage->SetStartTimeCode(static_cast<double>(*startTimeCode));
    }
    if (endTimeCode.has_value())
    {
        stage->SetEndTimeCode(static_cast<double>(*endTimeCode));
    }
    if (timeCodesPerSecond.has_value())
    {
        stage->SetTimeCodesPerSecond(static_cast<double>(*timeCodesPerSecond));
    }
}

void addReferenceToStage(int64_t stageId, const std::string& path, const std::string& usdPath, const std::string& typeName)
{
    if (!PXR_NS::SdfPath::IsValidPathString(path))
    {
        throw isaacsim::common::exceptions::PrimPathStringError(path);
    }
    PXR_NS::SdfLayerRefPtr sdfLayer = PXR_NS::SdfLayer::FindOrOpen(usdPath);
    if (!sdfLayer)
    {
        throw std::runtime_error("Unable to find/open the Sdf layer. The USD file (" + usdPath +
                                 ") might not exist or is not a valid USD file");
    }
    PXR_NS::UsdStageRefPtr stage = details::getStage(stageId, true);
    PXR_NS::UsdPrim prim = stage->GetPrimAtPath(PXR_NS::SdfPath(path));
    if (!prim.IsValid())
    {
        prim = stage->DefinePrim(PXR_NS::SdfPath(path), PXR_NS::TfToken(typeName));
    }
    if (!prim.GetReferences().AddReference(PXR_NS::SdfReference(usdPath)))
    {
        throw std::runtime_error("Unable to add reference to the USD file (" + usdPath + ")");
    }
}

std::string generateStageRepresentation(int64_t stageId, const std::string& mode)
{
    PXR_NS::UsdStageRefPtr stage = details::getStage(stageId, true);
    std::ostringstream oss;
    bool first = true;

    if (mode == "list")
    {
        for (const auto& prim : stage->Traverse())
        {
            if (!first)
                oss << "\n";
            oss << prim.GetPath().GetString() << " (" << prim.GetTypeName().GetString() << ")";
            first = false;
        }
    }
    else if (mode == "tree")
    {
        std::function<void(const PXR_NS::UsdPrim&, int)> generateTree;
        generateTree = [&](const PXR_NS::UsdPrim& prim, int indent)
        {
            std::string prefix;
            if (indent > 0)
            {
                for (int i = 0; i < indent - 1; ++i)
                    prefix += "│  ";
                prefix += "├─ ";
            }
            const std::string name = prim.GetPath().IsAbsoluteRootPath() ? "/" : prim.GetPath().GetName();
            if (!first)
                oss << "\n";
            oss << prefix << name << " (" << prim.GetTypeName().GetString() << ")";
            first = false;
            for (const auto& child : prim.GetChildren())
                generateTree(child, indent + 1);
        };
        generateTree(stage->GetPseudoRoot(), 0);
    }
    else
    {
        throw std::invalid_argument("Invalid mode: '" + mode + "'. Valid modes are: 'list', 'tree'");
    }

    return oss.str();
}

bool isValidPathString(const std::string& path)
{
    return PXR_NS::SdfPath::IsValidPathString(path);
}

bool isPrimValid(int64_t stageId, const std::string& path)
{
    auto stage = details::getStage(stageId, true);
    if (!PXR_NS::SdfPath::IsValidPathString(path))
    {
        throw isaacsim::common::exceptions::PrimPathStringError(path);
    }
    return details::getPrimAtPath(stage, path).IsValid();
}

std::string getName(int64_t stageId, const std::string& path)
{
    PXR_NS::UsdPrim prim = details::getPrimAtPath(stageId, path, true);
    return prim.GetName().GetString();
}

std::string getTypeName(int64_t stageId, const std::string& path)
{
    PXR_NS::UsdPrim prim = details::getPrimAtPath(stageId, path, true);
    return prim.GetTypeName().GetString();
}

std::string getParent(int64_t stageId, const std::string& path)
{
    PXR_NS::UsdPrim prim = details::getPrimAtPath(stageId, path, true);
    return prim.GetParent().GetPath().GetString();
}

std::vector<std::string> getChildren(int64_t stageId, const std::string& path)
{
    std::vector<std::string> result;
    PXR_NS::UsdPrim prim = details::getPrimAtPath(stageId, path, true);
    for (const auto& child : prim.GetChildren())
    {
        result.push_back(child.GetPath().GetString());
    }
    return result;
}

bool isA(int64_t stageId, const std::string& path, const std::string& schemaType)
{
    PXR_NS::UsdPrim prim = details::getPrimAtPath(stageId, path, true);
    PXR_NS::TfType type = PXR_NS::UsdSchemaRegistry::GetTypeFromSchemaTypeName(PXR_NS::TfToken(schemaType));
    if (type.IsUnknown())
    {
        throw std::invalid_argument("Unknown schema type: '" + schemaType + "'");
    }
    return prim.IsA(type);
}

bool hasApi(int64_t stageId,
            const std::string& path,
            const std::string& schemaType,
            const std::optional<std::string>& instanceName)
{
    PXR_NS::UsdPrim prim = details::getPrimAtPath(stageId, path, true);
    PXR_NS::TfType type = PXR_NS::UsdSchemaRegistry::GetTypeFromSchemaTypeName(PXR_NS::TfToken(schemaType));
    if (type.IsUnknown())
    {
        throw std::invalid_argument("Unknown schema type: '" + schemaType + "'");
    }
    if (!PXR_NS::UsdSchemaRegistry::IsAppliedAPISchema(type))
    {
        throw std::invalid_argument("Schema type '" + schemaType + "' is not an applied API schema");
    }
    return instanceName.has_value() ? prim.HasAPI(type, PXR_NS::TfToken(*instanceName)) : prim.HasAPI(type);
}

bool applyApi(int64_t stageId,
              const std::string& path,
              const std::string& schemaType,
              const std::optional<std::string>& instanceName)
{
    PXR_NS::UsdPrim prim = details::getPrimAtPath(stageId, path, true);
    PXR_NS::TfType type = PXR_NS::UsdSchemaRegistry::GetTypeFromSchemaTypeName(PXR_NS::TfToken(schemaType));
    if (type.IsUnknown())
    {
        throw std::invalid_argument("Unknown schema type: '" + schemaType + "'");
    }
    if (!PXR_NS::UsdSchemaRegistry::IsAppliedAPISchema(type))
    {
        throw std::invalid_argument("Schema type '" + schemaType + "' is not an applied API schema");
    }
    return instanceName.has_value() ? prim.ApplyAPI(type, PXR_NS::TfToken(*instanceName)) : prim.ApplyAPI(type);
}

bool removeApi(int64_t stageId,
               const std::string& path,
               const std::string& schemaType,
               const std::optional<std::string>& instanceName)
{
    PXR_NS::UsdPrim prim = details::getPrimAtPath(stageId, path, true);
    PXR_NS::TfType type = PXR_NS::UsdSchemaRegistry::GetTypeFromSchemaTypeName(PXR_NS::TfToken(schemaType));
    if (type.IsUnknown())
    {
        throw std::invalid_argument("Unknown schema type: '" + schemaType + "'");
    }
    if (!PXR_NS::UsdSchemaRegistry::IsAppliedAPISchema(type))
    {
        throw std::invalid_argument("Schema type '" + schemaType + "' is not an applied API schema");
    }
    return instanceName.has_value() ? prim.RemoveAPI(type, PXR_NS::TfToken(*instanceName)) : prim.RemoveAPI(type);
}

std::vector<std::string> getAppliedSchemas(int64_t stageId, const std::string& path)
{
    std::vector<std::string> result;
    PXR_NS::UsdPrim prim = details::getPrimAtPath(stageId, path, true);
    for (const auto& token : prim.GetAppliedSchemas())
    {
        result.push_back(token.GetString());
    }
    return result;
}

std::unordered_map<std::string, std::vector<std::string>> getVariants(int64_t stageId,
                                                                      const std::string& path,
                                                                      bool selection)
{
    std::unordered_map<std::string, std::vector<std::string>> result;
    PXR_NS::UsdPrim prim = details::getPrimAtPath(stageId, path, true);
    PXR_NS::UsdVariantSets variantSets = prim.GetVariantSets();
    for (const auto& setName : variantSets.GetNames())
    {
        PXR_NS::UsdVariantSet variantSet = variantSets.GetVariantSet(setName);
        if (selection)
        {
            result[setName] = { variantSet.GetVariantSelection() };
        }
        else
        {
            result[setName] = variantSet.GetVariantNames();
        }
    }
    return result;
}

void setVariants(int64_t stageId,
                 const std::string& path,
                 const std::unordered_map<std::string, std::vector<std::string>>& variants)
{
    PXR_NS::UsdPrim prim = details::getPrimAtPath(stageId, path, true);
    PXR_NS::UsdVariantSets variantSets = prim.GetVariantSets();
    const std::vector<std::string> availableSets = variantSets.GetNames();
    for (const auto& [setName, selections] : variants)
    {
        if (std::find(availableSets.begin(), availableSets.end(), setName) == availableSets.end())
        {
            throw std::invalid_argument("Invalid variant set: '" + setName + "'");
        }
        const std::string& selection = selections.empty() ? "" : selections[0];
        const std::vector<std::string> availableSelections = variantSets.GetVariantSet(setName).GetVariantNames();
        if (std::find(availableSelections.begin(), availableSelections.end(), selection) == availableSelections.end())
        {
            throw std::invalid_argument("Invalid variant selection (variant set: '" + setName + "'): '" + selection + "'");
        }
        variantSets.GetVariantSet(setName).SetVariantSelection(selection);
    }
}

bool createPrimAttribute(int64_t stageId,
                         const std::string& path,
                         const std::string& attributeName,
                         const std::string& typeName)
{
    PXR_NS::UsdPrim prim = details::getPrimAtPath(stageId, path, true);
    PXR_NS::SdfValueTypeName valueTypeName = PXR_NS::SdfSchema::GetInstance().FindType(typeName);
    if (!valueTypeName)
    {
        throw std::invalid_argument("Unknown attribute type name: '" + typeName + "'");
    }
    if (PXR_NS::UsdAttribute attribute = prim.GetAttribute(PXR_NS::TfToken(attributeName)))
    {
        if (attribute.GetTypeName() != valueTypeName)
        {
            throw std::invalid_argument("Prim at path '" + path + "' already has attribute '" + attributeName +
                                        "' with type '" + attribute.GetTypeName().GetAsToken().GetString() + "'");
        }
        return true;
    }
    return prim.CreateAttribute(PXR_NS::TfToken(attributeName), valueTypeName).IsValid();
}

bool removePrimAttribute(int64_t stageId, const std::string& path, const std::string& attributeName)
{
    PXR_NS::UsdPrim prim = details::getPrimAtPath(stageId, path, true);
    if (!prim.GetAttribute(PXR_NS::TfToken(attributeName)))
    {
        throw isaacsim::common::exceptions::AttributeNameError(attributeName, getPrimAttributeNames(stageId, path));
    }
    return prim.RemoveProperty(PXR_NS::TfToken(attributeName));
}

std::string getPrimAttributeTypeName(int64_t stageId, const std::string& path, const std::string& attributeName)
{
    PXR_NS::UsdPrim prim = details::getPrimAtPath(stageId, path, true);
    if (PXR_NS::UsdAttribute attribute = prim.GetAttribute(PXR_NS::TfToken(attributeName)))
    {
        return attribute.GetTypeName().GetAsToken().GetString();
    }
    throw isaacsim::common::exceptions::AttributeNameError(attributeName, getPrimAttributeNames(stageId, path));
}

std::vector<std::string> getPrimAttributeNames(int64_t stageId, const std::string& path)
{
    std::vector<std::string> result;
    PXR_NS::UsdPrim prim = details::getPrimAtPath(stageId, path, true);
    for (const auto& attribute : prim.GetAttributes())
    {
        result.push_back(attribute.GetName().GetString());
    }
    return result;
}

AttributeValues getPrimAttributeValues(int64_t stageId,
                                       const std::vector<std::string>& paths,
                                       const std::string& attributeName)
{
    if (paths.empty())
    {
        throw std::invalid_argument("The `paths` parameter must not be empty");
    }
    // if attribute type is asset, string, or token the details::getAttributeValue return std::string
    // if attribute type is asset[], string[], or token[] the details::getAttributeValue return std::vector<std::string>
    // otherwise the details::getAttributeValue return Array
    std::vector<details::AttributeValue> values;
    values.reserve(paths.size());
    for (const auto& path : paths)
    {
        PXR_NS::UsdPrim prim = details::getPrimAtPath(stageId, path, true);
        if (PXR_NS::UsdAttribute attribute = prim.GetAttribute(PXR_NS::TfToken(attributeName)))
        {
            values.push_back(details::getAttributeValue(attribute));
        }
        else
        {
            throw isaacsim::common::exceptions::AttributeNameError(attributeName, getPrimAttributeNames(stageId, path));
        }
    }

    return std::visit(
        [&values](const auto& first) -> AttributeValues
        {
            using T = std::decay_t<decltype(first)>;
            if constexpr (std::is_same_v<T, std::string> || std::is_same_v<T, std::vector<std::string>>)
            {
                std::vector<T> result;
                result.reserve(values.size());
                for (const auto& value : values)
                {
                    result.push_back(std::get<T>(value));
                }
                return result;
            }
            else
            {
                std::vector<int64_t> shape = first.shape().shape();
                if (shape.empty())
                {
                    shape = { 1 };
                }
                shape.insert(shape.begin(), static_cast<int64_t>(values.size()));

                T result = first.broadcastTo(isaacsim::common::array::Shape(shape), true);
                for (size_t i = 0; i < values.size(); ++i)
                {
                    result.at(static_cast<int64_t>(i)).set(std::get<T>(values[i]));
                }
                return result;
            }
        },
        values.front());
}


std::vector<bool> setPrimAttributeValues(int64_t stageId,
                                         const std::vector<std::string>& paths,
                                         const std::string& attributeName,
                                         const AttributeValues& values)
{
    if (paths.empty())
    {
        throw std::invalid_argument("The `paths` parameter must not be empty");
    }
    std::vector<bool> result;
    for (size_t i = 0; i < paths.size(); ++i)
    {
        PXR_NS::UsdPrim prim = details::getPrimAtPath(stageId, paths[i], true);
        if (PXR_NS::UsdAttribute attribute = prim.GetAttribute(PXR_NS::TfToken(attributeName)))
        {
            result.push_back(std::visit(
                [&attribute, i](auto value) { return details::setAttributeValue(attribute, value.at(i)); }, values));
        }
        else
        {
            throw isaacsim::common::exceptions::AttributeNameError(
                attributeName, getPrimAttributeNames(stageId, paths[i]));
        }
    }
    return result;
}

std::vector<std::string> findMatchingPrimPaths(int64_t stageId, const std::string& path, bool traverse)
{
    std::vector<std::string> result;
    PXR_NS::UsdStageRefPtr stage = details::getStage(stageId, true);
    if (!stage)
    {
        return result;
    }

    // Check for a valid prim path first to avoid unnecessary regex search.
    if (PXR_NS::SdfPath::IsValidPathString(path))
    {
        PXR_NS::UsdPrim prim = details::getPrimAtPath(stage, path);
        if (prim.IsValid())
        {
            if (traverse)
            {
                for (const auto& p : PXR_NS::UsdPrimRange(prim))
                {
                    result.push_back(p.GetPath().GetString());
                }
            }
            else
            {
                result.push_back(path);
            }
            return result;
        }
    }

    // Regex search.
    if (traverse)
    {
        std::regex pattern;
        try
        {
            pattern = std::regex(path);
        }
        catch (const std::regex_error&)
        {
            return result;
        }
        for (const auto& prim : stage->Traverse())
        {
            std::string primPath = prim.GetPath().GetString();
            if (std::regex_search(primPath, pattern, std::regex_constants::match_continuous))
            {
                result.push_back(primPath);
            }
        }
    }
    else
    {
        // Segment-wise search: split the path by "/" and match level by level
        auto start = path.find_first_not_of('/');
        auto end = path.find_last_not_of('/');
        std::string trimmed = (start == std::string::npos) ? "" : path.substr(start, end - start + 1);

        std::vector<std::regex> patterns;
        std::istringstream ss(trimmed);
        std::string token;
        try
        {
            while (std::getline(ss, token, '/'))
            {
                patterns.emplace_back("^" + token + "$");
            }
        }
        catch (const std::regex_error&)
        {
            return result;
        }

        std::vector<std::string> roots = { "/" };
        std::vector<std::string> matches;

        for (size_t i = 0; i < patterns.size(); ++i)
        {
            for (const auto& root : roots)
            {
                for (const auto& child : details::getPrimAtPath(stage, root).GetChildren())
                {
                    if (std::regex_match(child.GetName().GetString(), patterns[i]))
                    {
                        matches.push_back(child.GetPath().GetString());
                    }
                }
            }
            if (i < patterns.size() - 1)
            {
                roots = std::move(matches);
                matches.clear();
            }
        }
        result = std::move(matches);
    }

    return result;
}

void resetXformOpProperties(int64_t stageId, const std::string& path)
{
    PXR_NS::UsdGeomXformable xformable = details::getXformableAtPath(stageId, path, true, false);
    PXR_NS::UsdPrim prim = xformable.GetPrim();

    // TODO: Capture world pose before touching ops so we can restore it after reordering.

    // Remove non-standard rotation and transform ops.
    static const PXR_NS::TfToken s_kOpsToRemove[] = {
        PXR_NS::TfToken("xformOp:rotateX"),   PXR_NS::TfToken("xformOp:rotateXZY"),
        PXR_NS::TfToken("xformOp:rotateY"),   PXR_NS::TfToken("xformOp:rotateYXZ"),
        PXR_NS::TfToken("xformOp:rotateYZX"), PXR_NS::TfToken("xformOp:rotateZ"),
        PXR_NS::TfToken("xformOp:rotateZYX"), PXR_NS::TfToken("xformOp:rotateZXY"),
        PXR_NS::TfToken("xformOp:rotateXYZ"), PXR_NS::TfToken("xformOp:transform"),
    };
    for (const auto& token : s_kOpsToRemove)
    {
        if (prim.GetAttribute(token))
        {
            prim.RemoveProperty(token);
        }
    }

    // Collect stray `:unitsResolve` properties (skip :scale:unitsResolve - baked below).
    static constexpr std::string_view s_kUnitsResolveSuffix = ":unitsResolve";
    std::vector<PXR_NS::TfToken> unitsResolveToRemove;
    for (const auto& attribute : prim.GetAttributes())
    {
        const PXR_NS::TfToken token = attribute.GetName();
        const std::string_view name = token.GetString();
        if (name.find(s_kUnitsResolveSuffix) != std::string_view::npos && name.find(":scale:") == std::string_view::npos)
        {
            unitsResolveToRemove.push_back(token);
        }
    }
    for (const auto& token : unitsResolveToRemove)
    {
        prim.RemoveProperty(token);
    }

    // Ensure xformOp:translate (double precision).
    PXR_NS::UsdGeomXformOp xformOpTranslate;
    if (PXR_NS::UsdAttribute attribute = prim.GetAttribute(PXR_NS::TfToken("xformOp:translate")))
    {
        xformOpTranslate = PXR_NS::UsdGeomXformOp(attribute);
    }
    else
    {
        xformOpTranslate =
            xformable.AddXformOp(PXR_NS::UsdGeomXformOp::TypeTranslate, PXR_NS::UsdGeomXformOp::PrecisionDouble);
    }

    // Ensure xformOp:orient (double precision).
    PXR_NS::UsdGeomXformOp xformOpOrient;
    if (PXR_NS::UsdAttribute attribute = prim.GetAttribute(PXR_NS::TfToken("xformOp:orient")))
    {
        xformOpOrient = PXR_NS::UsdGeomXformOp(attribute);
    }
    else
    {
        xformOpOrient = xformable.AddXformOp(PXR_NS::UsdGeomXformOp::TypeOrient, PXR_NS::UsdGeomXformOp::PrecisionDouble);
    }

    // Ensure xformOp:scale (double precision), baking :unitsResolve into the scale value.
    PXR_NS::UsdGeomXformOp xformOpScale;
    if (PXR_NS::UsdAttribute scaleAttr = prim.GetAttribute(PXR_NS::TfToken("xformOp:scale")))
    {
        xformOpScale = PXR_NS::UsdGeomXformOp(scaleAttr);
        if (PXR_NS::UsdAttribute unitsResolveAttr = prim.GetAttribute(PXR_NS::TfToken("xformOp:scale:unitsResolve")))
        {
            PXR_NS::GfVec3d scale, unitsResolve;
            scaleAttr.Get(&scale);
            unitsResolveAttr.Get(&unitsResolve);
            scaleAttr.Set(
                PXR_NS::GfVec3d(scale[0] * unitsResolve[0], scale[1] * unitsResolve[1], scale[2] * unitsResolve[2]));
            prim.RemoveProperty(PXR_NS::TfToken("xformOp:scale:unitsResolve"));
        }
    }
    else
    {
        xformOpScale = xformable.AddXformOp(PXR_NS::UsdGeomXformOp::TypeScale, PXR_NS::UsdGeomXformOp::PrecisionDouble);
        xformOpScale.Set(PXR_NS::GfVec3d(1.0, 1.0, 1.0));
    }

    // Apply canonical op order.
    xformable.ClearXformOpOrder();
    xformable.SetXformOpOrder({ xformOpTranslate, xformOpOrient, xformOpScale });
}

array::Array getXformLocalScales(int64_t stageId, const std::vector<std::string>& paths)
{
    std::vector<std::vector<double>> scales;
    scales.reserve(paths.size());
    for (const auto& path : paths)
    {
        const PXR_NS::GfVec3d scale =
            details::getXformLocalScale(details::getXformableAtPath(stageId, path, true, false));
        scales.push_back({ scale[0], scale[1], scale[2] });
    }
    return array::Array(scales);
}

void setXformLocalScales(int64_t stageId, const std::vector<std::string>& paths, const array::Array& scales)
{
    const int64_t batchSize = static_cast<int64_t>(paths.size());
    if (scales.ndim() != 2 || scales.shape()[0] != batchSize || scales.shape()[1] != 3)
    {
        throw std::invalid_argument("The `scales` parameter must have shape (" + std::to_string(batchSize) + ", 3)");
    }
    for (size_t i = 0; i < paths.size(); ++i)
    {
        auto scale = const_cast<array::Array&>(scales).at(i).get<std::vector<double>>();
        details::setXformLocalScale(
            details::getXformableAtPath(stageId, paths[i], true, true), PXR_NS::GfVec3d(scale[0], scale[1], scale[2]));
    }
}

std::tuple<array::Array, array::Array> getXformLocalPoses(int64_t stageId, const std::vector<std::string>& paths)
{
    std::vector<std::vector<double>> translations, orientations;
    translations.reserve(paths.size());
    orientations.reserve(paths.size());
    for (const auto& path : paths)
    {
        const auto [translation, orientation] =
            details::getXformLocalPose(details::getXformableAtPath(stageId, path, true, false));
        const auto& imaginary = orientation.GetImaginary();
        translations.push_back({ translation[0], translation[1], translation[2] });
        orientations.push_back({ imaginary[0], imaginary[1], imaginary[2], orientation.GetReal() });
    }
    return { array::Array(translations), array::Array(orientations) };
}

void setXformLocalPoses(int64_t stageId,
                        const std::vector<std::string>& paths,
                        const std::optional<array::Array>& translations,
                        const std::optional<array::Array>& orientations)
{
    if (!translations.has_value() && !orientations.has_value())
    {
        throw std::invalid_argument("At least one of the `translations` or `orientations` parameters must be provided");
    }
    const int64_t batchSize = static_cast<int64_t>(paths.size());
    if (translations.has_value() &&
        (translations->ndim() != 2 || translations->shape()[0] != batchSize || translations->shape()[1] != 3))
    {
        throw std::invalid_argument("The `translations` parameter must have shape (" + std::to_string(batchSize) + ", 3)");
    }
    if (orientations.has_value() &&
        (orientations->ndim() != 2 || orientations->shape()[0] != batchSize || orientations->shape()[1] != 4))
    {
        throw std::invalid_argument("The `orientations` parameter must have shape (" + std::to_string(batchSize) + ", 4)");
    }

    std::optional<PXR_NS::GfVec3d> translation;
    std::optional<PXR_NS::GfQuatd> orientation;
    for (size_t i = 0; i < paths.size(); ++i)
    {
        translation = std::nullopt;
        orientation = std::nullopt;
        if (translations.has_value())
        {
            auto translationValue = const_cast<array::Array&>(*translations).at(i).get<std::vector<double>>();
            translation = PXR_NS::GfVec3d(translationValue[0], translationValue[1], translationValue[2]);
        }
        if (orientations.has_value())
        {
            auto orientationValue = const_cast<array::Array&>(*orientations).at(i).get<std::vector<double>>();
            orientation = PXR_NS::GfQuatd(
                orientationValue[3], PXR_NS::GfVec3d(orientationValue[0], orientationValue[1], orientationValue[2]));
        }
        details::setXformLocalPose(details::getXformableAtPath(stageId, paths[i], true, true), translation, orientation);
    }
}

std::tuple<array::Array, array::Array> getXformWorldPoses(int64_t stageId, const std::vector<std::string>& paths)
{
    std::vector<std::vector<double>> positions, orientations;
    positions.reserve(paths.size());
    orientations.reserve(paths.size());
    for (const auto& path : paths)
    {
        const auto [position, orientation] =
            details::getXformWorldPose(details::getXformableAtPath(stageId, path, true, false));
        const auto& imaginary = orientation.GetImaginary();
        positions.push_back({ position[0], position[1], position[2] });
        orientations.push_back({ imaginary[0], imaginary[1], imaginary[2], orientation.GetReal() });
    }
    return { array::Array(positions), array::Array(orientations) };
}

void setXformWorldPoses(int64_t stageId,
                        const std::vector<std::string>& paths,
                        const std::optional<array::Array>& positions,
                        const std::optional<array::Array>& orientations)
{
    if (!positions.has_value() && !orientations.has_value())
    {
        throw std::invalid_argument("At least one of the `positions` or `orientations` parameters must be provided");
    }
    const int64_t batchSize = static_cast<int64_t>(paths.size());
    if (positions.has_value() &&
        (positions->ndim() != 2 || positions->shape()[0] != batchSize || positions->shape()[1] != 3))
    {
        throw std::invalid_argument("The `positions` parameter must have shape (" + std::to_string(batchSize) + ", 3)");
    }
    if (orientations.has_value() &&
        (orientations->ndim() != 2 || orientations->shape()[0] != batchSize || orientations->shape()[1] != 4))
    {
        throw std::invalid_argument("The `orientations` parameter must have shape (" + std::to_string(batchSize) + ", 4)");
    }

    std::optional<PXR_NS::GfVec3d> position;
    std::optional<PXR_NS::GfQuatd> orientation;
    for (size_t i = 0; i < paths.size(); ++i)
    {
        position = std::nullopt;
        orientation = std::nullopt;
        if (positions.has_value())
        {
            auto positionValue = const_cast<array::Array&>(*positions).at(i).get<std::vector<double>>();
            position = PXR_NS::GfVec3d(positionValue[0], positionValue[1], positionValue[2]);
        }
        if (orientations.has_value())
        {
            auto orientationValue = const_cast<array::Array&>(*orientations).at(i).get<std::vector<double>>();
            orientation = PXR_NS::GfQuatd(
                orientationValue[3], PXR_NS::GfVec3d(orientationValue[0], orientationValue[1], orientationValue[2]));
        }
        details::setXformWorldPose(details::getXformableAtPath(stageId, paths[i], true, true), position, orientation);
    }
}

} // namespace openusd
} // namespace usd
} // namespace foundation
} // namespace isaacsim
