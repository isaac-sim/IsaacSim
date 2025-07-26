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
#include <pch/UsdPCH.h>
// clang-format on

#include "LidarConfigHelper.h"
#include "isaacsim/core/includes/UsdUtilities.h"

#include <carb/InterfaceUtils.h>
#include <carb/extras/Path.h>
#include <carb/settings/ISettings.h>
#include <carb/settings/SettingsUtils.h>
#include <carb/tokens/TokensUtils.h>

#include <omni/math/linalg/matrix.h>
#include <omni/math/linalg/quat.h>
#include <omni/math/linalg/vec.h>
#include <sys/types.h>

#include <math.h>

namespace isaacsim
{
namespace sensors
{
namespace rtx
{

void getTransformFromSensorPose(const omni::sensors::FrameAtTime& inPose, omni::math::linalg::matrix4d& matrixOutput)
{
    // Assign the position and orientation from the frame at time
    // inPose.orientation is ordered i,j,k,w, but the quatd constructor is ordered w,i,j,k
    omni::math::linalg::vec3d posM{ inPose.posM[0], inPose.posM[1], inPose.posM[2] };
    omni::math::linalg::quatd pose{ inPose.orientation[3], inPose.orientation[0], inPose.orientation[1],
                                    inPose.orientation[2] };
    matrixOutput.SetIdentity();
    matrixOutput.SetRotateOnly(pose);
    matrixOutput.SetTranslateOnly(posM);
}

float LidarConfigHelper::getFarRange() const
{

    return this->farRangeM;
}

float LidarConfigHelper::getNearRange() const
{
    return this->nearRangeM;
}

uint32_t LidarConfigHelper::getNumChannels() const
{
    return this->numberOfEmitters;
}

uint32_t LidarConfigHelper::getNumEchos() const
{
    return this->maxReturns;
}


uint32_t LidarConfigHelper::getReturnsPerScan() const
{
    return this->getNumEchos() * this->getNumChannels() * this->getTicksPerScan();
}

uint32_t LidarConfigHelper::getTicksPerScan() const
{
    return this->scanType == LidarScanType::kSolidState ? 1 : this->reportRateBaseHz / this->scanRateBaseHz;
}
// if inConfig and config are different, then the profile needs to be updated, and if there is an error, then
// return true only if config was updated.
bool LidarConfigHelper::updateLidarConfig(const char* renderProductPath)
{
    std::string curConfig = "";
    pxr::UsdAttribute configAttr =
        isaacsim::core::includes::getCameraAttributeFromRenderProduct("sensorModelConfig", renderProductPath);
    if (!configAttr.IsValid())
    {
        return false;
    }
    isaacsim::core::includes::safeGetAttribute(configAttr, curConfig);

    if (this->config == curConfig)
    {
        return false;
    }

    this->config = curConfig;

    if (this->config.empty())
    {
        this->scanType = LidarScanType::kUnknown;
        return false;
    }

    // omni::sensors::IProfileReaderPtr profileReader =
    //     carb::getFramework()->acquireInterface<omni::sensors::IProfileReaderFactory>()->createInstance();
    const auto json = this->getProfileJsonAtPaths(curConfig.c_str());
    this->init(json.c_str());
    // const auto dataSize = profileReader->dataSizeProfile();
    // this->profileBuffer.resize(dataSize);
    // bool updated = profileReader->update((void*)this->profileBuffer.data());
    // profileReader->release();
    // if (updated)
    // {
    //     this->profile = reinterpret_cast<LidarProfile*>(profileBuffer.data());
    //     this->scanType = this->profile->scanType;
    // }
    // else
    // {
    //     this->scanType = LidarScanType::kUnknown;
    // }

    auto& profile = (*m_doc)["profile"];
    const std::string scanType(profile["scanType"].GetString());
    if (scanType == "rotary" || scanType == "ROTARY" || scanType == "Rotary")
    {
        this->scanType = LidarScanType::kRotary;
    }
    else if (scanType == "solidState" || scanType == "SOLID_STATE" || scanType == "SolidState")
    {
        this->scanType = LidarScanType::kSolidState;
    }
    else
    {
        this->scanType = LidarScanType::kUnknown;
    }
    this->numberOfEmitters = 128;
    if (profile.HasMember("numberOfEmitters"))
    {
        numberOfEmitters = profile["numberOfEmitters"].GetInt();
    }
    this->nearRangeM = profile["nearRangeM"].GetFloat();
    this->farRangeM = profile["farRangeM"].GetFloat();
    this->azimuthStartDeg = profile["emitterStates"][0]["azimuthDeg"][0].GetFloat();
    this->azimuthEndDeg = profile["emitterStates"][0]["azimuthDeg"][this->numberOfEmitters - 1].GetFloat();
    this->is2D = true;

    this->emitterStateCount = profile["emitterStates"].Size();
    this->emitterStates.resize(this->emitterStateCount);
    for (uint32_t i = 0; i < this->emitterStateCount; i++)
    {
        this->emitterStates[i].azimuthDeg.resize(this->numberOfEmitters);
        this->emitterStates[i].elevationDeg.resize(this->numberOfEmitters);
        for (uint32_t j = 0; j < this->numberOfEmitters; j++)
        {
            if (profile["emitterStates"][i].HasMember("azimuthDeg"))
            {
                this->emitterStates[i].azimuthDeg[j] = profile["emitterStates"][i]["azimuthDeg"][j].GetFloat();
            }
            if (profile["emitterStates"][i].HasMember("elevationDeg"))
            {
                this->emitterStates[i].elevationDeg[j] = profile["emitterStates"][i]["elevationDeg"][j].GetFloat();
            }
        }
        for (uint32_t j = 0; j < this->numberOfEmitters; j++)
        {
            if (std::fabs(this->emitterStates[i].elevationDeg[j]) > 1e-3)
            {
                this->is2D = false;
                break;
            }
        }
    }
    const std::string rotationDirection = profile["rotationDirection"].GetString();
    this->reportRateBaseHz = static_cast<uint32_t>(profile["reportRateBaseHz"].GetFloat());
    this->scanRateBaseHz = static_cast<uint32_t>(profile["scanRateBaseHz"].GetFloat());
    this->rotationDirection = rotationDirection == "CW" ? LidarRotationDirection::CW : LidarRotationDirection::CCW;

    this->numLines = static_cast<uint32_t>(profile["numLines"].GetFloat());
    this->numRaysPerLine.resize(this->numLines);
    for (uint32_t i = 0; i < this->numLines; i++)
    {
        this->numRaysPerLine[i] = static_cast<uint32_t>(profile["numRaysPerLine"][i].GetFloat());
    }
    this->maxReturns = static_cast<uint32_t>(profile["maxReturns"].GetFloat());
    return true;
}

void LidarConfigHelper::init(const char* json)
{
    m_doc = std::make_unique<rapidjson::Document>();
    m_doc->Parse(json);
    // m_type = type;

    // if (isValid() && m_doc->HasMember("name"))
    // {
    //     m_name = (*m_doc)["name"].GetString();
    // }

    // m_dataSizeProfile = omni::sensors::nv::lidar::dataSizeOfProfile(m_doc.get(), m_type);
}

// TODO
omni::string LidarConfigHelper::getProfileJsonAtPaths(const char* inSensorProfileName)
{
    std::string sensorProfileName{ inSensorProfileName };
    std::string json{ "" };

    carb::tokens::ITokens* tokens = carb::getCachedInterface<carb::tokens::ITokens>();
    if (!tokens)
    {
        CARB_LOG_ERROR("getProfileJsonAtPaths failed to get carb::tokens::ITokens");
    }
    // Use at least the default path.
    std::vector<std::string> paths;

    // Use this local function to resolve strings and create a json file name.
    const auto get_resolved_path = [](const std::string& path, carb::tokens::ITokens* tokens)
    {
        return carb::extras::Path(carb::tokens::resolveString(tokens, { carb::cpp::unsafe_length, path.c_str() }).c_str())
            .getNormalized()
            .getString();
    };

    // If app.sensors.nv.lidar.profileBaseFolder is not empty get its strings.
    if (auto* iSettings = carb::getCachedInterface<carb::settings::ISettings>())
    {
        const char* settingPath = "/app/sensors/nv/lidar/profileBaseFolder";
        // settingPath = omni::sensors::nv::kLidarBaseFolderSetting;

        const size_t numPaths{ iSettings->getArrayLength(settingPath) };
        for (size_t i = 0; i < numPaths; ++i)
        {
            // Add trailing / for folder path
            std::string directory = carb::settings::getStringAt(iSettings, settingPath, i);
            if (directory.back() != '/')
            {
                directory += "/";
            }
            paths.push_back(directory);
        }
    }

    paths.push_back("${omni.sensors.nv.common}/data/lidar/");

    // Search all known paths for the LiDAR config file.
    for (const std::string& path : paths)
    {
        const std::string profilePath = get_resolved_path(path, tokens) + sensorProfileName + ".json";
        json = LidarConfigHelper::ReadWholeTextFile(profilePath);
        if (!json.empty())
        {
            break;
        }
    }

    // Use invalid profile to indicate wrong profile
    if (json.empty())
    {
        CARB_LOG_ERROR("getProfileJsonAtPaths could not find config file: \"%s\", in extension or in supplied paths:",
                       sensorProfileName.c_str());
        for (const std::string& path : paths)
        {
            CARB_LOG_ERROR("\t%s", (get_resolved_path(path, tokens) + sensorProfileName + ".json").c_str());
        }
        // CARB_LOG_ERROR("getProfileJsonAtPaths is creating a minimal modality specific profile");
        // json = omni::sensors::nv::lidar::invalidProfileJson(m_type);
    }
    omni::string outJson = omni::string(json.c_str());
    return outJson;
}

std::string LidarConfigHelper::ReadWholeTextFile(std::string fullPath)
{
    std::FILE* f = nullptr;

#if defined(_WIN32) || defined(_WIN64)
    ::fopen_s(&f, fullPath.c_str(), "rb");
#else
    f = std::fopen(fullPath.c_str(), "r");
#endif

    if (nullptr == f)
        return {};

    std::fseek(f, 0, SEEK_END);
    size_t size = ::ftell(f);
    std::fseek(f, 0, SEEK_SET);

    std::string str(size + 1, '\0');

    size_t bytesRead = std::fread(&str[0], 1, size, f);
    std::fclose(f);

    if (bytesRead != size)
    {
        CARB_LOG_WARN("ReadWholeTextFile: Expected to read %zu bytes but read %zu bytes from file: %s", size, bytesRead,
                      fullPath.c_str());
        return {};
    }

    return str;
}

}
}
}
