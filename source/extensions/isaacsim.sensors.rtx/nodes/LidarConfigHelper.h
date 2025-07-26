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

#include "rapidjson/document.h"

#include <omni/String.h>
#include <omni/math/linalg/matrix.h>

#include <GenericModelOutputTypes.h>

namespace isaacsim
{
namespace sensors
{
namespace rtx
{

/**
 * @brief Converts a sensor pose to a 4x4 transformation matrix
 * @details
 * This function takes a sensor pose containing position and orientation data and converts it
 * into a 4x4 transformation matrix. The position is extracted from the pose and set as the
 * translation component, while the orientation quaternion is used to set the rotation component.
 *
 * The function handles the quaternion component ordering conversion from the input format
 * (i,j,k,w) to the quaternion constructor format (w,i,j,k).
 *
 * @param[in] inPose The sensor pose containing position and orientation data
 * @param[out] matrixOutput The output 4x4 transformation matrix
 *
 * @note The input orientation quaternion is ordered as (i,j,k,w) but the quatd constructor
 *       expects (w,i,j,k) ordering, so the components are reordered during construction
 * @note The matrixOutput is first set to identity before applying rotation and translation
 */
void getTransformFromSensorPose(const omni::sensors::FrameAtTime& inPose, omni::math::linalg::matrix4d& matrixOutput);

/**
 * @brief Enumeration defining the types of LiDAR scanning patterns
 * @details Specifies whether the LiDAR uses a rotary scanning mechanism or solid-state scanning.
 */
enum class LidarScanType
{
    /** @brief Unknown or unspecified scan type */
    kUnknown,
    /** @brief Rotary scanning mechanism */
    kRotary,
    /** @brief Solid-state scanning mechanism */
    kSolidState
};

/**
 * @class LidarConfigHelper
 * @brief Helper class for managing LiDAR sensor configuration
 * @details Provides utilities for handling LiDAR profiles, scan types, and configuration parameters.
 *          Manages the storage and access of LiDAR-specific settings and profile data.
 *          Supports both rotary and solid-state LiDAR configurations with customizable parameters
 *          such as scan rates, resolution, and emitter configurations.
 */
class LidarConfigHelper
{
public:
    /** @brief Configuration string for the LiDAR sensor */
    std::string config;
    /** @brief Type of LiDAR scanning pattern */
    LidarScanType scanType{ LidarScanType::kUnknown };
    // /** @brief Pointer to the active LiDAR profile */
    // LidarProfile* profile;
    // /** @brief Buffer storing the raw profile data */
    // std::vector<uint8_t> profileBuffer;

    /**
     * @struct EmitterState
     * @brief Configuration state for individual LiDAR emitters
     * @details
     * Contains the angular configuration for LiDAR emitters, defining their pointing directions
     * in spherical coordinates. This structure stores elevation and azimuth angles that determine
     * where each emitter points in 3D space.
     */
    struct EmitterState
    {
        /**
         * @brief Elevation angles for each emitter in degrees
         * @details Array of elevation angles (vertical angles) for LiDAR emitters, measured from horizontal plane.
         *          Negative values point downward, positive values point upward.
         *          Values typically range from -15.0 to 10.0 degrees.
         */
        std::vector<float> elevationDeg{
            -15.0f,  -14.19f, -13.39f, -12.58f, -11.77f, -10.97f, -10.16f, -9.35f,  -8.55f,  -7.74f,  -6.94f,  -6.13f,
            -5.32f,  -4.52f,  -3.71f,  -2.9f,   -2.1f,   -1.29f,  -0.48f,  0.32f,   1.13f,   1.94f,   2.74f,   3.55f,
            4.35f,   5.16f,   5.97f,   6.77f,   7.58f,   8.39f,   9.19f,   10.0f,   -15.0f,  -14.19f, -13.39f, -12.58f,
            -11.77f, -10.97f, -10.16f, -9.35f,  -8.55f,  -7.74f,  -6.94f,  -6.13f,  -5.32f,  -4.52f,  -3.71f,  -2.9f,
            -2.1f,   -1.29f,  -0.48f,  0.32f,   1.13f,   1.94f,   2.74f,   3.55f,   4.35f,   5.16f,   5.97f,   6.77f,
            7.58f,   8.39f,   9.19f,   10.0f,   -15.0f,  -14.19f, -13.39f, -12.58f, -11.77f, -10.97f, -10.16f, -9.35f,
            -8.55f,  -7.74f,  -6.94f,  -6.13f,  -5.32f,  -4.52f,  -3.71f,  -2.9f,   -2.1f,   -1.29f,  -0.48f,  0.32f,
            1.13f,   1.94f,   2.74f,   3.55f,   4.35f,   5.16f,   5.97f,   6.77f,   7.58f,   8.39f,   9.19f,   10.0f,
            -15.0f,  -14.19f, -13.39f, -12.58f, -11.77f, -10.97f, -10.16f, -9.35f,  -8.55f,  -7.74f,  -6.94f,  -6.13f,
            -5.32f,  -4.52f,  -3.71f,  -2.9f,   -2.1f,   -1.29f,  -0.48f,  0.32f,   1.13f,   1.94f,   2.74f,   3.55f,
            4.35f,   5.16f,   5.97f,   6.77f,   7.58f,   8.39f,   9.19f,   10.0f
        };
        /**
         * @brief Azimuth angles for each emitter in degrees
         * @details Array of azimuth angles (horizontal angles) for LiDAR emitters, measured from forward direction.
         *          Negative values point to the left, positive values point to the right.
         *          Values typically range from -3.0 to 3.0 degrees.
         */
        std::vector<float> azimuthDeg{
            -3.0f, -3.0f, -3.0f, -3.0f, -3.0f, -3.0f, -3.0f, -3.0f, -3.0f, -3.0f, -3.0f, -3.0f, -3.0f, -3.0f, -3.0f,
            -3.0f, -3.0f, -3.0f, -3.0f, -3.0f, -3.0f, -3.0f, -3.0f, -3.0f, -3.0f, -3.0f, -3.0f, -3.0f, -3.0f, -3.0f,
            -3.0f, -3.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f,
            -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f,
            -1.0f, -1.0f, -1.0f, -1.0f, 1.0f,  1.0f,  1.0f,  1.0f,  1.0f,  1.0f,  1.0f,  1.0f,  1.0f,  1.0f,  1.0f,
            1.0f,  1.0f,  1.0f,  1.0f,  1.0f,  1.0f,  1.0f,  1.0f,  1.0f,  1.0f,  1.0f,  1.0f,  1.0f,  1.0f,  1.0f,
            1.0f,  1.0f,  1.0f,  1.0f,  1.0f,  1.0f,  3.0f,  3.0f,  3.0f,  3.0f,  3.0f,  3.0f,  3.0f,  3.0f,  3.0f,
            3.0f,  3.0f,  3.0f,  3.0f,  3.0f,  3.0f,  3.0f,  3.0f,  3.0f,  3.0f,  3.0f,  3.0f,  3.0f,  3.0f,  3.0f,
            3.0f,  3.0f,  3.0f,  3.0f,  3.0f,  3.0f,  3.0f,  3.0f
        };
    };
    /**
     * @enum LidarRotationDirection
     * @brief Defines the rotation direction of the LiDAR scanner
     * @details Specifies whether the LiDAR sensor rotates clockwise or counterclockwise during scanning.
     *          This affects the order in which points are collected during a scan.
     */
    enum class LidarRotationDirection
    {
        /** @brief Clockwise rotation direction */
        CW,
        /** @brief Counterclockwise rotation direction */
        CCW
    };
    /** @brief Minimum range of the LiDAR sensor in meters */
    float nearRangeM{ 0.3f };
    /** @brief Maximum range of the LiDAR sensor in meters */
    float farRangeM{ 200.0f };
    /** @brief Start azimuth angle of the LiDAR sensor in degrees */
    float azimuthStartDeg{ 0.0f };
    /** @brief End azimuth angle of the LiDAR sensor in degrees */
    float azimuthEndDeg{ 360.0f };
    /** @brief Horizontal resolution of the LiDAR sensor in degrees */
    float horizontalResolutionDeg{ 0.1f };
    /** @brief Report rate base frequency of the LiDAR sensor in Hz */
    uint32_t reportRateBaseHz{ 36000 };
    /** @brief Rotation rate of the LiDAR sensor in Hz */
    uint32_t scanRateBaseHz{ 10 };
    /** @brief Number of emitters in the LiDAR sensor */
    uint32_t numberOfEmitters{ 128 };
    /**
     * @brief Number of configured emitter states
     * @details Count of valid entries in the emitterStates vector that define emitter configurations.
     *          This may be less than the total number of emitters if some are not configured.
     */
    uint32_t emitterStateCount{ 0 };
    /**
     * @brief Maximum number of returns per laser beam
     * @details Specifies the maximum number of echo returns that can be detected from a single laser pulse.
     *          Higher values allow for better detection of partially transparent or reflective surfaces.
     */
    uint32_t maxReturns{ 2 };
    /**
     * @brief Number of rays per scan line
     * @details Vector specifying the number of laser rays for each horizontal scan line in the LiDAR pattern.
     *          This determines the vertical resolution of the scan.
     */
    std::vector<uint32_t> numRaysPerLine;
    /**
     * @brief Configuration states for all emitters
     * @details Vector containing EmitterState configurations that define the angular positioning of each emitter.
     *          Each entry specifies the elevation and azimuth angles for a specific emitter.
     */
    std::vector<EmitterState> emitterStates;
    /**
     * @brief Number of vertical scan lines
     * @details Total number of horizontal scan lines that make up the complete LiDAR scan pattern.
     *          This determines the vertical field of view and resolution.
     */
    uint32_t numLines{ 1 };
    /**
     * @brief Rotation direction of the LiDAR scanner
     * @details Specifies whether the scanner rotates clockwise or counterclockwise during operation.
     *          This affects the order in which points are collected during a scan.
     */
    LidarRotationDirection rotationDirection{ LidarRotationDirection::CW };
    /** @brief Whether the LiDAR sensor is 2D (single line) or 3D (multiple lines) */
    bool is2D{ false };


    /**
     * @brief JSON document for configuration parsing
     * @details Smart pointer to rapidjson document used for parsing LiDAR configuration files.
     *          This is used to store and process the JSON configuration data.
     */
    std::unique_ptr<rapidjson::Document> m_doc{ nullptr };


    /** @brief Gets the minimum range of the LiDAR sensor */
    float getNearRange() const;
    /** @brief Gets the maximum range of the LiDAR sensor */
    float getFarRange() const;
    /** @brief Gets the number of vertical channels in the LiDAR */
    uint32_t getNumChannels() const;
    /** @brief Gets the number of echoes per beam */
    uint32_t getNumEchos() const;
    /** @brief Gets the total number of returns per complete scan */
    uint32_t getReturnsPerScan() const;
    /** @brief Gets the number of ticks required for a complete scan */
    uint32_t getTicksPerScan() const;
    /**
     * @brief Updates the LiDAR configuration from a render product path
     * @param[in] renderProductPath Path to the render product configuration
     * @return True if update was successful, false otherwise
     * @throws std::runtime_error If the configuration file cannot be read or parsed
     */
    bool updateLidarConfig(const char* renderProductPath);

    /**
     * init document
     * @param json [in] json file name with path
     */
    void init(const char* json);

    // TODO: maybe pass in init the profile name as a string and use the getProfile internally
    // Then, we don't need an extra call to getProfileJsonAtPaths -- but we lose the flexibility to pass just a read
    // json

    /**
     * @brief Gets JSON content from given filename
     * @details Searches for the JSON file in internal paths or at given paths
     * @param[in] fileName JSON file name with path
     * @return String containing the JSON content from the file
     * @throws std::runtime_error If the file cannot be found or read
     */
    omni::string getProfileJsonAtPaths(const char* fileName);

    /**
     * @brief Reads the entire contents of a text file
     * @details
     * Utility function for reading complete text files into memory as a string.
     * Used for loading configuration files and other text-based data.
     *
     * @param[in] fullPath Complete file path to the text file to read
     * @return String containing the complete file contents
     * @throws std::runtime_error If file cannot be opened or read
     */
    static std::string ReadWholeTextFile(std::string fullPath);
};

}
}
}
