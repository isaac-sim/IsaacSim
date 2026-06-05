// SPDX-FileCopyrightText: Copyright (c) 2023-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

/** @file
 * @brief ROS 2 Quality of Service (QoS) definitions and utilities
 * @details
 * This file contains the definitions and utilities for handling ROS 2 Quality of Service (QoS)
 * settings in the Isaac Sim bridge. It provides enums, structures, and conversion utilities
 * for managing QoS profiles according to ROS 2 specifications.
 */
#pragma once

#include <nlohmann/json.hpp>

#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace isaacsim
{
namespace ros2
{
namespace core
{

/**
 * @enum Ros2QoSHistoryPolicy
 * @brief Enumerations of ROS 2 QoS History policy
 * @details
 * Defines the history policy options for ROS 2 QoS settings, determining how messages
 * are stored before being sent or after being received.
 *
 * @see [ROS 2 QoS policies](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Quality-of-Service-Settings.html)
 */
enum class Ros2QoSHistoryPolicy
{
    eSystemDefault, /**< Use the system default history setting */
    eKeepLast, /**< Keep only the last N messages (N defined by queue depth) */
    eKeepAll, /**< Keep all messages */
    eUnknown /**< Unknown or invalid history policy */
};

/**
 * @enum Ros2QoSReliabilityPolicy
 * @brief Enumerations of ROS 2 QoS Reliability policy
 * @details
 * Defines the reliability policy options for ROS 2 QoS settings, determining the
 * guarantees about message delivery.
 *
 * @see [ROS 2 QoS policies](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Quality-of-Service-Settings.html)
 */
enum class Ros2QoSReliabilityPolicy
{
    eSystemDefault, /**< Use the system default reliability setting */
    eReliable, /**< Guarantee message delivery with retries */
    eBestEffort, /**< No delivery guarantee, may drop messages */
    eUnknown /**< Unknown or invalid reliability policy */
};

/**
 * @enum Ros2QoSDurabilityPolicy
 * @brief Enumerations of ROS 2 QoS Durability policy
 * @details
 * Defines the durability policy options for ROS 2 QoS settings, determining how messages
 * are handled for late-joining subscribers.
 *
 * @see [ROS 2 QoS policies](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Quality-of-Service-Settings.html)
 */
enum class Ros2QoSDurabilityPolicy
{
    eSystemDefault, /**< Use the system default durability setting */
    eTransientLocal, /**< Store messages locally for late joiners */
    eVolatile, /**< No storage of messages for late joiners */
    eUnknown /**< Unknown or invalid durability policy */
};

/**
 * @enum Ros2QoSLivelinessPolicy
 * @brief Enumerations of ROS 2 QoS Liveliness policy
 * @details
 * Defines the liveliness policy options for ROS 2 QoS settings, determining how
 * the system monitors the presence of entities.
 *
 * @see [ROS 2 QoS policies](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Quality-of-Service-Settings.html)
 */
enum class Ros2QoSLivelinessPolicy
{
    eSystemDefault, /**< Use the system default liveliness setting */
    eAutomatic, /**< System automatically monitors liveliness */
    eManualByNode, /**< Node must manually assert liveliness (Deprecated) */
    eManualByTopic, /**< Publisher must manually assert liveliness */
    eUnknown /**< Unknown or invalid liveliness policy */
};

/**
 * @struct Ros2QoSTime
 * @brief ROS 2 QoS time representation
 * @details
 * Structure for representing time durations in ROS 2 QoS settings,
 * using seconds and nanoseconds components.
 */
struct Ros2QoSTime
{
    uint64_t sec; /**< Time in seconds */
    uint64_t nsec; /**< Additional time in nanoseconds */
};

/**
 * @struct Ros2QoSProfile
 * @brief ROS 2 QoS Profile configuration
 * @details
 * Comprehensive structure for configuring all aspects of ROS 2 Quality of Service.
 * Provides settings for history, reliability, durability, deadlines, lifespan,
 * and liveliness policies.
 */
struct Ros2QoSProfile
{
    Ros2QoSHistoryPolicy history; /**< History policy setting */
    size_t depth; /**< Size of the message queue */
    Ros2QoSReliabilityPolicy reliability; /**< Reliability policy setting */
    Ros2QoSDurabilityPolicy durability; /**< Durability policy setting */
    Ros2QoSTime deadline; /**< Period for expected message send/receive */
    Ros2QoSTime lifespan; /**< Maximum age for valid messages */
    Ros2QoSLivelinessPolicy liveliness; /**< Liveliness policy setting */
    Ros2QoSTime livelinessLeaseDuration; /**< Time window for liveliness assertion */
    bool avoidRosNamespaceConventions; /**< Flag to bypass ROS 2 namespace conventions */

    /**
     * @brief Default constructor initializing system default values
     * @details
     * Initializes the QoS profile with ROS 2 default values from rmw_qos_profile_default:
     * - History: Keep last 10 messages
     * - Reliability: Reliable delivery
     * - Durability: Volatile
     * - Other timing parameters: 0
     */
    Ros2QoSProfile()
    {
        // These are the values from the `rmw_qos_profile_default`
        history = Ros2QoSHistoryPolicy::eKeepLast;
        depth = 10;
        reliability = Ros2QoSReliabilityPolicy::eReliable;
        durability = Ros2QoSDurabilityPolicy::eVolatile;
        deadline = { 0, 0 };
        lifespan = { 0, 0 };
        liveliness = Ros2QoSLivelinessPolicy::eSystemDefault;
        livelinessLeaseDuration = { 0, 0 };
        avoidRosNamespaceConventions = false;
    }
};

namespace
{

/**
 * @brief Mapping from string representations to History policy enums
 * @details
 * Lookup table for converting string values to Ros2QoSHistoryPolicy enum values.
 * Used during JSON parsing of QoS configuration.
 */
const std::map<std::string, Ros2QoSHistoryPolicy> g_kRos2QoSHistoryString2PolicyMap = {
    { "systemDefault", Ros2QoSHistoryPolicy::eSystemDefault },
    { "keepLast", Ros2QoSHistoryPolicy::eKeepLast },
    { "keepAll", Ros2QoSHistoryPolicy::eKeepAll },
    { "unknown", Ros2QoSHistoryPolicy::eUnknown }
};

/**
 * @brief Mapping from string representations to Reliability policy enums
 * @details
 * Lookup table for converting string values to Ros2QoSReliabilityPolicy enum values.
 * Used during JSON parsing of QoS configuration.
 */
const std::map<std::string, Ros2QoSReliabilityPolicy> g_kRos2QoSReliabilityString2PolicyMap = {
    { "systemDefault", Ros2QoSReliabilityPolicy::eSystemDefault },
    { "reliable", Ros2QoSReliabilityPolicy::eReliable },
    { "bestEffort", Ros2QoSReliabilityPolicy::eBestEffort },
    { "unknown", Ros2QoSReliabilityPolicy::eUnknown }
};

/**
 * @brief Mapping from string representations to Durability policy enums
 * @details
 * Lookup table for converting string values to Ros2QoSDurabilityPolicy enum values.
 * Used during JSON parsing of QoS configuration.
 */
const std::map<std::string, Ros2QoSDurabilityPolicy> g_kRos2QoSDurabilityString2PolicyMap = {
    { "systemDefault", Ros2QoSDurabilityPolicy::eSystemDefault },
    { "transientLocal", Ros2QoSDurabilityPolicy::eTransientLocal },
    { "volatile", Ros2QoSDurabilityPolicy::eVolatile },
    { "unknown", Ros2QoSDurabilityPolicy::eUnknown }
};

/**
 * @brief Mapping from string representations to Liveliness policy enums
 * @details
 * Lookup table for converting string values to Ros2QoSLivelinessPolicy enum values.
 * Used during JSON parsing of QoS configuration.
 */
const std::map<std::string, Ros2QoSLivelinessPolicy> g_kRos2QoSLivelinessString2PolicyMap = {
    { "systemDefault", Ros2QoSLivelinessPolicy::eSystemDefault },
    { "automatic", Ros2QoSLivelinessPolicy::eAutomatic },
    { "manualByTopic", Ros2QoSLivelinessPolicy::eManualByTopic },
    { "unknown", Ros2QoSLivelinessPolicy::eUnknown }
};

} // namespace anonymous

/**
 * @brief Converts a JSON string to a ROS 2 QoS profile
 * @details
 * Parses a JSON string containing QoS settings and converts it into a Ros2QoSProfile
 * structure. The JSON must contain all required fields with appropriate types:
 * - history: string (matching Ros2QoSHistoryPolicy)
 * - depth: non-negative integer
 * - reliability: string (matching Ros2QoSReliabilityPolicy)
 * - durability: string (matching Ros2QoSDurabilityPolicy)
 * - deadline: non-negative float (seconds)
 * - lifespan: non-negative float (seconds)
 * - liveliness: string (matching Ros2QoSLivelinessPolicy)
 * - leaseDuration: non-negative float (seconds)
 *
 * @param[out] qos Reference to Ros2QoSProfile to store the converted settings
 * @param[in] jsonString JSON string containing QoS settings
 * @return bool True if conversion successful, false otherwise
 *
 * @note The function performs extensive validation of the JSON input
 */
inline static const bool jsonToRos2QoSProfile(Ros2QoSProfile& qos, const std::string& jsonString)
{
    // Define the required keys and their expected types
    std::map<std::string, std::function<bool(const nlohmann::json&)>> requiredKeys = {
        { "history", [](const nlohmann::json& json) { return json.is_string(); } },
        { "depth", [](const nlohmann::json& json) { return json.is_number_integer() && json.get<int>() >= 0; } },
        { "reliability", [](const nlohmann::json& json) { return json.is_string(); } },
        { "durability", [](const nlohmann::json& json) { return json.is_string(); } },
        { "deadline", [](const nlohmann::json& json) { return json.is_number_float() && json.get<double>() >= 0; } },
        { "lifespan", [](const nlohmann::json& json) { return json.is_number_float() && json.get<double>() >= 0; } },
        { "liveliness", [](const nlohmann::json& json) { return json.is_string(); } },
        { "leaseDuration", [](const nlohmann::json& json) { return json.is_number_float() && json.get<double>() >= 0; } }
    };

    // Parse the JSON string
    nlohmann::json json;
    try
    {
        json = nlohmann::json::parse(jsonString);
    }
    catch (nlohmann::json::parse_error& e)
    {
        std::cerr << "Parsing error: " << e.what() << '\n';
        return false;
    }

    // Check for the presence and type of required keys
    for (const auto& [key, validator] : requiredKeys)
    {
        if (!json.contains(key) || !validator(json[key]))
        {
            std::cerr << "Missing key: " << key << " Or invalid value: " << json[key] << '\n';
            return false;
        }
    }

    // Validate that the values for the keys exist in the corresponding maps
    if (g_kRos2QoSHistoryString2PolicyMap.find(json["history"].get<std::string>()) ==
        g_kRos2QoSHistoryString2PolicyMap.end())
    {
        std::cerr << "Invalid value for 'history'\n";
        return false;
    }
    if (g_kRos2QoSReliabilityString2PolicyMap.find(json["reliability"].get<std::string>()) ==
        g_kRos2QoSReliabilityString2PolicyMap.end())
    {
        std::cerr << "Invalid value for 'reliability'\n";
        return false;
    }
    if (g_kRos2QoSDurabilityString2PolicyMap.find(json["durability"].get<std::string>()) ==
        g_kRos2QoSDurabilityString2PolicyMap.end())
    {
        std::cerr << "Invalid value for 'durability'\n";
        return false;
    }
    if (g_kRos2QoSLivelinessString2PolicyMap.find(json["liveliness"].get<std::string>()) ==
        g_kRos2QoSLivelinessString2PolicyMap.end())
    {
        std::cerr << "Invalid value for 'liveliness'\n";
        return false;
    }

    // Lambda function to convert seconds to ROS 2 time format
    // Creates a Ros2QoSTime struct from a floating-point value representing seconds,
    // splitting it into seconds and nanoseconds components.
    auto createRos2QoSTimeType = [](double totalSeconds) -> Ros2QoSTime
    {
        uint64_t secs = static_cast<uint64_t>(totalSeconds);
        uint64_t nanoseconds = static_cast<uint64_t>((totalSeconds - secs) * 1e9);
        return Ros2QoSTime{ secs, nanoseconds };
    };

    qos.history = g_kRos2QoSHistoryString2PolicyMap.at(json["history"].get<std::string>());
    qos.depth = json["depth"];
    qos.reliability = g_kRos2QoSReliabilityString2PolicyMap.at(json["reliability"].get<std::string>());
    qos.durability = g_kRos2QoSDurabilityString2PolicyMap.at(json["durability"].get<std::string>());
    qos.deadline = createRos2QoSTimeType(json["deadline"]);
    qos.lifespan = createRos2QoSTimeType(json["lifespan"]);
    qos.liveliness = g_kRos2QoSLivelinessString2PolicyMap.at(json["liveliness"].get<std::string>());
    qos.livelinessLeaseDuration = createRos2QoSTimeType(json["leaseDuration"]);
    qos.avoidRosNamespaceConventions = false;
    return true;
}

} // namespace core
} // namespace ros2
} // namespace isaacsim
