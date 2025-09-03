// SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "ManusSDK.h"

#ifdef _MSC_VER
#    if ISAACSIM_XR_INPUT_DEVICES_EXPORT
#        define ISAACSIM_XR_INPUT_DEVICES_DLL_EXPORT __declspec(dllexport)
#    else
#        define ISAACSIM_XR_INPUT_DEVICES_DLL_EXPORT __declspec(dllimport)
#    endif
#else
#    define ISAACSIM_XR_INPUT_DEVICES_DLL_EXPORT __attribute__((visibility("default")))
#endif

namespace isaacsim
{
namespace xr
{
namespace input_devices
{

class ISAACSIM_XR_INPUT_DEVICES_DLL_EXPORT IsaacSimManusTracker
{
public:
    IsaacSimManusTracker();
    ~IsaacSimManusTracker();

    bool initialize();
    std::unordered_map<std::string, std::vector<float>> get_glove_data();
    void cleanup();

private:
    static IsaacSimManusTracker* s_instance;
    static std::mutex s_instance_mutex;

    // ManusSDK specific members
    void RegisterCallbacks();
    void ConnectToGloves();
    void DisconnectFromGloves();
    
    // Callback functions
    static void OnSkeletonStream(const SkeletonStreamInfo* skeleton_stream_info);
    static void OnLandscapeStream(const Landscape* landscape);
    static void OnErgonomicsStream(const ErgonomicsStream* ergonomics_stream);
    
    // Data storage (following isaac-deploy pattern)
    std::mutex output_map_mutex;
    std::mutex landscape_mutex;
    std::unordered_map<std::string, std::vector<float>> output_map;
    std::optional<uint32_t> left_glove_id;
    std::optional<uint32_t> right_glove_id;
    bool is_connected = false;
    
    // Legacy member for compatibility
    std::unordered_map<std::string, std::vector<float>> m_glove_data;
    std::mutex m_data_mutex;
};

} // namespace input_devices
} // namespace xr
} // namespace isaacsim
