# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Configuration definitions for supported Lidar sensors in Isaac Sim.

This module defines the supported Lidar sensor configurations and their variants
that can be used with the RTX sensor system. It includes configurations for various
manufacturers.
"""

# Expected name of Lidar prim variant sets
SUPPORTED_LIDAR_VARIANT_SET_NAME = "sensor"

# Map supported Lidar asset paths to their variants
SUPPORTED_LIDAR_CONFIGS = {
    "/Isaac/Sensors/HESAI/XT32_SD10/HESAI_XT32_SD10.usd": set(),
    # "/Isaac/Sensors/NVIDIA/Debug_Rotary.usda": set(),
    "/Isaac/Sensors/NVIDIA/Example_Rotary_2D.usda": set(),
    # "/Isaac/Sensors/NVIDIA/Example_Rotary_BEAMS.usda": set(),
    "/Isaac/Sensors/NVIDIA/Example_Rotary.usda": set(),
    # "/Isaac/Sensors/NVIDIA/Example_Solid_State_BEAMS.usda": set(),
    "/Isaac/Sensors/NVIDIA/Example_Solid_State.usda": set(),
    "/Isaac/Sensors/NVIDIA/Simple_Example_Solid_State.usda": set(),
    "/Isaac/Sensors/Ouster/OS0/OS0.usd": {
        "OS0_REV6_128ch10hz1024res",
        "OS0_REV6_128ch10hz2048res",
        "OS0_REV6_128ch10hz512res",
        "OS0_REV6_128ch20hz1024res",
        "OS0_REV6_128ch20hz512res",
        "OS0_REV7_128ch10hz1024res",
        "OS0_REV7_128ch10hz2048res",
        "OS0_REV7_128ch10hz512res",
        "OS0_REV7_128ch20hz1024res",
        "OS0_REV7_128ch20hz512res",
    },
    "/Isaac/Sensors/Ouster/OS1/OS1.usd": {
        "OS1_REV6_128ch10hz1024res",
        "OS1_REV6_128ch10hz2048res",
        "OS1_REV6_128ch10hz512res",
        "OS1_REV6_128ch20hz1024res",
        "OS1_REV6_128ch20hz512res",
        "OS1_REV6_32ch10hz1024res",
        "OS1_REV6_32ch10hz2048res",
        "OS1_REV6_32ch10hz512res",
        "OS1_REV6_32ch20hz1024res",
        "OS1_REV6_32ch20hz512res",
        "OS1_REV7_128ch10hz1024res",
        "OS1_REV7_128ch10hz2048res",
        "OS1_REV7_128ch10hz512res",
        "OS1_REV7_128ch20hz1024res",
        "OS1_REV7_128ch20hz512res",
    },
    "/Isaac/Sensors/Ouster/OS2/OS2.usd": {
        "OS2_REV6_128ch10hz1024res",
        "OS2_REV6_128ch10hz2048res",
        "OS2_REV6_128ch10hz512res",
        "OS2_REV6_128ch20hz1024res",
        "OS2_REV6_128ch20hz512res",
        "OS2_REV7_128ch10hz1024res",
        "OS2_REV7_128ch10hz2048res",
        "OS2_REV7_128ch10hz512res",
        "OS2_REV7_128ch20hz1024res",
        "OS2_REV7_128ch20hz512res",
    },
    "/Isaac/Sensors/Ouster/VLS_128/Ouster_VLS_128.usd": set(),
    # "/Isaac/Sensors/SICK/LRS4581R/SICK_LRS4581R.usd": {
    #     "Profile_Extended_1",
    #     "Profile_Extended_2",
    #     "Profile_Extended_3",
    #     "Profile_Extended_4",
    #     "Profile_Extended_5",
    #     "Profile_Extended_6",
    #     "Profile_Extended_7",
    #     "Profile_Extended_8",
    #     "Profile_1",
    #     "Profile_2",
    #     "Profile_3",
    #     "Profile_4",
    #     "Profile_5",
    #     "Profile_6",
    #     "Profile_7",
    #     "Profile_8",
    #     "Profile_9",
    #     "Profile_10",
    # },
    "/Isaac/Sensors/SICK/microScan3/SICK_microScan3.usd": {
        "Profile_1",
        "Profile_2",
        "Profile_3",
        "Profile_4",
        "Profile_5",
        "Profile_6",
    },
    # "/Isaac/Sensors/SICK/MRS1104C/SICK_MRS1104C.usd": {
    #     "Profile_1",
    # "Profile_2_interlaced",
    # "Profile_3_interlaced",
    # },
    "/Isaac/Sensors/SICK/multiScan136/SICK_multiScan136.usd": set(),
    "/Isaac/Sensors/SICK/multiScan165/SICK_multiScan165.usd": set(),
    "/Isaac/Sensors/SICK/nanoScan3/SICK_nanoScan3.usd": {
        "CAAZ30AN1",
    },
    "/Isaac/Sensors/SICK/picoScan150/SICK_picoScan150.usd": {
        "Profile_1",
        "Profile_2",
        "Profile_3",
        "Profile_4",
        "Profile_5",
        "Profile_6",
        "Profile_7",
        "Profile_8",
        "Profile_9",
        "Profile_10",
        "Profile_11",
    },
    "/Isaac/Sensors/SICK/TIM781/SICK_TIM781.usd": set(),
    "/Isaac/Sensors/Slamtec/RPLIDAR_S2E/Slamtec_RPLIDAR_S2E.usd": set(),
    "/Isaac/Sensors/ZVISION/ZVISION_ML30S.usda": set(),
    "/Isaac/Sensors/ZVISION/ZVISION_MLXS.usda": set(),
}
