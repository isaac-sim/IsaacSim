# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#: Default Lidar variant set name (used for entries whose variants are flat strings).
SUPPORTED_LIDAR_VARIANT_SET_NAME = "sensor"

#: Map of supported Lidar asset paths to their variant selections. Each value is either:
#:
#: - a ``set[str]`` of flat variant names — applied against the variant set named
#:   ``SUPPORTED_LIDAR_VARIANT_SET_NAME`` (i.e. ``"sensor"``); used by USDs that expose
#:   a single ``sensor`` variant set, or
#: - a ``list[dict[str, str]]`` of explicit ``{set_name: value, ...}`` mappings — required
#:   for USDs with multiple variant sets, or where the variant set name is not ``"sensor"``.
SUPPORTED_LIDAR_CONFIGS: dict[str, "set[str] | list[dict[str, str]]"] = {
    "/Isaac/Sensors/HESAI/XT32_SD10/HESAI_XT32_SD10.usd": set(),
    "/Isaac/Sensors/NVIDIA/Example_Rotary_2D.usda": set(),
    "/Isaac/Sensors/NVIDIA/Example_Rotary.usda": set(),
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
    # SICK family USDs use named variant sets ("Product", "Profile") that don't match
    # the default "sensor" set name, so all SICK entries use the dict form.
    "/Isaac/Sensors/SICK/LMS4000/SICK_LMS4000.usd": [
        {"Product": product, "Profile": profile}
        for product, profile in [
            ("LMS4111R", "Profile01_3p0m_0p0833deg"),
            ("LMS4121R", "Profile01_3p0m_0p0833deg"),
            ("LMS4124R", "Profile01_5p5m_0p0833deg"),
        ]
    ],
    "/Isaac/Sensors/SICK/LMS5xx/SICK_LMS5xx.usd": [
        {"Product": product, "Profile": profile}
        for product, profiles in [
            (
                "LMS5xxExtendedRange",
                [
                    "Profile00_25Hz_0p1667deg",
                    "Profile01_25Hz_0p25deg",
                    "Profile02_35Hz_0p25deg",
                    "Profile03_35Hz_0p5deg",
                    "Profile04_50Hz_0p3333deg",
                    "Profile05_50Hz_0p5deg",
                    "Profile06_75Hz_0p5deg",
                    "Profile07_75Hz_1p0deg",
                    "Profile08_100Hz_0p667deg",
                    "Profile09_100Hz_1p0deg",
                    "Profile10_50Hz_0p1667deg",
                    "Profile11_75Hz_0p25deg",
                    "Profile12_100Hz_0p1667deg",
                    "Profile13_100Hz_0p3333deg",
                    "Profile14_100Hz_0p5deg",
                    "Profile15_25Hz_0p083deg",
                    "Profile16_25Hz_0p042deg",
                ],
            ),
            (
                "LMS5xxLiteHR",
                [
                    "Profile01_25Hz_0p25deg",
                    "Profile03_35Hz_0p5deg",
                    "Profile05_50Hz_0p5deg",
                    "Profile06_75Hz_0p5deg",
                    "Profile07_75Hz_1p0deg",
                ],
            ),
            (
                "LMS5xxLiteSR",
                [
                    "Profile01_25Hz_0p25deg",
                    "Profile03_35Hz_0p5deg",
                    "Profile05_50Hz_0p5deg",
                    "Profile06_75Hz_0p5deg",
                    "Profile07_75Hz_1p0deg",
                ],
            ),
            (
                "LMS5xxProHR",
                [
                    "Profile00_25Hz_0p1667deg",
                    "Profile01_25Hz_0p25deg",
                    "Profile02_35Hz_0p25deg",
                    "Profile03_35Hz_0p5deg",
                    "Profile04_50Hz_0p3333deg",
                    "Profile05_50Hz_0p5deg",
                    "Profile06_75Hz_0p5deg",
                    "Profile07_75Hz_1p0deg",
                    "Profile08_100Hz_0p667deg",
                    "Profile09_100Hz_1p0deg",
                    "Profile10_50Hz_0p1667deg",
                    "Profile11_75Hz_0p25deg",
                    "Profile12_100Hz_0p1667deg",
                    "Profile13_100Hz_0p3333deg",
                    "Profile14_100Hz_0p5deg",
                    "Profile15_25Hz_0p083deg",
                    "Profile16_25Hz_0p042deg",
                ],
            ),
            (
                "LMS5xxProSR",
                [
                    "Profile00_25Hz_0p1667deg",
                    "Profile01_25Hz_0p25deg",
                    "Profile02_35Hz_0p25deg",
                    "Profile03_35Hz_0p5deg",
                    "Profile04_50Hz_0p3333deg",
                    "Profile05_50Hz_0p5deg",
                    "Profile06_75Hz_0p5deg",
                    "Profile07_75Hz_1p0deg",
                    "Profile08_100Hz_0p667deg",
                    "Profile09_100Hz_1p0deg",
                    "Profile10_50Hz_0p1667deg",
                    "Profile11_75Hz_0p25deg",
                    "Profile12_100Hz_0p1667deg",
                    "Profile13_100Hz_0p3333deg",
                    "Profile14_100Hz_0p5deg",
                    "Profile15_25Hz_0p083deg",
                    "Profile16_25Hz_0p042deg",
                ],
            ),
        ]
        for profile in profiles
    ],
    "/Isaac/Sensors/SICK/LRS4000/SICK_LRS4000.usd": [
        {"Profile": p}
        for p in [
            "Profile01_12p5Hz_0p04deg",
            "Profile02_12p5Hz_0p06deg",
            "Profile04_12p5Hz_0p1deg",
            "Profile05_12p5Hz_0p12deg",
            "Profile11_12p5Hz_0p02deg",
            "Profile31_12p5Hz_0p24deg",
            "Profile32_12p5Hz_0p12deg",
            "Profile33_12p5Hz_0p08deg",
            "Profile35_12p5Hz_0p04deg",
            "Profile61_25Hz_0p08deg",
            "Profile62_25Hz_0p12deg",
            "Profile64_25Hz_0p2deg",
            "Profile65_25Hz_0p24deg",
            "Profile71_25Hz_0p04deg",
            "Profile91_25Hz_0p48deg",
            "Profile92_25Hz_0p24deg",
            "Profile93_25Hz_0p16deg",
            "Profile95_25Hz_0p08deg",
        ]
    ],
    "/Isaac/Sensors/SICK/microScan3/SICK_microScan3.usd": [
        {"Profile": p}
        for p in [
            "Profile01_4p0m_33p3Hz",
            "Profile02_4p0m_25p0Hz",
            "Profile03_5p5m_33p3Hz",
            "Profile04_5p5m_25p0Hz",
            "Profile05_9p0m_25p0Hz",
            "Profile06_9p0m_20p0Hz",
        ]
    ],
    "/Isaac/Sensors/SICK/MRS1000/SICK_MRS1000.usd": [
        {"Profile": p}
        for p in [
            "Profile01_12p5Hz_0p25deg",
            "Profile02_6p25Hz_0p125deg",
            "Profile03_3p125Hz_0p0625deg",
        ]
    ],
    # Per-Product Profile differs: 136/166 use 0.125° resolution, 165/165S use 0.5°.
    "/Isaac/Sensors/SICK/multiScan100/SICK_multiScan100.usd": [
        {"Product": "multiScan136", "Profile": "Profile01_20Hz_0p125deg"},
        {"Product": "multiScan165", "Profile": "Profile01_20Hz_0p5deg"},
        {"Product": "multiScan165S", "Profile": "Profile01_20Hz_0p5deg"},
        {"Product": "multiScan166", "Profile": "Profile01_20Hz_0p125deg"},
    ],
    "/Isaac/Sensors/SICK/nanoScan3/SICK_nanoScan3.usd": set(),
    # picoScan150Pro listed first so it is the menu's default variant — the picoScan120
    # sub-model has only a single profile and may not author a fully-resolved OmniLidar
    # prim across all loaders.
    "/Isaac/Sensors/SICK/picoScan100/SICK_picoScan100.usd": [
        {"Product": product, "Profile": profile}
        for product, profiles in [
            (
                "picoScan150Pro",
                [
                    "Profile01_15Hz_0p5deg",
                    "Profile02_15Hz_0p33deg",
                    "Profile03_20Hz_0p1deg",
                    "Profile04_20Hz_0p25deg",
                    "Profile05_25Hz_0p25deg",
                    "Profile06_30Hz_0p1deg",
                    "Profile07_40Hz_0p25deg",
                    "Profile08_50Hz_0p25deg",
                    "Profile09_15Hz_0p05deg",
                    "Profile10_40Hz_0p125deg",
                    "Profile11_15Hz_1p0deg",
                ],
            ),
            (
                "picoScan150Prime",
                [
                    "Profile01_15Hz_0p5deg",
                    "Profile02_15Hz_0p33deg",
                    "Profile03_20Hz_0p1deg",
                    "Profile04_20Hz_0p25deg",
                    "Profile05_25Hz_0p25deg",
                    "Profile06_30Hz_0p1deg",
                    "Profile07_40Hz_0p25deg",
                    "Profile08_50Hz_0p25deg",
                    "Profile09_15Hz_0p05deg",
                    "Profile10_40Hz_0p125deg",
                    "Profile11_15Hz_1p0deg",
                ],
            ),
            (
                "picoScan150Core",
                [
                    "Profile02_15Hz_0p33deg",
                    "Profile05_25Hz_0p25deg",
                    "Profile11_15Hz_1p0deg",
                ],
            ),
            ("picoScan120", ["Profile03_20Hz_0p1deg"]),
        ]
        for profile in profiles
    ],
    "/Isaac/Sensors/SICK/TIM781/SICK_TIM781.usd": set(),
    "/Isaac/Sensors/Slamtec/RPLIDAR_S2E/Slamtec_RPLIDAR_S2E.usd": set(),
    "/Isaac/Sensors/ZVISION/ZVISION_ML30S.usda": set(),
    "/Isaac/Sensors/ZVISION/ZVISION_MLXS.usda": set(),
}
