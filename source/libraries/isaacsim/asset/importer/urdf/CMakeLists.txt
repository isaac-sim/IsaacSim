# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

isaacsim_add_python_module(
    NAME isaacsim.asset.importer.urdf
    PRESERVE_PACKAGE_LAYOUT
)

isaacsim_add_python_tests(
    MODULE isaacsim.asset.importer.urdf
    TIMEOUT 900
    LABELS
        unit
)
