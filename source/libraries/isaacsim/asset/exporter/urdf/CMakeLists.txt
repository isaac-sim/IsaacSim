# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

isaacsim_add_python_module(
    NAME isaacsim.asset.exporter.urdf
    PRESERVE_PACKAGE_LAYOUT
)

isaacsim_add_python_tests(
    MODULE isaacsim.asset.exporter.urdf
    LABELS
        unit
)
