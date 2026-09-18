# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

isaacsim_add_group(
    NAME isaacsim_performance
    MODULES
        isaacsim.performance.benchmarking
)

add_subdirectory(benchmarking)
