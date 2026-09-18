# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

isaacsim_add_group(
    NAME isaacsim_ovgl_viewport
    MODULES
        isaacsim.ovgl_viewport.debug
)

isaacsim_add_group_dependency(
    GROUP isaacsim_ovgl_viewport
    DEPENDS_ON isaacsim_common
)

isaacsim_add_group_dependency(
    GROUP isaacsim_ovgl_viewport
    DEPENDS_ON isaacsim_foundation
)

add_subdirectory(debug)
