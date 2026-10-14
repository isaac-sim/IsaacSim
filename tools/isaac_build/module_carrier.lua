-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
-- SPDX-License-Identifier: Apache-2.0

-- Keep the documented M1 entry points on the current wheel staging implementation.
dofile(path.join(path.getdirectory(_SCRIPT), "extension_wheel.lua"))
stage_isaacsim_module_carrier = stage_isaacsim_extension_wheel
stage_isaacsim_module_carrier_with_python_overlays = stage_isaacsim_extension_wheel_with_python_overlays
