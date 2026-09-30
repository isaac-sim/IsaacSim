<!--
SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
SPDX-License-Identifier: Apache-2.0
-->

# cuMotion RMPflow Follow Target

`main.py` uses cuMotion RMPflow to move Franka's `panda_leftfingertip` frame toward a Cartesian target and applies the
resulting joint targets through OvPhysX.
