<!--
SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
SPDX-License-Identifier: Apache-2.0
-->

# Differential Drive Controller

`main.py` converts a planar velocity command into the TurtleBot3 Burger robot's wheel targets with
`DifferentialDriveController`, applies them through OvPhysX, and renders the motion with OVGL.
