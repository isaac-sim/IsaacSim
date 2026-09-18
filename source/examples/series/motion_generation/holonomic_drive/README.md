<!--
SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
SPDX-License-Identifier: Apache-2.0
-->

# Holonomic Drive Controller

`main.py` converts forward speed, lateral speed, and yaw rate commands into the NVIDIA Kaya robot's wheel targets with
`HolonomicController`, applies them through OvPhysX, and renders the motion with OVGL.
