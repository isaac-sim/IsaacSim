<!--
SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
SPDX-License-Identifier: Apache-2.0
-->

# Pick-and-Place State Machine

`main.py` combines cuMotion RMPflow with a measurement-driven state machine to stack and unstack two physical cubes
using a Franka Panda robot. `state_machine.py` contains the bounded task phases and transition guards.
