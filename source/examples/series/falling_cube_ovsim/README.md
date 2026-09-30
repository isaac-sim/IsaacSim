<!--
SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
SPDX-License-Identifier: Apache-2.0
-->

# Falling Cube (OV SIM API)

The sequence compares in-process and gRPC clients without changing the cube-authoring and simulation workflow.

For the gRPC step, follow the platform-specific instructions in **Simulate a Cube over gRPC**, linked in the series
steps below. Configure the server paths and start the server in the first terminal before running the client in a
second terminal. The client requires `--endpoint 127.0.0.1:50051`; the server remains running until you press **Ctrl+C**.
