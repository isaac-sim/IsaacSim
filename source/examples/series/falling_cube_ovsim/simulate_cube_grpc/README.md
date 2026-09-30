<!--
SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
SPDX-License-Identifier: Apache-2.0
-->

# Simulate a Cube over gRPC

This example authors a cube locally, sends the stage to an OV SIM gRPC server, and advances the remote simulation.

## Run the example

First, follow the {ref}`server setup instructions <isaacsim-libraries-tutorial-ovsim-workflow-remote-simulation>`
to configure PATH and start the server. Keep that terminal open. In a second terminal, use the runner described
in {ref}`Build and run examples <isaacsim-libraries-run-examples>` with these arguments:

```text
run falling_cube_ovsim.simulate_cube_grpc -- --endpoint 127.0.0.1:50051
```

Stop the server with **Ctrl+C** when finished.
