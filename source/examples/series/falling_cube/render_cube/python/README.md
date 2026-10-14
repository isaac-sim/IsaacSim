<!--
SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
SPDX-License-Identifier: Apache-2.0
-->

# Render a cube

This step adds rendering to **Simulate a Cube**. It creates the same scene, simulates it with OvPhysX, and renders each
captured frame with `ovrtx`.

The physics engine and renderer share one OVStage. Foundation creates the scene with the `openusd` backend, imports it
into an OVStage-backed stage, and gives the same native address to both systems.

The Foundation Objects API creates the camera and light. The example adds the render product as a small USDA layer
because the Python API does not yet support creating arbitrary attributes and relationships.

After each simulation step, the physics manager publishes the cube's pose to the stage. Simulation does not update the
stage automatically. Publishing creates a new OVStage version that the next render can see.

## Run

```sh
python examples.py run falling_cube.render_cube.python
```

Frames are written to a new temporary directory by default. Pass `--output` to choose one:

```sh
python examples.py run falling_cube.render_cube.python -- --output /tmp/frames
```

The example prints the cube's travel, the number of captured frames, and:

```text
The cube fell.
The render followed the simulation.
```

## Requirements

The repository pins `ovrtx` in its `pixi.toml` runtime environments. It requires a GPU with a working RTX driver.
