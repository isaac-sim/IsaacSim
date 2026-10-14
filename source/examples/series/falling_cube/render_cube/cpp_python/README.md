<!--
SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
SPDX-License-Identifier: Apache-2.0
-->

# Render a cube with C++

This is the C++ version of **Render a Cube**. The code in `Bindings.cpp` creates the scene from **Simulate a Cube**,
simulates it with OvPhysX, and renders each captured frame with `ovrtx`. It uses only public C++ APIs.

The physics engine and renderer share one OVStage. Foundation creates the scene with the `openusd` backend, imports it
into an OVStage-backed stage, and gives the same native pointer to both systems. No conversion is needed:
`Stage::getStagePtr` returns the `ovstage_instance_t*` expected by `ovrtx_attach_ovstage`.

The Foundation Objects API creates the camera, light, render variable, and render product. The example adds only the
render product's camera and render-variable relationships to the exported USDA text because the public API does not
yet support creating those relationship targets.

After each simulation step, the physics manager publishes the cube's pose to the stage. Simulation does not update the
stage automatically. Publishing creates a revision that the next render can see. `RigidBodyEntity` reports the
simulated height used to confirm that the cube landed.

Unlike the Python example, `Workflow` owns the stage and shuts down the renderer, physics manager, and stage in the
required order. It writes frames as binary PPM files instead of PNG files, avoiding an extra compression dependency.

## Why a Python entry point

The example builds as a Python extension and starts through `main.py`. The Foundation OpenUSD backend needs symbols
from `usd-core`, which `isaacsim.foundation.usd.openusd` loads before the extension. A standalone executable would not
have those symbols available. All workflow logic remains in C++; `main.py` loads the dependency, selects the output
directory, and calls the extension.

## Run

```sh
python examples.py run falling_cube.render_cube.cpp_python
```

Frames are written to a new temporary directory by default. Pass `--output` to choose one:

```sh
python examples.py run falling_cube.render_cube.cpp_python -- --output /tmp/frames
```

The example prints the cube's travel, the number of captured frames, and:

```text
The cube fell.
The render followed the simulation.
```

## Requirements

The repository pins `ovrtx` in its `pixi.toml` runtime environments. The package provides C headers, the
`ovrtx::ovrtx` CMake target, and the renderer runtime. It requires a GPU with a working RTX driver.
