<!--
SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
SPDX-License-Identifier: Apache-2.0
-->

# Camera Benchmark

This benchmark measures what `ovrtx` costs per frame when it renders a ring of cameras. It places
`--num-sensors` cameras around an authored environment of ground, lights, and obstacles, renders
`--num-frames` frames through them, and maps the requested render variables back each frame.

Every sensor benchmark shares the same scene and command line, and all of them are measured by
`isaacsim.performance.benchmarking`, so a camera run is directly comparable to a lidar, radar, or
depth-sensor run.

## Run

From this directory, with the dependencies in `example.toml` installed:

```bash
python main.py --num-sensors 4 --resolution 1920 1080 --output /tmp/camera-4x1080p.json
```

Defaults are one camera at 1280x720, 100 warmup frames, and 1,000 measured frames read back on the CPU.

`--render-vars` selects what each camera produces, from `LdrColor`, `HdrColor`, `DepthSD`,
`DistanceToCameraSD`, `DistanceToImagePlaneSD`, `NormalSD`, `Camera3dPositionSD`, and
`DiffuseAlbedoSD`. Each one is a per-pixel image, so readback cost scales with `--resolution`.

Use `--help` for the full command line, including `--readback`, `--warmup-frames`, `--time-step`,
`--gpus`, and `--scene`.

## Results

A run writes its measured frame times and configuration to the JSON file selected by `--output`.
The default is `isaacsim-benchmarks/camera_result.json` under the system temporary directory;
`ISAACSIM_BENCHMARK_OUTPUT_DIR` overrides that default directory. The renderer log,
`camera_ovrtx.log`, is written beside the result.

Use the examples runner’s `--nsys` option before the benchmark ID to record the measured
phase (`nsys` must be on `PATH`). `--nsys-scope full` includes startup and warmup;
`--nsys-output PATH` selects the report path or prefix.

`--print-stage` prints the composed USDA and exits without creating a renderer. `--export-stage`
writes the same layer to `camera_stage.usda` beside the result, or to the supplied path,
and carries on with the run.

## Requirements

The benchmark declares `isaacsim_performance`, which supplies the measurement, the resource metrics,
and the written result. `ovrtx`, `ovstage`, NumPy, and Warp are declared in `example.toml`.
A measured run needs a GPU with a working RTX driver stack;
`--print-stage` does not. The manifest declares that `gpu` requirement, so the example runner
reports this benchmark's test as skipped on a machine without one rather than failing it.

## Test

`test.py` runs a short benchmark through this same command line and checks what it wrote: the
composed stage, whether the run was valid, the configuration it recorded, and the frames it
measured.

```bash
python test.py
```
