<!--
SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
SPDX-License-Identifier: Apache-2.0
-->

# Single-View Depth Sensor Benchmark

This benchmark measures what `ovrtx` costs per frame when it runs a ring of simulated stereo depth
sensors. It places `--num-sensors` cameras around an authored environment of ground, lights, and
obstacles, turns each one into a depth sensor by applying `OmniSensorDepthSensorSingleViewAPI` to
its render product, and maps the requested depth-sensor outputs back each frame.

The sensor is a renderer post-process, so the schema goes on the render product rather than the
camera, and the product must also request the render variables the post-process consumes.

Every sensor benchmark shares the same scene and command line, and all of them are measured by
`isaacsim.performance.benchmarking`, so a depth-sensor run is directly comparable to a camera,
lidar, or radar run.

## Reading a multi-sensor run

Only one of the render products gets its depth-sensor outputs written per frame, however many
sensors are placed. The others map as empty buffers, counted as `unwritten_outputs` in the result.
Which product is the written one follows the renderer's view index rather than the prim path, and
view assignment varies between runs. So `--num-sensors` above 1 scales the camera rendering the
post-process reads from, but not the post-process itself.

## Run

From this directory, with the dependencies in `example.toml` installed:

```bash
python main.py --num-frames 300 --render-vars DepthSensorDistance DepthSensorImager --output /tmp/depth.json
```

Defaults are one sensor at 1280x720, 100 warmup frames, and 1,000 measured frames read back on the CPU.

`--render-vars` selects the depth-sensor outputs, from `DepthSensorDistance`, `DepthSensorImager`,
`DepthSensorPointCloudPosition`, and `DepthSensorPointCloudColor`. Use `--help` for the full command
line, including the simulated `--baseline-mm`, `--focal-length-pixel`, `--max-disparity-pixel`, and
`--confidence-threshold`.

## Results

A run writes its measured frame times and configuration to the JSON file selected by `--output`.
The default is `isaacsim-benchmarks/depth_result.json` under the system temporary directory;
`ISAACSIM_BENCHMARK_OUTPUT_DIR` overrides that default directory. The renderer log,
`depth_ovrtx.log`, is written beside the result.

Use the examples runner’s `--nsys` option before the benchmark ID to record the measured
phase (`nsys` must be on `PATH`). `--nsys-scope full` includes startup and warmup;
`--nsys-output PATH` selects the report path or prefix.

`--print-stage` prints the composed USDA and exits without creating a renderer. `--export-stage`
writes the same layer to `depth_stage.usda` beside the result, or to the supplied path,
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
