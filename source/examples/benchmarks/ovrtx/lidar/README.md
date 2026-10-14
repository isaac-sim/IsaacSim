<!--
SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
SPDX-License-Identifier: Apache-2.0
-->

# RTX Lidar Benchmark

This benchmark measures what `ovrtx` costs per frame when it runs a ring of RTX lidars. It places
`--num-sensors` `OmniLidar` prims around an authored environment of ground, lights, and obstacles,
steps their `PointCloud` render products for `--num-frames` frames, and maps the returned points
back each frame.

Every sensor benchmark shares the same scene and command line, and all of them are measured by
`isaacsim.performance.benchmarking`, so a lidar run is directly comparable to a camera, radar, or
depth-sensor run.

## Scan cadence

A lidar sweeps over time, so by default it only hands out a point cloud once a scan completes. Two
defaults here override that so a step is a fixed unit of work: instant lidar makes every emission a
full scan, and partial outputs make the sensor emit on every step rather than only at scan
boundaries. Turn either off with `--no-instant-lidar` or `--no-partial-outputs` to profile the
sensor's real cadence instead.

`--frame-rate` defaults to the 10 Hz the generic lidar model is specified at. Raising it degenerates
the model's scan pattern — the sensor keeps emitting frames but they carry no returns — so the run
fails rather than saving a profile of empty work.

## Run

From this directory, with the dependencies in `example.toml` installed:

```bash
python main.py --num-sensors 4 --num-frames 300 --output /tmp/lidar-4.json
```

Defaults are one lidar at 10 Hz, 100 warmup frames, and 1,000 measured frames read back on the CPU.

Use `--help` for the full command line, including `--coords-type`, `--frame-of-reference`,
`--include-invalid-points`, `--motion-bvh`, `--readback`, and `--scene`.

## Results

A run writes its measured frame times and configuration to the JSON file selected by `--output`.
The default is `isaacsim-benchmarks/lidar_result.json` under the system temporary directory;
`ISAACSIM_BENCHMARK_OUTPUT_DIR` overrides that default directory. The renderer log,
`lidar_ovrtx.log`, is written beside the result.

Use the examples runner’s `--nsys` option before the benchmark ID to record the measured
phase (`nsys` must be on `PATH`). `--nsys-scope full` includes startup and warmup;
`--nsys-output PATH` selects the report path or prefix.

`--print-stage` prints the composed USDA and exits without creating a renderer. `--export-stage`
writes the same layer to `lidar_stage.usda` beside the result, or to the supplied path,
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
