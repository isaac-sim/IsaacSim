<!--
SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
SPDX-License-Identifier: Apache-2.0
-->

# RTX Radar Benchmark

This benchmark measures what `ovrtx` costs per frame when it runs a ring of RTX radars. It places
`--num-sensors` `OmniRadar` prims around an authored environment of ground, lights, and obstacles,
steps their `PointCloud` render products for `--num-frames` frames, and maps the returned detections
back each frame.

Every sensor benchmark shares the same scene and command line, and all of them are measured by
`isaacsim.performance.benchmarking`, so a radar run is directly comparable to a camera, lidar, or
depth-sensor run.

## Scan cadence

A radar emits detections at its own scan rate, so a step only produces a frame once a scan
completes. With the default 10 Hz rate the generic WPM DMAT model is specified at and the default
1/60 s step, that is one frame every six steps, so keep `--num-frames` well above six or the run
will not cover a whole scan. Raising `--frame-rate` degenerates the model's scan pattern — the
sensor keeps emitting frames but they carry no detections — so the run fails rather than saving a
profile of empty work.

## Run

From this directory, with the dependencies in `example.toml` installed:

```bash
python main.py --num-sensors 4 --num-frames 300 --output /tmp/radar-4.json
```

Defaults are one radar at 10 Hz, 100 warmup frames, and 1,000 measured frames read back on the CPU.

Use `--help` for the full command line, including `--coords-type`, `--frame-of-reference`,
`--aux-output-type`, `--motion-bvh`, `--readback`, and `--scene`.

## Results

A run writes its measured frame times and configuration to the JSON file selected by `--output`.
The default is `isaacsim-benchmarks/radar_result.json` under the system temporary directory;
`ISAACSIM_BENCHMARK_OUTPUT_DIR` overrides that default directory. The renderer log,
`radar_ovrtx.log`, is written beside the result.

Use the examples runner’s `--nsys` option before the benchmark ID to record the measured
phase (`nsys` must be on `PATH`). `--nsys-scope full` includes startup and warmup;
`--nsys-output PATH` selects the report path or prefix.

`--print-stage` prints the composed USDA and exits without creating a renderer. `--export-stage`
writes the same layer to `radar_stage.usda` beside the result, or to the supplied path,
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
