# Benchmarks

Measure simulation, control, and sensor workloads with
`isaacsim.performance.benchmarking`. Build the libraries, then run a workload:

```bash
source/libraries/build.sh -r
source/examples/examples.sh run benchmarks.robot_motion.controllers.differential_drive -- \
    --physics-device cpu --output /tmp/differential-cpu.json
```

See the [Benchmarks](../../../docs/isaacsim_libraries/benchmarks.rst) for
measurement conventions, repeated trials, result interpretation, and Nsight capture.
Each workload's README describes its completed frame, options, and limitations;
its `example.toml` declares the runner ID, dependencies, and smoke tests.

Simulation workloads default to a 0.005 s timestep, 100 excluded warmup frames,
1,000 measured frames, and rendering off. ovrtx sensor workloads use the same
warmup and sample counts, with a 1/60 s render interval. Check their README before
comparing results. Preserve raw JSON and record the hardware, dependencies,
configuration, and cache state alongside each run.

## Authoring

Use the existing workflow lifecycle: `setup`, `run`, `validate`, and `teardown`.
Keep initialization outside timing, complete one defined operation in `run`, and
validate that useful work occurred. Declare public dependencies and smoke tests in
`example.toml`; test correctness rather than machine-dependent FPS thresholds.

See the [benchmarking API](../../libraries/isaacsim/performance/docs/index.rst).
