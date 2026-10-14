..
   SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
   SPDX-License-Identifier: Apache-2.0

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

   http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.

.. _isaacsim-performance-library:

======================
isaacsim-performance
======================

The ``isaacsim-performance`` distribution (CMake group ``isaacsim_performance``)
provides standalone, Kit-independent benchmark authoring and execution through
``isaacsim.performance.benchmarking``. Use the ``benchmark`` facade to declare
operation, subsystem, and workflow benchmarks, collect named measurements, and
write structured JSON results.

Operation benchmarks calibrate their batch size before measurement. Subsystem
and workflow benchmarks run one unit of work per sample or frame. Every scope
supports warmup, deterministic random seeds, synchronous or asynchronous
targets, lifecycle hooks, validation, and optional process, GPU, and simulation
metrics.

.. code-block:: python

    from isaacsim.performance.benchmarking import benchmark

    @benchmark.operation(
        "isaacsim.example.increment",
        warmup_samples=2,
        measured_samples=10,
    )
    def increment(value: int = 1) -> int:
        return value + 1


    result = benchmark.run(increment, save=False)
    print(result["status"]["valid"])

Set ``ISAACSIM_BENCHMARK_OUTPUT_DIR`` to choose the default result root, pass an
explicit path with ``output``, or use ``save=False`` for in-memory results.
Request the ``process``, ``gpu``, or ``simulation`` metric profile from a
benchmark decorator. GPU metrics require an NVIDIA GPU visible to NVIDIA Management
Library (NVML); the builtin provider also requires the ``nvidia-ml-py`` runtime package.

.. _isaacsim-performance-resource-collection:

Resource collection
===================

``benchmark.run()`` and ``benchmark.run_async()`` accept ``resource_backend``:

* ``"auto"`` (default) uses ``ovperf-collector==0.3.1`` when installed on Linux
  x86_64. If the package is absent, it quietly uses the builtin process and
  Python NVML providers. An installed collector that fails to import or initialize
  makes the run invalid.
* ``"ovperf"`` requires the supported native collector. A missing package, version
  mismatch, or unavailable requested GPU observation makes the run invalid; it does
  not silently select another provider.
* ``"builtin"`` selects this library's process and Python NVML providers,
  including on Linux.

The native collector is an optional ``isaacsim-performance[ovperf]`` dependency
on Linux x86_64; public runtime installations do not require its internal wheel.
Internal CI installs the released collector explicitly. Benchmarks without
process or GPU profiles skip resource queries unless capture is requested; the
ovperf backend still initializes its native domain for tracing.

Both providers use the same benchmark lifecycle, timing, statistics, and metric
reduction rules. CPU time retains ``time.process_time()`` precision. Resource
observations are acquired outside each sample's stopwatch. GPU utilization uses
NVML utilization; device memory includes NVML reserved memory, matching the
builtin provider's accounting. Unavailable optional RSS or process VRAM values
are omitted. Process activity is independent of VRAM availability. Unknown GPU
activity makes the run invalid because device metrics cannot be attributed reliably.
When both boundaries confirm that the process is inactive on every GPU, process
VRAM is zero and device metrics are omitted. Incomplete GPU discovery fails before
calibration or warmup; benchmarks without GPU metrics do not require a GPU.

``device_gpu_utilization`` and ``device_vram_used_mib`` can include activity and
allocations from other processes on the selected GPUs, so control background load
before benchmarking.

By default, native collection uses fresh boundary snapshots without a periodic
sampler. Set ``capture_collector=True`` to additionally record the measured phase
and write a sibling JSON artifact. This requires ``save=True`` and the native
backend.

.. _isaacsim-performance-collector-example:

Run and inspect a capture
=========================

This example runs on Linux x86_64 with the optional ovperf collector installed.
Replace the sleep with completed application work; the runner does not
synchronize GPU work on the application's behalf.

.. code-block:: python

   import time
   from isaacsim.performance.benchmarking import benchmark

   @benchmark.subsystem(
       "example.step", metrics=("process",), warmup_samples=1, measured_samples=3
   )
   def step():
       with benchmark.measure("update"):
           time.sleep(0.01)
       benchmark.record("items", 1, "count")

   result = benchmark.run(
       step, resource_backend="ovperf", capture_collector=True, output="result.json"
   )
   if not result["status"]["valid"]:
       raise RuntimeError(result["status"]["errors"])
   collector_artifact = result["environment"]["collector"]["artifact"]
   print(collector_artifact)

Add ``"gpu"`` to ``metrics`` when GPU observations are required. In an existing
event loop, use ``await benchmark.run_async(...)`` with the same options.
To compare with the builtin providers, run the same target with
``resource_backend="builtin"`` and omit ``capture_collector``.

The capture example writes:

.. code-block:: text

   result.json                         # KPI samples, statistics, named regions, status
   result.<unique-id>.collector.json    # native history and exact boundary observations

``result.json`` contains three measured samples; the warmup is excluded.
``metrics.items`` contains the recorded values, and ``regions.update`` contains
the named timing region. Repeated ``benchmark.record()`` calls with the same name
and unit are averaged within each sample. Nested ``benchmark.measure()`` names
form paths such as ``update/physics`` in ``regions``; they do not create files.

.. _isaacsim-performance-collector-output:

Result and capture files
========================

Results use ``isaacsim.performance.benchmarking.result.v3``. The field layout is
unchanged; the version distinguishes corrected CPU boundaries and GPU attribution
from historical v2 results. Consumers that validate the schema must accept v3.
The optional artifact's basename is stored in ``environment.collector.artifact``;
keep it beside the result when moving files. Its
``isaacsim.performance.benchmarking.collector.v1`` document contains the native
run and the exact before/after inputs used for each result sample. Fresh snapshot
timestamps use absolute monotonic seconds; native run timestamps are relative to
collector initialization. Do not substitute periodic averages for the result's
boundary-based metrics.

When GPU metrics are enabled, collector artifacts can contain GPU memory series
for other NVML-visible PIDs; more recorded process series and longer captures can
increase artifact size.

The ``before.process_cpu_seconds`` and ``after.process_cpu_seconds`` values bracket
the workload stopwatch; their attached resource snapshots are taken outside that
interval. CPU KPIs exclude synchronous resource queries for both backends. They
include CPU consumed by all process threads during the workload, including the
periodic sampler when capture is enabled. Compare runs with the same capture setting.

Each captured run writes a separate artifact. Overwriting an explicit result path
retains earlier artifacts so readers holding the previous result can still open
them. Prefer the unique default result paths when retaining run history, and
remove old results and artifacts together according to your retention policy.

.. _isaacsim-performance-collector-tracing:

Tracing and collector ownership
===============================

When selected, the ovperf backend emits native NVTX ranges
for ``MEASURE`` and each ``sample/<index>`` in the
``isaacsim.performance.benchmarking`` domain, including with the default
``metrics=()``. Timing-only runs initialize the native domain without querying
resources or starting periodic collection. The builtin backend does not emit
these ranges.

The runner owns initialization for one run and rejects an already initialized
collector; close an application's existing collector before requesting the
native benchmark provider.

NVTX is enabled automatically under ``nsys profile``. This is independent of
``capture_collector``: Nsight can show the ranges with periodic capture disabled.
The runner does not mirror ``benchmark.record()`` values as NVTX counters or
``benchmark.measure()`` regions as NVTX ranges. Both remain in ``result.json``.

Use one ``benchmark.run()`` call per result. Its optional capture covers the
MEASURE phase and ends before validation and teardown. Do not start a second
collector inside the benchmark to create sessions or exports.

See the `collector usage guide
<https://github.com/NVIDIA-Omniverse/ovperf/blob/main/packages/ovperf-collector/docs/usage.md>`_
for direct collector sessions, scoped exports, custom NVTX counters, and timing
helpers when using the collector independently of this runner.
