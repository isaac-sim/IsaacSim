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
benchmark decorator. GPU metrics require the ``nvidia-ml-py`` runtime package
and an NVIDIA GPU that is visible to NVIDIA Management Library (NVML).
