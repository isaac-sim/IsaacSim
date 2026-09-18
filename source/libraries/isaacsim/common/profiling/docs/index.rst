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

.. _isaacsim-common-profiling:
.. _isaacsim-common-profiling-overview:

==========================
isaacsim.common.profiling
==========================

``isaacsim.common.profiling`` provides C, C++, and Python APIs for adding profiling
zones and events to Kit-independent libraries. The same instrumentation works with
the profiler supplied by Kit or with the package-owned Carbonite NVTX profiler.

.. _isaacsim-common-profiling-cpp:

C++ usage
---------

Include ``Profiling.hpp`` and use the convenience macros when event arguments should
only be evaluated while profiling is enabled:

.. code-block:: cpp

   ISAACSIM_COMMON_PROFILE_ZONE("simulation/step");
   ISAACSIM_COMMON_PROFILE_VALUE("simulation/body_count", bodyCount);
   ISAACSIM_COMMON_PROFILE_INSTANT("simulation/ready");
   ISAACSIM_COMMON_PROFILE_FLOW_BEGIN(taskId, "simulation/task");
   runTask();
   ISAACSIM_COMMON_PROFILE_FLOW_END(taskId);
   ISAACSIM_COMMON_PROFILE_FRAME("simulation/frame");

The ``_MASK`` variants select explicit Carbonite capture-mask bits. Zone names are
arbitrary lazy C++ expressions; formatting is left to the caller, so the profiling
API does not impose a formatting-library dependency. Prefer stable names and record
changing numeric data with ``ISAACSIM_COMMON_PROFILE_VALUE``.

.. _isaacsim-common-profiling-python:

Python usage
------------

Start the standalone NVTX profiler before running instrumented work and always stop
it during cleanup:

.. code-block:: python

   import isaacsim.common.profiling as profiling

   profiling.start()
   try:
       with profiling.zone("simulation/step"):
           run_simulation_step()
       profiling.frame("simulation/frame")
   finally:
       profiling.stop()

``start()`` raises :class:`RuntimeError` if a standalone session is already active,
if Kit or another application has attached a profiler, or if the packaged NVTX host
cannot start. ``stop()`` only stops a session created by ``start()`` and is otherwise
a no-op. Code running in Kit uses the application profiler automatically and should
not call ``start()`` or ``stop()``.

Use stable hierarchical event names. Put changing numbers in ``value()`` events and
use flow identifiers to correlate work across threads. When no profiler admits an
event, the event helpers return without emitting it. Callable Python zone names and
lazy C++ zone names are evaluated only when profiling is enabled.

.. _isaacsim-common-profiling-application-hosts:

Application hosts
-----------------

An application that already owns a Carbonite ``IProfiler`` can attach it through
``ProfilingHost.h``. The returned token owns that attachment. The application must
keep the profiler alive until it detaches the token. Detachment prevents new event
admission, waits for active calls, and closes any remaining facade zones before
returning.

.. toctree::
    :maxdepth: 2

    api_c
    api_cpp
    api_python
