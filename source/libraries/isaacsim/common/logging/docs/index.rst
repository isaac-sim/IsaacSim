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

.. _isaacsim-common-logging:
.. _isaacsim-common-logging-overview:

=======================
isaacsim.common.logging
=======================

``isaacsim.common.logging`` is a thin C, C++, and Python facade over the private, statically linked Carbonite logging
backend. Each feature library owns a lightweight logger with one immutable channel. Carbonite performs severity
filtering, rendering, destination routing, and asynchronous delivery.

Logger construction only caches its channel and does not select a backend. A native host may therefore attach its
Carbonite interface after feature loggers are constructed. The first record submission, admission query, or global
configuration operation selects the standalone backend when no host has attached. Host attachment must precede those
operations when the application owns Carbonite.

The standalone backend disables absolute timestamps and enables a millisecond elapsed-time index on Carbonite's
default logger. The elapsed index starts when that logger emits its first message. The application may subsequently
change process-wide Carbonite settings through ``configure_global``. The unconditional standard-output line produced
by ``report()`` contains the logger channel and caller's raw message; only its Carbonite copy receives the remaining
backend formatting.

The facade deliberately does not provide a second callback registry or filtering engine. It translates four delivered
severity levels, preserves provided source locations, and forwards already formatted messages to Carbonite. C++ macros
provide source locations automatically; Python callers opt in per submission with ``capture_source=True``. Its global
configuration object patches Carbonite's process-wide and per-channel settings rather than maintaining a second
configuration store. Backend unavailability makes log submission a no-op, causes admission queries to return false,
and does not affect feature-library error reporting.

.. _isaacsim-common-logging-configuration-ownership:

Global configuration ownership
------------------------------

Configure global logging only from the application or service entry point. Create one ``Logger`` in each feature
library, and leave process policy unchanged. Every optional C++ and Python configuration member, and every unselected
C field-mask bit, means "leave the current Carbonite setting unchanged." This lets you change the file destination
without resetting console formatting or a host's existing severity policy.

The configurable surface includes process admission, asynchronous delivery, standard-stream and debug-console
destinations, file path and append policy, destination thresholds and flush policy, absolute or elapsed timestamps,
source location fields, thread/process/trace identifiers, color output, and multi-process serialization. File paths
may be absolute or relative to the working directory at configuration time and may contain Carbonite's ``${pid}``
placeholder. Create parent directories before configuring a file destination.

Applications can patch channel enablement and minimum severity with explicit ``override`` or ``inherit`` behavior.
Inheritance clears the Carbonite per-channel setting and resumes using the process-wide policy. Feature loggers remain
immutable channel handles and do not own filtering state.

The facade serializes configuration calls with one another but does not stop concurrent logging. Carbonite applies
individual settings synchronously; there is no atomic multi-setting transaction. Carbonite's file configuration
setters do not report file-open failures, so a successful configuration result means the backend accepted the
settings, not that it opened a path successfully. Call ``flush()`` before shutdown or before consuming a log file.

Callback dispatch follows Carbonite's contract because the facade does not add a callback layer. With asynchronous
delivery, a record submitted by a Carbonite callback is queued after the current callback and can be delivered to that
callback later. Callback implementations that submit records must prevent unbounded record generation.

.. _isaacsim-common-logging-python-usage:

Python usage
------------

Create a logger at feature-library initialization time and retain it for that library's lifetime:

.. code-block:: python

    from isaacsim.common.logging import GlobalLoggingConfig, LogLevel, Logger, configure_global, flush

    config = GlobalLoggingConfig()
    config.file_path = "logs/isaac-sim-${pid}.log"
    config.file_append = True
    config.file_level = LogLevel.INFO
    configure_global(config)

    logger = Logger("isaacsim.simulation")
    logger.info("Simulation initialized")
    logger.warning("Fallback selected", capture_source=True)

    result = 6 * 7
    logger.report(f"Calculation result: {result}")

    # Use only when the application needs an explicit process-wide drain.
    flush()

The logger needs no explicit shutdown. ``flush()`` is process-wide because Carbonite, not an individual facade
instance, owns pending records.

``report()`` is for result-style output that must remain visible regardless of Carbonite's configured log level. It
synchronously writes and flushes ``[channel] message`` to standard output, then submits the unprefixed message to
Carbonite at ``INFO``.
The Carbonite submission remains subject to Carbonite's output policy. If Carbonite also routes the ``INFO`` record to
the console, the message can appear there a second time.

C++ formatting macros accept an explicit logger, use ``{fmt}`` syntax, and preserve source file, function, and line
metadata. The warning-once and deprecation-once macros are thread-safe and submit at most one admitted record per call
site. Logging is supplemental: disabling it does not change feature-library results or last-error behavior.

See :doc:`api_c`, :doc:`api_cpp`, and :doc:`api_python` for language-specific references.

.. toctree::
    :maxdepth: 2

    api_c
    api_cpp
    api_python
