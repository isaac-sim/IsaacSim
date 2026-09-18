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

.. _isaacsim-common-logging-api-cpp:

=========
C++ guide
=========

.. isaacsim-libraries-api-guide-start

Include ``isaacsim/common/logging/Logging.hpp`` and link ``isaacsim::common-logging``. Store one ``Logger`` for each
feature-library channel:

.. code-block:: cpp

    #include <isaacsim/common/logging/Logging.hpp>

    using isaacsim::common::logging::LogLevel;
    using isaacsim::common::logging::Logger;

    Logger logger("isaacsim.simulation");
    logger.log(LogLevel::eInfo, "Simulation initialized");

``Logger`` copies its non-empty channel during construction without selecting a backend, allowing a native host to
attach after feature loggers are created. Records have four delivered severities: ``eVerbose``, ``eInfo``,
``eWarning``, and ``eError``. Carbonite owns filtering and output policy; the facade has no callback registry,
per-logger policy, or last-error state.

The C++ macros use the pinned header-only ``{fmt}`` dependency, submit without a second message copy, and preserve
source location metadata:

.. code-block:: cpp

    ISAACSIM_LOG_INFO(logger, "Loaded {} objects", objectCount);
    ISAACSIM_LOG_WARN(logger, "Falling back to backend {}", backendName);
    ISAACSIM_LOG_WARN_ONCE(logger, "The fallback backend is active");
    ISAACSIM_LOG_DEPRECATION_ONCE(logger, "old_api() is deprecated; use new_api() instead");

``ISAACSIM_LOG_WARNING_ONCE`` and its ``ISAACSIM_LOG_WARN_ONCE`` alias submit at most one admitted record for each
macro call site, including when multiple threads reach that site concurrently. A filtered call does not consume the
call site's record. ``ISAACSIM_LOG_DEPRECATION_ONCE`` provides the same behavior at ``WARNING`` severity and leaves the
deprecation message unchanged. All formatting macros check admission before evaluating their formatting arguments.
Use ``Logger::isEnabled`` when expensive work must happen before the macro call or when calling ``Logger::log``
directly.

For code that already owns source metadata, pass ``SourceLocation`` to ``Logger::log`` or ``Logger::report`` directly:

.. code-block:: cpp

    using isaacsim::common::logging::SourceLocation;

    logger.log(LogLevel::eInfo, message, SourceLocation{ sourceFile, sourceFunction, sourceLine });

The logger borrows the three pointers only for the call. A default-constructed ``SourceLocation`` omits the metadata.

Use ``report`` for result-style output that must remain visible regardless of Carbonite's configured log level. The
formatted macro also preserves source metadata:

.. code-block:: cpp

    logger.report("Calculation complete");
    ISAACSIM_REPORT(logger, "Calculation result: {:.2f}", result);

Both forms synchronously write and flush ``[channel] message`` to standard output, then submit the unprefixed message
to Carbonite at ``INFO``. Carbonite still controls admission and routing of the log record, so console routing of
``INFO`` may display the message a second time.

Formatting failures are ignored because logging is supplemental. Callers that need compile-time format-string
checking with the pinned fmt 7 dependency can wrap a literal with ``FMT_STRING``.

.. _isaacsim-common-logging-cpp-configuration:

Application-level configuration
================================

``GlobalLoggingConfiguration`` is an application-level patch over Carbonite's process-wide default logger and named
channels.
Empty optionals leave current scalar settings unchanged, so applications can update one destination without resetting
the others:

.. code-block:: cpp

    using isaacsim::common::logging::ConfigureResult;
    using isaacsim::common::logging::GlobalLoggingConfiguration;
    using isaacsim::common::logging::configureGlobalLogging;

    GlobalLoggingConfiguration configuration;
    configuration.filePath = "logs/isaac-sim-${pid}.log";
    configuration.fileAppend = true;
    configuration.fileLevel = LogLevel::eInfo;
    if (configureGlobalLogging(configuration) != ConfigureResult::eSuccess)
    {
        // Invalid configuration or unavailable backend.
    }

An empty ``filePath`` string disables file logging. Create parent directories before configuring the destination.
Carbonite resolves relative paths from the working directory at configuration time and does not report file-open
failures through its configuration interface. Configure this process-wide policy only from the application entry
point.

Applications can also patch Carbonite's per-channel enablement and threshold. ``eOverride`` applies the supplied
value, ``eInherit`` clears the corresponding channel override, and ``eUnchanged`` leaves it untouched:

.. code-block:: cpp

    using isaacsim::common::logging::ChannelLoggingConfiguration;
    using isaacsim::common::logging::ChannelSettingBehavior;

    ChannelLoggingConfiguration channel;
    channel.channel = "isaacsim.sensors.camera";
    channel.enabledBehavior = ChannelSettingBehavior::eOverride;
    channel.enabled = false;

    GlobalLoggingConfiguration configuration;
    configuration.channels.push_back(channel);
    configureGlobalLogging(configuration);

Enablement and minimum-level behaviors are independent. Carbonite performs the filtering; ``Logger`` does not keep a
local policy copy.

Logging calls are ``noexcept`` and return ``void``. Use ``isaacsim::common::logging::flush()`` only when the application
needs to drain Carbonite's process-wide logging backend explicitly. Logger destruction does not flush global state.

The declarations and Doxygen comments in ``Logging.hpp`` are the authoritative C++ reference.
