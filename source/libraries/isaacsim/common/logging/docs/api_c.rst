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

.. _isaacsim-common-logging-api-c:

========
C guide
========

.. isaacsim-libraries-api-guide-start

Include ``isaacsim/common/logging/Logging.h`` and link ``isaacsim::common-logging``. The C API is a stateless facade
over the same private Carbonite backend used by C++ and Python:

.. code-block:: c

    #include <isaacsim/common/logging/Logging.h>

    ISAACSIM_COMMON_LOGGING_INFO("isaacsim.simulation", "Simulation initialized");

The four convenience macros submit ``VERBOSE``, ``INFO``, ``WARNING``, or ``ERROR`` records and automatically preserve
``__FILE__``, ``__func__``, and ``__LINE__``. Messages must already be formatted. Call
``isaacsimCommonLoggingWrite`` directly when explicit source metadata is required.

Use ``ISAACSIM_COMMON_LOGGING_REPORT`` for output that must be visible regardless of Carbonite's configured log level:

.. code-block:: c

    ISAACSIM_COMMON_LOGGING_REPORT("isaacsim.simulation", "Calculation result: 42");

It synchronously writes and flushes ``[channel] message`` to standard output, then submits the unprefixed message to
Carbonite at ``INFO`` with source metadata. Carbonite still controls admission and routing of the log record, so
console routing of ``INFO`` may display the message a second time. Call ``isaacsimCommonLoggingReport`` directly to
provide explicit source metadata.

The channel must be a non-empty null-terminated string, and the message must be a null-terminated string. The facade
ignores invalid input because logging is supplemental and does not introduce a second result or last-error system.
Source file and function pointers may be null. The call does not retain input strings.

.. _isaacsim-common-logging-c-configuration:

Application-level configuration
================================

Initialize a patch with ``ISAACSIM_COMMON_LOGGING_GLOBAL_CONFIG_INIT``, select fields, and apply it once at application
startup. Unselected fields leave the current process-wide Carbonite setting unchanged:

.. code-block:: c

    IsaacSimCommonLoggingGlobalConfig configuration = ISAACSIM_COMMON_LOGGING_GLOBAL_CONFIG_INIT;
    configuration.fields = ISAACSIM_COMMON_LOGGING_CONFIG_FILE_PATH |
                    ISAACSIM_COMMON_LOGGING_CONFIG_FILE_APPEND |
                    ISAACSIM_COMMON_LOGGING_CONFIG_FILE_LEVEL;
    configuration.filePath = "logs/isaac-sim-${pid}.log";
    configuration.fileAppend = 1;
    configuration.fileLevel = ISAACSIM_COMMON_LOGGING_LOG_INFO;

    if (isaacsimCommonLoggingConfigureGlobal(&configuration) != ISAACSIM_COMMON_LOGGING_CONFIGURE_SUCCESS)
    {
        /* Invalid configuration or unavailable backend. */
    }

Select ``FILE_PATH`` with a null or empty path to disable file logging. Create parent directories before configuring
the destination. Carbonite resolves relative paths from the working directory at configuration time and does not
report file-open failures through this API. Configure this global policy only from the application entry point.

Use ``CHANNELS`` to override Carbonite's process-wide enablement or threshold for specific channels:

.. code-block:: c

    IsaacSimCommonLoggingChannelConfig channel = ISAACSIM_COMMON_LOGGING_CHANNEL_CONFIG_INIT;
    channel.channel = "isaacsim.sensors.camera";
    channel.enabledBehavior = ISAACSIM_COMMON_LOGGING_CHANNEL_SETTING_OVERRIDE;
    channel.enabled = 0;

    IsaacSimCommonLoggingGlobalConfig configuration = ISAACSIM_COMMON_LOGGING_GLOBAL_CONFIG_INIT;
    configuration.fields = ISAACSIM_COMMON_LOGGING_CONFIG_CHANNELS;
    configuration.channelConfigs = &channel;
    configuration.channelConfigCount = 1;
    isaacsimCommonLoggingConfigureGlobal(&configuration);

Set a behavior to ``INHERIT`` to clear that channel override and resume using the process-wide setting. Set it to
``UNCHANGED`` to leave the corresponding Carbonite setting untouched. Enablement and minimum-level behaviors are
independent, and Carbonite performs the filtering.

The field mask covers process enablement and minimum level, asynchronous delivery, standard stream and Windows debug
console settings, file settings, timestamp and elapsed-time rendering, source metadata, identifiers, color output, and
multi-process serialization, plus named-channel enablement and thresholds. The declarations in ``Logging.h`` provide
the complete field inventory.

``isaacsimCommonLoggingFlush`` drains Carbonite's process-wide backend. The C API has no logger handles, callback
registry, local filtering engine, or per-instance lifecycle.

.. _isaacsim-common-logging-api-c-host-integration:

Host integration
================

``LoggingHost.h`` is reserved for native host adapters. A host that already owns Carbonite attaches its existing
``carb::logging::ILogging`` pointer before any logging operation selects the standalone backend and retains the
returned ownership token. Loggers may be constructed before attachment. Detaching that token waits for active
submissions and removes this library's channels without releasing the Carbonite framework. Feature libraries should
include only ``Logging.h`` and must not call this integration API.

The declarations and Doxygen comments in ``Logging.h`` and ``LoggingHost.h`` are the authoritative C reference.
