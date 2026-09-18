// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "isaacsim/common/logging/Logging.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/string_view.h>
#include <nanobind/stl/vector.h>

#include <cstddef>
#include <string>
#include <string_view>

namespace nb = nanobind;
using namespace isaacsim::common::logging;

namespace
{

struct PythonSourceLocation
{
    SourceLocation get() const noexcept
    {
        return { file.empty() ? nullptr : file.c_str(), function.empty() ? nullptr : function.c_str(), line };
    }

    std::string file;
    std::string function;
    unsigned int line{ 0 };
};

std::string getUnicodeAttribute(PyObject* object, const char* name)
{
    PyObject* attribute = PyObject_GetAttrString(object, name);
    if (attribute == nullptr)
    {
        PyErr_Clear();
        return {};
    }

    Py_ssize_t size = 0;
    const char* value = PyUnicode_AsUTF8AndSize(attribute, &size);
    std::string result;
    if (value != nullptr)
    {
        result.assign(value, static_cast<std::size_t>(size));
    }
    else
    {
        PyErr_Clear();
    }
    Py_DECREF(attribute);
    return result;
}

PythonSourceLocation capturePythonSourceLocation()
{
    PythonSourceLocation location;
    PyFrameObject* frame = PyEval_GetFrame();
    if (frame == nullptr)
    {
        return location;
    }

    const int line = PyFrame_GetLineNumber(frame);
    if (line > 0)
    {
        location.line = static_cast<unsigned int>(line);
    }
    else if (PyErr_Occurred())
    {
        PyErr_Clear();
    }

    PyCodeObject* code = PyFrame_GetCode(frame);
    if (code != nullptr)
    {
        PyObject* codeObject = reinterpret_cast<PyObject*>(code);
        location.file = getUnicodeAttribute(codeObject, "co_filename");
        location.function = getUnicodeAttribute(codeObject, "co_name");
        Py_DECREF(codeObject);
    }
    else
    {
        PyErr_Clear();
    }
    return location;
}

void logWithReleasedGil(const Logger& logger, LogLevel severity, std::string_view message, bool captureSource)
{
    if (!logger.isEnabled(severity))
    {
        return;
    }
    const PythonSourceLocation location = captureSource ? capturePythonSourceLocation() : PythonSourceLocation{};
    nb::gil_scoped_release release;
    logger.log(severity, message, location.get());
}

void reportWithReleasedGil(const Logger& logger, std::string_view message, bool captureSource)
{
    const bool shouldCaptureSource = captureSource && logger.isEnabled(LogLevel::eInfo);
    const PythonSourceLocation location = shouldCaptureSource ? capturePythonSourceLocation() : PythonSourceLocation{};
    nb::gil_scoped_release release;
    logger.report(message, location.get());
}

ConfigureResult configureGlobalWithReleasedGil(const GlobalLoggingConfiguration& configuration)
{
    const GlobalLoggingConfiguration snapshot = configuration;
    nb::gil_scoped_release release;
    return configureGlobalLogging(snapshot);
}

} // namespace

NB_MODULE(_bindings, module)
{
    module.doc() = "Channel-oriented logging and application-level Carbonite configuration.";

    nb::enum_<LogLevel>(module, "LogLevel", "Severity of a submitted log record.")
        .value("VERBOSE", LogLevel::eVerbose, "Detailed diagnostic information.")
        .value("INFO", LogLevel::eInfo, "Informational messages.")
        .value("WARNING", LogLevel::eWarning, "Potential problems that allow continued operation.")
        .value("ERROR", LogLevel::eError, "Failures that prevented an operation from completing.");

    nb::enum_<ConfigureResult>(module, "ConfigureResult", "Result of applying logging configuration.")
        .value("SUCCESS", ConfigureResult::eSuccess, "The selected settings were accepted.")
        .value("INVALID_ARGUMENT", ConfigureResult::eInvalidArgument, "A supplied setting was invalid.")
        .value("BACKEND_UNAVAILABLE", ConfigureResult::eBackendUnavailable, "The native backend is unavailable.");

    nb::enum_<OutputStream>(module, "OutputStream", "Selection of the standard logging stream.")
        .value("DEFAULT", OutputStream::eDefault, "Use stdout for lower severities and stderr for errors.")
        .value("STDERR", OutputStream::eStderr, "Send all standard-stream records to stderr.");

    nb::enum_<ElapsedTimeUnit>(module, "ElapsedTimeUnit", "Unit used for the elapsed-time prefix.")
        .value("DISABLED", ElapsedTimeUnit::eDisabled, "Do not include elapsed time.")
        .value("MILLISECONDS", ElapsedTimeUnit::eMilliseconds, "Include elapsed time in milliseconds.")
        .value("MICROSECONDS", ElapsedTimeUnit::eMicroseconds, "Include elapsed time in microseconds.")
        .value("NANOSECONDS", ElapsedTimeUnit::eNanoseconds, "Include elapsed time in nanoseconds.");

    nb::enum_<ChannelSettingBehavior>(
        module, "ChannelSettingBehavior", "Action to apply to one channel setting in a configuration patch.")
        .value("UNCHANGED", ChannelSettingBehavior::eUnchanged, "Leave the existing channel setting unchanged.")
        .value("INHERIT", ChannelSettingBehavior::eInherit, "Clear the override and inherit the global setting.")
        .value("OVERRIDE", ChannelSettingBehavior::eOverride, "Apply the supplied channel value.");

    nb::class_<ChannelLoggingConfiguration>(module, "ChannelLoggingConfig",
                                            R"doc(Patch-style settings for one logging channel.

Example:

.. code-block:: python

    >>> from isaacsim.common.logging import ChannelLoggingConfig

    >>> channel = ChannelLoggingConfig()
    >>> channel.channel = "isaacsim.sensors.camera"
)doc")
        .def(nb::init<>())
        .def_rw("channel", &ChannelLoggingConfiguration::channel, "The non-empty logging channel.")
        .def_rw("enabled_behavior", &ChannelLoggingConfiguration::enabledBehavior, "The action to apply to enabled.")
        .def_rw("enabled", &ChannelLoggingConfiguration::enabled, "The enabled value used by OVERRIDE.")
        .def_rw("minimum_level_behavior", &ChannelLoggingConfiguration::minimumLevelBehavior,
                "The action to apply to minimum_level.")
        .def_rw("minimum_level", &ChannelLoggingConfiguration::minimumLevel, "The threshold used by OVERRIDE.");

    nb::class_<GlobalLoggingConfiguration>(module, "GlobalLoggingConfig",
                                           R"doc(Application-level logging configuration patch.

An attribute set to ``None`` leaves the corresponding scalar setting unchanged.

Example:

.. code-block:: python

    >>> from isaacsim.common.logging import GlobalLoggingConfig, LogLevel

    >>> config = GlobalLoggingConfig()
    >>> config.minimum_level = LogLevel.WARNING
)doc")
        .def(nb::init<>())
        .def_rw("enabled", &GlobalLoggingConfiguration::enabled, "Whether process-wide logging is enabled.")
        .def_rw("minimum_level", &GlobalLoggingConfiguration::minimumLevel, "The process-wide admission threshold.")
        .def_rw("asynchronous", &GlobalLoggingConfiguration::asynchronous, "Whether backend delivery is asynchronous.")
        .def_rw("standard_stream_enabled", &GlobalLoggingConfiguration::standardStreamEnabled,
                "Whether stdout and stderr output is enabled.")
        .def_rw(
            "standard_stream_level", &GlobalLoggingConfiguration::standardStreamLevel, "The standard-stream threshold.")
        .def_rw("standard_stream_flush", &GlobalLoggingConfiguration::standardStreamFlush,
                "Whether stdout is flushed after every record.")
        .def_rw("output_stream", &GlobalLoggingConfiguration::outputStream,
                "Default stdout/stderr routing or forced stderr output.")
        .def_rw("debug_console_enabled", &GlobalLoggingConfiguration::debugConsoleEnabled,
                "Whether the Windows debug console destination is enabled.")
        .def_rw("debug_console_level", &GlobalLoggingConfiguration::debugConsoleLevel,
                "The Windows debug-console threshold.")
        .def_rw("file_path", &GlobalLoggingConfiguration::filePath, "Log path; an empty string disables file logging.")
        .def_rw(
            "file_append", &GlobalLoggingConfiguration::fileAppend, "Whether file output appends instead of replacing.")
        .def_rw("file_level", &GlobalLoggingConfiguration::fileLevel, "The file-destination threshold.")
        .def_rw("file_flush_level", &GlobalLoggingConfiguration::fileFlushLevel,
                "The minimum severity that flushes the file.")
        .def_rw("filename_included", &GlobalLoggingConfiguration::filenameIncluded,
                "Whether source filenames are included.")
        .def_rw("line_number_included", &GlobalLoggingConfiguration::lineNumberIncluded,
                "Whether source line numbers are included.")
        .def_rw("function_name_included", &GlobalLoggingConfiguration::functionNameIncluded,
                "Whether source function names are included.")
        .def_rw("timestamp_included", &GlobalLoggingConfiguration::timestampIncluded,
                "Whether absolute timestamps are included.")
        .def_rw("utc_timestamps", &GlobalLoggingConfiguration::utcTimestamps, "Whether absolute timestamps use UTC.")
        .def_rw("microsecond_timestamps", &GlobalLoggingConfiguration::microsecondTimestamps,
                "Whether absolute timestamps use microsecond precision.")
        .def_rw("elapsed_time", &GlobalLoggingConfiguration::elapsedTime, "The elapsed-time prefix unit.")
        .def_rw("thread_id_included", &GlobalLoggingConfiguration::threadIdIncluded,
                "Whether thread identifiers are included.")
        .def_rw("source_included", &GlobalLoggingConfiguration::sourceIncluded, "Whether logger channels are included.")
        .def_rw("process_id_included", &GlobalLoggingConfiguration::processIdIncluded,
                "Whether process identifiers are included.")
        .def_rw("trace_id_included", &GlobalLoggingConfiguration::traceIdIncluded,
                "Whether active trace identifiers are included.")
        .def_rw("color_included", &GlobalLoggingConfiguration::colorIncluded, "Whether terminal color codes are included.")
        .def_rw("force_ansi_color", &GlobalLoggingConfiguration::forceAnsiColor, "Whether ANSI color codes are forced.")
        .def_rw("multiprocess_group_id", &GlobalLoggingConfiguration::multiprocessGroupId,
                "A nonzero identifier for cross-process output serialization.")
        .def_rw("channels", &GlobalLoggingConfiguration::channels, "Per-channel enablement and threshold patches.");

    nb::class_<Logger>(module, "Logger",
                       R"doc(Logger with one immutable channel.

Construction caches the channel without selecting or initializing a logging backend.

Args:
    channel: Non-empty channel attached to every submitted record.

Raises:
    ValueError: If ``channel`` is empty.

Example:

.. code-block:: python

    >>> from isaacsim.common.logging import Logger

    >>> logger = Logger("isaacsim.simulation")
)doc")
        .def(nb::init<const std::string&>(), nb::arg("channel"))
        .def_prop_ro(
            "channel", [](const Logger& logger) { return std::string(logger.getChannel()); },
            "The logger's immutable channel.")
        .def("is_enabled", &Logger::isEnabled, nb::arg("severity"),
             R"doc(Return whether the backend currently admits a severity for this channel.

Args:
    severity: Severity to query.

Returns:
    Whether a record at the severity is currently admitted.

Example:

.. code-block:: python

    >>> logger.is_enabled(LogLevel.INFO)
    True
)doc")
        .def(
            "log",
            [](const Logger& logger, LogLevel severity, std::string_view message, bool captureSource)
            { logWithReleasedGil(logger, severity, message, captureSource); },
            nb::arg("severity"), nb::arg("message"), nb::kw_only(), nb::arg("capture_source") = false,
            R"doc(Submit an already formatted message.

Args:
    severity: Severity assigned to the record.
    message: Message to submit.
    capture_source: Whether to attach the immediate Python caller's filename, function, and line.

Example:

.. code-block:: python

    >>> logger.log(LogLevel.INFO, "Simulation initialized")
)doc")
        .def(
            "verbose",
            [](const Logger& logger, std::string_view message, bool captureSource)
            { logWithReleasedGil(logger, LogLevel::eVerbose, message, captureSource); },
            nb::arg("message"), nb::kw_only(), nb::arg("capture_source") = false,
            R"doc(Submit a VERBOSE record.

Args:
    message: Message to submit.
    capture_source: Whether to attach the immediate Python caller's filename, function, and line.

Example:

.. code-block:: python

    >>> logger.verbose("Loaded detailed simulation state")
)doc")
        .def(
            "info",
            [](const Logger& logger, std::string_view message, bool captureSource)
            { logWithReleasedGil(logger, LogLevel::eInfo, message, captureSource); },
            nb::arg("message"), nb::kw_only(), nb::arg("capture_source") = false,
            R"doc(Submit an INFO record.

Args:
    message: Message to submit.
    capture_source: Whether to attach the immediate Python caller's filename, function, and line.

Example:

.. code-block:: python

    >>> logger.info("Simulation initialized")
)doc")
        .def(
            "warning",
            [](const Logger& logger, std::string_view message, bool captureSource)
            { logWithReleasedGil(logger, LogLevel::eWarning, message, captureSource); },
            nb::arg("message"), nb::kw_only(), nb::arg("capture_source") = false,
            R"doc(Submit a WARNING record.

Args:
    message: Message to submit.
    capture_source: Whether to attach the immediate Python caller's filename, function, and line.

Example:

.. code-block:: python

    >>> logger.warning("Using the fallback backend")
)doc")
        .def(
            "warn",
            [](const Logger& logger, std::string_view message, bool captureSource)
            { logWithReleasedGil(logger, LogLevel::eWarning, message, captureSource); },
            nb::arg("message"), nb::kw_only(), nb::arg("capture_source") = false,
            R"doc(Submit a WARNING record.

This is a compatibility alias for :meth:`warning`.

Args:
    message: Message to submit.
    capture_source: Whether to attach the immediate Python caller's filename, function, and line.

Example:

.. code-block:: python

    >>> logger.warn("Using the fallback backend")
)doc")
        .def(
            "error",
            [](const Logger& logger, std::string_view message, bool captureSource)
            { logWithReleasedGil(logger, LogLevel::eError, message, captureSource); },
            nb::arg("message"), nb::kw_only(), nb::arg("capture_source") = false,
            R"doc(Submit an ERROR record.

Args:
    message: Message to submit.
    capture_source: Whether to attach the immediate Python caller's filename, function, and line.

Example:

.. code-block:: python

    >>> logger.error("Simulation initialization failed")
)doc")
        .def(
            "report",
            [](const Logger& logger, std::string_view message, bool captureSource)
            { reportWithReleasedGil(logger, message, captureSource); },
            nb::arg("message"), nb::kw_only(), nb::arg("capture_source") = false,
            R"doc(Print ``[channel] message`` to standard output and submit an INFO record.

Args:
    message: Message to print after the channel prefix and submit.
    capture_source: Whether to attach the immediate Python caller's filename, function, and line.

Example:

.. code-block:: python

    >>> logger.report("Calculation result: 42")
)doc");

    module.def("configure_global", &configureGlobalWithReleasedGil, nb::arg("config"),
               R"doc(Apply selected process-wide logging settings.

Args:
    config: Configuration patch to apply.

Returns:
    Result of applying the selected settings.

Example:

.. code-block:: python

    >>> configure_global(config)
    ConfigureResult.SUCCESS
)doc");
    module.def(
        "flush",
        []
        {
            nb::gil_scoped_release release;
            flush();
        },
        R"doc(Flush records pending in the process-wide native logging backend.

Example:

.. code-block:: python

    >>> flush()
)doc");
}
