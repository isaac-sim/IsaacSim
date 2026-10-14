# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Verify that the staged logging module submits through Kit's Carbonite backend."""

from __future__ import annotations

import asyncio
import inspect
import uuid

import carb.logging
import omni.kit.test
from isaacsim.common.logging import Logger, flush

LogRecord = tuple[str, int, str, int, str]

# Coverage discovery may import this module before the extension plugin starts. Constructing a logger here verifies
# that channel creation remains backend-neutral and does not prevent the plugin from attaching Kit's logging backend.
_PRESTART_LOGGER = Logger("isaacsim.common.tests.python.prestart")


def _emit_from_python_wrapper(logger: Logger, message: str) -> int:
    """Emit one record and return the wrapper line expected in its source metadata."""
    frame = inspect.currentframe()
    if frame is None:
        raise RuntimeError("The Python interpreter did not provide the current frame")
    expected_line = frame.f_lineno + 1
    logger.warning(message, capture_source=True)
    return expected_line


class TestLoggingIntegration(omni.kit.test.AsyncTestCase):
    """Exercise the wheel import and borrowed Kit backend together."""

    async def test_python_record_reaches_kit_logging_callback(self) -> None:
        """Observe a module record and safe recursive submission through Kit."""
        marker = f"isaacsim-common-test-{uuid.uuid4()}"
        recursive_marker = f"isaacsim-common-recursive-{uuid.uuid4()}"
        records: list[LogRecord] = []
        kit_logging = carb.logging.acquire_logging()
        logger = _PRESTART_LOGGER

        def capture(source: str, level: int, filename: str, line_number: int, message: str) -> None:
            if marker in message:
                records.append((source, level, filename, line_number, message))
                logger.warning(recursive_marker)

        handle = kit_logging.add_logger(capture)
        try:
            logger.warning(marker)
            flush()
            for _ in range(20):
                if records:
                    break
                await asyncio.sleep(0.01)
        finally:
            flush()
            kit_logging.remove_logger(handle)

        self.assertEqual(len(records), 1)
        self.assertEqual(records[0][0], logger.channel)
        self.assertEqual(records[0][1], carb.logging.LEVEL_WARN)
        self.assertFalse(records[0][2])
        self.assertEqual(records[0][3], 0)

    async def test_python_direct_call_captures_source_metadata(self) -> None:
        """Capture the test file and exact logger call line and show all source fields in Kit output."""
        channel = "isaacsim.common.tests.python.source.direct"
        marker = f"isaacsim-common-direct-source-{uuid.uuid4()}"
        records: list[LogRecord] = []
        kit_logging = carb.logging.acquire_logging()
        logger = Logger(channel)

        def capture(source: str, level: int, filename: str, line_number: int, message: str) -> None:
            if marker in message:
                records.append((source, level, filename, line_number, message))

        handle = kit_logging.add_logger(capture)
        try:
            frame = inspect.currentframe()
            self.assertIsNotNone(frame)
            expected_line = frame.f_lineno + 1
            logger.warning(marker, capture_source=True)
            flush()
            for _ in range(20):
                if records:
                    break
                await asyncio.sleep(0.01)
        finally:
            kit_logging.remove_logger(handle)

        self.assertEqual(len(records), 1)
        self.assertEqual(records[0][0], channel)
        self.assertEqual(records[0][1], carb.logging.LEVEL_WARN)
        self.assertTrue(records[0][2].endswith("test_logging.py"))
        self.assertEqual(records[0][3], expected_line)

    async def test_python_wrapper_captures_wrapper_source_metadata(self) -> None:
        """Verify that automatic capture attributes a wrapped call to the immediate Python wrapper."""
        channel = "isaacsim.common.tests.python.source.wrapper"
        marker = f"isaacsim-common-wrapper-source-{uuid.uuid4()}"
        records: list[LogRecord] = []
        kit_logging = carb.logging.acquire_logging()
        logger = Logger(channel)

        def capture(source: str, level: int, filename: str, line_number: int, message: str) -> None:
            if marker in message:
                records.append((source, level, filename, line_number, message))

        handle = kit_logging.add_logger(capture)
        try:
            expected_line = _emit_from_python_wrapper(logger, marker)
            flush()
            for _ in range(20):
                if records:
                    break
                await asyncio.sleep(0.01)
        finally:
            kit_logging.remove_logger(handle)

        self.assertEqual(len(records), 1)
        self.assertEqual(records[0][0], channel)
        self.assertEqual(records[0][1], carb.logging.LEVEL_WARN)
        self.assertTrue(records[0][2].endswith("test_logging.py"))
        self.assertEqual(records[0][3], expected_line)
