# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Regression coverage for the IsaacReadFilePath compute return value."""

import tempfile
from types import SimpleNamespace

import omni.kit.test
from isaacsim.core.nodes.ogn.python.nodes.OgnIsaacReadFilePath import OgnIsaacReadFilePath


class TestReadFilePathSuccess(omni.kit.test.AsyncTestCase):
    """Verify successful file reads report successful OmniGraph computation."""

    async def test_successful_read_returns_true(self) -> None:
        """A valid file should populate the output and return True."""
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8") as file:
            file.write("hello Isaac Sim")
            file.flush()

            db = SimpleNamespace(
                inputs=SimpleNamespace(path=file.name),
                outputs=SimpleNamespace(fileContents=""),
                log_warn=lambda _message: None,
            )

            self.assertTrue(OgnIsaacReadFilePath.compute(db))
            self.assertEqual(db.outputs.fileContents, "hello Isaac Sim")
