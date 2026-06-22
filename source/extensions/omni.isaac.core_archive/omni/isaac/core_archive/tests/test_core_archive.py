# SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

"""Verifies that the core archive extension packages all required third-party Python dependencies by importing each dependency in the Kit test environment."""

import omni.kit.test


class TestPipArchive(omni.kit.test.AsyncTestCase):
    """TestPipArchive implementation."""

    # import all packages to make sure dependencies were not missed
    async def test_import_all(self) -> None:
        """Verify import all."""
        import contourpy
        import cycler
        import dateutil
        import fontTools
        import kiwisolver
        import llvmlite
        import matplotlib
        import nest_asyncio
        import osqp
        import packaging
        import pyparsing
        import pyperclip
        import qdldl
        import six

        self.assertIsNotNone(contourpy)
        self.assertIsNotNone(cycler)
        self.assertIsNotNone(dateutil)
        self.assertIsNotNone(fontTools)
        self.assertIsNotNone(kiwisolver)
        self.assertIsNotNone(llvmlite)
        self.assertIsNotNone(matplotlib)
        self.assertIsNotNone(nest_asyncio)
        self.assertIsNotNone(osqp)
        self.assertIsNotNone(packaging)
        self.assertIsNotNone(pyparsing)
        self.assertIsNotNone(pyperclip)
        self.assertIsNotNone(qdldl)
        self.assertIsNotNone(six)
