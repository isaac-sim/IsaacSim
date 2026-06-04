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

"""Test for foundation."""

import isaacsim.core.experimental.utils.foundation as foundation_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import omni.kit.test
import usdrt
from pxr import Sdf


class TestFoundation(omni.kit.test.AsyncTestCase):
    """Test foundation."""

    async def setUp(self):
        """Method called to prepare the test fixture."""
        super().setUp()
        # create new stage
        await stage_utils.create_new_stage_async()

    async def tearDown(self):
        """Method called immediately after the test method has been called."""
        super().tearDown()

    # --------------------------------------------------------------------

    async def test_get_value_type_names(self):
        """Test get value type names."""
        [self.assertIsInstance(item, str) for item in foundation_utils.get_value_type_names(format=str)]
        [
            self.assertIsInstance(item, Sdf.ValueTypeName)
            for item in foundation_utils.get_value_type_names(format=Sdf.ValueTypeNames)
        ]
        [
            self.assertIsInstance(item, usdrt.Sdf.ValueTypeName)
            for item in foundation_utils.get_value_type_names(format=usdrt.Sdf.ValueTypeNames)
        ]

    async def test_resolve_value_type_name(self):
        """Test resolve value type name."""
        # str
        for item in foundation_utils.get_value_type_names(format=str):
            value_type_name = foundation_utils.resolve_value_type_name(item, backend="usd")
            self.assertIsInstance(value_type_name, Sdf.ValueTypeName)
            value_type_name = foundation_utils.resolve_value_type_name(item, backend="usdrt")
            self.assertIsInstance(value_type_name, usdrt.Sdf.ValueTypeName)
            value_type_name = foundation_utils.resolve_value_type_name(item, backend="fabric")
            self.assertIsInstance(value_type_name, usdrt.Sdf.ValueTypeName)
        # USD
        value_type_names = [
            getattr(Sdf.ValueTypeNames, item)
            for item in dir(Sdf.ValueTypeNames)
            if isinstance(getattr(Sdf.ValueTypeNames, item), Sdf.ValueTypeName)
        ]
        for item in value_type_names:
            if str(item).replace("[]", "") not in ["group", "opaque", "pathExpression"]:
                value_type_name = foundation_utils.resolve_value_type_name(item, backend="usd")
                self.assertIsInstance(value_type_name, Sdf.ValueTypeName)
                value_type_name = foundation_utils.resolve_value_type_name(item, backend="usdrt")
                self.assertIsInstance(value_type_name, usdrt.Sdf.ValueTypeName)
                value_type_name = foundation_utils.resolve_value_type_name(item, backend="fabric")
                self.assertIsInstance(value_type_name, usdrt.Sdf.ValueTypeName)
        # USDRT
        value_type_names = [
            getattr(usdrt.Sdf.ValueTypeNames, item)
            for item in dir(usdrt.Sdf.ValueTypeNames)
            if isinstance(getattr(usdrt.Sdf.ValueTypeNames, item), usdrt.Sdf.ValueTypeName)
        ]
        for item in value_type_names:
            if not item.GetAsToken().startswith("tag") and not item.GetAsToken() == "double6":
                value_type_name = foundation_utils.resolve_value_type_name(item, backend="usd")
                self.assertIsInstance(value_type_name, Sdf.ValueTypeName)
                value_type_name = foundation_utils.resolve_value_type_name(item, backend="usdrt")
                self.assertIsInstance(value_type_name, usdrt.Sdf.ValueTypeName)
                value_type_name = foundation_utils.resolve_value_type_name(item, backend="fabric")
                self.assertIsInstance(value_type_name, usdrt.Sdf.ValueTypeName)
