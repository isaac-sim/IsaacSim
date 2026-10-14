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

"""Tests for shared authored-sensor rule helpers."""

from __future__ import annotations

from unittest.mock import Mock

import omni.kit.test
from isaacsim.asset.validation.sensor_rule_utils import AttributeNamespace, make_bound_check
from pxr import Sdf, Usd


class TestBoundCheck(omni.kit.test.AsyncTestCase):
    """Check shared bound validation and reporting behavior."""

    async def test_missing_invalid_valid_and_conditional_values(self) -> None:
        """Fail missing and invalid values, and skip valid or inactive values."""
        stage = Usd.Stage.CreateInMemory()
        prim = stage.DefinePrim("/Sensor")
        checker = Mock()
        requirement = object()
        check = make_bound_check(
            lambda stage: [stage.GetPrimAtPath("/Sensor")],
            AttributeNamespace("test:"),
            "gain",
            lambda value: value > 0,
            "positive",
        )

        check(checker, stage, requirement)
        self.assertEqual(checker._AddFailedCheck.call_args.kwargs["at"], prim)
        self.assertEqual(checker._AddFailedCheck.call_args.kwargs["message"], "test:gain on /Sensor must be positive")

        attribute = prim.CreateAttribute("test:gain", Sdf.ValueTypeNames.Float)
        attribute.Set(0)
        checker.reset_mock()
        check(checker, stage, requirement)
        self.assertEqual(checker._AddFailedCheck.call_args.kwargs["at"], attribute)

        attribute.Set(1)
        checker.reset_mock()
        check(checker, stage, requirement)
        checker._AddFailedCheck.assert_not_called()

        attribute.Set(0)
        conditional_check = make_bound_check(
            lambda stage: [stage.GetPrimAtPath("/Sensor")],
            AttributeNamespace("test:"),
            "gain",
            lambda value: value > 0,
            "positive",
            condition=lambda prim: False,
        )
        conditional_check(checker, stage, requirement)
        checker._AddFailedCheck.assert_not_called()

    async def test_custom_failure_message(self) -> None:
        """Preserve modality-specific diagnostics when a formatter is supplied."""
        stage = Usd.Stage.CreateInMemory()
        prim = stage.DefinePrim("/Sensor")
        checker = Mock()
        check = make_bound_check(
            lambda stage: [stage.GetPrimAtPath("/Sensor")],
            AttributeNamespace("test:"),
            "gain",
            lambda value: value > 0,
            "must be positive",
            message=lambda prim, name, value, description: f"Sensor {prim.GetPath()} {name}={value!r} {description}",
        )

        check(checker, stage, object())
        self.assertEqual(
            checker._AddFailedCheck.call_args.kwargs["message"],
            "Sensor /Sensor gain=None must be positive",
        )
        self.assertEqual(checker._AddFailedCheck.call_args.kwargs["at"], prim)
