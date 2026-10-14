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

"""Verify gain-readiness warnings reach the final Asset Transformer result."""

import tempfile
from types import SimpleNamespace
from unittest.mock import Mock

import omni.kit.test
from isaacsim.asset.transformer.ui.window import AssetTransformerWindow


class TestConversionWarning(omni.kit.test.AsyncTestCase):
    """Exercise the report-to-dialog path without constructing an Omni UI window."""

    async def test_successful_conversion_with_missing_gains_shows_review_dialog(self) -> None:
        """A successful rule must still surface missing gains in the final result."""
        warning = "Control readiness warning: 2 new actuator(s) lack authored gain/bias parameters."
        result = SimpleNamespace(success=True, log=[{"message": warning}])
        with tempfile.TemporaryDirectory(prefix="conversion_warning_") as output_dir:
            report = SimpleNamespace(
                results=[result], package_root=output_dir, output_stage_path=None, to_dict=lambda: {}
            )
            window = SimpleNamespace(
                _file_data=SimpleNamespace(
                    autoload_option=SimpleNamespace(model=SimpleNamespace(get_value_as_bool=lambda: False))
                ),
                _resolve_input_stage_path=lambda: ("/tmp/input.usda", None),
                _build_profile_from_actions=lambda: SimpleNamespace(rules=[object()], output_package_root=output_dir),
                _manager=SimpleNamespace(run=lambda *_args, **_kwargs: report),
                _show_error_dialog=Mock(),
            )
            AssetTransformerWindow._run_actions_after_precheck(window)
            window._show_error_dialog.assert_called_once()
            self.assertEqual(window._show_error_dialog.call_args.args[0], "Conversion Needs Gain Review")
            self.assertIn(warning, window._show_error_dialog.call_args.args[1])
