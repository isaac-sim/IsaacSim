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

"""Verify that the assets the scenario suites load are present.

A missing asset otherwise surfaces as a stage-open failure deep inside whichever scenario happened to run
first, so this checks for them up front.
"""

from __future__ import annotations

import os

import pytest


class TestDataPaths:
    """Verify that required scenario test assets exist."""

    @pytest.mark.parametrize(
        "asset",
        ("Ant.usda", "CartPole.usda", "CartPoleNoRail.usda", "CartRailNoPole.usda", "Humanoid.usda"),
    )
    def test_data_paths(self, asset: str) -> None:
        """Verify that a required scenario asset exists.

        Args:
            asset: Asset file name supplied by the pytest parameter.

        """
        from _scenario import get_asset_root

        asset_root = get_asset_root()
        assert os.path.isdir(asset_root)
        assert os.path.isfile(os.path.join(asset_root, asset)), f"missing test asset {asset!r}"
