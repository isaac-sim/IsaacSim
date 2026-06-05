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

"""Shared test data paths for rules tests."""

import os

_TEST_DATA_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(__file__)))))),
    "data",
    "tests",
)

_UR10E_USD = os.path.join(_TEST_DATA_DIR, "ur10e", "ur10e.usd")
_UR10E_SHOULDER_USD = os.path.join(_TEST_DATA_DIR, "ur10e_shoulder", "ur10e.usda")
_TEST_ADVANCED_USD = os.path.join(_TEST_DATA_DIR, "test_advanced", "usdex", "test_advanced.usda")
_TEST_COLLISION_FROM_VISUALS_USD = os.path.join(
    _TEST_DATA_DIR, "test_collision_from_visuals", "test_collision_from_visuals.usda"
)
_INSPIRE_HAND_DIR = os.path.join(_TEST_DATA_DIR, "inspire_hand")
_INSPIRE_HAND_MATERIALS_USDA = os.path.join(_INSPIRE_HAND_DIR, "inspire_hand_materials.usda")
