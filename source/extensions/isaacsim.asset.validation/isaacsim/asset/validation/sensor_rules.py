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

"""Registration hub for authored RTX sensor asset requirements."""

from . import camera_core_rules as camera_core_rules
from . import core_sensor_contract_rules as core_sensor_contract_rules
from . import depth_single_view_rules as depth_single_view_rules
from . import generic_camera_isp_rules as generic_camera_isp_rules
from . import lens_lut_rules as lens_lut_rules
from . import lidar_sensor_rules as lidar_sensor_rules
from . import radar_sensor_rules as radar_sensor_rules
from . import render_graph_rules as render_graph_rules
from . import rgb_output_rules as rgb_output_rules
from . import shippable_asset_structure_rules as shippable_asset_structure_rules
from . import spg_sensor_rules as spg_sensor_rules
from .sensor_rule_utils import RULES_BY_REQUIREMENT_ID

__all__ = ["RULES_BY_REQUIREMENT_ID"]
