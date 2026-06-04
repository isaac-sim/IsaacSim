# SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

# NOTE: Import here your extension examples to be propagated to ISAAC SIM Extensions startup

"""Interactive humanoid robot policy example demonstrating a Unitree H1 robot with flat terrain locomotion using Isaac Lab."""

from .humanoid_example import HumanoidExample
from .humanoid_example_extension import HumanoidExampleExtension

__all__ = ["HumanoidExample"]
