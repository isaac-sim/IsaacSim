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

"""Configuration definitions for supported Acoustic sensors in Isaac Sim.

This module defines the supported Acoustic sensor configurations and their variants
that can be used with the RTX sensor system. It includes configurations for various
manufacturers.
"""

#: Expected name of Acoustic prim variant sets.
SUPPORTED_ACOUSTIC_VARIANT_SET_NAME = "sensor"

#: Map of supported Acoustic asset paths to their variant name sets.
SUPPORTED_ACOUSTIC_CONFIGS: dict[str, set[str]] = {}
